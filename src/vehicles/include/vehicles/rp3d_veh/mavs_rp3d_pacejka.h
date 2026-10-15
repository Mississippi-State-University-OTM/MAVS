/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
/**
* \class Pacejka
*
* Class for a Pacejka model. 
* Works for surfaces 'dry', 'wet', 'snow', 'ice', 'clay', and 'sand'
* For clay and sand, CI must be provided.
*
* Combined slip is handled in two stages:
*  1) Combined-slip weighting functions (Pacejka-style) reduce each pure-slip
*     force as the other slip grows: 
*        Fx *= cos(Cxa*atan(Bxa*alpha)),  Fy *= cos(Cyk*atan(Byk*slip))
*  2) A friction ellipse caps the resulting force vector at the surface limits.
*
* \author Chris Goodin
*
* \date 9/27/2019
*/
#ifndef MAVS_PACEJKA
#define MAVS_PACEJKA

#include <vector>
#include <string>
#include <reactphysics3d/reactphysics3d.h>

namespace mavs {
namespace vehicle {
namespace mavs_rp3d {

class MavsPacejka {
public:
	/// Method used to cap the combined tire force
	enum class FrictionLimit {
		kNone,                 ///< No cap, forces are independent
		kEllipse,              ///< Scale both components equally onto the friction ellipse
		kLongitudinalPriority  ///< Longitudinal force first, lateral gets the remaining capacity
	};

	/// Construct a default Pacekjka model
	MavsPacejka();

	/**
	* Get the comined longitudinal and lateral traction in the vehicle reference frame. 
	* Returns a vector where x is longitudinal and y is lateral.
	* \param normal_force Current normal force in Newtons
	* \param deflection Current tire deflection in meters
	* \param slip Current tire longitudinal slip
	* \param lat_slip Current slip angle in radians
	*/
	rp3d::Vector2 GetCombinedTraction(float normal_force, float deflection, float slip, float lat_slip);

	/**
	* Set the tire parameters.
	* These will be used in the friction calcuation for sand and clay
	* \param width Tire width in meters
	* \param diameter Tire diameter in meters
	* \param section_height Tire section height in meters
	*/
	void SetTireParameters(float width, float diameter, float section_height);

	/**
	* Set the default surface type. Options are
	* 'dry', 'wet', 'ice', 'snow', 'clay', and 'sand'
	* \param type The surface type
	*/
	void SetSurfaceType(std::string type);

	/**
	* Set the default surface type and strength. 
	* Surface type options are
	* 'dry', 'wet', 'ice', 'snow', 'clay', and 'sand'.
	* Strength is cone index in Pascals (the default is 250 psi = 250*6894.76 Pa).
	* Only applied to clay and sand.
	* \param type The surface type 
	* \param CI Cone index of the surface in Pa if it is clay or sand
	*/
	void SetSurfaceProperties(std::string type, float CI);

	/**
	* Set the slip angle at which the lateral force vs
	* slip curve "turns over", in DEGREES (converted to radians internally).
	* \param angle Angle in degrees
	*/
	void SetSlipAngleCrossover(float angle);

	/**
	* Set the method used to cap the combined force. Default is kEllipse.
	* \param mode The friction limit mode
	*/
	void SetFrictionLimitMode(FrictionLimit mode) { friction_limit_ = mode; }

	/**
	* Set the combined-slip weighting function coefficients.
	* Fx is reduced by cos(Cxa*atan(Bxa*alpha)), alpha in radians.
	* Fy is reduced by cos(Cyk*atan(Byk*slip)).
	* Larger B values make the force fall off faster with the other slip.
	* \param Bxa Stiffness of Fx reduction vs slip angle
	* \param Cxa Shape of Fx reduction vs slip angle
	* \param Byk Stiffness of Fy reduction vs longitudinal slip
	* \param Cyk Shape of Fy reduction vs longitudinal slip
	*/
	void SetCombinedSlipWeights(float Bxa, float Cxa, float Byk, float Cyk);

	/**
	* Enable or disable the combined-slip weighting functions. Default is enabled.
	* \param use True to enable
	*/
	void UseCombinedSlipWeighting(bool use) { use_weighting_ = use; }

	/**
	* Set the parameters that tie lateral friction to soil strength on clay and sand.
	* Clay: the peak lateral coefficient is the gross traction coefficient from the
	* clay traction equation evaluated at clay_ref_slip (near maximum pull).
	* Sand: the peak lateral coefficient is sand_mu_max*(1-exp(-Ns/sand_N0)),
	* where Ns is the Turnage sand numeric.
	* Both are floored at mu_min.
	* \param clay_ref_slip Reference slip for the clay lateral coefficient (default 0.2)
	* \param sand_mu_max Peak lateral coefficient on strong sand (default 0.7)
	* \param sand_N0 Sand numeric at which ~63% of sand_mu_max is reached (default 5)
	* \param mu_min Minimum lateral coefficient on very weak soil (default 0.05)
	*/
	void SetSoftSoilLateralParams(float clay_ref_slip, float sand_mu_max, float sand_N0, float mu_min);

private:
	void SetCoeffs(float W, float delta);
	float GetCrolla(float alpha, float beta);
	rp3d::Vector2 ApplyCombinedSlipWeighting(rp3d::Vector2 f, float slip, float alpha) const;
	rp3d::Vector2 ApplyFrictionLimit(rp3d::Vector2 f, float fx_max, float fy_max) const;
	float GetClayLateralCoeff(float Nc) const;
	float GetSandLateralCoeff(float W, float delta) const;
	float GetSandNumeric(float W, float delta) const;
	float B_;
	float C_;
	float D_;
	float D_lat_;
	float E_;
	float b_;
	float d_;
	float h_;
	float CI_;
	float high_slip_angle_cutoff_radians_;
	std::string surface_type_;

	// combined slip
	FrictionLimit friction_limit_;
	bool use_weighting_;
	float Bxa_;
	float Cxa_;
	float Byk_;
	float Cyk_;

	// soil-strength dependent lateral friction
	float clay_ref_slip_;
	float sand_mu_max_;
	float sand_N0_;
	float soft_mu_min_;
};

} // namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs

#endif
