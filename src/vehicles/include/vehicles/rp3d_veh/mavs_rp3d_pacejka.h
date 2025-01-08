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
	* Strength is cone index in PSI. Only applied to clay and sand
	* \param type The surface type 
	* \param CI Cone index of the surface if it is clay or sand
	*/
	void SetSurfaceProperties(std::string type, float CI);

	/**
	* Set the slip angle at which the lateral force vs
	* slip curve "turns over", in radians.
	* \param angle Angle in radians
	*/
	void SetSlipAngleCrossover(float angle);

private:
	void SetCoeffs(float W, float delta);
	float GetCrolla(float alpha, float beta);
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
};

} // namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs

#endif