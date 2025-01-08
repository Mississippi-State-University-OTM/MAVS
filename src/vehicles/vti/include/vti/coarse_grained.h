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
#ifndef VTI_CG_H
#define VTI_CG_H
/**
* \class CoarseGrained
*
* Class that has VTI equations for coarse-grained soil
*
* Based on papers:
*
* Dettwiller, I., Rais - Rohani, M., Vahedifard, F., Mason, G.L., &Priddy, J.D. (2017).
* Bayesian calibration of Vehicle-Terrain Interface algorithms for wheeled vehicles on loose sands.
* Journal of Terramechanics, 71, 45 - 56. [Dettwiller2017]
*
* Williams, J. M., Vahedifard, F., Mason, G. L., & Priddy, J. D. (2017).
* New algorithms for predicting longitudinal motion resistance of wheels on dry sand.
* The Journal of Defense Modeling and Simulation, 1548512917693119. [Williams2017]
*
* Mason, G. L., Vahedifard, F., Robinson, J. D., Howard, I. L., McKinley, G. B.,
* & Priddy, J. D. (2016).
* Improved sinkage algorithms for powered and unpowered wheeled vehicles operating on sand.
* Journal of Terramechanics, 67, 25-36. [Mason2016]
*
* Mason, G. L., Williams, J. M., Vahedifard, F., & Priddy, J. D. (2018).
* A unified equation for predicting traction for wheels on sand over a range of
* braked, towed, and powered operations.
* Journal of Terramechanics, 79, 33-40. [Mason2018]
*
* Note that variable names are taken from the papers
* 
* \author Chris Goodin
*
* \date 2/6/2019
*/
#include "vti/vti.h"
namespace mavs {
namespace vti {

class CoarseGrained : public MavsVti{
public:
	/// Construct a coarse-grained VTI
	CoarseGrained();

	/**
	* Update the outputs of the VTI model, must be called at each time step
	* \param rci Remold-Cone Index of the current terrain at the wheel (kPa)
	* \param load Current load on the wheel (kN)
	* \param deflection  Current deflection (delta) of the wheel (meters)
	* \param slip Slip of the current wheel
	* \param alpha Steering angle in radians
	*/
	void Update(double rci, double load, double deflection, double slip, double alpha);

	/**
	* Set the wheel properties
	* \param width Wheel width in meters
	* \param diameter Wheel diameter in meters
	* \param section_height Wheel section height in meters
	*/
	void SetWheelProperties(double width, double diameter, double section_height);

private:
	//-- Numerics calculated with each update ----//
	/// coarse-grained numeric
	double N_s_;
	/// coarse-graned sinkage numeric, powered wheel
	double N_s_p_;
	/// coarse-graned sinkage numeric, un-powered wheel
	double N_s_u_;
	/// intermediate values that are reused
	double a1_;
	/// Contact Pressure
	double CP_;
	/// Unified coefficients
	double A_, B_, C_, max_trac_, min_trac_, ratio_;
	void CalculateNumerics(double rci, double load, double deflection, double slip);

	//-- outputs calculated with each update ---//
	void CalculateTraction(double slip);
	void CalculatePoweredMotionResistance(double rci, double slip);
	void CalculateUnpoweredMotionResistance(double rci, double slip);
	void CalculatePoweredSinkage();
	void CalculateUnpoweredSinkage();
	void CalculateDrawbarPull(double slip);
};

} //namespace vti
} //namespace mavs

#endif