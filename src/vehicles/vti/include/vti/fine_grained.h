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
#ifndef VTI_FG_H
#define VTI_FG_H
/**
* \class FineGrained
*
* Class that has VTI equations for fine-grained soil
* Based on the paper:
* Dettwiller, I., Vahedifard, F., Rais-Rohani, M., Mason, G. L., & Priddy, J. D. (2018). 
* Improving accuracy of vehicle-terrain interface algorithms for wheeled vehicles on fine-grained soils through Bayesian calibration. 
* Journal of Terramechanics, 77, 59-68.
*
* Note that variable names are taken from the paper
* 
* \author Chris Goodin
*
* \date 2/6/2019
*/

#include "vti/vti.h"

namespace mavs {
namespace vti {

class FineGrained : public MavsVti {
public:
	/// Construct a fine-grained VTI model
	FineGrained();

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
	/// Fine-grained numeric
	double N_c_;
	/// Fine-grained sinkage numeric
	double N_cz_;
	/// Self-propelled slip
	double S_sp_;
	/// variables that are used repeatedly in calculations
	double a1_,d1_;
	/// method to calculate numerics based on current load, deflection, rci, and slip
	void CalculateNumerics(double rci, double load, double deflection, double slip);

	//-- outputs calculated with each update ---//
	void CalculateTraction(double slip);
	void CalculateMotionResistance();
	void CalculatePoweredSinkage();
	void CalculateUnpoweredSinkage();
	void CalculateDrawbarPull(double slip);
};

} //namespace vti
} //namespace mavs

#endif