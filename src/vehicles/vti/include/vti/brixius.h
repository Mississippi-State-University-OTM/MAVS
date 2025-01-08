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
#ifndef VTI_BRIXIUS_H
#define VTI_BRIXIUS_H
/**
* \class Brixius
*
* Class that has VTI equations from Brixius model
* Pan, W., Papelis, Y. E., & He, Y. (2004, September). 
* A vehicle-terrain system modeling and simulation approach 
* to mobility analysis of vehicles on soft terrain. 
* In Unmanned Ground Vehicle Technology VI (Vol. 5422, pp. 520-532). 
* International Society for Optics and Photonics.
*
* Note that variable names are taken from the paper
* 
* \author Chris Goodin
*
* \date 2/7/2019
*/

#include "vti/vti.h"

namespace mavs {
namespace vti {

class Brixius : public MavsVti {
public:
	/// Construct a fine-grained VTI model
	Brixius();

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

	void SetBiasPly() { is_bias_ply_ = true; }
	void SetRadial() { is_bias_ply_ = false; }

private:
	//-- Numerics calculated with each update ----//
	/// Wheel numeric
	double C_n_;
	/// Mobility number
	double B_n_;
	/// Lateral steering mobility number
	double B_n_1_;
	/// rolling radius
	double r_;
	/// rolling resistances
	double C_rr_road_, C_rr_tire_;
	/// intermediat calculation variables
	double d2_;
	/// method to calculate numerics based on current load, deflection, rci, and slip
	void CalculateNumerics(double rci, double load, double deflection, double slip);
	void CalculateResistance(double slip);
	void CalculateTraction (double slip);
	void CalculateLateralTraction(double steering);

	bool is_bias_ply_;

};

} //namespace vti
} //namespace mavs

#endif