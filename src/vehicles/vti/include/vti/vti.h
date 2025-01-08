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
#ifndef MAVS_VTI_H
#define MAVS_VTI_H
/**
* \class MavsVti
*
* Class that has VTI base functions and data
* Vehicle-Terrain Interaction is for a single wheel
*
* \author Chris Goodin
*
* \date 2/6/2019
*/

namespace mavs {
namespace vti {


struct TireForces {
	double lateral;
	double longitudinal;
	double normal;
};


class MavsVti {
public:
	/// Constructor for base vti
	MavsVti() {
		powered_ = true;
	}

	/**
	* Set the wheel properties
	* \param width Wheel width in meters
	* \param diameter Wheel diameter in meters
	* \param section_height Wheel section height in meters
	*/
	virtual void SetWheelProperties(double width, double diameter, double section_height) {}

	/**
	* Update the outputs of the VTI model
	* \param rci Remold-Cone Index of the current terrain at the wheel (kPa)
	* \param load Current load on the wheel (kN)
	* \param deflection  Current deflection (delta) of the wheel (meters)
	* \param slip Slip of the current wheel
	* \param alpha Steering angle in radians
	*/
	virtual void Update(double rci, double load, double deflection, double slip, double alpha) {}

	/**
	* Set if the wheel is powered or unpowerd
	* \param powered Set to true if powered, false if unpowered
	*/
	void SetPowered(bool powered) { powered_ = powered; }

	/// Return the motion resistance (Newtons) calculated at the last Update
	double GetLongitudinalMotionResistance() { return longi_motion_resistance_; }

	/// Return the drawbar pull (Newtons) calculated at the last Update
	double GetDrawbarPull() { return drawbar_pull_; }

	/// Return the longitudinal traction (Newtons) calculated at the last Update
	double GetLongitudinalTraction() { return longi_traction_; }

	/// Return the lateral traction (Newtons) calculated at the last Update
	double GetLateralTraction() { return lat_traction_; }

	/// Return the sinkage (meters) calculated at the last Update
	double GetSinkage() { return sinkage_; }

protected:
	// tire parameters
	double b_;
	double d_;
	double h_;
	bool powered_;

	// calculated parameters
	double drawbar_pull_;
	double sinkage_;
	double longi_motion_resistance_;
	double longi_traction_;
	double lat_traction_;
};

} //namespace vti
} //namespace mavs

#endif