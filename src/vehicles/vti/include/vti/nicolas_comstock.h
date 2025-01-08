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
#ifndef VTI_NICSTOCK_H
#define VTI_NICSTOCK_H
/**
* \class NicolasComstock
*
* Equations for converting from pure lateral and 
* longitudinal slip to combined slip
*
* Equations from the papers:
* Brach, R., & Brach, M. (2011).
* The tire-force ellipse (friction ellipse) and tire characteristics
* (No. 2011-01-0094). SAE Technical Paper.
*
* Brach, R. M., & Brach, R. M. (2009).
* Tire models for vehicle dynamic simulation and accident reconstruction
* (No. 2009-01-0102). SAE Technical Paper.
*
* \author Chris Goodin
*
* \date 2/8/2019
*/
#include "vti/vti.h"

namespace mavs {
namespace vti {

class NicolasComstock {
public:
	NicolasComstock();

	void SetParameters(double slip_stiffness, double slip_angle_stiffness) {
		C_s_ = slip_stiffness;
		C_a_ = slip_angle_stiffness;
		C_a_2_ = C_a_ * C_a_;
		C_s_2_ = C_s_ * C_s_;
	}

	TireForces GetForces(double pure_longi, double pure_lat, double slip, double alpha);

private:
	//slip stiffness
	double C_s_; //Newtons
	double C_s_2_;
	//slip angle stiffness
	double C_a_; //Newtons/radian
	double C_a_2_;
};

} //namespace vti
} //namespace mavs

#endif