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
#ifndef MAVS_TIRE_H
#define MAVS_TIRE_H
/**
* \class MavsVti
*
* Class that has complete MAVS tire model
* including coordinate transformations
*
* \author Chris Goodin
*
* \date 2/8/2019
*/

//#include <mavs_core/environment/environment.h>
#include "vti/spring_tire.h"
#include "vti/brixius.h"

namespace mavs {
namespace vti {

class Tire {
public:
	/// Constructor for tire
	Tire();

	void Update(double dt, double zpos, double ground_height, double slip, double slip_angle);

	double GetNormalForce() {
		return forces_.normal;
	}

	double GetLongitudinalForce() {
		return forces_.longitudinal;
	}

	double GetLateralForce() {
		return forces_.lateral;
	}

	void SetGeometry(double radius, double width) {
		radius_ = radius;
		double d = 2.0*radius;
		double h = 0.22*d;
		vti_.SetWheelProperties(width, d, h);
		spring_tire_.SetRadius(radius, h);
	}

private:
	SpringTire spring_tire_;
	Brixius vti_;
	double radius_;
	TireForces forces_;
};

} //namespace vti
} //namespace mavs

#endif