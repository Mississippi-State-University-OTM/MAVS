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
#ifndef VTI_SPRINGTIRE_H
#define VTI_SPRINGTIRE_H
/**
* \class SpringTire
*
* Simple single spring-damper model for tire vertical forces
*
* \author Chris Goodin
*
* \date 2/8/2019
*/

namespace mavs {
namespace vti {

class SpringTire {
public:
	SpringTire();

	double Update(double dt, double deflected_radius);

	void SetLoadDeflection(double load, double deflection);

	void SetSpringDamping(double k, double c);

	void SetRadius(double radius, double section_height) {
		radius_ = radius;
		section_height_ = section_height;
	}

private:
	void SetCriticalDamping(double mass);
	double spring_constant_;
	double damping_coefficient_;

	double current_compression_;
	double current_velocity_;

	double radius_;
	double section_height_;
};

} //namespace vti
} //namespace mavs

#endif