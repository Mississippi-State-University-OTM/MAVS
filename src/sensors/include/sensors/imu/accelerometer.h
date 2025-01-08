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
* \class Accelerometer
*
* An instance of a MEMS sensor with base units of
* m/s^2 that measures the linear acceleration along
* each axis of the sensor.
*
* Example parameters on
* https://www.mathworks.com/help/fusion/ref/accelparams-class.html
*
* \author Chris Goodin
*
* \date 10/12/2018
*/
#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <sensors/imu/mems_sensor.h>

namespace mavs {
namespace sensor {
namespace imu {

class Accelerometer : public MemsSensor {
public:
	/// Create an accelerometer sensor with default parameters
	Accelerometer() {
		measurement_range_ = 19.6f; //m/s^2
		resolution_ = 0.598E-3f; //(m/s^2)/LSB
		constant_bias_ = glm::vec3(0.49f, 0.49f, 0.49f); //m/s^2
		noise_density_ = glm::vec3(3920.0E-6f, 3920.0E-6f, 3920.0E-6f); //(m/s^2)/sqrt(Hz)
		bias_instability_ = glm::vec3(0.0f,0.0f,0.0f); //m/s^2
		random_walk_ = glm::vec3(0.0f,0.0f,0.0f); //(m/s^2)/sqrt(Hz)
		axis_misalignment_ = glm::vec3(0.0f, 0.0f, 0.0f);  // skew %
		SetAxisMisalignment(axis_misalignment_);
		temperature_bias_ = glm::vec3(0.294f, 0.294f, 0.294f); //(m/s^2)/C
		temperature_scale_factor_ = glm::vec3(0.02f, 0.02f, 0.02f); //%/C
		acceleration_bias_ = glm::vec3(0.0f, 0.0f, 0.0f); // (m/s^2)/(m/s^2)
	}

};

} //namespace imu
} //namespace sensor
} //namespace mavs

#endif
