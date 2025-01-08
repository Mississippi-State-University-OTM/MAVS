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
* \class Gyroscope
*
* An instance of a MEMS sensor with base units of
* rad/s that measures the angular velocity along
* each axis of the sensor.
*
* Example parameters on
* https://www.mathworks.com/help/fusion/ref/gyroparams-class.html
*
* \author Chris Goodin
*
* \date 10/12/2018
*/
#ifndef GYRO_H
#define GYRO_H

#include <sensors/imu/mems_sensor.h>

namespace mavs {
namespace sensor {
namespace imu {

	class Gyro : public MemsSensor {
	public:
		/// Create a gyroscope sensor with default parameters
		Gyro() {
			measurement_range_ = 4.363f; //rad/s
			resolution_ = 1.332E-4f; // (rad/s)/LSB
			constant_bias_ = glm::vec3(0.349f, 0.349f, 0.349f); // rad/s
			noise_density_ = glm::vec3(8.727E-4f, 8.727E-4f, 8.727E-4f); // (rad/s)/sqrt(Hz)
			bias_instability_ = glm::vec3(0.0f, 0.0f, 0.0f); // rad/s
			random_walk_ = glm::vec3(0.0f, 0.0f, 0.0f); // (rad/s)/sqrt(Hz)
			axis_misalignment_ = glm::vec3(0.0f, 0.0f, 0.0f); // skew %
			SetAxisMisalignment(axis_misalignment_);
			temperature_bias_ = glm::vec3(0.349f, 0.349f, 0.349f); //(rad/s)/C
			temperature_scale_factor_ = glm::vec3(0.02f, 0.02f, 0.02f); //%/C
			acceleration_bias_ = glm::vec3(0.178E-3f, 0.178E-3f, 0.178E-3f); // (rad/s)/(m/s^2)
		}
	
};

} //namespace imu
} //namespace sensor
} //namespace mavs

#endif
