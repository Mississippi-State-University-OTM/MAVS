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
* \class Magnetometer
*
* An instance of a MEMS Sensor with base units of
* micro-tesla (uT) that measures the magnetic
* field along each axis in the vicinity of the sensor.
*
* Example parameters on 
* https://www.mathworks.com/help/fusion/ref/magparams-class.html
*
* \author Chris Goodin
*
* \date 10/12/2018
*/
#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <sensors/imu/mems_sensor.h>

namespace mavs {
namespace sensor {
namespace imu {

class Magnetometer : public MemsSensor {
public:
	/// Create a magnetometer sensor with default parameters
	Magnetometer() {
		measurement_range_ = 1200.0f;// uT/s
		resolution_ = 0.1f; // (uT/s)/LSB
		constant_bias_ = glm::vec3(1.0f,1.0f,1.0f);// uT/s
		noise_density_ = glm::vec3(0.06f,0.06f,0.09f); // (uT)/sqrt(Hz)
		bias_instability_ = glm::vec3(0.0f,0.0f,0.0f); // uT
		random_walk_ = glm::vec3(0.0f,0.0f,0.0f); // (uT)/sqrt(Hz)
		axis_misalignment_ = glm::vec3(0.0f, 0.0f, 0.0f); // skew %
		SetAxisMisalignment(axis_misalignment_);
		temperature_bias_ = glm::vec3(0.8f, 0.8f, 2.4f); //(uT)/C
		temperature_scale_factor_ = glm::vec3(0.1f, 0.1f, 0.1f); //%/C
		acceleration_bias_ = glm::vec3(0.0f, 0.0f, 0.0f); // (uT)/(m/s^2)
	}
};

} //namespace imu
} //namespace sensor
} //namespace mavs

#endif
