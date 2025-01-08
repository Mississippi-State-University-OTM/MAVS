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
* \class MemsSensor
*
* Simulates a generic MEMS sensor, which could be a
* magnetometer, accelerometer, or gyroscope. While
* the parameters and equations are the same for the 
* different types of sensors, the units and values of 
* the parameters will change. There are "base units" 
* for each type of sensor.
*
* accelerometer base units = [m/s^2]
*
* gyroscope base units = [rad/s]
*
* magnetometer base units = [micro-tesla (uT)]
*
* Uses same framework as MATLAB model
* https://www.mathworks.com/help/fusion/ref/imusensor-system-object.html
*
* \author Chris Goodin
*
* \date 10/12/2018
*/

#ifndef MEMS_SENSOR_H
#define MEMS_SENSOR_H

#include <glm/glm.hpp>

namespace mavs {
namespace sensor {
namespace imu {

	class MemsSensor {
	public:
		/// MemsSensor Constructor
		MemsSensor() {}

		/// MemsSensor copy constructor
		MemsSensor(const MemsSensor &mems) {
			measurement_range_ = mems.measurement_range_;
			resolution_ = mems.resolution_;
			constant_bias_ = mems.constant_bias_;
			noise_density_ = mems.noise_density_;
			bias_instability_ = mems.bias_instability_;
			axis_misalignment_ = mems.axis_misalignment_;
			random_walk_ = mems.random_walk_;
			temperature_bias_ = mems.temperature_bias_;
			temperature_scale_factor_ = mems.temperature_scale_factor_;
			alignment_matrix_ = mems.alignment_matrix_;
			acceleration_bias_ = mems.acceleration_bias_;
		}

		/**
		* Set the measurement range to [-range,+range] in base units
		* \param range The desired maximum range
		*/
		void SetMeasurmentRange(float range) {
			measurement_range_ = range;
		}

		/**
		* Set the electronic resolution of the sensor 
		* in base units / LSB. 
		* \param res Desired resolution
		*/
		void SetResolution(float res) {
			resolution_ = res;
		}

		/**
		* Set the constant bias of the sensor
		* in base units, which will result in 
		* a constant offset value.
		* \param bias Desired constant offset in base units
		*/
		void SetConstantBias(glm::vec3 bias) {
			constant_bias_ = bias;
		}

		/**
		* Set the noise density of the sensor in units/sqrt(Hz)
		* This is the power spectral density of the sensor noise
		* \param nd The noise density value along each axis of the sensor
		*/
		void SetNoiseDensity(glm::vec3 nd) {
			noise_density_ = nd;
		}

		/**
		* Set the bias instability in base units
		* Results in noise that increases with larger measurments
		* \param bi The bias instability in base units
		*/
		void SetBiasInstability(glm::vec3 bi) {
			bias_instability_ = bi;
		}

		/**
		* Set the misalignment of the axis, in % skey
		* \param misalign The misalignment percentage in the local x-y-z of the sensor
		*/
		void SetAxisMisalignment(glm::vec3 misalign);

		/// Return the misalignment matrix for the sensor
		glm::mat3 GetAxisMisalignmentMatrix() {
			return alignment_matrix_;
		}

		/**
		* Set the random walk (white noise) error
		* in the sensor in units/sqrt(Hz)
		* \param rw The desired randow walk noise
		*/
		void SetRandomWalk(glm::vec3 rw) {
			random_walk_ = rw;
		}

		/**
		* Set how the sensor biases from temperature in units/Celsius
		* \param tb The temperature bias
		*/
		void SetTemperatureBias(glm::vec3 tb) {
			temperature_bias_ = tb;
		}

		/**
		* Set how the scale factor of the error with temperature, in 
		* percent/Celsius
		* \param tsf The temperature scale factor
		*/
		void SetTemperatureScaleFactor(glm::vec3 tsf) {
			temperature_scale_factor_ = tsf;
		}

		/**
		* For gyroscope sensor, set the sensor bias
		* caused by linear acceleration in (rad/s)/(m/s^2). 
		* Should be set to zeros for magnetometer 
		* and accelerometer.
		* \param ab The acceleration bias
		*/
		void SetAccelerationBias(glm::vec3 ab) {
			acceleration_bias_ = ab;
		}

		/**
		* Update the reading of the MEMS sensor
		* \param input The input vector, in base units in the LOCAL SENSOR FRAME!
		* \param temperature The ambient temperature of the sensor in degrees celsius
		* \param sample_rate The update rate of the sensor in Hz
		*/
		glm::vec3 Update(glm::vec3 input, float temperature, float sample_rate);

protected:
	float measurement_range_; 
	float resolution_; 
	glm::vec3 constant_bias_; 
	glm::vec3 noise_density_; 
	glm::vec3 bias_instability_; 
	glm::vec3 axis_misalignment_; 
	glm::vec3 random_walk_; 
	glm::vec3 temperature_bias_;
	glm::vec3 temperature_scale_factor_;
	glm::mat3 alignment_matrix_;
	glm::vec3 acceleration_bias_; 

	

private:
	float Filter1(float z, float sample_rate);
	float Filter2(float z);
};

} //namespace imu
} //namespace sensor
} //namespace mavs

#endif