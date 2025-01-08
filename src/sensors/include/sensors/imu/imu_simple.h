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
 * \class ImuSimple
 * 
 * Simulates an IMU sensor with a simple white noise + offset model
 *
 * \author Chris Goodin
 *
 * \date 10/6/21
 */

#ifndef IMU_SIMPLE_H
#define IMU_SIMPLE_H
#include <random>
#include <glm/glm.hpp>
#include <sensors/sensor.h>
//#include <mavs_core/math/constants.h>
#ifdef USE_MPI
#include <mpi.h>
#endif

namespace mavs{
namespace sensor{
namespace imu{

class ImuSimple : public Sensor {
 public:
  /// Create the simulated IMU
  ImuSimple();

  /// Imu destructor
  ~ImuSimple();
  
	/// Imu copy constructor
  ImuSimple(const ImuSimple &imu){
		local_acc_ = imu.local_acc_;
		local_angvel_ = imu.local_angvel_; 
		accel_readings_ = imu.accel_readings_; 
		gyro_readings_ = imu.gyro_readings_; 
		mag_readings_ = imu.mag_readings_; 
		accel_offsets_ = imu.accel_offsets_;
		accel_noise_ = imu.accel_noise_;
		gyro_offsets_ = imu.gyro_offsets_;
		gyro_noise_  = imu.gyro_noise_;
		mag_offsets_ = imu.mag_offsets_;
		mag_noise_ = imu.mag_noise_;
		subtract_g_ = subtract_g_;
		nsteps_ = 0;
  }

	/// make a deep copy
	virtual ImuSimple* clone() const {
		return new ImuSimple(*this);
	}

  /** 
   * Update the Imu sensor.
   * \param env The current environment.
   * \param dt The amount of time to simulate (seconds).
   */
  void Update(environment::Environment *env, double dt);

#ifdef USE_MPI
  /** 
   * Publish the current heading to the simulation 
   */
  void PublishData(int root, MPI_Comm broadcast_to);
#endif  

	/**
	* Returns the current reading from the accelerometer in 
	* m/s^2. Reading is in the LOCAL FRAME of the sensor.
	*/
	glm::vec3 GetAcceleration() {
		return accel_readings_;
	}

	/**
	* Returns the current reading from the gyroscope in
	* rad/s. Reading is in the LOCAL FRAME of the sensor.
	*/
	glm::vec3 GetAngularVelocity() {
		return gyro_readings_;
	}

	/**
	* Returns the current reading from the magnetometer in
	* uT. Reading is in the LOCAL FRAME of the sensor.
	*/
	glm::vec3 GetMagneticField() {
		return mag_readings_;
	}

	/**
	* Load a json file with input parameters for an IMU model
	* \param input_file The full path to the json input file.
	*/
	void Load(std::string input_file);

	/**
	* Tell the accelerometer wether to subtract g from the z reading
	* \param subtract_g True to subtract g, false to leave it
	*/
	void SetSubtractG(bool subtract_g) { subtract_g_ = subtract_g; }

	/**
	* Set parameters of noise model
	* \param sensor_type Can be "mag", "acc", or "gyr"
	* \param dimension Can be "x", "y" or "z"
	* \param noise_offset Constant offset of noise
	* \param noise_mag Magnitude of noise
	*/
	void SetNoiseModel(std::string sensor_type, std::string dimension, float noise_offset, float noise_mag);

protected:
	int nsteps_;
 private:
	 //input values in sensor coordinate frame
	 glm::vec3 local_acc_; // m/s^2
	 glm::vec3 local_angvel_; // rad/s

	 //output values
	 glm::vec3 accel_readings_; // m/s^2
	 glm::vec3 gyro_readings_; // rad/s
	 glm::vec3 mag_readings_; // micro-Tesla

	 glm::vec3 accel_offsets_;
	 glm::vec3 accel_noise_;
	 glm::vec3 gyro_offsets_;
	 glm::vec3 gyro_noise_;
	 glm::vec3 mag_offsets_;
	 glm::vec3 mag_noise_;

	 std::default_random_engine generator_;
	 std::uniform_real_distribution<float> acc_x_dist_; 
	 std::uniform_real_distribution<float> acc_y_dist_;
	 std::uniform_real_distribution<float> acc_z_dist_;
	 std::uniform_real_distribution<float> mag_x_dist_;
	 std::uniform_real_distribution<float> mag_y_dist_;
	 std::uniform_real_distribution<float> mag_z_dist_;
	 std::uniform_real_distribution<float> gyro_x_dist_;
	 std::uniform_real_distribution<float> gyro_y_dist_;
	 std::uniform_real_distribution<float> gyro_z_dist_;

	 bool subtract_g_;
};

} //namespace imu
} //namespace sensor
} //namespace mavs

#endif
