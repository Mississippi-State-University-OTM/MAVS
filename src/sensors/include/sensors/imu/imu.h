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
 * \class Imu
 * 
 * Simulates an IMU sensor with MEMS accelerometer,
 * gyroscope, and magnetometer. Uses same framework as MATLAB model
 * https://www.mathworks.com/help/fusion/ref/imusensor-system-object.html
 *
 * \author Chris Goodin
 *
 * \date 10/11/2018
 */

#ifndef IMU_H
#define IMU_H

#include <random>

#include <glm/glm.hpp>

#include <sensors/sensor.h>
#include <mavs_core/math/constants.h>
#include <sensors/imu/accelerometer.h>
#include <sensors/imu/gyro.h>
#include <sensors/imu/magnetometer.h>

#ifdef USE_MPI
#include <mpi.h>
#endif

namespace mavs{
namespace sensor{
namespace imu{

class Imu : public Sensor {
 public:
  /// Create the simulated IMU
  Imu();

  /// Imu destructor
  ~Imu();
  
	/// Imu copy constructor
  Imu(const Imu &imu){
		sample_rate_ = imu.sample_rate_;
		temperature_ = imu.temperature_;
		magnetic_field_ = imu.magnetic_field_;
		magnetometer_ = imu.magnetometer_;
		gyro_ = imu.gyro_;
		accelerometer_ = imu.accelerometer_;
		local_acc_ = imu.local_acc_;
		local_angvel_ = imu.local_angvel_; 
		accel_readings_ = imu.accel_readings_; 
		gyro_readings_ = imu.gyro_readings_; 
		mag_readings_ = imu.mag_readings_; 
		nsteps_ = 0;
  }

	/// make a deep copy
	virtual Imu* clone() const {
		return new Imu(*this);
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
	* Set the sample rate of the sensor in Hz
	* \param sr_hz The desired sample rate
	*/
	void SetSampleRate(float sr_hz) {
		sample_rate_ = sr_hz;
	}

	/**
	* Set the local ambient temperature
	* Will increase the noise in the sensor
	* \param temp The temperature in degrees Celsius
	*/
	void SetTemperature(float temp) {
		temperature_ = temp;
	}

	/**
	* Set the magnetic field in the vicinity of the sensor
	* in micro-Tesla
	* \param field The x-y-z components of the field in global coordiantes
	*/
	void SetMagneticField(glm::vec3 field) {
		magnetic_field_ = field;
	}

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

	/// Returns a pointer to the magnetometer of the IMU.
	Magnetometer* GetMagnetometer() { return &magnetometer_; }

	/// Returns a pointer to the gyroscope of the IMU.
	Gyro* GetGyro() { return &gyro_; }

	/// Returns a pointer to the accelerometer of the IMU.
	Accelerometer* GetAccelerometer() { return &accelerometer_; }

	/**
	* Load a json file with input parameters for an IMU model
	* \param input_file The full path to the json input file.
	*/
	void Load(std::string input_file);

	void SaveRaw();

protected:
	int nsteps_;
 private:
	 //properties
	 float sample_rate_;
	 float temperature_;
	 glm::vec3 magnetic_field_;

	 //devices
	 Magnetometer magnetometer_;
	 Gyro gyro_;
	 Accelerometer accelerometer_;

	 //input values in sensor coordinate frame
	 glm::vec3 local_acc_; // m/s^2
	 glm::vec3 local_angvel_; // rad/s

	 //output values
	 glm::vec3 accel_readings_; // m/s^2
	 glm::vec3 gyro_readings_; // rad/s
	 glm::vec3 mag_readings_; // micro-Tesla

};

} //namespace imu
} //namespace sensor
} //namespace mavs

#endif
