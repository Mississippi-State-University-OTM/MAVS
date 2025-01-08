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
* \class Ins
*
* Base class for a MAVS INS sensor
*
* \author Chris Goodin
*
* \date 1/4/2019
*/
#ifndef MAVS_INS_H
#define MAVS_INS_H
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <CImg.h>
#include <mavs_core/environment/environment.h>
#include <mavs_core/messages.h>
#include <sensors/gps/gps.h>
#include <sensors/imu/imu.h>

namespace mavs {
namespace sensor {
namespace ins {
class Ins : public Sensor {
public:
	/// Create Ins sensor
	Ins(){}

	/** Set the pose of the sensor, overrides the default methond in sensor.h
	* \param state The dynamic state of the vehicle that the sensor is attached to.
	*/
	void SetPose(mavs::VehicleState &state);

 	/**
	  * Set the pose of the sensor, overrides the default method in sensor.h
	  * \param position Position of the sensor in world coordinates
	  * \param orientation Quaternion orientation of the sensor in world coordiantes
	  */
	 void SetPose(glm::vec3 position, glm::quat orientation);

	/// Display the current INS output
	void Display();

	/// Return a pointer to the GPS sensor
	sensor::gps::Gps* GetGps() { return &gps_; }

	/** 
	 * Return the odometry message of the
	 * piksi. In ENU relative to the base station
	 */
	mavs::Odometry GetOdometryMessage();

protected:
	sensor::gps::Gps gps_;
	sensor::imu::Imu imu_;

	double sim_time_;
	int num_calls_;
	mavs::VehicleState current_pose_;

	cimg_library::CImgDisplay disp_;
}; // class Ins

} //namespace ins
} //namespace halo
} //namespace mavs


#endif