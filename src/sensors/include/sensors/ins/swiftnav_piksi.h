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
* \class SwiftnavPiksi
*
* Simulates a Swiftnav Piksi sensor, which includes a dual-band gps
* and an IMU with gyro, accelerometer, and magnetometer
* Assumed to be running at 100 Hz
* \author Chris Goodin
*
* \date 11/12/2018
*/
#ifndef SWIFTNAV_PIKSI_H
#define SWIFTNAV_PIKSI_H

#include <sensors/ins/ins.h>
#include <sensors/ins/swiftnav_msg.h>

namespace mavs {
namespace sensor {
namespace ins {
class SwiftnavPiksi : public Ins {
public:
	/// Create Piksi sensor
	SwiftnavPiksi();

	/**
	* Update the Piksi sensor
	* \param env The MAVS environment in which to perform the simulation
	* \param dt The time step, assumed to be 0.01 (100 Hz)
	*/
	void Update(environment::Environment *env, double dt);

	/**
	* Write the current Piksi state to a binary SBP file
	*/
	void WriteStateToSbp(std::string basename);

	/**
	* Append the current state to a binary sbp file
	*/
	void AppendStateToSbp(std::string filename);

	/**
	* Set the timestamp on the Piksi sensor
	* \param set_time Time to set, in seconds
	*/
	void SetTimeStamp(float set_time);

	/// Print the current sensor state to the screen
	void PrintState();

  private:
	swiftnav::sbpMessagesWithTimeStamp current_state_;
}; //class SwiftnavPiksi

} //namespace ins
} //namespace halo
} //namespace mavs


#endif