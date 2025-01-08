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
* \class AutonomousVehicle
*
* Class that contains high-level state information
* about an autonomous vehicle, as well as 
* simulation of the physical vehicle system
* via the MAVS vehicle base class. Note that
* the vehicle is assumed to update at 100 Hz.
*
* \author Chris Goodin
*
* \date 11/15/2018
*/
#ifndef AUTONOMOUS_VEHICLE_H
#define AUTONOMOUS_VEHICLE_H
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <mavs_core/environment/environment.h>
#include <vehicles/vehicle.h>
#include <vehicles/controllers/pid_controller.h>
#include <interfaces/drive_px2.h>

namespace mavs {
namespace vehicle {

class AutonomousVehicle {
public:
	/// Constructor
	AutonomousVehicle();

	/// Destructor
	~AutonomousVehicle();

	/**
	* Update the state of the vehicle. 
	* The Update function should be called at 100 Hz
	* \param env Pointer to the mavs environment
	*/
	void Update(environment::Environment *env);

	/**
	* Set the vehicle-dynamic model and simulator for 
	* the autonomous vehicle.
	* \param veh The vehicle dynamics simulation
	*/
	void SetVehicle(Vehicle *veh);

	/**
	* Send a reqest message to the vehicle, 
	* similar to the format recieved over CAN
	* \request The request to send
	*/
	void SetRequests(nvidia::VehicleReq request);

	/**
	* Get the vehicle feedback reply from the vehicle
	*/
	nvidia::VehicleFeedback GetFeedback();

	/**
	* Get a serial (binary) version of the 
	* vehicle reply message
	* \param msg The vehicle reply message to fill
	*/
	void GetFeedbackSerial(char *msg);

	/**
	* Write the vehicle feedback to a file
	* \param ofname The file to write
	*/
	void WriteFeedbackToFile(std::string ofname);

	/**
	* Append the vehicle feedback to an existing file
	* \param ofname The file to write
	*/
	void AppendFeedbackToFile(std::string ofname);

	/// Get a pointer to the vehicle model
	vehicle::Vehicle* GetVehicle() { return vehicle_; }

	/**
	* Set the timestamp to apply to the output data
	* \param ts The timestamp in seconds
	*/
	void SetTimeStamp(float ts) {
		current_feedback_.timestamp = (uint64_t)(ts / 1.0E-9);
	}

private:
	vehicle::Vehicle *vehicle_;
	float current_steering_angle_degrees_;
	int prnd_;
	float elapsed_time_;
	float requested_speed_;
	float actual_speed_;
	float speed_diff_;
	bool auto_mode_;
	float max_steer_angle_degrees_;
	nvidia::VehicleReq current_request_;
	nvidia::VehicleFeedback current_feedback_;
	PidController speed_control_;
	bool append_file_opened_;
};

} //namespace vehicle
} //namespace mavs

#endif
