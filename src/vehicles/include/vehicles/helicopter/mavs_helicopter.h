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
* \class MavsHelicopter
*
* Simple helicopter class that is basically a car with the ability to fly.
*
* \author Chris Goodin
*
* \date 11/18/25
*/
#ifndef MAVS_HELICOPTER
#define MAVS_HELICOPTER
#include <iostream>
#include <vector>
#ifdef None
#undef None
#endif
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>

namespace mavs {
namespace vehicle {

class MavsHelicopter {
public:
	/// Constructor
	MavsHelicopter();

	/// Destructor
	~MavsHelicopter();

	/**
	* Load a vehicle input file
	* \param input_file Full path to the vehicle input file
	*/
	void Load(std::string input_file, environment::Environment* env, glm::vec3 init_pos, glm::quat init_ori);

	/**
	* Method to update the state of the vehicle.
	* Inherited from MAVS vehicle base class.
	* \param env Reference to the current environment.
	* \param throttle Commanded throttle from 0 to 1.
	* \param steer Commanded steering angle (radians).
	* \param dt The time step for the update
	*/
	void Update(environment::Environment *env, float throttle, float steer, float brake, float dt);

	/// Get a pointer to the underlying rp3d vehicle
	Rp3dVehicle* GetVehicle() { return &vehicle_; }

	VehicleState GetState() { return vehicle_.GetState(); }

	void IncreaseLift(float dt);
	
	void DecreaseLift(float dt);

	void SetHover() { hover_ = true; }

	bool GetRp3dActive() { return use_rp3d_; }

private:
	void CheckTireContact(environment::Environment* env);
	Rp3dVehicle vehicle_;
	glm::vec3 current_lift_;
	float max_lift_;
	bool use_rp3d_;
	int veh_id_;
	bool hover_;
	float g_;
	float mass_;
	float tire_offset_;
};

}// namespace vehicle
}// namespace mavs

#endif
