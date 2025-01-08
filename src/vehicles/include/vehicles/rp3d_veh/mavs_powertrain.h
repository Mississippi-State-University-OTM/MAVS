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
* \class MavsPowertrain
*
* Class for a MAVS powertrain to be used with the RP3D vehicle model
* Calculates torque on wheels from engine throug drivetrain
*
* \author Chris Goodin
*
* \date 9/13/2019
*/
#ifndef MAVS_POWERTRAIN
#define MAVS_POWERTRAIN

namespace mavs {
namespace vehicle {
namespace mavs_rp3d {

class MavsPowertrain {
public:
	/// Create a default powertrain
	MavsPowertrain() {
		current_gear_ratio_ = 2.0;
		final_drive_ratio_ = 1.0;
		max_torque_ = 500.0; //N*m
		max_rpm_ = 6000.0; //rmp
		braking_torque_ = 500.0; //N*m
		idle_rpm_ = 1000.0;
	}

	/**
	* Get the current engine speed in RPM
	* \param v_long The current longintudinal vehicle speed in m/s
	* \param wheel_radius The deflected radius of the wheel
	*/
	float GetEngineSpeed(float v_long, float wheel_radius) {
		float engine_rpm = v_long * 60.0f * current_gear_ratio_*final_drive_ratio_ / (6.283185f*wheel_radius);
		if (engine_rpm < idle_rpm_)engine_rpm = idle_rpm_;
		return engine_rpm;
	}

	/**
	* Get the current engine torque based on RPM
	* \param engine_speed The current engine rpm
	*/
	float GetEngineTorque(float engine_speed) {
		//electric motor type model
		float engine_torque = max_torque_ - engine_speed * (max_torque_ / max_rpm_);
		return engine_torque;
	}

	/**
	* Get the torque on the wheel based on the current vehicle state
	* \param current_velocity Current longitudinal velocity of the vehicle in m/s
	* \param throttle Throttle from 0-1
	* \param brake Braking value from 0-1
	* \param tire_radius Deflected radius of the tire
	* \param is_powered True if the wheel is powered, false if it is not
	*/
	float CalculateWheelTorque(float current_velocity, float throttle, float brake, float tire_radius, bool is_powered) {
		float applied_torque = 0.0f;
		float current_rpm = GetEngineSpeed(current_velocity, tire_radius);
		if (brake == 0.0f && is_powered) {
			applied_torque = throttle * GetEngineTorque(current_rpm)*current_gear_ratio_*final_drive_ratio_;
		}
		else if (brake > 0.0f) {
			applied_torque = -braking_torque_ * brake;
		}
		return applied_torque;
	}

	/**
	* Set the final drive gear ratio
	* \param fdr The desired ratio
	*/
	void SetFinalDriveRatio(float fdr) { final_drive_ratio_ = fdr; }

	/**
	* Set the max torque in N/m
	* \param max_torque The desired maximum torque
	*/
	void SetMaxTorque(float max_torque) { max_torque_ = max_torque; }

	/**
	* Set the max rpm
	* \param max_rpm The desired maximum rpm
	*/
	void SetMaxRpm(float max_rpm) { max_rpm_ = max_rpm; }

	/**
	* Set the max braking torque in N/m
	* \param max_braking The desired maximum braking torque
	*/
	void SetMaxBrakingTorque(float max_braking) { braking_torque_ = max_braking; }

	/**
	* Set the engine rpm at idle
	* \param rpm The desired idle rpm
	*/
	void SetIdleRpm(float rpm) { idle_rpm_ = rpm; }

private:

	float current_gear_ratio_; 
	float final_drive_ratio_; 
	float max_torque_; 
	float max_rpm_; 
	float braking_torque_; 
	float idle_rpm_; 
};

} // namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs

#endif