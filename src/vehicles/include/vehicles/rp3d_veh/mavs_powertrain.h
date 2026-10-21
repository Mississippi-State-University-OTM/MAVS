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
* The transmission is an automatic with any number of forward gears. At each call
* it selects the gear that gives the most wheel torque at the current speed, with
* hysteresis to prevent hunting between gears. This gives high torque at low speed
* (launch, slopes, soft soil) without leaving excess torque available at high speed,
* where it would spin the tires and cost lateral grip. With a single gear ratio
* (the default, 2.0) the behavior is identical to the original fixed-ratio model.
*
* Optional traction control reduces drive torque on a wheel whose longitudinal slip
* exceeds a target. It is off by default and is only active through the overload of
* CalculateWheelTorque that takes the wheel slip.
*
* \author Chris Goodin
*
* \date 9/13/2019
*/
#ifndef MAVS_POWERTRAIN
#define MAVS_POWERTRAIN

#include <vector>
#include <algorithm>
#include <cmath>

namespace mavs {
namespace vehicle {
namespace mavs_rp3d {

class MavsPowertrain {
public:
	/// Create a default powertrain
	MavsPowertrain() {
		gear_ratios_.assign(1, 2.0f);
		current_gear_ = 0;
		final_drive_ratio_ = 1.0f;
		max_torque_ = 500.0f; //N*m
		max_rpm_ = 6000.0f; //rmp
		braking_torque_ = 500.0f; //N*m
		idle_rpm_ = 1000.0f;
		shift_hysteresis_ = 0.05f;
		tc_enabled_ = false;
		tc_slip_target_ = 0.15f;
		tc_slip_band_ = 0.15f;
	}

	/**
	* Get the current engine speed in RPM, in the current gear
	* \param v_long The current longintudinal vehicle speed in m/s
	* \param wheel_radius The deflected radius of the wheel
	*/
	float GetEngineSpeed(float v_long, float wheel_radius) {
		return GetEngineSpeedInGear(v_long, wheel_radius, current_gear_);
	}

	/**
	* Get the current engine torque based on RPM.
	* Torque falls linearly to zero at max rpm and goes negative above it,
	* which acts as a rev limiter.
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
		UpdateGear(current_velocity, tire_radius);
		float applied_torque = 0.0f;
		float current_rpm = GetEngineSpeed(current_velocity, tire_radius);
		if (brake == 0.0f && is_powered) {
			applied_torque = throttle * GetEngineTorque(current_rpm)*CurrentRatio()*final_drive_ratio_;
		}
		else if (brake > 0.0f) {
			applied_torque = -braking_torque_ * brake;
		}
		return applied_torque;
	}

	/**
	* Get the torque on the wheel, with traction control applied to the drive torque
	* if it is enabled.
	* \param current_velocity Current longitudinal velocity of the vehicle in m/s
	* \param throttle Throttle from 0-1
	* \param brake Braking value from 0-1
	* \param tire_radius Deflected radius of the tire
	* \param is_powered True if the wheel is powered, false if it is not
	* \param wheel_slip Current longitudinal slip of this wheel
	*/
	float CalculateWheelTorque(float current_velocity, float throttle, float brake, float tire_radius, bool is_powered, float wheel_slip) {
		float applied_torque = CalculateWheelTorque(current_velocity, throttle, brake, tire_radius, is_powered);
		if (tc_enabled_ && brake == 0.0f && applied_torque > 0.0f) {
			applied_torque *= GetTractionControlScale(wheel_slip);
		}
		return applied_torque;
	}

	/**
	* Set the forward gear ratios. They are sorted from lowest gear (largest ratio)
	* to highest gear. Non-positive ratios are ignored. An empty list leaves the
	* ratios unchanged.
	* \param ratios The gear ratios, not including the final drive
	*/
	void SetGearRatios(const std::vector<float> &ratios) {
		std::vector<float> r;
		for (size_t i = 0; i < ratios.size(); i++) {
			if (ratios[i] > 0.0f) r.push_back(ratios[i]);
		}
		if (r.empty()) return;
		std::sort(r.begin(), r.end(), [](float a, float b) { return a > b; });
		gear_ratios_ = r;
		current_gear_ = 0;
	}

	/**
	* Set a single fixed gear ratio (no shifting)
	* \param ratio The gear ratio, not including the final drive
	*/
	void SetGearRatio(float ratio) {
		SetGearRatios(std::vector<float>(1, ratio));
	}

	/**
	* Set the shift hysteresis. A different gear is selected only when it gives
	* this fraction more wheel torque than the current gear. Default 0.05.
	* \param hyst The hysteresis fraction
	*/
	void SetShiftHysteresis(float hyst) { shift_hysteresis_ = std::max(0.0f, hyst); }

	/**
	* Configure traction control. Drive torque is unchanged while |slip| <= slip_target,
	* ramps down linearly above it, and reaches zero at slip_target + slip_band.
	* Only active through the CalculateWheelTorque overload that takes wheel slip.
	* \param enabled True to enable traction control
	* \param slip_target Slip at which torque reduction starts (default 0.15)
	* \param slip_band Additional slip over which torque goes to zero (default 0.15)
	*/
	void SetTractionControl(bool enabled, float slip_target = 0.15f, float slip_band = 0.15f) {
		tc_enabled_ = enabled;
		tc_slip_target_ = std::max(0.0f, slip_target);
		tc_slip_band_ = std::max(1.0e-3f, slip_band);
	}

	/// Get the current gear, 0 is the lowest gear
	int GetCurrentGear() const { return current_gear_; }

	/// Get the number of forward gears
	int GetNumGears() const { return (int)gear_ratios_.size(); }

	/**
	* Get the highest speed at which the engine still produces positive torque
	* in top gear, in m/s. This is the gearing limit on top speed; drag and
	* rolling resistance will limit it to a lower value.
	* \param wheel_radius The deflected radius of the wheel
	*/
	float GetGearingLimitedTopSpeed(float wheel_radius) const {
		float top = gear_ratios_.back()*final_drive_ratio_;
		return max_rpm_ * 6.283185f*wheel_radius / (60.0f*top);
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
	float CurrentRatio() const { return gear_ratios_[current_gear_]; }

	float GetEngineSpeedInGear(float v_long, float wheel_radius, int gear) const {
		float engine_rpm = v_long * 60.0f * gear_ratios_[gear] * final_drive_ratio_ / (6.283185f*wheel_radius);
		if (engine_rpm < idle_rpm_)engine_rpm = idle_rpm_;
		return engine_rpm;
	}

	float WheelTorqueInGear(float v_long, float wheel_radius, int gear) {
		return GetEngineTorque(GetEngineSpeedInGear(v_long, wheel_radius, gear))*gear_ratios_[gear] * final_drive_ratio_;
	}

	void UpdateGear(float v_long, float wheel_radius) {
		int n = (int)gear_ratios_.size();
		if (n <= 1 || wheel_radius <= 0.0f) {
			current_gear_ = 0;
			return;
		}
		if (current_gear_ >= n) current_gear_ = n - 1;
		// Pick the gear with the most wheel torque at this speed. Switch only when the
		// gain beats the hysteresis, so the selection does not hunt between gears.
		float current = WheelTorqueInGear(v_long, wheel_radius, current_gear_);
		int best = current_gear_;
		float best_torque = current;
		for (int i = 0; i < n; i++) {
			float t = WheelTorqueInGear(v_long, wheel_radius, i);
			if (t > best_torque) {
				best_torque = t;
				best = i;
			}
		}
		if (best != current_gear_ && best_torque > current + shift_hysteresis_ * fabs(current)) {
			current_gear_ = best;
		}
	}

	float GetTractionControlScale(float slip) const {
		float excess = fabs(slip) - tc_slip_target_;
		if (excess <= 0.0f) return 1.0f;
		return std::max(0.0f, 1.0f - excess / tc_slip_band_);
	}

	std::vector<float> gear_ratios_;
	int current_gear_;
	float final_drive_ratio_; 
	float max_torque_; 
	float max_rpm_; 
	float braking_torque_; 
	float idle_rpm_; 
	float shift_hysteresis_;
	bool tc_enabled_;
	float tc_slip_target_;
	float tc_slip_band_;
};

} // namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs

#endif
