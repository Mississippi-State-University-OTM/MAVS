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
* \class Uav
*
* The base class for a unmanned aerial vehicle simulation.
* Integrated with MAVS terrains / visualization.
* see: Weitz, L. A. (2015). Derivation of a point-mass aircraft model used for fast-time simulation. MITRE CORP MCLEAN VA MCLEAN.
*
* \author Chris Goodin
*
* \date 11/18/2024
*/
#ifndef MAVS_UAV_H
#define MAVS_UAV_H
// mavs includes
#include <glm/glm.hpp>

namespace mavs {
namespace vehicle {
class Uav {
public:
	Uav();

	glm::vec3 GetPosition() {
		return glm::vec3(x_, y_, z_);
	}

	float GetAltitude() { return z_; }

	float GetHeadingDegrees();

	float GetHeadingRadians() { return psi_; }

	glm::quat GetOrientation();

	float GetAirspeed() { return airspeed_; }

	glm::vec3 GetVelocity() {
		glm::vec3 vel(cosf(gamma_) * cosf(psi_) * airspeed_, cosf(gamma_) * sinf(psi_) * airspeed_, sinf(gamma_) * airspeed_);
		return vel;
	}

	float GetMaxAirspeed() { return max_airspeed_; }

	float GetCurrentThrottle() { return current_throttle_; }

	void SetPosition(float x, float y, float z) {
		x_ = x;
		y_ = y;
		z_ = z;
	}

	void SetAirspeed(float v) {
		airspeed_ = v;
	}

	void SetHeadingRadians(float h) {
		psi_ = h;
	}

	void SetMaxVelocity(float mv) { max_airspeed_ = mv; }

	void SetMass(float m) { mass_ = m; }

	void SetLiftCoeff(float cl) { coeff_lift_ = cl; }

	void SetDragCoeff(float cd) { coeff_drag_ = cd; }

	void SetMaxRollRate(float mrr) { max_roll_rate_ = mrr; }

	void SetWingArea(float wa) { wing_area_ = wa; }

	void SetMaxThrust(float mt) { max_thrust_ = mt; }

	void Update(float dt, float droll, float dpitch, float throttle);

	float GetRoll() { return phi_; }

	float GetPitch() { return gamma_; }

	float GetYaw() { return psi_; }

private:
	void GetControl(float droll, float throttle, float dgamma);

	// enviromental params
	float g_; // acceleration due to gravity, m/s^2
	float rho_air_; // air density in kg/m^3

	// airplane params
	float wing_area_; // wing area in m^2 - both wings combined
	float mass_; // plane mass in kg
	float coeff_lift_; // lift coefficient,  ~[0.5-2.0]
	float coeff_drag_; // drag coefficient, ~[0.05-0.2]
	float max_airspeed_; // m/s
	float max_thrust_;
	float max_roll_rate_; // rad/s

	// state variables
	float gamma_; //pitch, radians
	float psi_; // yaw, radians
	float phi_; // roll, radians
	float x_; // x in world coordinates
	float y_; // y in world coordinates
	float z_; // height in meters
	float airspeed_; // longitudinal velocity in m/s

	// control variables
	float current_throttle_;

	// forces
	float current_lift_;
	float current_drag_;
	float current_thrust_;
}; // class Uav
} // namespace  vehicle
} // namespace mavs

#endif // include gaurd