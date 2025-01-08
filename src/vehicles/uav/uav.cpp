/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// class definition
#include "vehicles/uav/uav.h"
// c++ includes
#include <algorithm>
// mavs includes
#include <mavs_core/math/utils.h>

namespace mavs {
namespace vehicle {
Uav::Uav() {
	// env params
	g_ = 9.806f;
	rho_air_ = 1.225f;

	// plane params, defaults to albatross from applied aeronautics
	max_airspeed_ = 35.8f;
	mass_ = 5.0f;
	wing_area_ = 0.3f;
	coeff_lift_ = 0.5f;
	coeff_drag_ = 0.05f;
	max_thrust_ = 10.0f;
	max_roll_rate_ = 0.35f;

	// state variables
	x_ = 0.0f;
	y_ = 0.0f;
	z_ = 0.0f;
	gamma_ = 0.0f;
	psi_ = 0.0f;
	phi_ = 0.0f;
	airspeed_ = 0.0f;

	// control variables
	current_throttle_ = 0.0f;
	current_thrust_ = 0.0f;
	current_drag_ = 0.0f;
	current_lift_ = 0.0f;
}

float Uav::GetHeadingDegrees() {
	return psi_ * (float)mavs::kRadToDeg;
}

glm::quat Uav::GetOrientation() {
	// Abbreviations for the various angular functions
	float cy = cosf(psi_ * 0.5f);
	float sy = sinf(psi_ * 0.5f);
	float cp = cosf(gamma_ * 0.5f);
	float sp = sinf(gamma_ * 0.5f);
	float cr = cosf(phi_ * 0.5f);
	float sr = sinf(phi_ * 0.5f);

	glm::quat q;
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;

	return q;
}

void Uav::Update(float dt, float droll, float dpitch, float throttle) {
	// see: Weitz, L. A. (2015). Derivation of a point-mass aircraft model used for fast-time simulation. MITRE CORP MCLEAN VA MCLEAN.
	droll = std::max(-dt * max_roll_rate_, std::min(droll, dt * max_roll_rate_));
	dpitch = std::max(-0.25f * dt * max_roll_rate_, std::min(dpitch, 0.25f * dt * max_roll_rate_));
	GetControl(droll, throttle, dpitch);

	float cg = cosf(gamma_);
	float sg = sinf(gamma_);
	float dx = airspeed_ * cg * cosf(psi_);
	float dy = airspeed_ * cg * sinf(psi_);
	float dz = -airspeed_ * sg;
	float dv = ((current_thrust_ - current_drag_) / mass_) + g_ * sg;
	float dgamma = 0.0f;
	float dpsi = 0.0f;
	if (airspeed_ > 0.0f) {
		//dgamma = -(current_lift_*cosf(phi_) / (mass_*airspeed_)) + (g_*cg / airspeed_);
		float L = std::max(0.0f, current_lift_ - mass_ * g_);
		dgamma = -(L * cosf(phi_) / (mass_ * airspeed_));
		dpsi = -(current_lift_ * sinf(phi_) / (mass_ * airspeed_ * cg));
	}
	x_ += dt * dx;
	y_ += dt * dy;
	z_ += dt * dz;
	airspeed_ += dt * dv;
	gamma_ += dt * dgamma;
	psi_ += dt * dpsi;
}

void Uav::GetControl(float droll, float throttle, float dgamma) {
	//current_throttle_ += dthrottle;
	current_throttle_ = throttle;
	current_throttle_ = std::min(1.0f, std::max(current_throttle_, 0.0f));

	phi_ += droll;
	current_thrust_ = current_throttle_ * max_thrust_;

	float cc = 0.5f * wing_area_ * rho_air_ * airspeed_ * airspeed_;

	current_lift_ = coeff_lift_ * cc;
	current_drag_ = coeff_drag_ * cc;

	if (current_lift_ - mass_ * g_ > 0.0f || dgamma > 0.0) {
		gamma_ += dgamma;
	}
}
} // namespace vehicle
} // namespace mavs
