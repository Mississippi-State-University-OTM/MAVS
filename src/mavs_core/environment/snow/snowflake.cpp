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
#include <mavs_core/environment/snow/snowflake.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/math/constants.h>

namespace mavs {
namespace environment {

Snowflake::Snowflake(float temperature) {
	g_ = 9.806f;
	gvec_ = glm::vec3(0.0f, 0.0f, -g_);
	age_ = 0.0f;
	if (temperature > -0.061f) {
		diameter_ = 0.04f;
	}
	else {
		diameter_ = (float)(0.015*pow(fabs(temperature), -0.35));
	}
	// hack alert!!!!
	diameter_ = 0.2f*diameter_;
	radius_ = 0.5f*diameter_;
	if (temperature > -1.0f) {
		wet_ = true;
	}
	else {
		wet_ = false;
	}
	if (wet_) {
		terminal_velocity_ = mavs::math::rand_in_range(0.5f, 1.5f);
	}
	else {
		terminal_velocity_ = mavs::math::rand_in_range(1.0f, 2.0f);
	}
	terminal_velocity_squared_ = terminal_velocity_ * terminal_velocity_;
	velocity_ = glm::vec3(0.0f, 0.0f, terminal_velocity_);
	position_ = glm::vec3(0.0f, 0.0f, 0.0f);
	if (wet_) {
		density_ = 0.724f / diameter_;
	}
	else {
		density_ = 0.170f / diameter_;
	}

	mass_ = (float)(density_ * 4.0*mavs::kPi*radius_*radius_*radius_ / 3.0);

	rotational_radius_ = mavs::math::rand_in_range(0.0f, 2.0f);
	float tval = mavs::math::rand_in_range(-1.0f, 1.0f);
	tval = tval / tval;
	rotational_velocity_ = tval * (float)mavs::math::rand_in_range(mavs::kPi / 4.0, mavs::kPi / 3.0);
	rotfac_ = rotational_radius_ * rotational_velocity_;
}

void Snowflake::UpdatePosition(float dt) {
	glm::vec3 wind(0.0f, 0.0f, 0.0f);
	UpdatePosition(wind,dt);
}

void Snowflake::UpdatePosition(glm::vec3 wind, float dt) {
	age_ += dt;
	glm::vec3 v_fluid = wind - velocity_;
	float vfmag = glm::length(v_fluid);
	glm::vec3 v_circ = ( vfmag/ glm::length(velocity_))*rotfac_*glm::vec3((float)-sin(rotational_velocity_*age_), (float)cos(rotational_velocity_*age_), 0.0f);
	float drag_mag = vfmag * vfmag*g_ / (terminal_velocity_squared_);
	glm::vec3 acc_drag = drag_mag * (v_fluid) / glm::length(v_fluid);
	glm::vec3 a = gvec_ + acc_drag;
	position_ = position_ + (velocity_ + v_circ)*dt + 0.5f*a*dt*dt;
	velocity_ = velocity_ + a * dt;
}

} // namespace environment
} // namespace mavs

