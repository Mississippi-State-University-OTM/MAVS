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
#include <vehicles/rp3d_veh/mavs_tire.h>
#include <math.h>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace vehicle {
namespace mavs_rp3d {

MavsTire::MavsTire() {
	Init(60.0f, 0.5f, 0.25f, 0.25f, 470000.0f);
	angular_velocity_ = 0.0f;
	current_rotation_angle_ = 0.0f;
	current_slip_ = 0.0f;
	left_ = true;
	elapsed_time_ = 0.0f;
	tire_id_ = 0;
	num_slices_ = 12;
	dtheta_slice_ = 0.1f;
	startup_ = true;
}

MavsTire::MavsTire(float tire_mass, float tire_radius, float tire_width, float tire_section_height, float tire_spring_constant) {
	Init(tire_mass, tire_radius, tire_width, tire_section_height, tire_spring_constant);
	num_slices_ = 12;
	dtheta_slice_ = 0.1f;
	startup_ = true;
}

void MavsTire::Init(float tire_mass, float tire_radius, float tire_width, float tire_section_height, float tire_spring_constant) {
	// class members
	angular_velocity_ = 0.0f;
	height_function_set_ = false;
	soil_type_set_ = false;
	current_deflection_ = 0.0f;
	// Setting physical properties of tire
	mass_ = tire_mass;
	k_ = tire_spring_constant;
	CalcDamping();
	radius_ = tire_radius;
	width_ = tire_width;
	section_height_ = tire_section_height;
	angular_velocity_ = 0.0f;
	//moment_ = 0.5f*tire_mass * tire_radius*tire_radius;
	moment_ = 10.0f*tire_mass * tire_radius*tire_radius;
	viscous_friction_coeff_ = 0.5f;
	angular_velocity_ = 0.0f;
	current_rotation_angle_ = 0.0;
	current_slip_ = 0.0f;
	left_ = true;
	elapsed_time_ = 0.0f;;
	tire_id_ = 0;
	startup_ = true;
}

void MavsTire::SetTerrainHeightFunction(std::string terrain_type, std::vector<float> args) {
	height_function_set_ = true;
	terrain_type_ = terrain_type;
	terrain_type_args_ = args;
}

void MavsTire::SetSoilType(std::string soil_type, float soil_strength) {
	soil_type_set_ = true;
	soil_type_ = soil_type;
	soil_strength_ = soil_strength;
}

float MavsTire::GetTerrainHeightFromFunction(float x, float y) {
	float z = 0.0f;
	if (terrain_type_ == "flat") {
		if (terrain_type_args_.size() > 0) {
			z = terrain_type_args_[0];
		}
	}
	if (terrain_type_ == "rough") {
		terrain_noise_.SetFrequency(1.0 / terrain_type_args_[0]);
		z = (float)(terrain_type_args_[1] * terrain_noise_.GetPerlin(x, y));
	}
	else if (terrain_type_ == "sloped") {
		z = terrain_type_args_[0] * x;
	}
	else if (terrain_type_ == "sine") {
		z = (float)(terrain_type_args_[1] * sin(mavs::k2Pi*x / terrain_type_args_[0]));
	}
	else {
		z = 0.0f;
	}
	return z;
}

rp3d::Vector3 MavsTire::RodRot(rp3d::Vector3 v, rp3d::Vector3 k, float theta) {
	float c = cos(theta);
	float s = sin(theta);
	rp3d::Vector3 vrot = c * v + s * k.cross(v) + (k.dot(v))*(1.0f - c)*k;
	return vrot;
}

void MavsTire::CalcDamping() {
	c_ = (float)(2.0*sqrt(k_*mass_)); //critical damping
}

float MavsTire::CalcSlip(float vx) {
	float slip = 0.0f;
	float rw = radius_ * angular_velocity_;
	
	//if (vx < 0.0f && rw < 0.0f) {
	if (rw < 0.0f) {
		if (vx < rw) {
			if (vx != 0.0f) {
				slip = (rw / vx) - 1.0f;
			}
		}
		else {
			if (rw != 0.0f) {
				slip = 1.0f - (vx / rw);
			}
		}
		slip = -1.0f*slip;
	}
	else {
		if (vx > rw) {
			if (vx != 0.0f && !startup_) {
				slip = (rw / vx) - 1.0f;
			}
		}
		else {
			if (rw != 0.0f) {
				slip = 1.0f - (vx / rw);
			}
		}
	}
	//std::cout << "Slip calculation " << rw << " " << vx << " " << slip << std::endl;
	slip = std::min(1.0f, std::max(slip, -1.0f));
	return slip;
}

rp3d::Quaternion AxisAngleToQuat(rp3d::Vector3 axis, float angle) {
	double at = 0.5*angle;
	double s = sin(at);
	rp3d::Quaternion q;
	q.x = axis.x * s;
	q.y = axis.y * s;
	q.z = axis.z * s;
	q.w = cos(at);
	return q;
}

rp3d::Vector3 MavsTire::Update(environment::Environment *env, float dt, rp3d::Transform tire_pose, rp3d::Vector3 tire_velocity, float torque, float steer_angle) {
	
	if (!radial_spring_tire_.IsInitialized()) {
		//radial_spring_tire_.Initialize(width_, radius_, k_, 2.5f, 3);
		radial_spring_tire_.Initialize(width_, radius_, k_, dtheta_slice_, num_slices_);
		//radial_spring_tire_.Initialize(width_, radius_, k_, 2.5f, 5);
	}
	if (torque != 0.0)startup_ = false;
	
	rp3d::Vector3 force(0.0f, 0.0f, 0.0f);
	pacejka_.SetTireParameters(width_, 2.0f*radius_, section_height_);
	// Get the "look" vectors in the tire frame
	rp3d::Matrix3x3 rot_mat = tire_pose.getOrientation().getMatrix();
	rp3d::Vector3 look_to = rot_mat.getColumn(0); // default is (1,0,0)
	rp3d::Vector3 look_side = rot_mat.getColumn(1); // default is (0,1,0)
	rp3d::Vector3 look_up(0.0f, 0.0f, 1.0f); // = rot_mat.getColumn(2); default is (0, 0, 1)

	look_to.normalize();
	look_up.normalize();
	look_side.normalize();
	//if (!left_)look_side = -look_side;
	look_to = RodRot(look_to, look_up, steer_angle);
	look_side = RodRot(look_side, look_up, steer_angle);
	look_to.normalize();
	look_side.normalize();

	// determine if tire is in contact
	rp3d::Vector3 tpos = tire_pose.getPosition(); 
	float surf_height = 0.0f;
	if (height_function_set_) {
		surf_height = GetTerrainHeightFromFunction(tpos.x, tpos.y);
	}
	else {
		glm::vec4 h_and_n = env->GetScene()->GetSurfaceHeightAndNormal(tpos.x, tpos.y, tpos.z);
		surf_height = h_and_n.w;
	}

	float viscous_friction = angular_velocity_ * viscous_friction_coeff_;

	float dh = surf_height + radius_;
	if (tpos.z < dh) {
		rp3d::Vector3 fz;
		float dz = dh - tpos.z;
		current_deflection_ = dz / section_height_;
		float normal_force = 0.0f;
		if (height_function_set_) {
			//Get normal force
			fz = rp3d::decimal(k_*dz - c_ * tire_velocity.z)*look_up;
			normal_force = fz.length();
		}
		else {
			// get the force with the radial spring tire
			radial_spring_tire_.SetPosition(glm::vec3(tpos.x, tpos.y, tpos.z));
			glm::mat3 tire_ori;
			for (int ii = 0; ii < 3; ii++) { for (int jj = 0; jj < 3; jj++) { tire_ori[ii][jj] = rot_mat[ii][jj]; } }
			radial_spring_tire_.SetOrientation(tire_ori);
			float fn_temp = radial_spring_tire_.GetNormalForce(env);
			dz = radial_spring_tire_.GetCurrentEquivalentDeflection();
			//fz = rp3d::decimal(normal_force)*look_up;
			fz = rp3d::decimal(k_*dz - c_ * tire_velocity.z)*look_up;
			normal_force = fz.length();
		}

		// Get velocities in tire frame
		float vx = tire_velocity.dot(look_to); //longitudinal velocity
		float vy = tire_velocity.dot(look_side); //lateral velocity
		//calculate the longintudinal and lateral slip
		float slip = CalcSlip(vx);
		current_slip_ = slip;
		float alpha = 0.0f;
		if (fabs(vx) > 0.001f) {
			alpha = (float)atan(std::min(std::max((double)(vy / fabs(vx)), -mavs::kPi_2), mavs::kPi_2))-steer_angle;
		}

		// get the combined lateral and longitudinal forces on the tire
		if (soil_type_set_) {
			pacejka_.SetSurfaceProperties(soil_type_, soil_strength_);
		}
		else {
			pacejka_.SetSurfaceProperties(env->GetSurfaceType(), env->GetSurfaceRCI());
		}
		rp3d::Vector2 tire_forces = pacejka_.GetCombinedTraction(normal_force, dz, slip, alpha);

		//update the angular velocity of the tire
		angular_velocity_ = angular_velocity_ + (dt / moment_)*(torque - tire_forces.x*radius_) - dt * viscous_friction;
		if (startup_) {
			tire_forces.x = 0.0f;
			tire_forces.y = 0.0f;
			angular_velocity_ = 0.0f;
		}

		rp3d::Vector3 fx = tire_forces.x * look_to;
		rp3d::Vector3 fy = tire_forces.y * look_side;
		
		rp3d::Vector3 static_friction = (2.0f * tire_velocity) * fz.length() / powf(std::max(1.0f, fabsf(angular_velocity_)), 2.0f);
		// don't let the friction exceed a stable value
		float fric_max = 0.25f * tire_velocity.length() / dt;
		if (static_friction.length() > fric_max) {
			static_friction = static_friction * (fric_max / static_friction.length());
		}
		// on startup, set friction to totally staic
		if (startup_) {
			static_friction = tire_velocity * dt;
		}
		force = fx + fy + fz - static_friction;
	}
	else { // wheel is not touching, just update angular velocity
		//update the angular velocity of the tire
		angular_velocity_ = angular_velocity_ + (dt / moment_)*(torque) - dt * viscous_friction;
		current_slip_ = 0.0f;
	}

	current_rotation_angle_ += angular_velocity_ * dt;
	while (current_rotation_angle_ >= mavs::k2Pi)current_rotation_angle_ -= mavs::k2Pi;
	while (current_rotation_angle_ <= -mavs::k2Pi)current_rotation_angle_ += mavs::k2Pi;

	// spin wheels
	rp3d::Quaternion q_steer = AxisAngleToQuat(look_up, steer_angle);
	rp3d::Quaternion q_roll = AxisAngleToQuat(look_side, current_rotation_angle_);
	current_orientation_ = q_roll * q_steer * tire_pose.getOrientation();

	elapsed_time_ += dt;

	return force;
}

} // namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs
