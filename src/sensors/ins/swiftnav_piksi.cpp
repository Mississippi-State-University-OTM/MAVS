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
#include <sensors/ins/swiftnav_piksi.h>

#include <string>
#include <fstream>
#include <chrono>

#include <mavs_core/environment/date_time.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/messages.h>

namespace mavs {
namespace sensor {
namespace ins {
	
SwiftnavPiksi::SwiftnavPiksi() {
	gps_.SetType("dual band");
	sim_time_ = 0.0;
	num_calls_ = 0;
}

void SwiftnavPiksi::SetTimeStamp(float set_time) {
	current_state_.time = (uint64_t)(set_time * (1.0E6));
}

void SwiftnavPiksi::Update(environment::Environment *env, double dt) {
	//if (num_calls_ % 25 == 0)
	gps_.Update(env, dt);
	imu_.Update(env, dt);

	// pos_llh
	glm::dvec3 lla = gps_.GetRecieverPositionLLA();
	current_state_.pos_llh.lat = lla.x;
	current_state_.pos_llh.lon = lla.y;
	current_state_.pos_llh.height = lla.z;
	current_state_.pos_llh.h_accuracy = 0;
	current_state_.pos_llh.v_accuracy = 0;
	current_state_.pos_llh.n_sats = (uint8_t)gps_.GetNumSignals();
	current_state_.pos_llh.flags = 2;
	current_state_.flags.pos_llh = 1;

	// vel_ned, velocity units are mm/s
	glm::dvec3 vel = gps_.GetRecieverVelocityENU();
	current_state_.vel_ned.n = (int32_t)(1000.0f*vel.y);
	current_state_.vel_ned.e = (int32_t)(1000.0f*vel.x);
	current_state_.vel_ned.d = (int32_t)(-vel.z*1000.0f);
	current_state_.vel_ned.h_accuracy = 0;
	current_state_.vel_ned.v_accuracy = 0;
	current_state_.vel_ned.n_sats = (uint8_t)(gps_.GetNumSignals());
	current_state_.vel_ned.flags = 1;
	current_state_.flags.vel_ned = 1;

	// dops
	glm::dvec3 dop = gps_.GetPositionDop();
	current_state_.dops.pdop = (uint16_t)glm::length(dop);
	current_state_.dops.hdop = (uint16_t)dop.x;
	current_state_.dops.vdop = (uint16_t)dop.y;
	current_state_.flags.dops = 1;

	// utc_time
	environment::DateTime date_time = env->GetDateTime();
	current_state_.utc_time.year = date_time.year;
	current_state_.utc_time.month = date_time.month;
	current_state_.utc_time.day = date_time.day;
	current_state_.utc_time.hours = date_time.hour;
	current_state_.utc_time.minutes = date_time.minute;
	current_state_.utc_time.seconds = date_time.second;
	current_state_.utc_time.ns = (uint32_t)current_state_.time;
	current_state_.flags.utc_time = 1;

	//orient_euler
	double microdegrees_scale = mavs::kRadToDeg*1000000.0;
	double pitch, roll, yaw;
	mavs::math::QuatToEulerAngle(current_pose_.pose.quaternion, pitch, roll, yaw);
	current_state_.orient_euler.pitch = (uint32_t)(microdegrees_scale * pitch);
	current_state_.orient_euler.roll = (uint32_t)(microdegrees_scale * roll);
	current_state_.orient_euler.yaw = (uint32_t)(microdegrees_scale * yaw);
	current_state_.orient_euler.pitch_accuracy = 0.0f;
	current_state_.orient_euler.roll_accuracy = 0.0f;
	current_state_.orient_euler.yaw_accuracy = 0.0f;
	current_state_.flags.orient_euler = 1;

	//orient_quat
	current_state_.orient_quat.w = (int32_t)current_pose_.pose.quaternion.w;
	current_state_.orient_quat.x = (int32_t)current_pose_.pose.quaternion.x;
	current_state_.orient_quat.y = (int32_t)current_pose_.pose.quaternion.y;
	current_state_.orient_quat.z = (int32_t)current_pose_.pose.quaternion.z;
	current_state_.orient_quat.w_accuracy = 0.0f;
	current_state_.orient_quat.x_accuracy = 0.0f;
	current_state_.orient_quat.y_accuracy = 0.0f;
	current_state_.orient_quat.z_accuracy = 0.0f;
	current_state_.flags.orient_quat = 1;

	//angular_rate, in microdegrees
	glm::vec3 gps_rot = imu_.GetAngularVelocity();
	current_state_.angular_rate.x = (int32_t)(microdegrees_scale * gps_rot.x);
	current_state_.angular_rate.y = (int32_t)(-microdegrees_scale * gps_rot.y);
	current_state_.angular_rate.z = (int32_t)(-microdegrees_scale * gps_rot.z);
	current_state_.flags.angular_rate = 1;

	//imu_raw, not sure about units
	glm::vec3 acc = imu_.GetAcceleration();
	current_state_.imu_raw.acc_x = (int16_t)(acc.x);
	current_state_.imu_raw.acc_y = (int16_t)(acc.y);
	current_state_.imu_raw.acc_z = (int16_t)(acc.z);
	glm::vec3 rot = imu_.GetAngularVelocity();
	current_state_.imu_raw.gyr_x = (int16_t)(rot.x);
	current_state_.imu_raw.gyr_y = (int16_t)(rot.y);
	current_state_.imu_raw.gyr_z = (int16_t)(rot.z);
	current_state_.flags.imu_raw = 1;

	//mag_raw, not sure about units
	glm::vec3 mag = imu_.GetMagneticField();
	current_state_.mag_raw.mag_x = (int16_t)(mag.x);
	current_state_.mag_raw.mag_y = (int16_t)(mag.y);
	current_state_.mag_raw.mag_z = (int16_t)(mag.z);
	current_state_.flags.mag_raw = 1;
	num_calls_++;
	sim_time_ += dt;
}


void SwiftnavPiksi::AppendStateToSbp(std::string filename) {
	if (mavs::utils::file_exists(filename)) {
		if (num_calls_ == 0) {
			std::ofstream fout;
			fout.open(filename.c_str(), std::ios::binary | std::ios::out);
			fout.close();
		}
	}
	current_state_.AppendToFile(filename);
}


void SwiftnavPiksi::WriteStateToSbp(std::string basename) {
	std::string filename = basename + "_swiftnav.sbp";
	current_state_.Write(filename);
}

void SwiftnavPiksi::PrintState() {
	std::cout << current_state_ << std::endl;
}

} //namespace ins
} //namespace halo
} //namespace mavs