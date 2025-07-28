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
#include <sensors/imu/imu_simple.h>

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <glm/glm.hpp>
#include <mavs_core/math/utils.h>

#ifdef Bool
#undef Bool
#endif
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

namespace mavs {
namespace sensor {
namespace imu {

ImuSimple::ImuSimple() {
	type_ = "imu";
	subtract_g_ = false;
	accel_offsets_ = glm::vec3(0.0f, 0.0f, 0.0f);
	accel_noise_ = glm::vec3(0.0f, 0.0f, 0.0f);
	gyro_offsets_ = glm::vec3(0.0f, 0.0f, 0.0f);
	gyro_noise_ = glm::vec3(0.0f, 0.0f, 0.0f);
	mag_offsets_ = glm::vec3(0.0f, 0.0f, 0.0f);
	mag_noise_ = glm::vec3(0.0f, 0.0f, 0.0f);
	acc_x_dist_ = std::uniform_real_distribution<float>(0.0f, 0.0f);
	acc_y_dist_ = std::uniform_real_distribution<float>(0.0f, 0.0f);
	acc_z_dist_ = std::uniform_real_distribution<float>(0.0f, 0.0f);

	mag_x_dist_ = std::uniform_real_distribution<float>(0.0f, 0.0f);
	mag_y_dist_ = std::uniform_real_distribution<float>(0.0f, 0.0f);
	mag_z_dist_ = std::uniform_real_distribution<float>(0.0f, 0.0f);

	gyro_x_dist_ = std::uniform_real_distribution<float>(0.0f, 0.0f);
	gyro_y_dist_ = std::uniform_real_distribution<float>(0.0f, 0.0f);
	gyro_z_dist_ = std::uniform_real_distribution<float>(0.0f, 0.0f);

	dead_reckon_orientation_ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
}

ImuSimple::~ImuSimple() {
}

void ImuSimple::SetGyroNoise(float offset, float magnitude) {
	SetNoiseModel("gyr", "x", offset, magnitude);
	SetNoiseModel("gyr", "y", offset, magnitude);
	SetNoiseModel("gyr", "z", offset, magnitude);
}

void ImuSimple::SetAccelerometerNoise(float offset, float magnitude) {
	SetNoiseModel("acc", "x", offset, magnitude);
	SetNoiseModel("acc", "y", offset, magnitude);
	SetNoiseModel("acc", "z", offset, magnitude);
}

void ImuSimple::SetMagnetometerNoise(float offset, float magnitude) {
	SetNoiseModel("mag", "x", offset, magnitude);
	SetNoiseModel("mag", "y", offset, magnitude);
	SetNoiseModel("mag", "z", offset, magnitude);
}

void ImuSimple::SetNoiseModel(std::string sensor_type, std::string dimension, float noise_offset, float noise_mag) {
	int dim = -1;
	if (dimension == "x") {
		dim = 0;
	}
	else if (dimension == "y") {
		dim = 1;
	}
	else if (dimension == "z") {
		dim = 2;
	}
	else {
		std::cerr << "Warning, dimension " << dimension << " not recognzied when calling ImuSimple::SetNoiseModel. Must be \"x\", \"y\", or \"z\"." << std::endl;
		return;
	}
	if (sensor_type == "mag") {
		if (dim == 0)mag_x_dist_ = std::uniform_real_distribution<float>(noise_offset - 0.5f * noise_mag, noise_offset + 0.5f * noise_mag);
		if (dim == 1)mag_y_dist_ = std::uniform_real_distribution<float>(noise_offset - 0.5f * noise_mag, noise_offset + 0.5f * noise_mag);
		if (dim == 2)mag_z_dist_ = std::uniform_real_distribution<float>(noise_offset - 0.5f * noise_mag, noise_offset + 0.5f * noise_mag);
	}
	else if (sensor_type == "gyr") {
		if (dim == 0)gyro_x_dist_ = std::uniform_real_distribution<float>(noise_offset - 0.5 * noise_mag, noise_offset + 0.5 * noise_mag);
		if (dim == 1)gyro_y_dist_ = std::uniform_real_distribution<float>(noise_offset - 0.5 * noise_mag, noise_offset + 0.5 * noise_mag);
		if (dim == 2)gyro_z_dist_ = std::uniform_real_distribution<float>(noise_offset - 0.5 * noise_mag, noise_offset + 0.5 * noise_mag);
	}
	else if (sensor_type == "acc") {
		if (dim == 0)acc_x_dist_ = std::uniform_real_distribution<float>(noise_offset - 0.5 * noise_mag, noise_offset + 0.5 * noise_mag);
		if (dim == 1)acc_y_dist_ = std::uniform_real_distribution<float>(noise_offset - 0.5 * noise_mag, noise_offset + 0.5 * noise_mag);
		if (dim == 2)acc_z_dist_ = std::uniform_real_distribution<float>(noise_offset - 0.5 * noise_mag, noise_offset + 0.5 * noise_mag);
	}
	else {
		std::cerr << "Warning, sensor_type " << sensor_type << " not recognzied when calling ImuSimple::SetNoiseModel. Must be \"mag\", \"gyr\", or \"acc\"." << std::endl;
	}
}

static char imu_json_read_buffer[65536];

void ImuSimple::Load(std::string input_file) {

	if (!utils::file_exists(input_file)) {
		std::cerr << "ERROR! Attempted to load IMU input file " << input_file << ", but it does not exist." << std::endl;
		std::cerr << "EXITING!" << std::endl;
		exit(12);
	}

	FILE* fp = fopen(input_file.c_str(), "rb");
	;
	rapidjson::FileReadStream is(fp, imu_json_read_buffer, sizeof(imu_json_read_buffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);


	if (d.HasMember("Accelerometer")) {
		if (d["Accelerometer"].HasMember("Offsets")) {
			if (d["Accelerometer"]["Offsets"].Capacity() == 3) {
				accel_offsets_.x = d["Accelerometer"]["Offsets"][0].GetFloat();
				accel_offsets_.y = d["Accelerometer"]["Offsets"][1].GetFloat();
				accel_offsets_.z = d["Accelerometer"]["Offsets"][2].GetFloat();
			}
		}
		if (d["Accelerometer"].HasMember("Noise")) {
			if (d["Accelerometer"]["Noise"].Capacity() == 3) {
				accel_noise_.x = d["Accelerometer"]["Noise"][0].GetFloat();
				accel_noise_.y = d["Accelerometer"]["Noise"][1].GetFloat();
				accel_noise_.z = d["Accelerometer"]["Noise"][2].GetFloat();
			}
		}
		if (d["Accelerometer"].HasMember("Subtract G")) {
			subtract_g_ = d["Accelerometer"]["Subtract G"].GetBool();
		}
	} // accelerometer

	if (d.HasMember("Gyroscope")) {
		if (d["Gyroscope"].HasMember("Offsets")) {
			if (d["Gyroscope"]["Offsets"].Capacity() == 3) {
				gyro_offsets_.x = d["Gyroscope"]["Offsets"][0].GetFloat();
				gyro_offsets_.y = d["Gyroscope"]["Offsets"][1].GetFloat();
				gyro_offsets_.z = d["Gyroscope"]["Offsets"][2].GetFloat();
			}
		}
		if (d["Gyroscope"].HasMember("Noise")) {
			if (d["Gyroscope"]["Noise"].Capacity() == 3) {
				gyro_noise_.x = d["Gyroscope"]["Noise"][0].GetFloat();
				gyro_noise_.y = d["Gyroscope"]["Noise"][1].GetFloat();
				gyro_noise_.z = d["Gyroscope"]["Noise"][2].GetFloat();
			}
		}
	} // gyroscope

	if (d.HasMember("Magnetometer")) {
		if (d["Magnetometer"].HasMember("Offsets")) {
			if (d["Magnetometer"]["Offsets"].Capacity() == 3) {
				mag_offsets_.x = d["Magnetometer"]["Offsets"][0].GetFloat();
				mag_offsets_.y = d["Magnetometer"]["Offsets"][1].GetFloat();
				mag_offsets_.z = d["Magnetometer"]["Offsets"][2].GetFloat();
			}
		}
		if (d["Magnetometer"].HasMember("Noise")) {
			if (d["Magnetometer"]["Noise"].Capacity() == 3) {
				mag_noise_.x = d["Magnetometer"]["Noise"][0].GetFloat();
				mag_noise_.y = d["Magnetometer"]["Noise"][1].GetFloat();
				mag_noise_.z = d["Magnetometer"]["Noise"][2].GetFloat();
			}
		}
	} // magnetometer

	acc_x_dist_ = std::uniform_real_distribution<float>(accel_offsets_.x - 0.5f * accel_noise_.x, accel_offsets_.x + 0.5f * accel_noise_.x);
	acc_y_dist_ = std::uniform_real_distribution<float>(accel_offsets_.y - 0.5f * accel_noise_.y, accel_offsets_.y + 0.5f * accel_noise_.y);
	acc_z_dist_ = std::uniform_real_distribution<float>(accel_offsets_.z - 0.5f * accel_noise_.z, accel_offsets_.z + 0.5f * accel_noise_.z);

	mag_x_dist_ = std::uniform_real_distribution<float>(mag_offsets_.x - 0.5f * mag_noise_.x, mag_offsets_.x + 0.5f * mag_noise_.x);
	mag_y_dist_ = std::uniform_real_distribution<float>(mag_offsets_.y - 0.5f * mag_noise_.y, mag_offsets_.y + 0.5f * mag_noise_.y);
	mag_z_dist_ = std::uniform_real_distribution<float>(mag_offsets_.z - 0.5f * mag_noise_.z, mag_offsets_.z + 0.5f * mag_noise_.z);

	gyro_x_dist_ = std::uniform_real_distribution<float>(gyro_offsets_.x - 0.5f * gyro_noise_.x, gyro_offsets_.x + 0.5f * gyro_noise_.x);
	gyro_y_dist_ = std::uniform_real_distribution<float>(gyro_offsets_.y - 0.5f * gyro_noise_.y, gyro_offsets_.y + 0.5f * gyro_noise_.y);
	gyro_z_dist_ = std::uniform_real_distribution<float>(gyro_offsets_.z - 0.5f * gyro_noise_.z, gyro_offsets_.z + 0.5f * gyro_noise_.z);
}

// Updates orientation quaternion using angular velocity and time step
void ImuSimple::UpdateOrientationDeadReckoning(float dt) {
	float angle = glm::length(local_angvel_) * dt;

	if (angle < 1.0E-6f)return;

	// Normalize angular velocity to get rotation axis
	glm::vec3 axis = glm::normalize(local_angvel_);

	// Compute delta quaternion using exponential map
	glm::quat delta = glm::angleAxis(angle, axis);

	// Apply rotation: new orientation = delta * current
	glm::quat updated = delta * dead_reckon_orientation_;

	dead_reckon_orientation_ = glm::normalize(updated);
}

void ImuSimple::Update(environment::Environment* env, double dt) {
	glm::mat3 Ri = glm::inverse(orientation_);
	local_acc_ = Ri * acceleration_;
	local_angvel_ = Ri * angular_velocity_;

	accel_readings_.x = local_acc_.x + acc_x_dist_(generator_);
	accel_readings_.y = local_acc_.y + acc_y_dist_(generator_);
	accel_readings_.z = local_acc_.z + acc_z_dist_(generator_);

	gyro_readings_.x = local_angvel_.x + gyro_x_dist_(generator_);
	gyro_readings_.y = local_angvel_.y + gyro_y_dist_(generator_);
	gyro_readings_.z = local_angvel_.z + gyro_z_dist_(generator_);

	//mag_readings_.x = magnetic_field_.x + mag_x_dist_(generator_);
	//mag_readings_.y = magnetic_field_.y + mag_y_dist_(generator_);
	//mag_readings_.z = magnetic_field_.z + mag_z_dist_(generator_);

	UpdateOrientationDeadReckoning(dt);

	local_sim_time_ += local_time_step_;
	updated_ = true;
	nsteps_++;
}

#ifdef USE_MPI
void ImuSimple::PublishData(int root, MPI_Comm broadcast_to) {
	MPI_Bcast(&accel_readings_[0], 3, MPI_FLOAT, root, broadcast_to);
	MPI_Bcast(&gyro_readings_[0], 3, MPI_FLOAT, root, broadcast_to);
	MPI_Bcast(&mag_readings_[0], 3, MPI_FLOAT, root, broadcast_to);
}
#endif

} //namespace imu
} //namespace sensor
} //namespace mavs
