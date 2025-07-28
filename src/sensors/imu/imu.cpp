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
#include <sensors/imu/imu.h>

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>
#include <mavs_core/math/utils.h>

#ifdef Bool
#undef Bool
#endif
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>


namespace mavs{
namespace sensor{
namespace imu{

static char read_imu_input_buffer[65536];
static std::ofstream fout_;

Imu::Imu(){
  type_ = "imu";
  prefix_ = "./";
	temperature_ = 25.0f; //Celsius
	sample_rate_ = 100.0f; //Hz
	magnetic_field_ = glm::vec3(27.5550, -2.4169, -16.0849); //micro-tesla
	dead_reckon_orientation_ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	nsteps_ = 0;
	accel_readings_ = glm::vec3(0.0f, 0.0f, 0.0f);
	gyro_readings_ = glm::vec3(0.0f, 0.0f, 0.0f);
	mag_readings_ = glm::vec3(0.0f, 0.0f, 0.0f);
	local_acc_ = glm::vec3(0.0f, 0.0f, 0.0f);
	local_angvel_ = glm::vec3(0.0f, 0.0f, 0.0f);
}

Imu::~Imu(){
}

void Imu::Load(std::string input_file) {
	FILE* fp = fopen(input_file.c_str(), "rb");
	rapidjson::FileReadStream is(fp, read_imu_input_buffer, sizeof(read_imu_input_buffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	if (d.HasMember("Sample Rate")) {
		sample_rate_ = d["Sample Rate"].GetFloat();
	}
	if (d.HasMember("Temperature")) {
		temperature_ = d["Temperature"].GetFloat();
	}
	if (d.HasMember("Magnetic Field")) {
		for (int i = 0; i < 3; i++) {
			magnetic_field_[i] = d["Magnetic Field"][i].GetFloat();
		}
	}

	if (d.HasMember("Accelerometer")) {
		if (d["Accelerometer"].HasMember("Measurement Range")) {
			gyro_.SetMeasurmentRange(d["Accelerometer"]["Measurement Range"].GetFloat());
		}
		if (d["Accelerometer"].HasMember("Resolution")) {
			gyro_.SetResolution(d["Accelerometer"]["Resolution"].GetFloat());
		}
		if (d["Accelerometer"].HasMember("Constant Bias")) {
			glm::vec3 vin(d["Accelerometer"]["Constant Bias"][0].GetFloat(),
				d["Accelerometer"]["Constant Bias"][1].GetFloat(),
				d["Accelerometer"]["Constant Bias"][2].GetFloat());
			gyro_.SetConstantBias(vin);
		}
		if (d["Accelerometer"].HasMember("Noise Density")) {
			glm::vec3 vin(d["Accelerometer"]["Noise Density"][0].GetFloat(),
				d["Accelerometer"]["Noise Density"][1].GetFloat(),
				d["Accelerometer"]["Noise Density"][2].GetFloat());
			gyro_.SetNoiseDensity(vin);
		}
		if (d["Accelerometer"].HasMember("Bias Instability")) {
			glm::vec3 vin(d["Accelerometer"]["Bias Instability"][0].GetFloat(),
				d["Accelerometer"]["Bias Instability"][1].GetFloat(),
				d["Accelerometer"]["Bias Instability"][2].GetFloat());
			gyro_.SetBiasInstability(vin);
		}
		if (d["Accelerometer"].HasMember("Axis Misalignment")) {
			glm::vec3 vin(d["Accelerometer"]["Axis Misalignment"][0].GetFloat(),
				d["Accelerometer"]["Axis Misalignment"][1].GetFloat(),
				d["Accelerometer"]["Axis Misalignment"][2].GetFloat());
			gyro_.SetAxisMisalignment(vin);
		}
		if (d["Accelerometer"].HasMember("Random Walk")) {
			glm::vec3 vin(d["Accelerometer"]["Random Walk"][0].GetFloat(),
				d["Accelerometer"]["Random Walk"][1].GetFloat(),
				d["Accelerometer"]["Random Walk"][2].GetFloat());
			gyro_.SetRandomWalk(vin);
		}
		if (d["Accelerometer"].HasMember("Temperature Bias")) {
			glm::vec3 vin(d["Accelerometer"]["Temperature Bias"][0].GetFloat(),
				d["Accelerometer"]["Temperature Bias"][1].GetFloat(),
				d["Accelerometer"]["Temperature Bias"][2].GetFloat());
			gyro_.SetTemperatureBias(vin);
		}
		if (d["Accelerometer"].HasMember("Temperature Scale Factor")) {
			glm::vec3 vin(d["Accelerometer"]["Temperature Scale Factor"][0].GetFloat(),
				d["Accelerometer"]["Temperature Scale Factor"][1].GetFloat(),
				d["Accelerometer"]["Temperature Scale Factor"][2].GetFloat());
			gyro_.SetTemperatureScaleFactor(vin);
		}
	}

	if (d.HasMember("Gyroscope")) {
		if (d["Gyroscope"].HasMember("Measurement Range")) {
			gyro_.SetMeasurmentRange(d["Gyroscope"]["Measurement Range"].GetFloat());
		}
		if (d["Gyroscope"].HasMember("Resolution")) {
			gyro_.SetResolution(d["Gyroscope"]["Resolution"].GetFloat());
		}
		if (d["Gyroscope"].HasMember("Constant Bias")) {
			glm::vec3 vin(d["Gyroscope"]["Constant Bias"][0].GetFloat(),
				d["Gyroscope"]["Constant Bias"][1].GetFloat(),
				d["Gyroscope"]["Constant Bias"][2].GetFloat());
			gyro_.SetConstantBias(vin);
		}
		if (d["Gyroscope"].HasMember("Noise Density")) {
			glm::vec3 vin(d["Gyroscope"]["Noise Density"][0].GetFloat(),
				d["Gyroscope"]["Noise Density"][1].GetFloat(),
				d["Gyroscope"]["Noise Density"][2].GetFloat());
			gyro_.SetNoiseDensity(vin);
		}
		if (d["Gyroscope"].HasMember("Bias Instability")) {
			glm::vec3 vin(d["Gyroscope"]["Bias Instability"][0].GetFloat(),
				d["Gyroscope"]["Bias Instability"][1].GetFloat(),
				d["Gyroscope"]["Bias Instability"][2].GetFloat());
			gyro_.SetBiasInstability(vin);
		}
		if (d["Gyroscope"].HasMember("Axis Misalignment")) {
			glm::vec3 vin(d["Gyroscope"]["Axis Misalignment"][0].GetFloat(),
				d["Gyroscope"]["Axis Misalignment"][1].GetFloat(),
				d["Gyroscope"]["Axis Misalignment"][2].GetFloat());
			gyro_.SetAxisMisalignment(vin);
		}
		if (d["Gyroscope"].HasMember("Random Walk")) {
			glm::vec3 vin(d["Gyroscope"]["Random Walk"][0].GetFloat(),
				d["Gyroscope"]["Random Walk"][1].GetFloat(),
				d["Gyroscope"]["Random Walk"][2].GetFloat());
			gyro_.SetRandomWalk(vin);
		}
		if (d["Gyroscope"].HasMember("Temperature Bias")) {
			glm::vec3 vin(d["Gyroscope"]["Temperature Bias"][0].GetFloat(),
				d["Gyroscope"]["Temperature Bias"][1].GetFloat(),
				d["Gyroscope"]["Temperature Bias"][2].GetFloat());
			gyro_.SetTemperatureBias(vin);
		}
		if (d["Gyroscope"].HasMember("Temperature Scale Factor")) {
			glm::vec3 vin(d["Gyroscope"]["Temperature Scale Factor"][0].GetFloat(),
				d["Gyroscope"]["Temperature Scale Factor"][1].GetFloat(),
				d["Gyroscope"]["Temperature Scale Factor"][2].GetFloat());
			gyro_.SetTemperatureScaleFactor(vin);
		}
		if (d["Gyroscope"].HasMember("Acceleration Bias")) {
			glm::vec3 vin(d["Gyroscope"]["Acceleration Bias"][0].GetFloat(),
				d["Gyroscope"]["Acceleration Bias"][1].GetFloat(),
				d["Gyroscope"]["Acceleration Bias"][2].GetFloat());
			gyro_.SetAccelerationBias(vin);
		}
	}

	if (d.HasMember("Magnetometer")) {
		if (d["Magnetometer"].HasMember("Measurement Range")) {
			gyro_.SetMeasurmentRange(d["Magnetometer"]["Measurement Range"].GetFloat());
		}
		if (d["Magnetometer"].HasMember("Resolution")) {
			gyro_.SetResolution(d["Magnetometer"]["Resolution"].GetFloat());
		}
		if (d["Magnetometer"].HasMember("Constant Bias")) {
			glm::vec3 vin(d["Magnetometer"]["Constant Bias"][0].GetFloat(),
				d["Magnetometer"]["Constant Bias"][1].GetFloat(),
				d["Magnetometer"]["Constant Bias"][2].GetFloat());
			gyro_.SetConstantBias(vin);
		}
		if (d["Magnetometer"].HasMember("Noise Density")) {
			glm::vec3 vin(d["Magnetometer"]["Noise Density"][0].GetFloat(),
				d["Magnetometer"]["Noise Density"][1].GetFloat(),
				d["Magnetometer"]["Noise Density"][2].GetFloat());
			gyro_.SetNoiseDensity(vin);
		}
		if (d["Magnetometer"].HasMember("Bias Instability")) {
			glm::vec3 vin(d["Magnetometer"]["Bias Instability"][0].GetFloat(),
				d["Magnetometer"]["Bias Instability"][1].GetFloat(),
				d["Magnetometer"]["Bias Instability"][2].GetFloat());
			gyro_.SetBiasInstability(vin);
		}
		if (d["Magnetometer"].HasMember("Axis Misalignment")) {
			glm::vec3 vin(d["Magnetometer"]["Axis Misalignment"][0].GetFloat(),
				d["Magnetometer"]["Axis Misalignment"][1].GetFloat(),
				d["Magnetometer"]["Axis Misalignment"][2].GetFloat());
			gyro_.SetAxisMisalignment(vin);
		}
		if (d["Magnetometer"].HasMember("Random Walk")) {
			glm::vec3 vin(d["Magnetometer"]["Random Walk"][0].GetFloat(),
				d["Magnetometer"]["Random Walk"][1].GetFloat(),
				d["Magnetometer"]["Random Walk"][2].GetFloat());
			gyro_.SetRandomWalk(vin);
		}
		if (d["Magnetometer"].HasMember("Temperature Bias")) {
			glm::vec3 vin(d["Magnetometer"]["Temperature Bias"][0].GetFloat(),
				d["Magnetometer"]["Temperature Bias"][1].GetFloat(),
				d["Magnetometer"]["Temperature Bias"][2].GetFloat());
			gyro_.SetTemperatureBias(vin);
		}
		if (d["Magnetometer"].HasMember("Temperature Scale Factor")) {
			glm::vec3 vin(d["Magnetometer"]["Temperature Scale Factor"][0].GetFloat(),
				d["Magnetometer"]["Temperature Scale Factor"][1].GetFloat(),
				d["Magnetometer"]["Temperature Scale Factor"][2].GetFloat());
			gyro_.SetTemperatureScaleFactor(vin);
		}
	}
}

// Updates orientation quaternion using angular velocity and time step
void Imu::UpdateOrientationDeadReckoning(float dt) {
	float angle = glm::length(local_angvel_) * dt;

	if (angle < 1.0E-6f) return;

	// Normalize angular velocity to get rotation axis
	glm::vec3 axis = glm::normalize(local_angvel_);

	// Compute delta quaternion using exponential map
	glm::quat delta = glm::angleAxis(angle, axis);

	// Apply rotation: new orientation = delta * current
	glm::quat updated = delta * dead_reckon_orientation_;

	dead_reckon_orientation_ = glm::normalize(updated);
}


void Imu::Update(environment::Environment *env, double dt){
	glm::mat3 Ri = glm::inverse(orientation_);
	local_acc_ =  Ri * acceleration_;
	//std::cout << "Angular velocity:" << angular_velocity_.x << " " << angular_velocity_.y << " " << angular_velocity_.z << std::endl;
	local_angvel_ = Ri * angular_velocity_;

	UpdateOrientationDeadReckoning((float)dt);

	accel_readings_ = accelerometer_.Update(local_acc_, temperature_, sample_rate_);
	gyro_readings_ = gyro_.Update(local_angvel_, temperature_, sample_rate_);
	mag_readings_ = magnetometer_.Update(magnetic_field_, temperature_, sample_rate_);

  local_sim_time_ += local_time_step_;
  updated_ = true;
  nsteps_++;
}

void Imu::SaveRaw() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << nsteps_;
	std::string num_string(ss.str());

	std::string base_name = prefix_ + type_ + name_ + num_string;

	std::string outname = base_name + ".txt";

	std::ofstream fout;
	fout.open(outname.c_str());


	fout << "accel_readings: [ " << accel_readings_[0] << ", " << accel_readings_[1] << ", " << accel_readings_[2] << "]" << std::endl;
	fout << "gyro_readings: [ " << gyro_readings_[0] << ", " << gyro_readings_[1] << ", " << gyro_readings_[2] << "]" << std::endl;
	fout << "mag_readings: [ " << mag_readings_[0] << ", " << mag_readings_[1] << ", " << mag_readings_[2] << "]" << std::endl;

	fout.close();

}

#ifdef USE_MPI
void Imu::PublishData(int root,MPI_Comm broadcast_to){
  MPI_Bcast(&accel_readings_[0],3,MPI_FLOAT,root,broadcast_to);
	MPI_Bcast(&gyro_readings_[0], 3, MPI_FLOAT, root, broadcast_to);
	MPI_Bcast(&mag_readings_[0], 3, MPI_FLOAT, root, broadcast_to);
}
#endif

} //namespace imu
} //namespace sensor
} //namespace mavs

