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
#include <vehicles/autonomous_vehicle.h>

#include <algorithm>
#include <sstream>

#include <mavs_core/math/utils.h>

namespace mavs {
namespace vehicle {

AutonomousVehicle::AutonomousVehicle() {
	vehicle_ = NULL;

	current_steering_angle_degrees_ = 0.0f;
	prnd_ = 0;
	requested_speed_ = 0.0f;
	actual_speed_ = 0.0f;
	speed_diff_ = 0.0f;
	auto_mode_ = 0;
	max_steer_angle_degrees_ = 25.0f;
	append_file_opened_ = false;
	elapsed_time_ = 0.0f;
}

AutonomousVehicle::~AutonomousVehicle() {
	//if (vehicle_ != NULL)delete vehicle_;
}

void AutonomousVehicle::SetVehicle(Vehicle *veh) {
	vehicle_ = veh;
}

void AutonomousVehicle::SetRequests(nvidia::VehicleReq request) {
	current_request_ = request;
}

nvidia::VehicleFeedback AutonomousVehicle::GetFeedback() {
	return current_feedback_;
}

void AutonomousVehicle::GetFeedbackSerial(char *msg) {
	msg = (char *)&current_feedback_;
}

void AutonomousVehicle::WriteFeedbackToFile(std::string ofname) {
	std::ofstream writefile;
	writefile.open(ofname.c_str(), std::ios::binary | std::ios::out);
	writefile.write((char *)&current_feedback_, sizeof(nvidia::VehicleFeedback));
	writefile.close();
}

void AutonomousVehicle::AppendFeedbackToFile(std::string ofname) {
	if (mavs::utils::file_exists(ofname)) {
		if (!append_file_opened_) {
			std::ofstream fout;
			fout.open(ofname.c_str(), std::ios::binary | std::ios::out);
			fout.close();
			append_file_opened_ = true;
		}
	}
	std::ofstream writefile;
	writefile.open(ofname.c_str(), std::ios::binary | std::ios::out | std::ios::app);
	writefile.write((char *)&current_feedback_, sizeof(nvidia::VehicleFeedback));
	writefile.close();
}

void AutonomousVehicle::Update(environment::Environment *env) {
	float requested_steering_angle = ((float)current_request_.SteeringAngleReq) / 0.1f;
	float steering = requested_steering_angle / max_steer_angle_degrees_;
	steering = std::min(std::max(-1.0f, steering), 1.0f);
	float dt = 0.01f;
	requested_speed_ = 0.44704f*((float)current_request_.nVehReq) / 0.1f;
	actual_speed_ = (float)glm::length(vehicle_->GetState().twist.linear);
	speed_control_.SetSetpoint(requested_speed_);
	float throttle = (float)speed_control_.GetControlVariable(actual_speed_, dt);

	vehicle_->Update(env, throttle, steering, 0.0, dt);
	actual_speed_ = (float)glm::length(vehicle_->GetState().twist.linear);
	speed_diff_ = actual_speed_-requested_speed_;
	
	current_feedback_.nVeh = (uint64_t)(2.23694f*actual_speed_ / 0.1f);
	current_feedback_.PRND = 3;
	current_feedback_.nVehError = (int64_t)(2.23694f*speed_diff_ / 0.01f);
	current_feedback_.AutoMode = 1;
	current_feedback_.timestamp = (uint64_t)(elapsed_time_ / 1.0E-6);
	current_feedback_.SteeringAngle = (int64_t)(mavs::kRadToDeg*vehicle_->GetSteeringAngle() / 0.1f);
	elapsed_time_ += dt;
}

} //namespace vehicle
} //namespace mavs