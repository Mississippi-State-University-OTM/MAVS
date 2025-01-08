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
#include "vehicles/uav/uav_sim.h"
// mavs includes
#include <mavs_core/math/utils.h>

namespace mavs {
namespace vehicle {
UavSim::UavSim() {
	use_camera_ = false;
	use_gimbal_ = true;
	use_lwir_ = false;
	//show_map_ = false;
	save_data_ = false;
	los_to_ugv_ = false;
	use_waypoints_ = false;
	num_steps_ = 0;
	dwell_radius_ = 250.0f;
	status_ = "connected";
	render_debug_ = false;
	terrain_elev_ = 0.0f;
	image_updated_ = false;
	controller_active_ = true;
}

UavSim::~UavSim() {

}

bool UavSim::Crashed() {
	if (uav_.GetAltitude() < terrain_elev_) {
		return true;
	}
	else {
		return false;
	}
}

std::vector<glm::vec2> UavSim::GenerateWaypoints(float x, float y, float r, float theta_step_deg) {
	float theta = 0.0f;
	float dtheta = 3.14159f * theta_step_deg / 180.0f;
	std::vector<glm::vec2> wp;
	while (theta <= 6.28318530718f) {
		glm::vec2 p(x + r * cosf(theta), y + r * sinf(theta));
		wp.push_back(p);
		theta += dtheta;
	}
	return wp;
}

void UavSim::SetUgvPath(std::vector<glm::vec2> ugv_path, bool los_to_ugv) {
	ugv_path_ = ugv_path;
	if (ugv_path_.size() > 1 && !use_waypoints_) {
		glm::vec2 pos = ugv_path_[ugv_path_.size() - 1];
		glm::vec2 last_pos = ugv_path_[ugv_path_.size() - 2];
		glm::vec2 lt = pos - last_pos;
		lt = lt / glm::length(lt);
		pos = pos + dwell_radius_ * lt;
		std::vector<glm::vec2> wp = GenerateWaypoints(pos.x, pos.y, dwell_radius_, 15.0f);
		controller_.SetWaypoints(wp);
	}
	los_to_ugv_ = los_to_ugv;

	// update the desired altitude
}

FlightControl UavSim::GetControlsFromWindow() {
	FlightControl control;
	control.roll = uav_.GetRoll();
	control.pitch = uav_.GetPitch();
	control.throttle = uav_.GetCurrentThrottle();
	if (flight_camera_.GetDisplay()->is_keyARROWLEFT()) {
		control.roll += -0.01f;
	}
	else if (flight_camera_.GetDisplay()->is_keyARROWRIGHT()) {
		control.roll += 0.01f;
	}
	if (flight_camera_.GetDisplay()->is_keyARROWUP()) {
		control.pitch += 0.01f;
	}
	else if (flight_camera_.GetDisplay()->is_keyARROWDOWN()) {
		control.pitch += -0.01f;
	}
	if (flight_camera_.GetDisplay()->is_keyW()) {
		control.throttle += 0.01f;
	}
	else if (flight_camera_.GetDisplay()->is_keyS()) {
		control.throttle += -0.01f;
	}

	return control;
}

void UavSim::SetDesiredAltitude(float des_alt, float min_hgt) {
	desired_altitude_ = des_alt;
	min_height_ = min_hgt;
}

void UavSim::UpdateDesiredAltitude(mavs::environment::Environment* env) {
	float heading = uav_.GetHeadingRadians();
	//look 5.0 seconds into the future
	float lad = uav_.GetAirspeed() * 5.0f;
	glm::vec2 lt(cosf(heading), sinf(heading));
	glm::vec2 p(uav_.GetPosition().x, uav_.GetPosition().y);
	glm::vec2 ground_point = p + lad * lt;
	terrain_elev_ = env->GetGroundHeight(ground_point.x, ground_point.y);
	float min_altitude = terrain_elev_ + min_height_;
	float des_alt = std::max(min_altitude, desired_altitude_);
	controller_.SetDesiredAltitude(des_alt);
}


void UavSim::Update(mavs::environment::Environment* env, float dt) {
	if (num_steps_ == 0 && render_debug_)flight_camera_.Display();

	FlightControl control;
	if (controller_active_) {
		UpdateDesiredAltitude(env);
		control = controller_.UpdateControl(&uav_, dt);
	}
	else {
		control = GetControlsFromWindow();
	}

	//env->AdvanceTime(dt);

	uav_.Update(dt, control.roll - uav_.GetRoll(), control.pitch - uav_.GetPitch(), control.throttle);

	UpdateSensors(env, dt);

	num_steps_++;

}

/*sensor_msgs::msg::Image UavSim::GetBlurredImage(int blur_radius) {
	sensor_msgs::msg::Image img_out;
	if (use_lwir_) {
		lwir_camera_.GetCImg()->blur_box(blur_radius, blur_radius, 0, 1, 2);
		//if (render_debug_) lwir_camera_.Display();
		mavs_ros_utils::CImgToImage(lwir_camera_.GetCImg(), img_out);
	}
	else {
		rgb_camera_.GetCImg()->blur_box(blur_radius, blur_radius, 0, 1, 2);
		//if (render_debug_) rgb_camera_.Display();
		mavs_ros_utils::CImgToImage(rgb_camera_.GetCImg(), img_out);
	}
	return img_out;
}*/

mavs::Image UavSim::GetCurrentImage() {
	if (use_camera_) {
		if (use_lwir_) {
			//if (render_debug_) lwir_camera_.Display();
			return lwir_camera_.GetRosImage();
		}
		else {
			//if (render_debug_)rgb_camera_.Display();
			return rgb_camera_.GetRosImage();
		}
	}
	else {
		//if (render_debug_)rgb_camera_.Display();
		return rgb_camera_.GetRosImage();
	}
}

void UavSim::UpdateSensors(mavs::environment::Environment* env, float dt) {
	image_updated_ = false;
	if (num_steps_ % 25 == 0) {
		env->SetActorPosition(0, uav_.GetPosition(), uav_.GetOrientation());
		image_updated_ = true;
		if (render_debug_) {
			flight_camera_.SetPose(uav_.GetPosition(), uav_.GetOrientation());
			flight_camera_.Update(env, 10 * dt);
			UpdateReadout();
			flight_camera_.Display();
		}

		if (use_camera_) {
			glm::quat ori = uav_.GetOrientation();
			if (use_gimbal_) {
				ori = glm::quat(0.7071f, 0.0f, 0.7071f, 0.0f);
				float yaw = uav_.GetHeadingRadians();
				glm::quat rot(cosf(0.5f * yaw), 0.0f, 0.0f, sinf(0.5f * yaw));
				ori = rot * ori;
			}
			if (use_lwir_) {
				lwir_camera_.SetPose(uav_.GetPosition(), ori);
				lwir_camera_.Update(env, 10 * dt);
				//lwir_camera_.Display();
			}
			else {
				rgb_camera_.SetPose(uav_.GetPosition(), ori);
				rgb_camera_.Update(env, 10 * dt);
				//rgb_camera_.Display();
			}
		}

		if (save_data_ && render_debug_) {
			flight_camera_.SaveImage(mavs::utils::ToString(num_steps_, 6) + "_image.bmp");
		}

	}

	if (num_steps_ % 100 == 0) {
		flightpath_.push_back(glm::vec2(uav_.GetPosition().x, uav_.GetPosition().y));
		//if (show_map_) {
		//	UpdateWaypoints();
		//	map_.UpdateMap(env);
		//}
	}
}

std::vector<glm::vec2> UavSim::GetFilteredWaypoints() {
	filtered_ugv_waypoints_.clear();
	glm::vec2 pos(uav_.GetPosition().x, uav_.GetPosition().y);
	for (int i = 0; i < (int)ugv_waypoints_.size(); i++) {
		glm::vec2 p = ugv_waypoints_[i];
		float dist = glm::length(pos - p);
		if (dist < 1.5f * uav_.GetPosition().z)filtered_ugv_waypoints_.push_back(p);
	}
	return filtered_ugv_waypoints_;
}

/*void UavSim::UpdateWaypoints() {
	map_.AddWaypoints(controller_.GetWaypoints(), vis::orange, true);
	map_.AddWaypoints(flightpath_, vis::magenta);
	map_.AddWaypoints(filtered_ugv_waypoints_, vis::cyan);
	map_.AddWaypoints(ugv_path_, vis::yellow);
	if (ugv_path_.size() > 0) {
		glm::vec2 ugv_pos = ugv_path_[ugv_path_.size() - 1];
		glm::vec2 uav_pos(uav_.GetPosition().x, uav_.GetPosition().y);
		if (los_to_ugv_) { map_.AddLine(uav_pos, ugv_pos, vis::green); }
		else { map_.AddLine(uav_pos, ugv_pos, vis::red); }
	}
} //Update Waypoints*/

void UavSim::UpdateReadout() {
	const glm::ivec3 yellow(255, 255, 0);
	const glm::ivec3 white(255, 255, 255);
	const glm::ivec3 grey(128, 128, 128);
	//int im_h = flight_camera_.GetCImg()->height();

	std::string veltext = "Velocity (m/s): ";
	veltext.append(mavs::utils::ToString(uav_.GetAirspeed()));
	flight_camera_.GetCImg()->draw_text(0, 0, veltext.c_str(), (int*)&yellow);

	std::string alttext = "Altitude (m): ";
	alttext.append(mavs::utils::ToString(uav_.GetPosition().z));
	flight_camera_.GetCImg()->draw_text(0, 12, alttext.c_str(), (int*)&yellow);

	std::string throttext = "Throttle: ";
	flight_camera_.GetCImg()->draw_text(0, 24, throttext.c_str(), (int*)&yellow);
	int xstart = 50;
	flight_camera_.GetCImg()->draw_rectangle(xstart, 26, xstart + 100, 34, (int*)&grey);
	flight_camera_.GetCImg()->draw_rectangle(xstart, 26, xstart + (int)(100 * uav_.GetCurrentThrottle()), 34, (int*)&yellow);

	std::string status_text = "Status: " + status_;
	flight_camera_.GetCImg()->draw_text(0, 36, status_text.c_str(), (int*)&yellow);


	int arrow_length = 30;
	int w = flight_camera_.GetCImg()->width();
	int x0 = w - arrow_length - 5;
	int y0 = arrow_length + 5;
	int x1 = x0;
	int y1 = 5;
	flight_camera_.GetCImg()->draw_arrow(x0, y0, x1, y1, (int*)&white);
	flight_camera_.GetCImg()->draw_text(x1 - 10, y1, "N", (int*)&white);
	x1 = x0 + (int)(arrow_length * cosf(uav_.GetHeadingRadians()));
	y1 = y0 - (int)(arrow_length * sinf(uav_.GetHeadingRadians()));
	flight_camera_.GetCImg()->draw_arrow(x0, y0, x1, y1, (int*)&yellow);
} // UpdateReadout
} // namespace vehicle
} //namespae mavs
