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
#include <vehicles/controllers/pure_pursuit_controller.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <mavs_core/math/segment.h>

namespace mavs {
namespace vehicle {
PurePursuitController::PurePursuitController() {
	//default wheelbase and steer angle
	// set to MRZR values
	wheelbase_ = 2.731f; // meters
	max_steering_angle_ = 0.69f; //39.5 degrees
	max_stable_speed_ = 35.0f; //5.0;

	// tunable parameters
	min_lookahead_ = 5.0f;
	max_lookahead_ = 25.0f;
	k_ = 2.0f; //0.5;

	//vehicle state parameters
	veh_x_ = 0.0f;
	veh_y_ = 0.0f;
	veh_speed_ = 0.0f;
	veh_heading_ = 0.0f;

	desired_speed_ = 2.5f; //m/s

	complete_ = false;
	looping_ = false;
	goal_thresh_ = 4.0f; //meters

	max_throttle_rate_ = 0.125f; // 0.25f;
	last_throttle_ = 0.0f;
}

float GetHeadingFromOrientation(Quaternion orientation) {
	glm::quat q((float)orientation.w, (float)orientation.x, (float)orientation.y, (float)orientation.z);
	float yaw = glm::yaw(q);
	return yaw;
}

void PurePursuitController::SetVehicleState(Odometry state){
// Set the current state of the vehicle, which should be the first pose in the path
	veh_x_ = (float)state.pose.pose.position.x;
	veh_y_ = (float)state.pose.pose.position.y;
	float vx = (float)state.twist.twist.linear.x;
	float vy = (float)state.twist.twist.linear.y;
	veh_speed_ = sqrt(vx*vx + vy*vy);
	veh_heading_ = GetHeadingFromOrientation(state.pose.pose.quaternion);
}

void PurePursuitController::SetVehicleState(float x_pos, float y_pos, float speed, float heading) {
	veh_x_ = x_pos;
	veh_y_ = y_pos;
	veh_speed_ = speed;
	veh_heading_ = heading;
}

//Twist PurePursuitController::GetDcFromTraj(Path traj) {
void PurePursuitController::GetDrivingCommand(float &throttle, float &steering, float &braking, float dt) {

	//initialize the driving command
	throttle = 0.0f;
	braking = 0.0f;
	steering = 0.0f;
	if (complete_) return;
	//make sure the path contains some points
	int np = (int)path_.size();

	if (np < 2) return;

	//calculate the lookahead distance based on current speed
	glm::vec2 currpos(veh_x_, veh_y_);
	float d = glm::length(currpos - path_.back());
	if (d < goal_thresh_ && !looping_) {
		complete_ = true;
		return;
	}

	float path_length = glm::length(path_[np - 1] - currpos);
	float lookahead = k_ * veh_speed_;

	if (lookahead > max_lookahead_)lookahead = max_lookahead_;
	if (lookahead < min_lookahead_)lookahead = min_lookahead_;
	if (lookahead > path_length && path_length > min_lookahead_)lookahead = path_length - 0.01f;

	//first find the closest segment on the path , and distance to it
	float closest = 1.0E9;
	int start_seg = 0;
	for (int i = 0; i < np - 1; i++) {
		float d0 = (float)mavs::math::PointToSegmentDistance(path_[i], path_[i + 1], currpos);
		if (d0 < closest) {
			closest = d0;
			start_seg = i;
		}
	}
	if (looping_) {
		int i = (int)(path_.size() - 1);
		float d0 = (float)mavs::math::PointToSegmentDistance(path_[i], path_[0], currpos);
		if (d0 < closest) {
			closest = d0;
			start_seg = i;
		}
	}

	std::vector<int> indices;
	int idx = start_seg;
	for (int i = 0; i < np; i++) {
		indices.push_back(idx);
		idx++;
		if (idx == np)idx = 0;
	}
	glm::vec2 goal = path_[start_seg];
	float target_speed = desired_speed_;
	if (closest < lookahead) {
		//find point on path at lookahead distance away
		float accum_dist = closest;
		//for (int i = start_seg; i < np - 1; i++) {
		//for (int j=0;j<indices.size();j++){
		for (int j = 0; j<(indices.size()-1); j++) {
			glm::vec2 v = path_[indices[j+1]] - path_[indices[j]];
			float seg_dist = glm::length(v);
			if ((accum_dist + seg_dist) > lookahead) {
				glm::vec2 dir = v / seg_dist;
				float t = lookahead - accum_dist;
				goal = path_[indices[j]] + t * dir;
				target_speed = desired_speed_; 
				if (target_speed > max_stable_speed_)target_speed = max_stable_speed_;
				break;
			}
			else {
				accum_dist += seg_dist;
			}
		}
	}

	//find the angle, alpha, between the current orientation and the goal
	glm::vec2 curr_dir(cos(veh_heading_), sin(veh_heading_));
	glm::vec2 to_goal(goal.x - veh_x_,goal.y-veh_y_);
	to_goal = to_goal / glm::length(to_goal);
	//float alpha = atan2(curr_dir.y,curr_dir.x)-atan2(to_goal.y,to_goal.x);
	float alpha = (float)atan2(to_goal.y, to_goal.x) - atan2(curr_dir.y, curr_dir.x);
	//determine the desired normalized steering angle
	float sangle = (float)atan2(2 * wheelbase_*sin(alpha), lookahead);
	sangle = sangle / max_steering_angle_;
	sangle = std::min(1.0f, sangle);
	sangle = std::max(-1.0f, sangle);
	//dc.angular.z = sangle;
	steering = sangle;

	//Use the speed controller to get throttle/braking
	//addjust the target speed so you back off during hard turns
	float adj_speed = (float)(target_speed * exp(-0.69*pow(fabs(steering), 4.0)));
	speed_controller_.SetSetpoint(adj_speed);
	throttle = (float)speed_controller_.GetControlVariable(veh_speed_, 0.1);
	if (throttle < 0.0) { //braking
		throttle = 0.0f;
		braking = std::max(-1.0f, throttle);
	}
	else {
		braking = 0.0;
		throttle = std::min(1.0f, throttle);
	}

	float throttle_rate = (throttle - last_throttle_) / dt;
	if (throttle_rate > max_throttle_rate_) {
		throttle = max_throttle_rate_ * dt + last_throttle_;
	}

	last_throttle_ = throttle;

	return;
}

} //namespace vehicle
} //namespace mavs
