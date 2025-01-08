/*
Non-Commercial License - Mississippi State UnivMIT License

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
#include <mavs_core/environment/actors/actor.h>

#include <iostream>

#include <glm/gtx/euler_angles.hpp>

namespace mavs {
namespace actor {
	
Actor::Actor() {
	complete_ = false;
	speed_ = 1.4; //m/s
	current_waypoint_ = 0;
	tolerance_ = 0.1;
	orientation_ = glm::quat(1, 0, 0, 0);
	lock_to_ground_ = false;
	auto_update_ = true;
}

Actor::~Actor() {

}

void Actor::Update(double dt) {

	if (path_.NumWaypoints() <= 0) {
		return;
	}

	glm::dvec2 goal = path_.GetWaypoint(current_waypoint_);
	glm::dvec2 p(position_.x, position_.y);
	glm::dvec2 to_goal = goal - p;
	double dist_to_goal = length(to_goal);
	if (dist_to_goal<tolerance_) {
		current_waypoint_++;
		if (current_waypoint_ >= (int)path_.NumWaypoints()) {
			current_waypoint_ = 0;
		}
		goal = path_.GetWaypoint(current_waypoint_);
		to_goal = goal - p;
		dist_to_goal = length(to_goal);
	}

	glm::vec3 euler = glm::eulerAngles(orientation_);
	float yaw = euler.z;

	if (dist_to_goal > 0.0) {
		to_goal = to_goal / dist_to_goal;
		yaw = (float)atan2(to_goal.y, to_goal.x);
	}

	orientation_ = glm::eulerAngleYXZ(0.0f, 0.0f, yaw);

	double dist_traveled = speed_ * dt;
	if (dist_traveled > (dist_to_goal + tolerance_))dist_traveled = dist_to_goal + 0.8*tolerance_;
	p = p + to_goal * dist_traveled;
	position_.x = (float)p.x;
	position_.y = (float)p.y;
}

void Actor::LoadAnimation(std::string infile) {

}

} //namespace actor
} //namespace mavs