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
#include <raytracers/animation.h>
#include <fstream>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace raytracer {

static glm::mat3 GetIdentity() {
	glm::mat3 r;
	r[0][0] = 1.0f; r[0][1] = 0.0f; r[0][2] = 0.0f;
	r[1][0] = 0.0f; r[1][1] = 1.0f; r[1][2] = 0.0f;
	r[2][0] = 0.0f; r[2][1] = 0.0f; r[2][2] = 1.0f;
	return r;
}

Animation::Animation(){
	time_unit_ = 1000.0; //milli-seconds
	elapsed_time_ = 0;
	total_time_ = 0.0f;
	frame_rate_ = 30.0f;
	frame_time_ = (int)(time_unit_ / frame_rate_);
	sequence_time_ = 1000; 
	current_frame_ = 0;
	next_frame_ = 1;

	z_ = 0.0f;
	//variables for waypoint following
	current_waypoint_ = 0;
	waypoints_loaded_ = false;
	position_ = glm::vec3(0.0f, 0.0f, 0.0f);
	orientation_ = GetIdentity();
	base_rotation_ = GetIdentity();
	speed_ = 0.0f;
	mesh_scale_ = 1.0f;
	rotate_y_to_x_ = false;
	rotate_y_to_z_ = false;
	behavior_set_ = false;
	random_goal_age_ = 100.0;
}

void Animation::SetRotateYToX(bool ryx) {
	rotate_y_to_x_ = ryx;
	glm::mat3 rot_mat = GetIdentity();
	if (rotate_y_to_z_) {
		glm::mat3 ryz = GetIdentity();
		ryz[1][1] = 0.0;
		ryz[1][2] = -1.0;
		ryz[2][1] = 1.0;
		ryz[2][2] = 0.0;
		rot_mat = ryz * rot_mat;
	}
	if (rotate_y_to_x_) {
		glm::mat3 ryx = GetIdentity();
		ryx[0][0] = 0.0;
		ryx[0][1] = 1.0;
		ryx[1][0] = -1.0;
		ryx[1][1] = 0.0;
		rot_mat = ryx * rot_mat;
	}
	base_rotation_ = rot_mat;
}

void Animation::SetRotateYToZ(bool ryz) {
	rotate_y_to_z_ = ryz;
	glm::mat3 rot_mat = GetIdentity();
	if (rotate_y_to_z_) {
		glm::mat3 ryz = GetIdentity();
		ryz[1][1] = 0.0;
		ryz[1][2] = -1.0;
		ryz[2][1] = 1.0;
		ryz[2][2] = 0.0;
		rot_mat = ryz * rot_mat;
	}
	if (rotate_y_to_x_) {
		glm::mat3 ryx = GetIdentity();
		ryx[0][0] = 0.0;
		ryx[0][1] = 1.0;
		ryx[1][0] = -1.0;
		ryx[1][1] = 0.0;
		rot_mat = ryx * rot_mat;
	}
	base_rotation_ = rot_mat;
}

void Animation::SetBehavior(std::string behave) {
	if (behave == "wander") {
		behavior_ = behave;
		behavior_set_ = true;
	}
	else if (behave == "straight") {
		behavior_ = behave;
		behavior_set_ = true;
	}
	else if (behave == "circle") {
		behavior_ = behave;
		behavior_set_ = true;
	}
	else if (behave == "crowd") {
		behavior_ = behave;
		behavior_set_ = true;
	}
	else {
		behavior_ = "wander";
		behavior_set_ = true;
	}
}

void Animation::LoadFrameList(std::string frame_file) {
	std::ifstream fin;
	fin.open(frame_file.c_str());
	if (!fin.is_open()) {
		std::cerr << "Unable to open frame file list " << frame_file << std::endl;
		exit(23);
	}
	while (!fin.eof()) {
		std::string f;
		fin >> f;
		keyframe_list_.push_back(f);
	}
	keyframe_list_.pop_back();
	fin.close();
	if (keyframe_list_.size() >= 1) {
		current_mesh_.Load(path_to_frames_, (path_to_frames_ + "/" + keyframe_list_[0]));
		vertices_.resize(current_mesh_.GetNumVertices());
	}
	if (keyframe_list_.size() >= 2)next_mesh_.Load(path_to_frames_, (path_to_frames_ + "/" + keyframe_list_[1]));

	sequence_time_ = (int)(keyframe_list_.size()*time_unit_ / frame_rate_);
}

void Animation::LoadPathFile(std::string fname) {
	waypoints_.Load(fname);
	if (waypoints_.NumWaypoints() > 1) {
		waypoints_loaded_ = true;
		current_waypoint_ = 1;
		position_ = glm::vec3(waypoints_.GetWaypoint(0).x, waypoints_.GetWaypoint(0).y, 0.0f);
	}
}

Mesh Animation::GetMesh(int meshnum) {
	if (meshnum == current_frame_) {
		return current_mesh_;
	}
	else if (meshnum == next_frame_) {
		return next_mesh_;
	}
	else if (meshnum >= 0 && meshnum < keyframe_list_.size()) {
		Mesh mesh;
		mesh.Load(path_to_frames_, (path_to_frames_ + "/" + keyframe_list_[next_frame_]));
		return mesh;
	}
	else {
		Mesh mesh;
		return mesh;
	}
}

float Animation::UpdateFrameCount(float dt) {
	float interp_variable = 0.0;
	int dt_sec = (int)(dt*time_unit_);
	elapsed_time_ += dt_sec;
	int curr_loop_time = elapsed_time_ % sequence_time_;
	int frame_num = (int)floor(curr_loop_time / ((float)frame_time_));
	if (frame_num >= keyframe_list_.size()) {
		frame_num = frame_num - (int)keyframe_list_.size();
	}
	
	if (frame_num != current_frame_) {
		current_frame_ = frame_num;
		if (current_frame_ = next_frame_) {
			current_mesh_ = next_mesh_;
		}
		else {
			current_mesh_.ClearAll();
			current_mesh_.Load(path_to_frames_, (path_to_frames_ + "/" + keyframe_list_[current_frame_]));
		}
		next_frame_ = current_frame_ + 1;
		if (next_frame_ >= keyframe_list_.size()) {
			next_frame_ = next_frame_ - (int)keyframe_list_.size();
		}
		next_mesh_.ClearAll();
		next_mesh_.Load(path_to_frames_, (path_to_frames_ + "/" + keyframe_list_[next_frame_]));
	}
	interp_variable = (float)((curr_loop_time - frame_num * frame_time_) / ((float)frame_time_));
	interp_variable = interp_variable - floor(interp_variable);
	return interp_variable;
}

void Animation::Update(float dt) {
	float interp_variable = UpdateFrameCount(dt);
	InterpolateVerts(interp_variable);
	if (waypoints_loaded_) {
		MoveVertsWaypoints(dt);
	}
	else if (behavior_set_) {
		MoveVertsBehavior(dt);
	}
	total_time_ += dt;
}

void Animation::MoveWander(float dt) {
	if (random_goal_age_ > 5.0f){
		random_goal_ = glm::vec3(math::rand_in_range(-1000.0f, 1000.0f), math::rand_in_range(-1000.0f, 1000.0f), 0.0f);
		random_goal_age_ = mavs::math::rand_in_range(0.0f,5.0f);
	}

	MoveToWaypoint(dt, random_goal_);

	random_goal_age_ += dt;
}

void Animation::MoveStraight(float dt) {
	glm::vec3 lt(1.0f, 0.0f, 0.0f);
	lt = orientation_ * lt;
	
	glm::vec3 wp = position_ + lt;
	MoveToWaypoint(dt, wp);
}

void Animation::MoveCircle(float dt) {
	float d = (float)sqrt(position_.x*position_.x + position_.y*position_.y);
	if (d > 0.0f) {
		glm::vec3 p = position_ / length(position_);
		float theta = atan2(p.y, p.x);
		glm::vec3 lt(sin(theta), cos(theta), 0.0f);
		glm::vec3 wp = position_ + lt;
		MoveToWaypoint(dt, wp);
	}
	else {
		SetPosition(10.0f, 0.0f);
	}
}

void Animation::MoveVertsBehavior(float dt) {
	if (behavior_ == "wander") {
		MoveWander(dt);
	}
	else if (behavior_ == "straight") {
		MoveStraight(dt);
	}
	else if (behavior_ == "circle") {
		MoveCircle(dt);
	}
}

void Animation::MoveVertsWaypoints(float dt) {
	glm::vec3 wp(waypoints_.GetWaypoint(current_waypoint_).x, waypoints_.GetWaypoint(current_waypoint_).y, z_);
	glm::vec3 r = wp - position_;
	float dist = glm::length(r);
	float rangefac = (2.0f*speed_*dt);
	if (dist < rangefac) {
		current_waypoint_++;
		if (current_waypoint_ >= waypoints_.NumWaypoints())current_waypoint_ = 0;
	}
	MoveToWaypoint(dt, wp);
}

void Animation::SetHeading(float theta) {
	orientation_ = GetIdentity();
	orientation_[0][0] = cos(theta);
	orientation_[0][1] = sin(theta);
	orientation_[1][0] = -sin(theta);
	orientation_[1][1] = cos(theta);
}

static void PrintMatrix(std::string title, glm::mat3 rot_mat) {
	std::cout << title << ":" << std::endl;
	std::cout << "|" << rot_mat[0][0] << " " << rot_mat[0][1] << " " << rot_mat[0][2] << "|" << std::endl;
	std::cout << "|" << rot_mat[1][0] << " " << rot_mat[1][1] << " " << rot_mat[1][2] << "|" << std::endl;
	std::cout << "|" << rot_mat[2][0] << " " << rot_mat[2][1] << " " << rot_mat[2][2] << "|" << std::endl << std::endl;
}

void Animation::MoveToWaypoint(float dt, glm::vec3 wp) {
	glm::vec3 r = wp - position_;
	
	float dist = glm::length(r);
	if (dist > 0.0f) {
		float theta = atan2(r.y, r.x);
		SetHeading(theta);
		glm::mat3 rot_mat = orientation_ * base_rotation_;
		position_ = position_ + (speed_ * dt)*(r / dist);
		position_.z = z_;
		for (int i = 0; i < (int)vertices_.size(); i++) {
			glm::vec3 v = mesh_scale_ * vertices_[i];
			glm::vec3 vp = position_ + rot_mat * v;
			vertices_[i] = vp;
		}
	}
}

void Animation::InterpolateVerts(float x) {
	if (keyframe_list_.size() <= 1) return;
	float xp = 1.0f - x;
	if (current_mesh_.GetNumVertices() != next_mesh_.GetNumVertices()) {
		std::cerr << "ERROR: Keyframe meshes " << current_frame_ << " and " << next_frame_ <<
			" do not have the same number of vertices. " << current_mesh_.GetNumVertices() <<" " << 
			next_mesh_.GetNumVertices() << std::endl;
		exit(23);
	}
	for (int i = 0; i < (int)current_mesh_.GetNumVertices(); i++) {
		glm::vec3 v0 = current_mesh_.GetVertex(i);
		glm::vec3 v1 = next_mesh_.GetVertex(i);
		vertices_[i] = xp * v0 + x * v1;
	}
}

glm::vec3 Animation::GetVertex(int i) {
	if (i >= 0 && i < vertices_.size()) {
		glm::vec3 v;
		v.x = vertices_[i].x;
		v.y = vertices_[i].y;
		v.z = vertices_[i].z;
		return v;
	}
	else {
		std::cerr << "Error: Requested vertex " << i << " from animation with " << vertices_.size() << " vertices " << std::endl;
		exit(23);
	}
}

void MoveCrowd(float dt, std::vector<Animation> &animations) {
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
	for (int i = 0; i < (int)animations.size(); i++) {
		if (animations[i].GetBehavior() == "crowd") {
			float interp_variable = animations[i].UpdateFrameCount(dt);
			animations[i].InterpolateVerts(interp_variable);
			//animations[i].MoveToWaypoint(dt, glm::vec3(100.0, 0.0, 0.0));
			animations[i].MoveWander(dt);
		}
	}
}

} //namespace raytracer 
} //namespace mavs
