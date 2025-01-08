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
#include <sensors/rtk/rtk.h>

#include <string>

#include <mavs_core/math/utils.h>
#include <mavs_core/math/constants.h>
#include <mavs_core/messages.h>
#include <glm/gtc/quaternion.hpp>

namespace mavs {
namespace sensor {
namespace rtk {

Rtk::Rtk() {
	error_ = 0.25f;
	rtk_elapsed_time_ = 0.0f;
	drop_time_elapsed_ = 0.0f;
	dropped_out_ = false;
	float dx = mavs::math::rand_in_range(-1.0f,1.0f)*error_;
	float dy = mavs::math::rand_in_range(-1.0f,1.0f)*error_;
	float dz = mavs::math::rand_in_range(-1.0f,1.0f)*error_;
	offset_direction_ = glm::vec3(dx,dy,dz);
	offset_direction_ = offset_direction_/glm::length(offset_direction_);
	srand ((unsigned int)time(NULL));
	int seed1 = rand();
	int seed2 = rand();
	random_noise_.SetSeed(seed1);
	random_noise_.SetFrequency(1.0 / 500.0); // noise features
	pop_noise_.SetSeed(seed2);
	pop_noise_.SetFrequency(1.0/850.0f); // pop error features
}


void Rtk::SetDropoutRate(float rate){
	float d_rate = 5.228E-4f*rate;
	pop_noise_.SetFrequency(d_rate);
}

void Rtk::Update(environment::Environment *env, double dt){

	// ---- position error ---- //
	//noise
	float rep = (float)random_noise_.GetPerlin(rtk_elapsed_time_, rtk_elapsed_time_);
	float reu = mavs::math::rand_in_range(-0.01f, 0.01f);
	float rand_error = error_*(0.02f*rep + reu);
	
	//pop
	float pop_error = 0.0f; 
	float pop_raw = (float)pop_noise_.GetPerlin(rtk_elapsed_time_,rtk_elapsed_time_);
	if (fabs(pop_raw)>0.35f){
		//pop_error = 0.1f*error_; 
		pop_error = 0.125f*(float)sqrt(0.25f+drop_time_elapsed_)*error_;
		dropped_out_ = true;
		drop_time_elapsed_ += (float)dt;
	}
	else {
		dropped_out_ = false;
		drop_time_elapsed_ = 0.0f;
	}
	// time based error 
	float base_position_error = error_*exp(-rtk_elapsed_time_/400.0f);
	//total error
	float total_error = base_position_error + rand_error + pop_error;
	// added to constant direction offset
	//glm::vec3 pos_err = glm::ve0c3(offset_direction_.x*total_error, offset_direction_.y*total_error, offset_direction_.z*2.0f*total_error);
	glm::vec3 pos_err = glm::vec3(offset_direction_.x*total_error, offset_direction_.y*total_error, 0.25f*offset_direction_.z*total_error);

	// ----- heading error ----///
	float heading_scale_factor = 0.025f;
	float lateral_error = total_error*sqrt(offset_direction_.x*offset_direction_.x + offset_direction_.y*offset_direction_.y);
	float testval = look_to_.x*offset_direction_.x + look_to_.y*offset_direction_.y;
	float heading_err = mavs::math::get_sign(testval)*heading_scale_factor*lateral_error; 
	glm::quat ori_err((float)cos(0.5f*heading_err),0.0f, 0.0f, (float)sin(0.5f*heading_err));
	glm::quat curr_ori = glm::quat_cast(orientation_); 
	glm::quat ori_w_err = ori_err*curr_ori;

	//---- velocity error proportional to position error ---//
	glm::vec3 vel_err = 0.286f*pos_err;
	glm::vec3 ang_vel_err = 0.286f*heading_err*glm::vec3(mavs::math::rand_in_range(-0.5f,0.5f),mavs::math::rand_in_range(-0.5f,0.5f),mavs::math::rand_in_range(-0.5f,0.5f));
	odom_.pose.pose.position = position_ + pos_err - look_to_*offset_.x - look_side_*offset_.y-look_up_*offset_.z; 
	odom_.pose.pose.quaternion = ori_w_err; 
	odom_.twist.twist.linear = velocity_ + vel_err;
	odom_.twist.twist.angular = angular_velocity_ + ang_vel_err; 
	rtk_elapsed_time_ += (float)dt;
}

mavs::Odometry Rtk::GetOdometryMessage(){
	return odom_;
}

void Rtk::PrintCurrentOdometry(std::string err_to_print){
	glm::quat ori = glm::quat_cast(orientation_); 
	float dx = (float)(odom_.pose.pose.position.x - position_.x);
	float dy = (float)(odom_.pose.pose.position.y - position_.y);
	float dz = (float)(odom_.pose.pose.position.z - position_.z);
	float err = (float)sqrt(dx*dx + dy*dy + dz*dz);
	if (err_to_print=="all"){
		std::cout<<rtk_elapsed_time_<<" "<<odom_.pose.pose.position.x<<" "<<odom_.pose.pose.position.y<<" "<<odom_.pose.pose.position.z<<" "<<position_.x<<" "<<position_.y<<" "<<position_.z<<" "<<" "<<dx<<" "<<dy<<" "<<dz<<" "<<err<<std::endl;
		std::cout<<odom_.pose.pose.quaternion.w<<" "<<odom_.pose.pose.quaternion.x<<" "<<odom_.pose.pose.quaternion.y<<" "<<odom_.pose.pose.quaternion.z<<std::endl;
		std::cout<<odom_.twist.twist.linear.x<<" "<<odom_.twist.twist.linear.y<<" "<<odom_.twist.twist.linear.z<<std::endl;
		std::cout<<odom_.twist.twist.angular.x<<" "<<odom_.twist.twist.angular.y<<" "<<odom_.twist.twist.angular.z<<std::endl<<std::endl;
	}
	else if (err_to_print=="position"){
		std::cout<< rtk_elapsed_time_ << " " << odom_.pose.pose.position.x<<" "<<odom_.pose.pose.position.y<<" "<<odom_.pose.pose.position.z<<" "<<position_.x<<" "<<position_.y<<" "<<position_.z<<" "<<dx<<" "<<dy<<" "<<dz<<" "<<err<<" "<<dropped_out_<<std::endl;
	}
}

} //namespace Rtk
} //namespace sensor
} //namespace mavs