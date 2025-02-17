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
// MPI has to be included first or 
// the compiler chokes up
#ifdef USE_MPI
#include <mpi.h>
#endif

#ifdef USE_OMP
#include <omp.h>
#endif

#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
#include <tinyxml2.h>

#include <sensors/camera/camera.h>

#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <numeric>
#include <algorithm>
#include <limits>
#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>

#include <mavs_core/math/utils.h>
#include <mavs_core/plotting/mavs_plotting.h>
#include <raytracers/bounding_box.h>

namespace mavs {
namespace sensor {
namespace camera {

static const glm::vec3 zero(0.0f, 0.0f, 0.0f);

Camera::Camera() {
	img_saturation_ = 1.0f;
	img_temperature_ = 6500.0f; // K
	render_shadows_ = true;
	updated_ = false;
	raindrops_on_lens_ = false;
	local_sim_time_ = 0.0;
	Initialize(512, 512, 0.0035f, 0.0035f, 0.0035f);
	gamma_ = 0.75f;
	gain_ = 1.0f;
	exposure_time_ = 1.0f / 1000.0f; // 0.0005f; // 0.002f; // 1.0 / 250.0;
	type_ = "camera";
	first_display_ = true;
	first_seg_display_ = true;
	ncalled_ = 0;
	pixel_sample_factor_ = 1;
	prefix_ = "";
	use_distorted_ = false;
	is_fisheye_ = false;
	anti_aliasing_type_ = "none";
	disp_is_free_ = false;
	key_step_size_ = 0.75f;
	key_rot_rate_ = 0.05f;
	key_commands_.resize(4, false);
	frame_grabbed_ = false;
	target_brightness_ = 150.0f;
	snow_noise_.SetFrequency(50.0f);
}

Camera::~Camera() {

}

void Camera::Load(std::string input_file) {
	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	if (d.HasMember("Pixels")) {
		if (d["Pixels"].Capacity() == 2) {
			num_horizontal_pix_ = d["Pixels"][0].GetInt();
			num_vertical_pix_ = d["Pixels"][1].GetInt();
		}
	}

	if (d.HasMember("FocalLength")) {
		focal_length_ = d["FocalLength"].GetFloat();
	}

	if (d.HasMember("FocalPlaneDimensions")) {
		if (d["FocalPlaneDimensions"] == 2) {
			focal_array_width_ = d["FocalPlaneDimensions"][0].GetFloat();
			focal_array_height_ = d["FocalPlaneDimensions"][1].GetFloat();
		}
	}

	if (d.HasMember("Gamma")) {
		gamma_ = d["Gamma"].GetFloat();
	}

	if (d.HasMember("Gain")) {
		gain_ = d["Gain"].GetFloat();
	}

	if (d.HasMember("Exposure Time")) {
		exposure_time_ = d["Exposure Time"].GetFloat();
	}

	if (d.HasMember("Distortion")) {
		glm::vec2 cc, fc;
		float alpha_c = 0.0f;
		std::vector<float> kc;
		kc.resize(5, 0.0f);
		cc.x = 0.5f*num_horizontal_pix_;
		cc.y = 0.5f*num_vertical_pix_;
		fc.x = (num_horizontal_pix_ / focal_array_width_)*focal_length_;
		fc.y = (num_vertical_pix_ / focal_array_height_)*focal_length_;
		if (d["Distortion"].HasMember("CC")) {
			cc.x = d["Distortion"]["CC"][0].GetFloat();
			cc.y = d["Distortion"]["CC"][1].GetFloat();
		}
		if (d["Distortion"].HasMember("FC")) {
			fc.x = d["Distortion"]["FC"][0].GetFloat();
			fc.y = d["Distortion"]["FC"][1].GetFloat();
		}
		if (d["Distortion"].HasMember("KC")) {
			kc[0] = d["Distortion"]["KC"][0].GetFloat();
			kc[1] = d["Distortion"]["KC"][1].GetFloat();
			kc[2] = d["Distortion"]["KC"][2].GetFloat();
			kc[3] = d["Distortion"]["KC"][3].GetFloat();
			kc[4] = d["Distortion"]["KC"][4].GetFloat();
		}
		if (d["Distortion"].HasMember("alpha")) {
			alpha_c = d["Distortion"]["alpha"].GetFloat();
		}
		dm_.SetDistortionParameters(cc, fc, alpha_c, kc);
		dm_.SetNominalFocalLength(focal_length_);
		use_distorted_ = true;
	}

	if (d.HasMember("Fisheye Projection")) {
		fisheye_projection_ = d["Fisheye Projection"].GetString();
		is_fisheye_ = true;
	}

	if (d.HasMember("Anti Aliasing")) {
		if (d["Anti Aliasing"].HasMember("Samples Per Pixel")) {
			pixel_sample_factor_ = d["Anti Aliasing"]["Samples Per Pixel"].GetInt();
		}
		if (d["Anti Aliasing"].HasMember("Type")) {
			anti_aliasing_type_ = d["Anti Aliasing"]["Type"].GetString();
		}
	}
	Initialize(num_horizontal_pix_, num_vertical_pix_, focal_array_width_,
		focal_array_height_, focal_length_);
}

std::vector<bool> Camera::GetKeyCommands() {
	// Get translations
	//W
	key_commands_[0] = false;
	if (disp_.is_keyW()) {
		key_commands_[0] = true;
	}
	//S
	key_commands_[1] = false;
	if (disp_.is_keyS()) {
		key_commands_[1] = true;
	}
	//A
	key_commands_[2] = false;
	if (disp_.is_keyA()) {
		key_commands_[2] = true;
	}
	//D
	key_commands_[3] = false;
	if (disp_.is_keyD()) {
		key_commands_[3] = true;
	}
	return key_commands_;
}

void Camera::UpdatePoseKeyboard() {
	// Get translations
	if (disp_.is_keyW()) {
		position_ += key_step_size_ * look_to_;
	}
	if (disp_.is_keyS()) {
		position_ -= key_step_size_ * look_to_;
	}
	if (disp_.is_keyA()) {
		position_ += key_step_size_ * look_side_;
	}
	if (disp_.is_keyD()) {
		position_ -= key_step_size_ * look_side_;
	}
	if (disp_.is_keyPAGEUP()) {
		position_ += key_step_size_ * look_up_;
	}
	if (disp_.is_keyPAGEDOWN()) {
		position_ -= key_step_size_ * look_up_;
	}

	//rotate around look_side_
	if (disp_.is_keyARROWUP() || disp_.is_keyARROWDOWN()) {
		float pitch = key_rot_rate_;
		if (disp_.is_keyARROWUP()) pitch = -key_rot_rate_;
		look_to_ = glm::rotate(look_to_, pitch, look_side_);
		look_up_ = glm::rotate(look_up_, pitch, look_side_);
		orientation_[0] = look_to_;
		orientation_[2] = look_up_;
	}
	//rotate around look_up_
	if (disp_.is_keyARROWLEFT() || disp_.is_keyARROWRIGHT()) {
		glm::vec3 zhat(0.0f, 0.0f, 1.0f);
		float yaw = key_rot_rate_;
		if (disp_.is_keyARROWRIGHT()) yaw = -key_rot_rate_;
		look_to_ = glm::rotate(look_to_, yaw, zhat);
		look_side_ = glm::rotate(look_side_, yaw, zhat);
		orientation_[0] = look_to_;
		orientation_[1] = look_side_;
	}
	//rotate about look_to_
	if (disp_.is_keyHOME() || disp_.is_keyEND()) {
		float roll = key_rot_rate_;
		if (disp_.is_keyEND()) roll = -key_rot_rate_;
		look_side_ = glm::rotate(look_side_, roll, look_to_);
		look_up_ = glm::rotate(look_up_, roll, look_to_);
		orientation_[1] = look_side_;
		orientation_[2] = look_up_;
	}

	//grab frame
	if (disp_.is_keyR()) {
		frame_grabbed_ = true;
	}

}

void Camera::Initialize(int num_horizontal_pix, int num_vertical_pix,
	float focal_array_width, float focal_array_height,
	float focal_length) {
	num_horizontal_pix_ = num_horizontal_pix;
	num_vertical_pix_ = num_vertical_pix;
	focal_length_ = focal_length;
	focal_array_width_ = focal_array_width;
	focal_array_height_ = focal_array_height;
	//image_buffer_.resize(num_horizontal_pix_*num_vertical_pix_,zero);
	range_buffer_.resize(num_horizontal_pix_*num_vertical_pix_, 0.0f);
	normal_buffer_.resize(num_horizontal_pix_*num_vertical_pix_, glm::vec3(0.0f, 0.0f, -1.0f));
	segment_buffer_.resize(num_horizontal_pix_*num_vertical_pix_, 0);
	label_buffer_.resize(num_horizontal_pix_*num_vertical_pix_, "sky");
	horizontal_pixdim_ = focal_array_width / (1.0f*num_horizontal_pix);
	vertical_pixdim_ = focal_array_height / (1.0f*num_vertical_pix);
	image_.assign(num_horizontal_pix_, num_vertical_pix_, 1, 3, 0.);
	segmented_image_.assign(num_horizontal_pix_, num_vertical_pix_, 1, 3, 0.);
	half_horizontal_dim_ = 0.5f*num_horizontal_pix_;
	half_vertical_dim_ = 0.5f*num_vertical_pix_;
	horizontal_mag_scale_ = horizontal_pixdim_ / focal_length_;
	vertical_mag_scale_ = vertical_pixdim_ / focal_length_;
	pixel_solid_angle_ = (float)(2.0*atan(0.25*(horizontal_pixdim_ + vertical_pixdim_) / focal_length_));
}

void Camera::ZeroBuffer() {
	glm::vec3 zero(0.0f, 0.0f, 0.0f);
	//std::fill(image_buffer_.begin(),image_buffer_.end(),zero);
	std::fill(segment_buffer_.begin(), segment_buffer_.end(), 0);
	std::fill(label_buffer_.begin(), label_buffer_.end(), "sky");
	std::fill(range_buffer_.begin(), range_buffer_.end(), std::numeric_limits<float>::max());
	std::fill(normal_buffer_.begin(), normal_buffer_.end(), glm::vec3(0.0f,0.0f,1.0f));
	image_ = 0.0f;
	//image_.assign(num_horizontal_pix_,num_vertical_pix_,1,3,0.);
}

void Camera::ReduceImageBuffer() {
#ifdef USE_MPI
	int buff_size = num_horizontal_pix_ * num_vertical_pix_;
	MPI_Allreduce(MPI_IN_PLACE, image_.data(), image_.size(),
		MPI_FLOAT, MPI_MAX, comm_);
	//MPI_Allreduce(MPI_IN_PLACE, &range_buffer_[0], buff_size,
	//	MPI_FLOAT, MPI_MAX, comm_);
	MPI_Allreduce(MPI_IN_PLACE, &range_buffer_[0], buff_size,
		MPI_FLOAT, MPI_MIN, comm_);
#endif
}

void Camera::RenderFog(mavs::environment::Environment *env) {
	environment::Fog *fog = env->GetFog();
	/*
	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;
	float i2 = half_horizontal_dim_ - 0.5f;
	float j2 = half_vertical_dim_ - 0.5f;
	*/
	glm::vec3 skycol = env->GetAvgSkyRgb();
	float fog_rho = 0.65f*(skycol.x + skycol.y + skycol.z) / 3.0f;
	glm::vec3 cfog(fog_rho, fog_rho, fog_rho);
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic)
#endif
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j);
			float r = std::min(1000.0f, range_buffer_[n]);
			/*glm::vec3 direction = look_to_f
				+ (i - i2)*look_side_d
				+ (j - j2)*look_up_d;
			direction = glm::normalize(direction);
			glm::vec3 p1 = position_ + direction * r;
			float k = env->GetFogK(position_, p1);*/
			float k = fog->GetK();
			float f = (float)(exp(-k * r));
			glm::vec3 cpix(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
			glm::vec3 color = f * cpix + (1.0f - f)*cfog;
			image_.draw_point(i, j, (float *)&color);
		}
	}
}

void Camera::RenderSnow(mavs::environment::Environment *env, double dt) {
	//25 mm/h = heavy snow
	mavs::environment::Snow * snow = env->GetSnow();
	glm::vec3 skycol = env->GetAvgSkyRgb();
	float avgcol = (skycol.x + skycol.y + skycol.z) / 3;
	skycol = glm::vec3(avgcol, avgcol, avgcol);
	//define directions
	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;
	//float alpha = -0.00025f*env->GetSnowRate();
	//rasmussen 1999 suggests the factor should be -3E-5
	float alpha = -2.0E-4f*env->GetSnowRate();
	float i2 = half_horizontal_dim_ - 0.5f;
	float j2 = half_vertical_dim_ - 0.5f;
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic)
#endif
	for (int i = 0; i < num_horizontal_pix_; i++) {
		float ix = (i - i2)*horizontal_mag_scale_;
		float id = (float)i;
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j);
//#ifdef USE_MPI
//			if ((n) % comm_size_ == comm_rank_) {
//#endif
				float jd = (float)j;
				if (use_distorted_) {
					glm::vec2 meters(ix,
						(j - j2)*vertical_mag_scale_);
					glm::vec2 pu = dm_.Distort(meters);
					id = pu.x;
					jd = pu.y;
				}
				glm::vec3 direction = look_to_f
					+ (id - i2)*look_side_d
					+ (jd - j2)*look_up_d;
				direction = glm::normalize(direction);
				//glm::vec3 color = snow->GetClosestIntersection(position_, direction, horizontal_pixdim_);
				//float opacity = glm::length(color);
				float snowdist = std::numeric_limits<float>::max();
				float opacity = snow->GetClosestIntersection(position_, direction, horizontal_pixdim_, snowdist);
				//color = skycol;
				//if (glm::length(color) > 0.0f) {
				//	image_.draw_point(i, j, (float *)&color);
				//}
				if (opacity > 0.0f && snowdist < range_buffer_[n]) {
					glm::vec3 bg_col(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
					float sky_intens = glm::length(skycol);
					float bg_intens = glm::length(bg_col);
					float intens = std::max(bg_intens, sky_intens);
					glm::vec3 color(intens, intens, intens);
					glm::vec3 new_color = opacity * color + (1.0f - opacity)*bg_col;
					image_.draw_point(i, j, (float *)&new_color);
				}
				else {
					/*
					// did not intersect a snowflake
					// Get accumulation
					int n = GetFlattenedIndex(i, j);
					if (range_buffer_[n] < 10000.0f) {
						float f_inc = normal_buffer_[n].z;
						float f_exp = 0.0f;
						if (f_inc > 0.0f) {
							glm::vec3 inter_point = position_ + 0.999f*direction*range_buffer_[n];
							bool shadowed = env->GetAnyIntersection(inter_point, normal_buffer_[n]);
							if (!shadowed)f_exp = 1.0f;
							float noise = (float)snow_noise_.GetPerlin(inter_point.x, inter_point.y);
							float testval = (1.0f + noise) / 2.0f;
							if (f_inc < testval) {
								f_inc = 0.0f;
							}
							else {
								f_inc += std::max(0.0f, testval);
							}
						}
						f_inc = f_inc * 0.5f;
						float f_p = f_inc * f_exp;
						glm::vec3 pixcol(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
						glm::vec3 newcol = f_p * skycol + (1.0f - f_p)*pixcol;
						image_.draw_point(i, j, (float *)&newcol);
					} // done with accumulation
					*/
					if (range_buffer_[n] > 0.0) {
						// light scattered by distant snow
						float r = std::min(1000.0f, range_buffer_[n]);
						float abs_fac = exp(alpha*r);
						glm::vec3 pixcol(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
						glm::vec3 newcol = abs_fac * pixcol + (1.0f - abs_fac)*skycol;
						image_.draw_point(i, j, (float *)&newcol);
					}
				}
//#ifdef USE_MPI
//			}
//#endif
		}
	}
} //RenderSnow

void Camera::CreateRainMask(mavs::environment::Environment *env, double dt) {
	std::vector< std::vector< float > > rain_mask = mavs::utils::Allocate2DVector(num_horizontal_pix_, num_vertical_pix_, 0.0f);
	//environmental factors
	glm::vec3 skycol = env->GetAvgSkyRgb();
	float rate = env->GetRainRate(); //mm/h
	glm::vec3 wind(env->GetWind().x, env->GetWind().y, 0.0);

	//empirical stuff
	float alpha = -2.0f*0.0001f*rate;
	float raindrop_rho = 0.02f*(skycol.x + skycol.y + skycol.z) / 3.0f;
	glm::vec3 rain_col(raindrop_rho, raindrop_rho, raindrop_rho);
	
	/*float mean = (float)image_.mean();
	float alpha = -2.0f*0.00009f*rate;
	float max_alpha = 1000.0f*alpha;
	float raindrop_rho = 0.1f*mean; // 0.1f*(skycol.x + skycol.y + skycol.z);
	glm::vec3 rain_col = glm::vec3(raindrop_rho, raindrop_rho, raindrop_rho);*/

	float min_pix_frac = 0.1f; //10% of pixel
	float n_t = 172.0f*pow(rate, 0.21f); // #/m^3
	//Feingold & Levin 1986, Eq 39
	float d_g = 0.75f*pow(rate, 0.21f); //mm
	d_g = d_g / 1000.0f; //meters
	float vterm = 3.1654f*log(d_g) + 26.006f; //m/s
	wind.z = vterm;
	glm::vec2 wind_projected;
	wind_projected.x = glm::dot(look_side_, wind);
	wind_projected.y = glm::dot(look_up_, wind);

	// Geometry stuff
	//float range_max = focal_length_ * d_g*(2.0f/min_pix_frac) / (horizontal_pixdim_ + vertical_pixdim_);
	//float range_max = 15.0;
	//float range_max = std::max(20.0f, 4500.0f*focal_length_);
	float range_max = std::min(25.0f, 2500.0f*focal_length_);
		float width_mag = 0.5f*focal_array_width_ / focal_length_;
	float height_mag = 0.5f*focal_array_height_ / focal_length_;
	float dh = 2.0f*range_max * height_mag;
	float dw = 2.0f*range_max * width_mag;
	float rain_vol = range_max * dw*dh / 3.0f; //volume of view frustum
	int total_num_drops = (int)(n_t * rain_vol);
	float halfwidth = 0.5f*focal_array_width_;
	float halfheight = 0.5f*focal_array_height_;

	//float altitude = position_.z - env->GetGroundHeight(position_.x, position_.y);
	
	// do streaks
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
	for (int i = 0; i < total_num_drops; i++) {
		float z = mavs::math::rand_in_range(0.0f, range_max);
		float dx = width_mag * z;
		float dy = height_mag * z;
		float x = mavs::math::rand_in_range(-dx, dx);
		float y = mavs::math::rand_in_range(-dy, dy);
		float m = focal_length_ / z;
		x = m * x;
		y = m * y;
		int u = (int)((x + halfwidth) / horizontal_pixdim_);
		int v = (int)((y + halfheight) / vertical_pixdim_);
		//get the streak length and direction in pixels
		glm::vec2 l_streak(m * wind_projected.x*exposure_time_ / horizontal_pixdim_,
			m * wind_projected.y*exposure_time_ / vertical_pixdim_);
		float l_streak_mag = glm::length(l_streak);
		l_streak = l_streak / l_streak_mag;
		glm::vec2 l_pos(u, v);
		for (int ns = 0; ns <= (int)l_streak_mag; ns++) {
			glm::vec2 distorted_pos = l_pos;
			if (use_distorted_) {
				glm::vec2 meters((l_pos.x - half_horizontal_dim_ + 0.5f)*horizontal_mag_scale_,
					(l_pos.y - half_vertical_dim_ + 0.5f)*vertical_mag_scale_);
				distorted_pos = dm_.Distort(meters);
			}
			u = (int)distorted_pos.x;
			v = (int)distorted_pos.y;
			if (u >= 0 && u < num_horizontal_pix_ && v >= 0 && v < num_vertical_pix_) {
				int n = GetFlattenedIndex(u, v);
				if (z < range_buffer_[n] || range_buffer_[n] == 0.0f) {
					rain_mask[u][v] = rain_mask[u][v] + raindrop_rho; // to_add;
				}
			}
			else {
				break;
			}
			l_pos += l_streak;
		}
	}
	
	//calculate average and max intensity of image
	//float intens_max = 0.0f;
	//float intens_avg = 0.0f;
	//int bfs = (int)(image_.height()*image_.width()); // image_buffer_.size();
//#pragma omp parallel for schedule(dynamic) shared(intens_avg, intens_max)
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif
//for (int n = 0; n < bfs; n++) {
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			//------ Light absorbed by the rain ------------------------
			int n = GetFlattenedIndex(i, j);
			if (range_buffer_[n] > 0.0) {
				float r = std::min(1000.0f, range_buffer_[n]);
				float abs_fac = exp(alpha*r);
				for (int k = 0; k < 3; k++)image_(i, j, k) = abs_fac * image_(i, j, k) + 20.0f*(1.0f - abs_fac)*rain_col[k];
			}
			//---- end light absorbed by rain -----------------------------
			//float intens = glm::length(image_(i, j));
			//if (intens > intens_max)intens_max = intens;
			//intens_avg += intens;
		}
	}
	//intens_avg = intens_avg / (1.0f*bfs);
	//float sf = (intens_max - intens_avg) / (intens_max*intens_max);
	
	//apply rain mask
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic) 
#endif
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			//int n = GetFlattenedIndex(i, j);
			//float intens = glm::length(image_buffer_[n]);
			float r = image_(i, j, 0);
			float g = image_(i, j, 1);
			float b = image_(i, j, 2);
			float intens = (float)sqrt(r*r + g * g + b * b);
			//float scale_fac = (intens_max - intens)*sf;
			//image_buffer_[n] = image_buffer_[n] + rain_mask[i][j] *scale_fac;
			for (int k = 0; k < 3; k++) {
				//image_(i, j, k) = image_(i, j, k) + rain_mask[i][j] * scale_fac;
				image_(i, j, k) = image_(i, j, k) + rain_mask[i][j];
			}
		}
	}

	if (raindrops_on_lens_) {
		float lens_area = horizontal_pixdim_ * vertical_pixdim_;
		float numfac = (float)(lens_area * rate * 1.0E10);
		int num_raindrops = (int)(10.0f*(1.0 - exp(-0.1*numfac)));
		int num_to_add = num_raindrops - (int)droplets_.size();
		for (int i=0;i<num_to_add;i++) {
			LensDrop drop;
			drop.SetRadius(0.2f*mavs::math::rand_in_range(0.75f*d_g, 1.1f*d_g));
			drop.SetRadiusPixels((int)(drop.GetRadius() / horizontal_pixdim_));
			drop.SetCenterPixels(mavs::math::rand_in_range(0, num_horizontal_pix_), mavs::math::rand_in_range(0, num_vertical_pix_));
			drop.SetColor(rain_col.x, rain_col.y, rain_col.z);
			drop.SetLifetime(mavs::math::rand_in_range(2.0f, 5.0f));
			droplets_.push_back(drop);
		}
		for (int d = 0; d < droplets_.size(); d++) {
			droplets_[d].SetAge(droplets_[d].GetAge() + (float)dt);
			if (droplets_[d].GetAge() > droplets_[d].GetLifetime()) {
				droplets_.erase(droplets_.begin() + d);
			}
		}
		cimg_library::CImg<float> new_image = image_;
		for (int d = 0; d < droplets_.size(); d++) {
			glm::ivec2 c = droplets_[d].GetCenterPixels();
			int rp = droplets_[d].GetRadiusPixels();
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic) 
#endif
			for (int i = c.x - rp; i <= c.x + rp; i++) {
				for (int j = c.y - rp; j <= c.y + rp; j++) {
					if (i >= 0 && i < num_horizontal_pix_ && j >= 0 && j < num_vertical_pix_) {
						glm::vec2 pixel(i, j);
						glm::vec2 center((float)c.x, (float)c.y);
						float r = glm::length(pixel - center);
						if (droplets_[d].PixelInDrop(i,j) && r <= rp){
							//blur filter
							glm::vec3 newcol(0.0f, 0.0f, 0.0f);
							int nfilt = 5;
							int nr = (nfilt - 1) / 2;
							for (int ii = -nr; ii <= nr; ii++) {
								int iii = ii + i;
								for (int jj = -nr; jj <= nr; jj++) {
									int jjj = jj + j;
									if (iii >= 0 && iii < num_horizontal_pix_ && jjj >= 0 && jjj < num_vertical_pix_) {
										newcol = newcol + glm::vec3(image_(iii, jjj, 0), image_(iii, jjj, 1), image_(iii, jjj, 2));
									}
								}
							}
							newcol = newcol / (0.8f*nfilt*nfilt);
							glm::vec3 oldcol = glm::vec3(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
							float s = 1.0f - exp(-((float)r / rp));
							newcol = (1.0f-s) * newcol + s*oldcol;
							new_image.draw_point(i, j, (float *)&newcol);
						}
					}
				}
			}
		}
		image_ = new_image;
	}
}

void Camera::CopyBufferToImage() {
	image_.mirror("x");
	image_.mirror("y");
}

void Camera::WriteImageToText(std::string filename) {
	std::ofstream fout(filename.c_str());
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			glm::vec3 p = (glm::vec3)image_(i, j);//image_buffer_[j+i*num_vertical_pix_];
			fout << i << " " << j << " " << p.x << " " << p.y << " " << p.z << std::endl;
		}
	}
	fout.close();
}


#ifdef USE_MPI
void Camera::PublishData(int root, MPI_Comm broadcast_to) {
	MPI_Bcast(&updated_, 1, MPI_LOGICAL, root, broadcast_to);
	if (updated_) {
		int bsize = (int)(image_.height()*image_.width()*image_.spectrum());   //image_buffer_.size();
		MPI_Bcast(&bsize, 1, MPI_INT, root, broadcast_to);
		//if (image_buffer_.size()!=bsize)image_buffer_.resize(bsize);
		MPI_Bcast(image_.data(), bsize, MPI_FLOAT, root, broadcast_to);
		updated_ = false;
	}
}
#endif
Image Camera::GetRosImage() {
	Image image;
	image.height = GetHeight();
	image.width = GetWidth();
	image.is_bigendian = false;
	image.encoding = "rgb8";
	//glm::vec3 *buff = GetImageBuffer();
	//float * buff = GetImageBuffer();
	image.data.resize(3 * image.height * image.width);
	//image.data[0] = (uint8_t)buff;
	int n = 0;
	//for (int j = image.height - 1; j >= 0; j--){
	for (int j = 0; j < (int)image.height; j++) {
		for (int i = 0; i < (int)image.width; i++) {
			for (int k = 0; k < 3; k++) {
				//image.data[n] = (uint8_t)buff[j + i * image.height][k];
				image.data[n] = (uint8_t)image_(i, j, k);
				n++;
			}
		}
	}
	image.step = (uint32_t)(sizeof(uint8_t) * image.data.size() / image.height);
	return image;
	}

void Camera::DisplaySegmentedImage() {
	if (first_seg_display_) {
		seg_display_.assign(segmented_image_);
		first_seg_display_ = false;
	}
	else {
		seg_display_ = segmented_image_;
	}
}

void Camera::Display() {
	if (comm_rank_ == 0) {
		if (first_display_) {
			disp_.assign(num_horizontal_pix_, num_vertical_pix_, name_.c_str(), 0);
			disp_.set_title(name_.c_str());
			//disp_.assign(num_horizontal_pix_,num_vertical_pix_);
			//disp_.set_title("%s",name_.c_str());
			first_display_ = false;
		}
		disp_._normalization = 0;
		disp_ = image_;
		disp_._normalization = 0;
	}
}

void Camera::SaveImage(std::string file_name) {
	if (comm_rank_ == 0) {
		//image_.normalize(0,255);
		image_.save(file_name.c_str());
	}
}

void Camera::SaveRangeImage(std::string file_name) {
	cimg_library::CImg<float> range_image;
	range_image.assign(num_horizontal_pix_, num_vertical_pix_, 1, 1);
	for (int i = 0; i < num_horizontal_pix_; i++) {
		int ii = num_horizontal_pix_ - i - 1;
		for (int j = 0; j < num_vertical_pix_; j++) {
			int jj = num_vertical_pix_ - j - 1;
			int n = GetFlattenedIndex(i, j);
			if (range_buffer_[n] > 0.5f*std::numeric_limits<float>::max() || range_buffer_[n] < 0.0f) {
				range_image(ii, jj) = 0.0f;
			}
			else {
				//range_image(ii, jj) = sqrt(range_buffer_[n]);
                                range_image(ii, jj) = powf(range_buffer_[n],1.25f);
				//range_image(ii, jj) = 10.0f*logf(range_buffer_[n] + 1);
			}
		}
	}
	//range_image.normalize(0, 255);
	range_image.save(file_name.c_str());
}

void Camera::SaveNormalsToText() {
	std::ofstream fout("normals.txt");
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j);
			fout << i << " " << j << " " << normal_buffer_[n].x << " " << normal_buffer_[n].y << " " << normal_buffer_[n].z << std::endl;
		}
	}
	fout.close();
}


Image Camera::GetDisparityImage(environment::Environment *env, float baseline) {
	glm::vec3 look_to_f = focal_length_ * glm::vec3(1.0f, 0.0f, 0.0f);				// * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * glm::vec3(0.0f, 1.0f, 0.0f); // * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * glm::vec3(0.0f, 0.0f, 1.0f);			//look_up_;
	Image image;
	image.height = GetHeight();
	image.width = GetWidth();
	image.is_bigendian = false;
	image.encoding = "rgb8";
	image.data.resize(3 * image.height * image.width);
	float i2 = half_horizontal_dim_ - 0.5f;
	float j2 = half_vertical_dim_ - 0.5f;
	float pixdim = 0.5f * (horizontal_pixdim_ + vertical_pixdim_);
	int n = 0;
	glm::vec3 ls = horizontal_pixdim_ * look_side_;
	glm::vec3 lu = vertical_pixdim_ * look_up_;

#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic)
#endif
	for (int j = 0; j < (int)image.height; j++) {
		for (int i = 0; i < (int)image.width; i++) {
			glm::vec3 direction = look_to_f + (i - i2) * look_side_d + (j - j2) * look_up_d;
			direction = glm::normalize(direction);
			if (range_buffer_[n] >= 0.0) {
				float dist = direction.x * range_buffer_[n];
				int disp = 0;
				if (dist > 0.0) {
					disp = (int)((baseline * focal_length_ / dist) / pixdim);
				}
				for (int k = 0; k < 3; k++) {
					image.data[n] = (uint8_t)disp;
					n++;
				}
			}
		}
	}
	image.step = (uint32_t)(sizeof(uint8_t) * image.data.size() / image.height);
	return image;
}

std::vector<std::vector<glm::vec3> > Camera::GetPointsFromImage() {
	std::vector<std::vector<glm::vec3> > points = mavs::utils::Allocate2DVector(num_horizontal_pix_, num_vertical_pix_, glm::vec3(0.0f, 0.0f, 0.0f));

	glm::vec3 look_to_f = focal_length_  * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;

	float i2 = half_horizontal_dim_ - 0.5f;
	float j2 = half_vertical_dim_ - 0.5f;
	for (int i = 0; i < num_horizontal_pix_; i++) {
		float ix = (i - i2)*horizontal_mag_scale_;
		float id = (float)i;
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j); // j + i * num_vertical_pix_;
			float jd = (float)j;
			if (use_distorted_) {
				glm::vec2 meters(ix,
					(j - j2)*vertical_mag_scale_);
				glm::vec2 pu = dm_.Distort(meters);
				id = pu.x;
				jd = pu.y;
			}
			glm::vec3 direction = look_to_f
				+ (id - i2)*look_side_d
				+ (jd - j2)*look_up_d;
			direction = glm::normalize(direction);
			if (range_buffer_[n] >= 0.0) {
				points[i][j] = position_ + direction * range_buffer_[n];
			}
		}
	}
	return points;
}

PointCloud2 Camera::GetPointCloudFromImage() {
	PointCloud2 pc;
	pc.height = num_vertical_pix_;
	pc.width = num_horizontal_pix_;
	pc.is_bigendian = false;
	pc.is_dense = false;
	pc.point_step = sizeof(glm::vec4);
	pc.row_step = pc.point_step*pc.width;
	PointField x_field, y_field, z_field, intens_field;
	x_field.name = "x";
	x_field.datatype = x_field.FLOAT32;
	x_field.offset = 0;
	x_field.count = 1;
	y_field.name = "y";
	y_field.datatype = y_field.FLOAT32;
	y_field.offset = 4;
	y_field.count = 1;
	z_field.name = "z";
	z_field.datatype = z_field.FLOAT32;
	z_field.offset = 8;
	z_field.count = 1;
	intens_field.name = "intensity";
	intens_field.datatype = intens_field.FLOAT32;
	intens_field.offset = 12;
	intens_field.count = 1;
	pc.fields.push_back(x_field);
	pc.fields.push_back(y_field);
	pc.fields.push_back(z_field);
	pc.fields.push_back(intens_field);
	pc.data.resize(pc.height*pc.width, glm::vec4(0.0f, 0.0f, 0.0f, 0.0f));

	glm::vec3 look_to_f = focal_length_ * glm::vec3(1.0f, 0.0f, 0.0f); // * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * glm::vec3(0.0f, 1.0f, 0.0f); // * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * glm::vec3(0.0f, 0.0f, 1.0f); //look_up_;

	float i2 = half_horizontal_dim_ - 0.5f;
	float j2 = half_vertical_dim_ - 0.5f;

#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic)
#endif
	for (int i = 0; i < num_horizontal_pix_; i++) {
		float ix = (i - i2)*horizontal_mag_scale_;
		float id = (float)i;
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j); // j + i * num_vertical_pix_;
#ifdef USE_MPI
			if ((n) % comm_size_ == comm_rank_) {
#endif			
				float jd = (float)j;
				if (use_distorted_) {
					glm::vec2 meters(ix,
						(j - j2)*vertical_mag_scale_);
					glm::vec2 pu = dm_.Distort(meters);
					id = pu.x;
					jd = pu.y;
				}
				glm::vec3 direction = look_to_f
					+ (id - i2)*look_side_d
					+ (jd - j2)*look_up_d;
				direction = glm::normalize(direction);
				if (range_buffer_[n] >= 0.0) {
					//glm::vec3 point = position_ + direction*range_buffer_[n];
					glm::vec3 point = direction * range_buffer_[n];
					pc.data[n].x = point.x;
					pc.data[n].y = point.y;
					pc.data[n].z = point.z;
					pc.data[n].w = image_(i, j, 0) / 255.0f;
				}
#ifdef USE_MPI
			}
#endif
		}
	}
	return pc;
}

void Camera::AnnotateFull(environment::Environment *env) {

	struct ObjectPixelLimits {
		ObjectPixelLimits() {
			ll_x = 100000;
			ll_y = 100000;
			ur_x = 0;
			ur_y = 0;
		}
		int ll_x;
		int ll_y;
		int ur_x;
		int ur_y;
	};

	std::vector<int> objects_in_frame;
	std::vector<ObjectPixelLimits> object_limits;
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j); // j + i * num_vertical_pix_;
			glm::vec3 col(0.0f, 0.0f, 0.0f);
			int obj_num = segment_buffer_[n];
			if (!(std::find(objects_in_frame.begin(), objects_in_frame.end(), obj_num) != objects_in_frame.end())) {
				objects_in_frame.push_back(obj_num);
				ObjectPixelLimits lims;
				lims.ll_x = i;
				lims.ur_x = i;
				lims.ll_y = j;
				lims.ur_y = j;
				object_limits.push_back(lims);
			}
			else {
				for (int on = 0; on < (int)objects_in_frame.size(); on++) {
					if (obj_num == objects_in_frame[on]) {
						// update the limits
						if (i > object_limits[on].ur_x)object_limits[on].ur_x = i;
						if (j > object_limits[on].ur_y)object_limits[on].ur_y = j;
						if (i < object_limits[on].ll_x)object_limits[on].ll_x = i;
						if (j < object_limits[on].ll_y)object_limits[on].ll_y = j;
					}
				}
			}
			if (obj_num >= 0) {
				col = anno_colors_.GetColor(obj_num);
			}
			segmented_image_.draw_point(num_horizontal_pix_ - i - 1, num_vertical_pix_ - j - 1,
				(float*)&col);
		}
	}

	object_annotations_.clear();
	std::map<int, Annotation>::iterator iter;
	//std::map<std::string, Annotation>::iterator iter;
	for (int n = 0; n < (int)objects_in_frame.size(); n++) {
		// Store annotation information
		//iter = annotations_.begin();
		iter = object_annotations_.begin();
		int obj_num = objects_in_frame[n];
		
		std::string mesh_name = env->GetObjectName(obj_num);
		iter = object_annotations_.find(obj_num);
		//iter = annotations_.find(mesh_name);

		/*
		raytracer::BoundingBox bb = env->GetObjectBoundingBox(obj_num);
		std::vector<glm::vec3> corners = bb.GetCorners();
		int imin = num_horizontal_pix_;
		int imax = 0;
		int jmin = num_vertical_pix_;
		int jmax = 0;
		for (int c = 0; c < (int)corners.size(); c++) {
			glm::ivec2 pc;
			bool inframe = WorldToPixel(corners[c], pc);
			if (inframe) {
				int ip = (int)pc.x;
				int jp = (int)pc.y;
				if (ip < imin)imin = ip;
				if (ip > imax)imax = ip;
				if (jp < jmin)jmin = jp;
				if (jp > jmax)jmax = jp;
			}
		}
		*/
		//if (iter == annotations_.end()) {
			//iter = annotations_.begin();
		if (iter == object_annotations_.end()) {
			iter = object_annotations_.begin();
			//annotations_.insert(std::make_pair(mesh_name, Annotation(mesh_name, imin, jmin, imax, jmax)));
			//annotations_.insert(std::make_pair(mesh_name, Annotation(mesh_name, object_limits[n].ll_x, object_limits[n].ll_y, object_limits[n].ur_x, object_limits[n].ur_y)));
			object_annotations_.insert(std::make_pair(obj_num, Annotation(mesh_name, object_limits[n].ll_x, object_limits[n].ll_y, object_limits[n].ur_x, object_limits[n].ur_y)));
		}
	}

	//draw_rectangles
	//for (iter = annotations_.begin(); iter != annotations_.end(); iter++) {
	for (iter = object_annotations_.begin(); iter != object_annotations_.end(); iter++) {
		if (draw_annotations_) {
			DrawRectangle(iter->second.GetLLPixel(), iter->second.GetURPixel());
		}
	}

}

void Camera::AnnotateFrame(environment::Environment *env, bool semantic) {
	//std::map<int, Annotation>::iterator iter;
	std::map<std::string, Annotation>::iterator iter;
	annotations_.clear();
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = j + i * num_vertical_pix_;
			glm::vec3 col(0.0f, 0.0f, 0.0f);
			int obj_num = segment_buffer_[n];
			int lab_num = 0;
			std::string label_name = label_buffer_[n]; // env->GetLabel(mesh_name);
			//std::string mesh_name = env->GetObjectName(obj_num);
			// Store annotation information
			iter = annotations_.begin();
			//iter = annotations_.find(segment_buffer_[n]);
			iter = annotations_.find(label_name);
			if (iter == annotations_.end()) {
				iter = annotations_.begin();
				if (semantic) {
					lab_num = env->GetLabelNum(label_name);
					//annotations_.insert(std::make_pair(lab_num, Annotation(label_name, i, j, i, j)));
					glm::vec3 color = 255.0f*env->GetLabelColor(label_name); // anno_colors_.GetColor(lab_num);
					//annotations_.insert(std::make_pair(lab_num, Annotation(label_name, (int)color.x, (int)color.y,(int)color.z)));
					annotations_.insert(std::make_pair(label_name, Annotation(label_name, (int)color.x, (int)color.y, (int)color.z)));
				}
				else {
					std::string mesh_name = env->GetObjectName(obj_num);
					//annotations_.insert(std::make_pair(obj_num, Annotation(mesh_name, i, j, i, j)));
					annotations_.insert(std::make_pair(label_name, Annotation(mesh_name, i, j, i, j)));
				}
			}
			else {
				if (i < iter->second.GetLLPixel().x) { iter->second.SetLLPixel(i, iter->second.GetLLPixel().y); }
				if (i > iter->second.GetURPixel().x) { iter->second.SetURPixel(i, iter->second.GetURPixel().y); }
				if (j < iter->second.GetLLPixel().y) { iter->second.SetLLPixel(iter->second.GetLLPixel().x, j); }
				if (j > iter->second.GetURPixel().y) { iter->second.SetURPixel(iter->second.GetURPixel().x, j); }
			}
			if (semantic) {
				col = 255.0f*env->GetLabelColor(label_name); //anno_colors_.GetColor(lab_num);
			}
			else {
				if (obj_num >= 0) {
					col = anno_colors_.GetColor(obj_num);
				}
			}
			segmented_image_.draw_point(num_horizontal_pix_ - i - 1, num_vertical_pix_ - j - 1,
				(float*)&col);
		}
	}

	annotations_.erase(prev(annotations_.end()));
	if (!semantic) {
		//draw_rectangles
		for (iter = annotations_.begin(); iter != annotations_.end(); ++iter) {
			if (draw_annotations_) {
				if (iter->second.GetName() != "textured_surface") DrawRectangle(iter->second.GetLLPixel(), iter->second.GetURPixel());
			}
		}
	}
}

void Camera::DrawRectangle(glm::ivec2 ll, glm::ivec2 ur) {
	// draw top, bottom
	glm::vec3 blue(0.0f, 0.0f, 255.0f);
	if (ll.x < 0)ll.x = 0;
	if (ur.x >= num_horizontal_pix_)ur.x = num_horizontal_pix_ - 1;
	if (ll.y < 0)ll.y = 0;
	if (ur.y >= num_vertical_pix_)ur.y = num_vertical_pix_ - 1;
	for (int i = ll.x; i < ur.x; i++) {
		segmented_image_.draw_point(num_horizontal_pix_ - i - 1,
			num_vertical_pix_ - ur.y - 1, (float*)&blue);
		segmented_image_.draw_point(num_horizontal_pix_ - i - 1,
			num_vertical_pix_ - ll.y - 1, (float*)&blue);
	}
	// draw sides
	for (int j = ll.y; j < ur.y; j++) {
		segmented_image_.draw_point(num_horizontal_pix_ - ur.x - 1,
			num_vertical_pix_ - j - 1, (float*)&blue);
		segmented_image_.draw_point(num_horizontal_pix_ - ll.x - 1,
			num_vertical_pix_ - j - 1, (float*)&blue);
	}
}

void Camera::SaveAnnotationsCsv(std::string fname) {
	// Handle frame annotations
	std::ofstream annotationFile;
	annotationFile.open(fname.c_str());
	annotationFile << "Name, Min_X, Min_Y, Max_X, Max_Y" << std::endl;
	//std::map<int, Annotation>::iterator iter;
	std::map<std::string, Annotation>::iterator iter;
	iter = annotations_.begin();
	for (iter = annotations_.begin(); iter != annotations_.end(); ++iter) {
		if (iter->second.GetName() != "textured_surface") {
			annotationFile << iter->second.GetName() << ", " << iter->second.GetLLPixel().x << ", " << iter->second.GetLLPixel().y << ", " <<
				iter->second.GetURPixel().x << ", " << iter->second.GetURPixel().y << std::endl;
		}
	}
	annotationFile.close();
}

void Camera::SaveBoxAnnotationsCsv(std::string fname) {
	// Handle frame annotations
	std::ofstream annotationFile;
	annotationFile.open(fname.c_str());
	annotationFile << "Name, Min_X, Min_Y, Max_X, Max_Y" << std::endl;
	std::map<int, Annotation>::iterator iter;
	//std::map<std::string, Annotation>::iterator iter;
	iter = object_annotations_.begin();
	for (iter = object_annotations_.begin(); iter != object_annotations_.end(); ++iter) {
		annotationFile << iter->second.GetName() << ", " << 
			num_horizontal_pix_ - iter->second.GetURPixel().x - 1 << ", " <<
			num_vertical_pix_ - iter->second.GetURPixel().y - 1 << ", " <<
			num_horizontal_pix_ - iter->second.GetLLPixel().x - 1 << ", " <<
			num_vertical_pix_ - iter->second.GetLLPixel().y - 1 << std::endl;
	}
	annotationFile.close();
}

void Camera::SaveSemanticAnnotationsCsv(std::string fname) {
	// Handle frame annotations
	std::ofstream annotationFile;
	annotationFile.open(fname.c_str());
	annotationFile << "Name, Red, Green, Blue" << std::endl;
	//std::map<int, Annotation>::iterator iter;
	std::map<std::string, Annotation>::iterator iter;
	iter = annotations_.begin();
	for (iter = annotations_.begin(); iter != annotations_.end(); ++iter) {
		glm::ivec3 col = iter->second.GetColor();
		annotationFile << iter->second.GetName() << ", " << col.x << ", " << col.y << ", " << col.z << std::endl;
	}
	annotationFile.close();
}

void Camera::SaveAnnotationsXmlVoc(std::string fname) {
	//create the xml document and root
	tinyxml2::XMLDocument xml_doc;
	tinyxml2::XMLNode * p_root = xml_doc.NewElement("annotation");
	xml_doc.InsertFirstChild(p_root);

	//------ Add if image is segmented ----
	tinyxml2::XMLElement * segment_element = xml_doc.NewElement("segmented");
	segment_element->SetText(1);
	p_root->InsertEndChild(segment_element);
	//------- add image size info ----
	tinyxml2::XMLElement * size_element = xml_doc.NewElement("size");
	tinyxml2::XMLElement * width_element = xml_doc.NewElement("width");
	tinyxml2::XMLElement * height_element = xml_doc.NewElement("height");
	tinyxml2::XMLElement * depth_element = xml_doc.NewElement("depth");
	width_element->SetText(num_horizontal_pix_);
	height_element->SetText(num_vertical_pix_);
	depth_element->SetText(3);
	size_element->InsertEndChild(width_element);
	size_element->InsertEndChild(height_element);
	size_element->InsertEndChild(depth_element);
	p_root->InsertEndChild(size_element);
	//------- add annotation info ---
	//std::map<int, Annotation>::iterator iter;
	std::map<std::string, Annotation>::iterator iter;
	iter = annotations_.begin();
	for (iter = annotations_.begin(); iter != annotations_.end(); ++iter) {
		tinyxml2::XMLElement * object_element = xml_doc.NewElement("object");
		tinyxml2::XMLElement * name_element = xml_doc.NewElement("name");
		tinyxml2::XMLElement * box_element = xml_doc.NewElement("bndbox");
		tinyxml2::XMLElement * xmin_element = xml_doc.NewElement("xmin");
		tinyxml2::XMLElement * ymin_element = xml_doc.NewElement("ymin");
		tinyxml2::XMLElement * xmax_element = xml_doc.NewElement("xmax");
		tinyxml2::XMLElement * ymax_element = xml_doc.NewElement("ymax");
		name_element->SetText(iter->second.GetName().c_str());
		xmin_element->SetText(iter->second.GetLLPixel().x);
		ymin_element->SetText(iter->second.GetLLPixel().y);
		xmax_element->SetText(iter->second.GetURPixel().x);
		ymax_element->SetText(iter->second.GetURPixel().y);
		box_element->InsertEndChild(xmin_element);
		box_element->InsertEndChild(ymin_element);
		box_element->InsertEndChild(xmax_element);
		box_element->InsertEndChild(ymax_element);
		object_element->InsertEndChild(name_element);
		object_element->InsertEndChild(box_element);
		p_root->InsertEndChild(object_element);
	}

	//save the document
	tinyxml2::XMLError e_result = xml_doc.SaveFile(fname.c_str());
}

void Camera::SaveSegmentedImage(std::string file_name) {
	segmented_image_.save(file_name.c_str());
}


void Camera::SaveAnnotation(std::string ofname) {
	std::string imout = ofname + ".bmp";
	std::string csvout = ofname + ".csv";
	SaveSegmentedImage(imout);
	SaveSemanticAnnotationsCsv(csvout);
}

void Camera::SaveAnnotation() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << ncalled_;
	std::string num_string(ss.str());
	std::string outname = prefix_ + type_ + name_ + num_string + "_annotated";
	SaveAnnotation(outname);
}

void Camera::SaveRaw() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << ncalled_;
	std::string num_string(ss.str());
	std::string outname = prefix_ + type_ + name_ + num_string + ".bmp";
	SaveImage(outname);
	ncalled_++;
}

void Camera::ApplyRccbFilter() {
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			float new_green = 0.3f*image_(i, j, 0) + 0.59f*image_(i, j, 1) + 0.11f*image_(i, j, 2);
			image_(i, j, 1) = new_green;
		}
	}
}

glm::vec3 Camera::RgbToHsl(float r, float g, float b) {
	// see: https://github.com/ratkins/RGBConverter/blob/master/RGBConverter.cpp
	float rd = (float)r / 255.0f;
	float gd = (float)g / 255.0f;
	float bd = (float)b / 255.0f;
	float max = std::max(std::max(rd, gd), bd);
	float min = std::min(std::min(rd, gd), bd);
	float h, s, l = (max + min) / 2;

	if (max == min) {
		h = s = 0.0f; // achromatic
	}
	else {
		float d = max - min;
		s = l > 0.5 ? d / (2 - max - min) : d / (max + min);
		if (max == rd) {
			h = (gd - bd) / d + (gd < bd ? 6 : 0);
		}
		else if (max == gd) {
			h = (bd - rd) / d + 2;
		}
		else if (max == bd) {
			h = (rd - gd) / d + 4;
		}
		h /= 6;
	}
	glm::vec3 hsl(h, s, l);
	return hsl;
}

static float Hue2Rgb(float p, float q, float t) {
	// see: https://github.com/ratkins/RGBConverter/blob/master/RGBConverter.cpp
	if (t < 0.0f) t += 1.0f;
	if (t > 1.0f) t -= 1.0f;
	if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
	if (t < 1.0f / 2.0f) return q;
	if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
	return p;
}

glm::vec3 Camera::HslToRgb(float h, float s, float l) {
	// see: https://github.com/ratkins/RGBConverter/blob/master/RGBConverter.cpp
	float r, g, b;

	if (s == 0.0f) {
		r = g = b = l; // achromatic
	}
	else {
		float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
		float p = 2.0f * l - q;
		r = Hue2Rgb(p, q, h + 1.0f / 3.0f);
		g = Hue2Rgb(p, q, h);
		b = Hue2Rgb(p, q, h - 1.0f / 3.0f);
	}
	glm::vec3 rgb(255.0f * r, 255.0f * g, 255.0f * b);
	return rgb;
}

void Camera::AdjustSaturationAndTemperature() {
	// saturation
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			glm::vec3 hsl = RgbToHsl(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
			hsl[1] *= img_saturation_;
			glm::vec3 rgb = HslToRgb(hsl[0], hsl[1], hsl[2]);
			for (int k = 0; k < 3; k++) image_(i, j, k) = rgb[k];
		}
	}

	// temperature
	float t1 = (img_temperature_ - 6500.0f) / 10000.0f;
	float t2 = t1 * t1;
	float t3 = t2 * t1;
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			image_(i, j, 0) = std::max(0.0f, std::min(255.0f,image_(i, j, 0) + 128.0f*( 0.6038f*t3 - 0.6634f*t2 + 0.4610f*t1)));
			image_(i, j, 1) = std::max(0.0f, std::min(255.0f, image_(i, j, 1) + 128.0f*(-0.2755f*t3 + 0.2372f*t2 - 0.0677f*t1)));
			image_(i, j, 2) = std::max(0.0f, std::min(255.0f, image_(i, j, 2) + 128.0f*(-1.3989f*t3 + 1.4863f*t2 - 0.8055f*t1)));
		}
	}
}

bool Camera::WorldToPixel(glm::vec3 point_world, glm::ivec2 &pixel) {
	glm::vec3 v = point_world - position_;
	float z = glm::dot(v, look_to_);
	if (z <= 0.0f) {
		return false;
	}
	float x = glm::dot(v, look_side_);
	float y = glm::dot(v, look_up_);

	pixel = glm::ivec2((int)((focal_length_*x / z) / horizontal_pixdim_),
		(int)((focal_length_*y / z) / vertical_pixdim_));
	return true;
}

void Camera::RenderParticleSystems(environment::Environment *env) {
	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;

	glm::vec3 suncolor = env->GetSunColor();
	environment::ParticleSystem *systems = env->GetParticleSystems();
	for (int ps = 0; ps < (int)env->GetNumParticleSystems(); ps++) {
		environment::Particle *particles = systems[ps].GetParticles();
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
		for (int p = 0; p < (int)systems[ps].GetNumParticles(); p++) {
			float r = particles[p].radius_;
			glm::vec3 to_particle = particles[p].position_ - position_;
			float x = glm::dot(to_particle, look_side_);
			float y = glm::dot(to_particle, look_up_);
			float R = glm::dot(to_particle, look_to_);
			float M = focal_length_ / R;
			float M_h = M / horizontal_pixdim_;
			float M_v = M / vertical_pixdim_;
			int px_l = (int)(half_horizontal_dim_ + floor(M_h*(x - r)));
			int px_h = (int)(half_horizontal_dim_ + ceil(M_h*(x + r)));
			int py_l = (int)(half_vertical_dim_ + floor(M_v*(y - r)));
			int py_h = (int)(half_vertical_dim_ + ceil(M_v*(y + r)));
			if (px_l < 0) px_l = 0;
			if (px_l >= num_horizontal_pix_) px_l = num_horizontal_pix_ - 1;
			if (px_h < 0) px_h = 0;
			if (px_h >= num_horizontal_pix_) px_h = num_horizontal_pix_ - 1;
			if (py_l < 0) py_l = 0;
			if (py_l >= num_vertical_pix_) py_l = num_vertical_pix_ - 1;
			if (py_h < 0) py_h = 0;
			if (py_h >= num_vertical_pix_) py_h = num_vertical_pix_ - 1;
			for (int u = px_l; u <= px_h; u++) {
				for (int v = py_l; v < py_h; v++) {
					int n = GetFlattenedIndex(u, v); // v + u * num_vertical_pix_;

					float ud = (float)u;
					float vd = (float)v;
					if (use_distorted_) {
						glm::vec2 meters((u - half_horizontal_dim_ + 0.5f)*horizontal_mag_scale_,
							(v - half_vertical_dim_ + 0.5f)*vertical_mag_scale_);
						glm::vec2 pu = dm_.Distort(meters);
						ud = pu.x;
						vd = pu.y;
					}

					glm::vec3 direction = (float)(ud - half_horizontal_dim_)*look_side_d +
						(float)(vd - half_vertical_dim_)*look_up_d + look_to_f;

					//glm::vec3 direction = (float)(u-half_horizontal_dim_)*look_side_d +
					//	(float)(v-half_vertical_dim_)*look_up_d + look_to_f;

					direction = direction / glm::length(direction);
					glm::vec2 d = particles[p].GetIntersection(position_, direction);
					if (d.y > 0.0f && d.y < range_buffer_[n]) {
						if (d.x<1.0f && d.x>0.0f) {
							float s = 1.0f - d.x;
							float t = 1.0f - s * (1.0f - particles[p].transparency_);
							glm::vec3 col;
							int vert_index = num_vertical_pix_ - v - 1;
							for (int c = 0; c < 3; c++) {
								//col[c] = (1.0f-t)*suncolor_[c]*particles[p].color_[c] +t*image_buffer_[n][c];
								col[c] = (1.0f - t)*suncolor[c] * particles[p].color_[c] + t * image_(u, v, c);
								image_(u, v, c) = col[c];
							}
							//image_buffer_[n] = col;
						}
					}
				}
			}
		}
	}
} // RenderParticleSystems

void Camera::AddLidarPointsToImage(std::vector<glm::vec4> registered_xyzi) {
	mavs::utils::Mplot plotter;
	float fh = focal_length_ / horizontal_pixdim_;
	float fv = focal_length_ / vertical_pixdim_;
	//mavs::Pose pose = GetPose();
	int pixrad = (int)floor(0.5f*num_horizontal_pix_*num_vertical_pix_ / 1000000.0f);
	for (int i = 0; i < (int)registered_xyzi.size(); i++) {
		glm::vec3 p(registered_xyzi[i].x, registered_xyzi[i].y, registered_xyzi[i].z);
		/*glm::vec3 v = p - position_; // (glm::vec3)pose.position;
		// check if point is behind me
		if (glm::dot(v, look_to_) < 0.0f)continue;

		//project it to camera frame
		float x = glm::dot(v, look_side_);
		float y = glm::dot(v, look_up_);
		float dp = glm::length(v);
		int px = (int)floor(( (fh * (x / dp)) + half_horizontal_dim_));
		int py = (int)floor(( (fv * (y / dp)) + half_vertical_dim_));
		*/
		glm::ivec2 pix;
		bool in_image = WorldToPixel(p, pix);
		// if in frame, render
		if (pix.x>=0 && pix.x<num_horizontal_pix_ && pix.y>=0 && pix.y<num_vertical_pix_) {
			int n = GetFlattenedIndex(pix.x, pix.y);
			float dp = glm::length(p - position_);
			if (dp < range_buffer_[n]) {
				mavs::utils::Color color = plotter.MorelandColormap(registered_xyzi[i].w, 0.0f, 1.0f);
				if (pixrad <= 0) {
					image_.draw_point(num_horizontal_pix_ - pix.x, num_vertical_pix_ - pix.y, (float *)&color);
				}
				else {
					image_.draw_circle(num_horizontal_pix_ - pix.x, num_vertical_pix_ - pix.y, pixrad, (float *)&color);
				}
			}
		}
	}
}

} //namespace camera
} //namespace sensor
} //namespace mavs
