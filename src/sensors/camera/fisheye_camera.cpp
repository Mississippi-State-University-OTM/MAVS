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
#include <sensors/camera/fisheye_camera.h>

#include <mavs_core/math/utils.h>

namespace mavs {
namespace sensor {
namespace camera {

FisheyeCamera::FisheyeCamera() {
	// Peleng 3.5 / 8A on a APS-C camera
	Initialize(810, 540, 0.0225f, 0.015f, 0.008f);
	pixel_sample_factor_ = 1;
	camera_type_ = "fisheye";
}

glm::vec3 FisheyeCamera::Distort(glm::vec2 pix) {
	float x = pix.x;
	float y = pix.y;
	float r = sqrt(x*x + y * y);
	float x_y = x / y;
	float thetap;
	if (fisheye_projection_ == "equidistant") {
		thetap = r / focal_length_;
	}
	else if (fisheye_projection_ == "equisolid") {
		thetap = 2.0f*asin(0.5f*r / focal_length_);
	}
	else if (fisheye_projection_ == "stereographic") {
		thetap = 2.0f*atan(0.5f*r / focal_length_);
	}
	else { //default to equidistant
		thetap = r / focal_length_;
	}
	float rp = focal_length_ * tan(thetap);
	glm::vec3 distorted;
	distorted.y = sqrt(rp*rp / (1 + x_y * x_y))*math::get_sign(y);
	distorted.x = x_y * distorted.y;
	distorted.z = thetap;
	return distorted;
}

void FisheyeCamera::Update(environment::Environment *env, double dt) {
	CheckFreq(dt);
	if (local_sim_time_ <= 0.0 || !env_set_)SetEnvironmentProperties(env);

	ZeroBuffer();

	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;

	//note that looping over i-j is faster than manually
	// unrolling the loop
#ifdef USE_OMP  
#if defined(_WIN32) || defined(WIN32)
#pragma omp parallel for schedule(dynamic)
#else
#pragma omp parallel for schedule(dynamic) collapse(2)
#endif
#endif  
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = j + i * num_vertical_pix_;
#ifdef USE_MPI
			if ((n) % comm_size_ == comm_rank_) {
#endif
			RgbPixel pixel;
			float num_dist = 0.0f;
			float dist_accum = 0.0f;
			for (int k = 0; k < pixel_sample_factor_; k++) {
				glm::vec2 pix_undistort; 
				if (k == 0) {
					pix_undistort.x = (i - half_horizontal_dim_ + 0.5f)*horizontal_pixdim_;
					pix_undistort.y = (j - half_vertical_dim_ + 0.5f)*vertical_pixdim_;
				}
				else {
					pix_undistort.x = (i + mavs::math::rand_in_range(0.1f, 0.9f) - half_horizontal_dim_)*horizontal_pixdim_;
					pix_undistort.y = (j + mavs::math::rand_in_range(0.1f, 0.9f) - half_vertical_dim_)*vertical_pixdim_;

				}
				glm::vec3 pix = Distort(pix_undistort);
				float thetap = pix.z;
				if (thetap <= kPi_2 && fabs(pix.x) < half_horizontal_dim_ && fabs(pix.y) < half_vertical_dim_) {
					glm::vec3 direction = look_to_f + pix.x * look_side_ + pix.y * look_up_;
					direction = glm::normalize(direction);
					RgbPixel this_pix = RenderPixel(env, direction);
					pixel.color += this_pix.color / (float)pixel_sample_factor_;
					pixel.object_id = this_pix.object_id;
					if (this_pix.distance > 0.0f) {
						num_dist = num_dist + 1.0f;
						dist_accum += this_pix.distance;
					}
				}
			}
			//image_buffer_[n] = pixel.color;
			for (int c = 0; c < 3; c++)image_(i, j, c) = pixel.color[c];
			if (num_dist > 0.0f)range_buffer_[n] = dist_accum / num_dist;
			segment_buffer_[n] = pixel.object_id;
#ifdef USE_MPI
			}
#endif
		}
	}

#ifdef USE_MPI  
	ReduceImageBuffer();
#endif

	RenderParticleSystems(env);

	ApplyElectronics(dt);

	CopyBufferToImage();

	if (log_data_) {
		SaveImage(utils::ToString(local_sim_time_) + log_file_name_);
	}

	local_sim_time_ += local_time_step_;
	updated_ = true;
}

bool FisheyeCamera::WorldToPixel(glm::vec3 point_world, glm::ivec2 &pixel) {
	// first put in the pixel reference frame. 
	glm::vec2 pixfloat;
	glm::vec3 v = point_world - position_;
	float z = glm::dot(v, look_to_);
	if (z <= 0.0f) {
		return false;
	}
	float x = glm::dot(v, look_side_);
	float y = glm::dot(v, look_up_);
	glm::vec2 p_meters_undistort(focal_length_*x / z, focal_length_*y / z);

	pixfloat = Distort(p_meters_undistort);
	pixfloat = glm::vec2(pixfloat.x / horizontal_pixdim_, pixfloat.y / vertical_pixdim_);

	pixfloat.x += half_horizontal_dim_;
	pixfloat.y += half_vertical_dim_;
	pixel = glm::ivec2((int)pixfloat.x, (int)pixfloat.y);

	return true;
}

} //namespace camera
} //namespace sensor
} //namespace mavs
