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

#include <sensors/camera/spherical_camera.h>

#include <iostream>
#include <ctime>

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>

#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace camera{

SphericalCamera::SphericalCamera(){
  updated_ = false;
  local_sim_time_ = 0.0;
  Initialize(512,512,0.0035f,0.0035f,0.0035f);
	camera_type_ = "spherical";
}

SphericalCamera::~SphericalCamera(){

}

void SphericalCamera::Update(environment::Environment *env, double dt){
	CheckFreq(dt);

	if (disp_is_free_)UpdatePoseKeyboard();

	float phi_range = (float)(360.0*mavs::kDegToRad);
	float phi_min = (float)(-180.0*mavs::kDegToRad);
	float phi_step = phi_range / (1.0f*num_horizontal_pix_);
	float theta_range = (float)(180.0*mavs::kDegToRad);
	float theta_min = 90.0f;
	float theta_step = -theta_range / (1.0f*num_vertical_pix_);

	//define directions
	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;

	ZeroBuffer();
	float phi = 0.0f;
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
	for (int i = 0; i < num_horizontal_pix_; i++) {
		float theta = 0.0f;
		for (int j = num_vertical_pix_ - 1; j >= 0; j--) {
			int n = j + i * num_vertical_pix_;
#ifdef USE_MPI
			if ((n) % comm_size_ == comm_rank_) {
#endif
				float s = sin(theta);
				glm::vec3 direction(s*cos(phi), s*sin(phi), cos(theta));
				direction = orientation_ * direction;
				glm::vec3 pix_color(135, 206, 250);
				raytracer::Intersection inter =
					env->GetClosestIntersection(position_, direction);
				if (inter.dist > 0) {
					pix_color = 255.0f*inter.color;
					segment_buffer_[n] = inter.object_id;
					range_buffer_[n] = inter.dist;
				}
				theta += theta_step;
				//image_buffer_[n] = pix_color;
				for (int c = 0; c < 3; c++)image_(i, j, c) = pix_color[c];
#ifdef USE_MPI
			}
#endif
		}
		phi += phi_step;
	}

  ReduceImageBuffer();

	CopyBufferToImage();

	image_.normalize(0, 255);

  if (log_data_) {
    SaveImage(utils::ToString(local_sim_time_)+log_file_name_); }

  local_sim_time_ += local_time_step_;
  updated_ = true;
}

} //namespace camera
} //namespace sensor
} //namespace mavs
