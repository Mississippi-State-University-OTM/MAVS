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

#include <sensors/camera/ortho_camera.h>

#include <iostream>
#include <ctime>

#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace camera{

OrthoCamera::OrthoCamera(){
  updated_ = false;
  local_sim_time_ = 0.0f;
  Initialize(512,512,0.0035f,0.0035f,0.0035f);
  gamma_ = 0.75f;
	gain_ = 1.0f; 
	background_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
	camera_type_ = "ortho";
}

OrthoCamera::~OrthoCamera(){

}

void OrthoCamera::Update(environment::Environment *env, double dt){
	CheckFreq(dt);
	if (disp_is_free_)UpdatePoseKeyboard();
  //define directions
  glm::vec3 look_to_f = focal_length_*look_to_;
  glm::vec3 look_side_d = horizontal_pixdim_*look_side_;
  glm::vec3 look_up_d = vertical_pixdim_*look_up_;

	points_ = mavs::utils::Allocate2DVector(num_horizontal_pix_, num_vertical_pix_, glm::vec3(0.0f, 0.0f, 0.0f));

  ZeroBuffer();
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
  for (int i=0;i<num_horizontal_pix_;i++){
    for (int j=0;j<num_vertical_pix_;j++){
			int n = j + i * num_vertical_pix_;
#ifdef USE_MPI
      if ( (n)%comm_size_==comm_rank_){
#endif
				glm::vec3 direction = look_to_f 
					+ (i-half_horizontal_dim_+0.5f)*look_side_d 
					+ (j-half_vertical_dim_+0.5f)*look_up_d;
					direction = glm::normalize(direction);

					//glm::vec3 origin(position_.x + (i - 0.5*num_horizontal_pix_)*horizontal_pixdim_,
					//	y_pos + (j - 0.5*num_vertical_pix_)*vertical_pixdim_,
					//	height);
					glm::vec3 origin = position_ + look_side_ * (i - 0.5f*num_horizontal_pix_)*horizontal_pixdim_ +
						look_up_ * (j - 0.5f*num_vertical_pix_)*vertical_pixdim_;
					glm::vec3 pix_color = background_color_;  
					raytracer::Intersection inter = 
						env->GetClosestIntersection(origin,look_to_);
					if (inter.dist > 0) {
						//assume sun is directly overhead
						//if (inter.normal.z < 0.0f)inter.normal = -1.0f*inter.normal;
						//float rho = 0.25f + 0.75f*std::max(inter.normal.z,0.0f) / glm::length(inter.normal);
						inter.normal = inter.normal / (glm::length(inter.normal));
						float rho = std::fabs(glm::dot(inter.normal,look_to_));
						pix_color = rho*255.0f*inter.color;
						segment_buffer_[n] = inter.object_id;
						range_buffer_[n] = inter.dist;
						points_[i][j] = origin + inter.dist*look_to_;
					}
					//image_buffer_[n] = pix_color;
					for (int c = 0; c < 3; c++)image_(i, j, c) = pix_color[c];
#ifdef USE_MPI
      }
#endif
    }
  }

  ReduceImageBuffer();

	CopyBufferToImage();

	image_.normalize(0, 255);

  if (log_data_) {
    SaveImage(utils::ToString(local_sim_time_)+log_file_name_); }

  local_sim_time_ += local_time_step_;
  updated_ = true;
}

void OrthoCamera::AddLine(std::vector<glm::ivec2> points, glm::vec3 color) {
	for (int i = 0; i < (int)(points.size()-1); i++) {
		image_.draw_line(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y, (float *)&color);
	}
}

} //namespace camera
} //namespace sensor
} //namespace mavs
