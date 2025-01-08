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

#include <sensors/camera/micasense_red_edge.h>

#include <iostream>
#include <ctime>

#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace camera{

MicaSenseRedEdge::MicaSenseRedEdge(){
  updated_ = false;
  local_sim_time_ = 0.0f;
  Initialize(1280, 960, 0.02032f, 0.01524f, 0.02326f);
  gamma_ = 0.75f;
	gain_ = 1.0f; 
	background_color_ = 0.0f;
	image_.assign(num_horizontal_pix_, num_vertical_pix_, 1, 5, 0.);
	wavelengths_.push_back(0.475f); // blue
	wavelengths_.push_back(0.560f); // green
	wavelengths_.push_back(0.668f); // red
	wavelengths_.push_back(0.717f); // edge
	wavelengths_.push_back(0.840f); // nir
	camera_type_ = "rededge";
}

MicaSenseRedEdge::~MicaSenseRedEdge(){

}

void MicaSenseRedEdge::Update(environment::Environment *env, double dt){
	CheckFreq(dt);
	if (disp_is_free_)UpdatePoseKeyboard();
  //define directions
  glm::vec3 look_to_f = focal_length_*look_to_;
  glm::vec3 look_side_d = horizontal_pixdim_*look_side_;
  glm::vec3 look_up_d = vertical_pixdim_*look_up_;
	env->GetScene()->TurnOnSpectral();
	glm::vec3 sun_direction = env->GetSolarDirection();
  ZeroBuffer();

	float rho_diff = 0.25f;
	float rho_direct = 0.75f;
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
					//glm::vec3 pix_color(background_color_,background_color_,background_color_);  
					float pix_color[5];
					for (int wl = 0; wl < 5; wl++)pix_color[wl] = 0.0f;
					raytracer::Intersection inter = 
						env->GetClosestIntersection(position_,direction);
					
					if (inter.dist > 0.0f) { // intersected

						bool shadowed = false;
						glm::vec3 point = position_ + 0.999f*inter.dist*direction;
						glm::vec3 diffcol = zero_vec3;
						glm::vec3 speccol = zero_vec3;
						inter.normal = glm::normalize(inter.normal);
						if (render_shadows_)shadowed = env->GetAnyIntersection(point, sun_direction);

						if (!shadowed) { //lighted pixel calculation
							//assume sun is overhead
							if (inter.normal.z < 0.0f)inter.normal = -1.0f*inter.normal;
							float brdf = rho_diff + rho_direct*glm::dot(inter.normal, sun_direction);
							if (inter.spectrum_name.size() > 0) {
								for (int wl = 0; wl < wavelengths_.size(); wl++) {
									float rho = env->GetScene()->GetReflectance(inter.spectrum_name, wavelengths_[wl]);
									pix_color[wl] = brdf * 255.0f*rho;
								}
							}
						}
						else { // shadowed
							if (inter.normal.z < 0.0f)inter.normal = -1.0f*inter.normal;
							float brdf = rho_diff;
							if (inter.spectrum_name.size() > 0) {
								for (int wl = 0; wl < wavelengths_.size(); wl++) {
									float rho = env->GetScene()->GetReflectance(inter.spectrum_name, wavelengths_[wl]);
									pix_color[wl] = brdf * 255.0f*rho;
								}
							}
						} // shadow pixel calculation
						segment_buffer_[n] = inter.object_id;
						range_buffer_[n] = inter.dist;
					}
					for (int c = 0; c < 5; c++)image_(i, j, c) = pix_color[c];
#ifdef USE_MPI
      }
#endif
    }
  }

  ReduceImageBuffer();

	CopyBufferToImage();

	image_.normalize(0, 255);

  if (log_data_) {
    SaveImage(utils::ToString(local_sim_time_)+log_file_name_); 
	}

	env->GetScene()->TurnOffSpectral();

  local_sim_time_ += local_time_step_;
  updated_ = true;
}

void MicaSenseRedEdge::Display() {
	cimg_library::CImg<float> disp_image;
	disp_image.assign(num_horizontal_pix_, num_vertical_pix_, 1, 3, 0.);
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			for (int k = 0; k < 3; k++) {
				disp_image(i, j, k) = image_(i, j, 2-k);
			}
		}
	}
	disp_ = disp_image;
}

void MicaSenseRedEdge::SaveImage(std::string ofname) {
	cimg_library::CImg<float> disp_image;
	disp_image.assign(num_horizontal_pix_, num_vertical_pix_, 1, 3, 0.);
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			for (int k = 0; k < 3; k++) {
				disp_image(i, j, k) = image_(i, j, 2-k);
			}
		}
	}
	disp_image.save(ofname.c_str());
}

void MicaSenseRedEdge::SaveFalseColor(int band1, int band2, int band3, std::string fname) {
	cimg_library::CImg<float> disp_image;
	disp_image.assign(num_horizontal_pix_, num_vertical_pix_, 1, 3, 0.);
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			disp_image(i, j, 0) = image_(i, j, band1);
			disp_image(i, j, 1) = image_(i, j, band2);
			disp_image(i, j, 2) = image_(i, j, band3);			
		}
	}
	disp_image.save(fname.c_str());
}


void MicaSenseRedEdge::SaveBands(std::string fname) {
	cimg_library::CImg<float> band1, band2, band3, band4, band5;
	band1.assign(num_horizontal_pix_, num_vertical_pix_, 1, 1, 0.0f);
	band2.assign(num_horizontal_pix_, num_vertical_pix_, 1, 1, 0.0f);
	band3.assign(num_horizontal_pix_, num_vertical_pix_, 1, 1, 0.0f);
	band4.assign(num_horizontal_pix_, num_vertical_pix_, 1, 1, 0.0f);
	band5.assign(num_horizontal_pix_, num_vertical_pix_, 1, 1, 0.0f);
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			band1(i, j) = image_(i, j, 0);
			band2(i, j) = image_(i, j, 1);
			band3(i, j) = image_(i, j, 2);
			band4(i, j) = image_(i, j, 3);
			band5(i, j) = image_(i, j, 4);
		}
	}
	band1.save(("band1_" + fname).c_str());
	band2.save(("band2_" + fname).c_str());
	band3.save(("band3_" + fname).c_str());
	band4.save(("band4_" + fname).c_str());
	band5.save(("band5_" + fname).c_str());
}

} //namespace camera
} //namespace sensor
} //namespace mavs
