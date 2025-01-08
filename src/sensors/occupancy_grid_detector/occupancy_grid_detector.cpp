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
#include <sensors/occupancy_grid_detector/occupancy_grid_detector.h>

#include <algorithm>

#include <mavs_core/math/utils.h>

namespace mavs {
namespace sensor {
namespace ogd {

OccupancyGridDetector::OccupancyGridDetector() {
	Initialize(200.0f, 200.0f, 0.5f);
	max_height_ = 1.0;
	first_display_ = true;
}

OccupancyGridDetector::~OccupancyGridDetector() {

}

OccupancyGridDetector::OccupancyGridDetector(float x_size, float y_size, float resolution_meters) {
	Initialize(x_size, y_size, resolution_meters);
	max_height_ = 1.0;
	first_display_ = true;
	n_ignore_ = (int)ceil(3.0 / resolution_);
}


void OccupancyGridDetector::Update(environment::Environment *env, double dt) {
	glm::vec3 dir(0.0f, 0.0f, -1.0f);
	float xl = position_.x - 0.5f*width_;
	float yl = position_.y - 0.5f*height_;
	float h = 1000.0f;
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			grid_[i][j] = 0;
		}
	}
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
	for (int i = 0; i < nx_; i++) {
		float x = xl + resolution_ * i;
		for (int j = 0; j < ny_; j++) {
			if ((i < (int)(0.5*nx_+n_ignore_) && i >= (int)(0.5*nx_ - n_ignore_)) &&
				(j < (int)(0.5*ny_+n_ignore_) && j >= (int)(0.5*ny_ - n_ignore_)) ) {
				grid_[i][j] = 0;
			}
			else {
				float y = yl + resolution_ * j;
				grid_[i][j] = 0;
				float max_cell_height = -h;
				for (int ii=-1;ii<=1;ii++){
					float xx = x + (ii *0.7f*resolution_);
					for (int jj=-1;jj<=1;jj++){
						float yy = y + (jj *0.7f*resolution_);
						glm::vec3 pos(xx, yy, h);
						mavs::raytracer::Intersection inter = env->GetClosestIntersection(pos, dir);
						if (inter.dist > 0.0) {
							float top = h - inter.dist;
							float ground = env->GetGroundHeight(x, y);
							float height = top - ground;
							if (height>max_cell_height){
								float s = std::max(0.0f, std::min(100.0f, 100.0f*height / max_height_));
								grid_[i][j] = (int)s;
								max_cell_height = height;
							}
						}
					}
				}
			}
		}
	}
}

OccupancyGrid OccupancyGridDetector::GetGrid() {
	mavs::OccupancyGrid og;
	og.info.width = nx_;
	og.info.height = ny_;
	
	og.info.resolution = resolution_;
	og.info.origin.position.x = position_.x - 0.5*width_;
	og.info.origin.position.y = position_.y - 0.5*height_;
	og.info.origin.position.z = position_.z;
	glm::quat q(orientation_);
	og.info.origin.quaternion.w = q.w;
	og.info.origin.quaternion.x = q.x;
	og.info.origin.quaternion.y = q.y;
	og.info.origin.quaternion.z = q.z;
	og.data.resize(nx_*ny_, 0);
	int n = 0;
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			og.data[n] = grid_[i][j];
			n++;
		}
	}
	return og;
}

OccupancyGrid OccupancyGridDetector::GetBinaryGrid(float thresh) {
	mavs::OccupancyGrid og;
	og.info.width = nx_;
	og.info.height = ny_;
	
	og.info.resolution = resolution_;
	og.info.origin.position.x = position_.x - 0.5*width_;
	og.info.origin.position.y = position_.y - 0.5*height_;
	og.info.origin.position.z = position_.z;
	glm::quat q(orientation_);
	og.info.origin.quaternion.w = q.w;
	og.info.origin.quaternion.x = q.x;
	og.info.origin.quaternion.y = q.y;
	og.info.origin.quaternion.z = q.z;
	og.data.resize(nx_*ny_, 0);
	int n = 0;
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			if (grid_[i][j]>thresh){
				og.data[n]=100;
			}
			else{
				og.data[n]=0;
			}
			n++;
		}
	}
	return og;
}

void OccupancyGridDetector::Initialize(float x_size, float y_size,
	float resolution_meters) {
	width_ = x_size;
	height_ = y_size;
	resolution_ = resolution_meters;
	nx_ = (int)ceil(width_ / resolution_);
	ny_ = (int)ceil(height_ / resolution_);
	if (nx_%2!=0) nx_ = nx_+1;
	if (ny_%2!=0) ny_ = ny_+1;
	grid_ = mavs::utils::Allocate2DVector(nx_, ny_, 0);
	n_ignore_ = (int)ceil(3.0 / resolution_);
}


void OccupancyGridDetector::Display() {
	if (first_display_) {
		image_.assign(nx_, ny_, 1, 3, 0.0);
		disp_.assign(nx_, ny_, "occupancy_grid");
		first_display_ = false;
	}

	image_ = 0.0f;

	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			float color[3];
			color[0] = 2.55f*grid_[i][j];
			color[1] = color[0];
			color[2] = color[1];
			image_.draw_point(i,j, (float *)&color);
		}
	}
	disp_ = image_;
} //Display

} //namespace ogd
} //namespace sensor
} //namespace mavs