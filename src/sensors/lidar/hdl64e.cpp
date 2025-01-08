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
#include <sensors/lidar/hdl64e.h>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

Hdl64E::Hdl64E(){
  lower_block_.SetBeamSpotRectangular(0.0033f, 0.0007f);
  upper_block_.SetBeamSpotRectangular(0.0033f, 0.0007f);
  max_range_ = 90.0f; //100.0;
  min_range_ = 1.0f;
  is_planar_ = false;
  comm_set_ = false;
}

void Hdl64E::SetRotationRate(float rot_hz){
  rot_hz = math::clamp(rot_hz,5.0f,15.0f);
	rotation_rate_ = (float)rot_hz;
  double dt = 1.0/rot_hz;
  SetTimeStep(dt);
  lower_block_.SetTimeStep(dt);
  upper_block_.SetTimeStep(dt);
  int n_lower = (int)((250000.0/rot_hz)/32.0);
  int n_upper = 4*n_lower;
  float lower_res = 360.0f/(1.0f*n_lower-1.0f);
  float upper_res = 360.0f/(1.0f*n_upper-1.0f);
  lower_block_.SetScanSize(-180.0f,180.0f-lower_res,n_lower, -24.8f, -11.6127f, 32);
  upper_block_.SetScanSize(-180.0f,180.0f-upper_res,n_upper, -11.1873f, 2.0f, 32);
}

void Hdl64E::Update(environment::Environment *env, double dt) {
#ifdef USE_MPI  
	if (!comm_set_) {
		upper_block_.SetComm(comm_);
		lower_block_.SetComm(comm_);
	}
#endif

	lower_block_.SetPose(position_, glm::quat_cast(orientation_));
	upper_block_.SetPose(position_, glm::quat_cast(orientation_));
	lower_block_.Update(env, dt);
	upper_block_.Update(env, dt);
	points_.clear();
	point_colors_.clear();
	distances_.clear();
	intensities_.clear();
	registered_points_.clear();
	segment_points_.clear();
	//buffer the points in sets of 4 sets of 32
	// 3 from the upper block, 1 from the lower block
	int n_low = 0;
	for (int ii = 0; ii < (int)upper_block_.GetNumPoints() / 32; ii++) {
		for (int j = 0; j < 32; j++) {
			int i = ii * 32 + j;
			if (ii % 4 == 3) {
				points_.push_back(lower_block_.GetPoint(n_low));
				intensities_.push_back(lower_block_.GetIntensity(n_low));
				distances_.push_back(lower_block_.GetDistance(n_low));
				registered_points_.push_back(lower_block_.GetRegisteredPoint(n_low));
				segment_points_.push_back(lower_block_.GetSegmentPoint(n_low));
				point_colors_.push_back(lower_block_.GetPointColor(n_low));
				n_low++;
			}
			else {
				points_.push_back(upper_block_.GetPoint(i));
				intensities_.push_back(upper_block_.GetIntensity(i));
				distances_.push_back(upper_block_.GetDistance(i));
				registered_points_.push_back(upper_block_.GetRegisteredPoint(i));
				segment_points_.push_back(upper_block_.GetSegmentPoint(i));
				point_colors_.push_back(upper_block_.GetPointColor(i));
			}
		}
	}

	if (log_data_) {
		WritePointsToText(utils::ToString(local_sim_time_) + log_file_name_);
	}
	local_sim_time_ += local_time_step_;
	updated_ = true;
	nsteps_++;
}

/*std::vector<glm::vec4> Hdl64E::GetRegisteredPointsXYZI() {
	std::glm::vec4 points;
	u
}*/

} //namespace lidar
} //namespace sensor
} //namespace mavs
