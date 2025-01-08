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
#include <mavs_core/pose_readers/trail.h>
#include <mavs_core/math/utils.h>
#include <iostream>
namespace mavs {

Trail::Trail() {
	track_width_ = 0.6f;
	wheelbase_ = 2.5f;
	trail_width_ = 5.0f;

	// derived parameters
	float half_wb = 0.5f*wheelbase_;
	float half_tw = 0.5f*track_width_;
	tw_lo_ = half_wb - half_tw;
	tw_hi_ = half_wb + half_tw;
	half_trail_width_ = 0.5f*trail_width_;
}

void Trail::AddPose(glm::vec3 position, glm::quat orientation) {
	mavs::Pose pose;
	pose.position = position;
	pose.quaternion = orientation;
	poses_.push_back(pose);
}

void Trail::SaveToAnvelVprp(std::string fname) {
	std::ofstream fout(fname.c_str());
	fout << "QS VANE REPLAY FILE" << std::endl;
	fout << "Version : 1" << std::endl;
	fout << "Replay Type : PositionReplay" << std::endl;
	fout << "Vehicle File : Generic4x4.vehicle.xml" << std::endl;
	fout << "Vehicle Objects : 0" << std::endl;
	fout << "#Start Date : 01 / 01 / 2000" << std::endl;
	fout << "#Start Time : 08 : 00 : 00" << std::endl;
	fout<<"@Data"<<std::endl;
	float dt = 0.01f;
	if (poses_.size() > 0) {
		for (int i = 0; i < (int)poses_.size(); i++) {
			fout << dt * (i+1) << ":" << poses_[i].position.x << " " << poses_[i].position.y << " " << poses_[i].position.z<<":"<<
				poses_[i].quaternion.w << " "<<poses_[i].quaternion.x<<" "<<poses_[i].quaternion.y<<" "<<poses_[i].quaternion.z << std::endl;
		}
	}
	else {
		for (int i = 1; i < (int)path_.NumWaypoints(); i++) {
			glm::vec2 to_vec = path_.GetWaypoint(i) - path_.GetWaypoint(i - 1);
			float theta = 0.5f*atan2(to_vec.y, to_vec.x);
			fout << dt * i << ":" << path_.GetWaypoint(i).x << " " << path_.GetWaypoint(i).y << " 0.0:" << cos(theta) << " 0.0 0.0 " << sin(theta) << std::endl;
		}
	}
	fout << "@END OF REPLAY FILE" << std::endl;
	fout.close();
}

void Trail::LoadPath(std::string infile) {
	path_file_ = infile;
	if (!mavs::utils::file_exists(infile)) {
		std::cerr << "ERROR, ANVEL vprp file " << infile << " does not exist" << std::endl;
		exit(2);
	}
	path_.CreateFromAnvelVprp(infile);
	path_.SetMapRes(trail_width_);
}

bool Trail::IsPointInTracks(glm::vec2 point) {
	float dist = path_.DistanceToPath(point);
	if (dist > tw_lo_ && dist < tw_hi_) {
		return true;
	}
	else {
		return false;
	}
}

} // namespace mavs