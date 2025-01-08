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
#include <sensors/lidar/os2.h>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

OusterOS2::OusterOS2(){
	SetBeamSpotCircular((float)(0.09*mavs::kDegToRad));
  SetRotationRate(10.0,1024);
  max_range_ = 250.0;
  min_range_ = 1.0;
  //vertical_readout_ = true;
}

void OusterOS2::SetRotationRate(float rot_hz, int mode){
	if (rot_hz < 10.0)rot_hz = 10.0;
	if (rot_hz > 20.0)rot_hz = 20.0;
	if (rot_hz != 10.0 && rot_hz != 20.0) {
		if (rot_hz < 15.0) {
			rot_hz = 10.0;
		}
		else {
			rot_hz = 20.0;
		}
	}
	if (mode == 2048) rot_hz = 10.0;
	if (mode != 512 && mode != 1024 && mode != 2048) {
		std::cerr << "ERROR in setting mode of OS2 lidar." << std::endl;;
		std::cerr << "Mode must be 512,1024, or 2048" << std::endl;
		exit(10);
	}
	rotation_rate_ = rot_hz;
	float res = 360.0f / (1.0f*mode);
	float vres = 22.5f / 63.0f;
  SetScanPattern(-180.0f,180.0f-res,res,-11.25,11.25,vres);
  SetTimeStep(1.0/rot_hz);
}

void OusterOS2::WriteOs2Pcd(std::string fname) {
	int np = 0;
	float max_intens = 0.0f;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			if (intensities_[i] > max_intens)max_intens = intensities_[i];
			np++;
		}
	}
	std::ofstream fout(fname.c_str());
	fout << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
	fout << "VERSION 0.7" << std::endl;
	fout << "FIELDS x y z intensity t reflectivity ring noise range" << std::endl;
	fout << "SIZE 4 4 4 4 4 2 1 2 4" << std::endl;
	fout << "TYPE F F F F U U U U U" << std::endl;
	fout << "COUNT 1 1 1 1 1 1 1 1 1" << std::endl;
	fout << "WIDTH 2048" << std::endl;
	fout << "HEIGHT 64" << std::endl;
	fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
	fout << "POINTS " << np << std::endl;
	fout << "DATA ascii" << std::endl;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			int intens = (int)(450.0*intensities_[i] / max_intens);
			float r = glm::length(points_[i]);
			int refl = (int)(0.0979f*r*r*intens);
			double zenith = asin(points_[i].z / r);
			int ring = (int)(32.783 - 109.4*zenith);
			double azimuth = atan2(points_[i].y, points_[i].z);
			int t = (int)(1000.0*(1589.4*azimuth + 7501.8));
			int range = (int)(1000.0*r);
			int noise = (int)(0.01*range);
			fout << points_[i].x << " " << points_[i].y << " " << points_[i].z << " " <<
				intens << " " << t << " " << refl << " " << ring << " " << noise << " " << range << std::endl;
		}
	}
	fout.close();
}

void OusterOS2::WriteOs2LabeledPcd(std::string fname) {
	int np = 0;
	float max_intens = 0.0f;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			if (intensities_[i] > max_intens)max_intens = intensities_[i];
			np++;
		}
	}
	std::ofstream fout(fname.c_str());
	fout << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
	fout << "VERSION 0.7" << std::endl;
	fout << "FIELDS x y z intensity t reflectivity ring noise range label" << std::endl;
	fout << "SIZE 4 4 4 4 4 2 1 2 4 1" << std::endl;
	fout << "TYPE F F F F U U U U U U" << std::endl;
	fout << "COUNT 1 1 1 1 1 1 1 1 1 1" << std::endl;
	fout << "WIDTH 2048" << std::endl;
	fout << "HEIGHT 64" << std::endl;
	fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
	fout << "POINTS " << np << std::endl;
	fout << "DATA ascii" << std::endl;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			int intens = (int)(450.0*intensities_[i]/max_intens);
			float r = glm::length(points_[i]);
			int refl = (int)(0.0979f*r*r*intens);
			double zenith = asin(points_[i].z / r);
			int ring = (int)(32.783-109.4*zenith);
			double azimuth = atan2(points_[i].y, points_[i].z);
			int t = (int)(1000.0*(1589.4*azimuth + 7501.8));
			int range = (int)(1000.0*r);
			int noise = (int)(0.01*range);
			fout << points_[i].x << " " << points_[i].y << " " << points_[i].z << " " << 
				intens << " " << t << " " << refl << " " << ring << " " << noise << " " << range << " " << segment_points_[i] << std::endl;
		}
	}
	fout.close();
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
