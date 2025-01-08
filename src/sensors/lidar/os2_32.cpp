/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
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
	float vres = 22.5f / 31.0f;
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
	fout << "HEIGHT 32" << std::endl;
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
	fout << "HEIGHT 32" << std::endl;
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
