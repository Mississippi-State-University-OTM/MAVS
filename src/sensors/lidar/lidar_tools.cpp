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
#include <sensors/lidar/lidar_tools.h>

#include <fstream>
#include <iostream>
#include <algorithm>
#include <mavs_core/math/utils.h>
#include <sensors/annotation_colors.h>
#include <mavs_core/math/constants.h>
#include <glm/gtx/quaternion.hpp>
#include <cnpy.h>


namespace mavs {
namespace sensor {
namespace lidar {

LidarTools::LidarTools() {
	v_res_ = 1.0f; 
	h_res_ = 1.0f; 
	v_fov_ = glm::vec2(-19.0f, 10.0f);
	h_fov_ = glm::vec2(-180.0f, 180.0f);
	val_ = "reflectance";
	display_ = false;
}

void LidarTools::AnalyzeCloud(std::vector<mavs::sensor::lidar::labeled_point> &points, std::string name, int num, Lidar *lidar) {
	name.append(mavs::utils::ToString(num, 6));
	std::string cname = name;
	std::string pcd_name = name;
	cname.append(".txt");
	pcd_name.append(".pcd");
	std::string npy_name = name;
	//mavs::sensor::lidar::LidarTools pc_analyzer;
	//pc_analyzer.SetColorBy("label");
	//pc_analyzer.WriteLabeledCloudToText(points, cname);
	//pc_analyzer.WriteCloudToPcd(points, pcd_name);
	//pc_analyzer.ProjectAndSave(points, npy_name);
	SetColorBy("label");
	WriteLabeledCloudToText(points, cname);
	//WriteCloudToPcd(points, pcd_name);
	ProjectAndSave(points, npy_name, lidar);
}

void LidarTools::WriteLabeledCloudToText(std::vector<labeled_point> &cloud, std::string fname) {
	std::ofstream fout(fname.c_str());
	for (int i = 0; i < (int)cloud.size(); i++) {
		fout << cloud[i].x << " " << cloud[i].y << " " << cloud[i].z << " " << cloud[i].intensity << " " << cloud[i].label << std::endl;
	}
	fout.close();
}

void LidarTools::ProjectAndSave(std::vector<labeled_point> &cloud, std::string fname, Lidar *lidar) {
	// Calculate the distances of the point clouds
	std::vector<float> d_lidar;
	d_lidar.resize(cloud.size());
	for (int i = 0; i < (int)cloud.size(); i++) {
		d_lidar[i] = sqrt(cloud[i].x*cloud[i].x + cloud[i].y*cloud[i].y + cloud[i].z*cloud[i].z);
	}

	//Get geometry of Lidar
	int vert_channels = lidar->GetNumVerticalChannels();
	const unsigned int azimuth_levels = 512;
	const unsigned int zenith_levels = (unsigned int)vert_channels;
	float h_res_rad = (float)mavs::k2Pi / (float)azimuth_levels; 
	float vert_min = lidar->GetVerticalFovMin();
	float v_range = lidar->GetVerticalFovMax() - vert_min;
	float v_res_rad = v_range / (float)zenith_levels;

	//put the points into theta/phi spherical coordinates
	std::vector<float> x_img, y_img;
	x_img.resize(cloud.size());
	y_img.resize(cloud.size());
	for (int i = 0; i < (int)cloud.size(); i++) {
		float r = sqrt(cloud[i].x*cloud[i].x + cloud[i].y*cloud[i].y);
		float x = atan2(cloud[i].y, cloud[i].x); 
		float y = atan2(cloud[i].z, d_lidar[i]); 
		x_img[i] = (x + (float)mavs::kPi) / h_res_rad;
		y_img[i] = (y - vert_min) / v_res_rad;
	}

	std::vector< std::vector < float > > padded_points_with_range;
	for (int i = 0; i < (int)cloud.size(); i++) {
		std::vector < float > point;
		point.resize(6, 0.0f);
		point[0] = cloud[i].x;
		point[1] = cloud[i].y;
		point[2] = cloud[i].z;
		point[3] = (float)ceil(125.0*cloud[i].intensity); // std::min(100.0f, (float)ceil(1000.0f*cloud[i].intensity));
		point[4] = d_lidar[i];
		//std::cout << i << " " << cloud[i].label << std::endl;
		point[5] = 1.0f*cloud[i].label;
		padded_points_with_range.push_back(point);
	}
	AnnotationColors label_colors;
	cimg_library::CImg<float> image;
	if (display_) image.assign(azimuth_levels, zenith_levels, 1, 3, 0.0);
	
	//create the array for holding the sphere projection
	std::vector<float> sphere_projection;
	sphere_projection.resize(azimuth_levels * zenith_levels * 6, 0.0f);
	for (int ndx = 0; ndx < (int)x_img.size(); ndx++) {
		unsigned int yp = zenith_levels-(unsigned int)(y_img[ndx])-1;
		if (yp < zenith_levels && yp>=0) {
			unsigned int xp = (unsigned int)x_img[ndx];
			if (display_) {
				glm::vec3 col = label_colors.GetColor((int)padded_points_with_range[ndx][5]);
				image.draw_point(xp, yp, (float*)&col);
			}
			if (xp >= 0 && xp < azimuth_levels) {
				for (int k = 0; k < 6; k++) {
					//int n = k + 6 * yp + 6 * zenith_levels*xp;	
					int n = k + 6 * xp + 6 * 512*yp;
					sphere_projection[n] = padded_points_with_range[ndx][k];
				}
			}
		}
	}


	image.resize(1024, 256);
	if (display_) lidar_tools_display_ = image;
	//std::string img_name = fname;
	//img_name.append(".bmp");
	//image.save(img_name.c_str());

	//save sphere projection to a .npy file
	std::string npy_name = fname;
	npy_name.append("_");
	npy_name.append(mavs::utils::ToString(zenith_levels));
	npy_name.append(".npy");
	cnpy::npy_save(npy_name.c_str(), &sphere_projection[0], { zenith_levels,azimuth_levels,6 }, "w");
	//cnpy::npy_save(npy_name.c_str(), &sphere_projection[0], { azimuth_levels,zenith_levels,6 }, "w");

	//2/5/2019 - image not needed
	/*
	//# SHIFT COORDINATES TO MAKE 0, 0 THE MINIMUM
	float x_min = -360.0f / h_res_ / 2.0f;   //# Theoretical min x value based on sensor specs
	for (int i = 0; i < (int)x_img.size(); i++) {
	x_img[i] -= x_min;              //# Shift NOTE : this array holds our azimuth values
	}
	float x_max = 360.0f / h_res_ ;       //# Theoretical max x value after shifting
	//float x_max = 360.0 / h_res_/2.0f;

	float y_min = -180.0f / v_res_ / 2.0f;  //# Theoretical min x value based on sensor specs
	for (int i = 0; i < (int)y_img.size(); i++) {
	//y_img[i] -= y_min;              //# Shift NOTE : this array holds our azimuth values
	y_img[i] += y_min;              //# Shift NOTE : this array holds our azimuth values
	}
	float y_max = 180.0f / v_res_;      //# Theoretical max x value after shifting
	//float y_max = v_fov_[1] / v_res_; // / 2.0f;
	//# y_min = v_fov[0] / v_res    # theoretical min y value based on sensor specs
	//# y_img -= y_min              # Shift NOTE : this array holds our zenith values
	//# y_max = v_fov_total / v_res # Theoretical max x value after shifting
	//# y_max += y_fudge            # Fudge factor if the calculations based on
	//# spec sheet do not match the range of
	//# angles collected by in the data.

	std::vector<float> pixel_values;
	pixel_values.resize(cloud.size());
	if (val_ == "reflectance") {
	for (int i = 0; i < (int)cloud.size(); i++) {
	pixel_values[i] = cloud[i].intensity;
	}
	}
	else if (val_ == "height") {
	for (int i = 0; i < (int)cloud.size(); i++) {
	pixel_values[i] = cloud[i].z;
	}
	}
	else if (val_ == "label") {
	for (int i = 0; i < (int)cloud.size(); i++) {
	pixel_values[i] = (float)cloud[i].label;
	}
	}
	else {  //if (val == "depth") {
	for (int i = 0; i < (int)cloud.size(); i++) {
	pixel_values[i] = -d_lidar[i];
	}
	}
	*/
	/*
	//float dpi = 100.0f;
	//float pix_angle_res = 0.25;
	//int nx = (int)(360.0f / pix_angle_res);
	//int ny = (int)(180.0f / pix_angle_res);
	int nx = (int)(x_max); // / dpi);
	int ny = (int)(y_max); // / dpi);
	cimg_library::CImg<float> image;
	image.assign(nx, ny, 1, 3, 0.0);
	for (int i = 0; i < (int)cloud.size(); i++) {
		//int px = (int)floor(x_img[i]/pix_angle_res);
		//int py = ny-(int)floor(y_img[i]/pix_angle_res)-1;
		int px = (int)floor(x_img[i] );
		int py = ny - (int)floor(y_img[i] ) - 1;
		//int py = (int)floor(y_img[i]);
		if (px >= 0 && py >= 0 && px < nx && py < ny) {
			glm::vec3 color(0.0f, 0.0f, 0.0f);
			if (val_ == "label") {
				color = cloud[i].color;
			}
			else {
				color = glm::vec3(pixel_values[i], pixel_values[i], pixel_values[i]);
				color = 255.0f * color;
			}
			image.draw_point(px, py, (float*)(&color));
		}
	}
	//image.resize(768, 384);
	//image.dilate(3);

	std::string img_name = fname;
	img_name.append(".bmp");
	image.save(img_name.c_str());
	*/
}

void LidarTools::WriteCloudToText(std::vector<glm::vec4> &cloud, std::string fname) {
	std::ofstream fout(fname.c_str());
	for (int i = 0; i < (int)cloud.size(); i++) {
		//if (cloud[i].w > 0.0f) {
		fout << cloud[i].x << " " << cloud[i].y << " " << cloud[i].z << " " << cloud[i].w << std::endl;
		//}
	}
	fout.close();
}

void LidarTools::WriteCloudToPcd(std::vector<glm::vec4> &cloud, std::string fname) {
	glm::quat orient(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 pos(0.0f, 0.0f, 0.0f);
	WriteCloudToPcd(cloud, pos, orient, fname);
}

void LidarTools::WriteCloudToPcd(std::vector<labeled_point> &cloud_in, std::string fname) {
	glm::quat orient(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 pos(0.0f, 0.0f, 0.0f);
	std::vector <glm::vec4> cloud;
	cloud.resize(cloud_in.size());
	for (int i = 0; i < (int)cloud_in.size(); i++) {
		cloud[i].x = cloud_in[i].x;
		cloud[i].y = cloud_in[i].y;
		cloud[i].z = cloud_in[i].z;
		cloud[i].w = cloud_in[i].intensity;
	}
	WriteCloudToPcd(cloud, pos, orient, fname);
}

void LidarTools::WriteCloudToPcd(std::vector<glm::vec4> &cloud, glm::vec3 position, glm::quat orientation, std::string fname) {
	int np = (int)cloud.size();
	std::ofstream fout(fname.c_str());
	fout << "VERSION 0.7" << std::endl;
	fout << "FIELDS x y z intensity" << std::endl;
	fout << "SIZE 4 4 4 4" << std::endl;
	fout << "TYPE F F F F" << std::endl;
	fout << "COUNT 1 1 1 1" << std::endl;
	fout << "WIDTH " << np << std::endl;
	fout << "HEIGHT " << 1 << std::endl;
	glm::quat q = orientation;
	fout << "VIEWPOINT " << position.x << " " << position.y << " " << position.z << " " <<
		q.w << " " << q.x << " " << q.y << " " << q.z << std::endl;
	fout << "POINTS " << np << std::endl;
	fout << "DATA ascii" << std::endl;
	for (int i = 0; i<(int)cloud.size(); i++) {
		fout << cloud[i].x << " " << cloud[i].y << " " <<
			cloud[i].z << " " << cloud[i].w << std::endl;
	}
	fout.close();
}

std::vector<labeled_point> LidarTools::AppendClouds(std::vector<labeled_point> &cloud1, std::vector<labeled_point> &cloud2) {
	std::vector<labeled_point> merged_points = cloud1;
	merged_points.insert(merged_points.end(), cloud2.begin(), cloud2.end());
	return merged_points;
}

std::vector<labeled_point> LidarTools::TransformCloud(std::vector<labeled_point> &cloud, glm::vec3 position, glm::quat orientation) {
	std::vector<labeled_point> trans_cloud;
	trans_cloud.resize(cloud.size());
	for (int i = 0; i < (int)cloud.size(); i++) {
		trans_cloud[i].x = cloud[i].x - position.x;
		trans_cloud[i].y = cloud[i].y - position.y;
		trans_cloud[i].z = cloud[i].z - position.z;
		trans_cloud[i].intensity = cloud[i].intensity;
		trans_cloud[i].label = cloud[i].label;
	}
	
	glm::mat3 R = glm::toMat3(orientation);
	glm::mat3 Ri = glm::inverse(R);
	for (int i = 0; i < (int)cloud.size(); i++) {
		glm::vec3 p(trans_cloud[i].x, trans_cloud[i].y, trans_cloud[i].z);
		glm::vec3 p_prime = Ri * p;
		trans_cloud[i].x = p_prime.x;
		trans_cloud[i].y = p_prime.y;
		trans_cloud[i].z = p_prime.z;
	}
	
	return trans_cloud;
}

} //namespace lidar
} //namespace sensor
} //namespace mavs