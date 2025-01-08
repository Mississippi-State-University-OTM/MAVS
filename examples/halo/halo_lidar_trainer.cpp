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
/**
* \file halo_lidar_trainer.cpp
*
* Generates labeled lidar data in a given scene along a given path
*
* Usage: >./halo_lidar_trainer scenefile.json anvel_replay.vprp
*
* where scenefile.json is a mavs scene file, examples in 
* mavs/data/scenes, 
*
* and anvel_replay.vprp is an ANVEL replay file
* in text format, examples in mavs/data/waypoints.
*
* Output is annotated lidar data.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <sensors/mavs_sensors.h>
#include <sensors/camera/rgb_camera.h>
#include <sensors/lidar/lidar_tools.h>
#include <mavs_core/pose_readers/anvel_vprp_reader.h>
#include <mavs_core/math/utils.h>
#include <iostream>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif 


int main(int argc, char *argv[]) {
#ifdef USE_EMBREE
	if (argc<3) {
		std::cerr << "Usage: ./halo_lidar_trainer scenefile.json anvel_replay.vprp" << std::endl;
		return 1;
	}

	// create the scene and load file
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	std::cout << "Loading " << scene_file << std::endl;
	scene.Load(scene_file);

	//create the pose structure and load
	std::string pose_file(argv[2]);
	mavs::AnvelVprpReader pose_reader;
	std::vector<mavs::Pose> poses = pose_reader.Load(pose_file,10);

	//create the environment
	mavs::environment::Environment env;
	env.SetRaytracer(&scene);

	//create a camera for debug purposes
	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);

	//create the lidar sensors and set properties
	float rot_rate = 10.0f;
	float dt = 1.0f / rot_rate;
	mavs::sensor::lidar::MEight left, right, top;
	//mavs::sensor::lidar::Hdl64E left, right, top;
	left.SetNoiseCutoff(0.0);
	right.SetNoiseCutoff(0.0);
	top.SetNoiseCutoff(0.0);
	left.SetRotationRate(rot_rate);
	right.SetRotationRate(rot_rate);
	top.SetRotationRate(rot_rate);
	left.SetDrawAnnotations();
	right.SetDrawAnnotations();
	top.SetDrawAnnotations();
	left.SetName("Left Lidar");
	right.SetName("Right Lidar");
	top.SetName("Top Lidar");

	// Set the relative poses of the lidar sensors
	double angle = 65.0;
	double fix_angle = mavs::kDegToRad*34.7;
	glm::dvec3 x_hat(1, 0, 0);
	glm::dvec3 z_hat(0, 0, 1);
	glm::dvec3 top_pos(-2.18756, 0.0, 1.830);
	glm::dvec3 left_pos(-0.1, 0.779, 0.689);
	glm::dvec3 right_pos(-0.1, -0.779, 0.689);
	glm::dquat top_ang(1.0, 0.0, 0.0, 0.0);
	glm::dquat left_ang = glm::angleAxis(-mavs::kDegToRad*angle, x_hat)*
		glm::angleAxis(-fix_angle, z_hat);
	glm::dquat right_ang = glm::angleAxis(mavs::kDegToRad*angle, x_hat)*
		glm::angleAxis(fix_angle, z_hat);
	top.SetRelativePose(top_pos, top_ang);
	left.SetRelativePose(left_pos, left_ang);
	right.SetRelativePose(right_pos, right_ang);
	
	camera.SetRelativePose(top_pos, top_ang);

	mavs::sensor::lidar::LidarTools pc_analyzer;
	//pc_analyzer.SetColorBy("depth");
	pc_analyzer.SetColorBy("label");
	for (int i = 0; i < (int)poses.size(); i++) {
	//for (int i = 0; i < 1; i++) {
		//update the point cloud for each sensor
		camera.SetPose(poses[i].position, poses[i].quaternion);
		right.SetPose(poses[i].position, poses[i].quaternion);
		left.SetPose(poses[i].position, poses[i].quaternion);
		top.SetPose(poses[i].position, poses[i].quaternion);
		right.Update(&env, dt);
		left.Update(&env, dt);
		top.Update(&env, dt);
		top.AnnotateFrame(&env, true);
		left.AnnotateFrame(&env, true);
		right.AnnotateFrame(&env, true);
		camera.Update(&env, dt);
		camera.Display();
		std::string im_name = "image";
		im_name.append(mavs::utils::ToString(i,4));
		im_name.append(".bmp");
		camera.SaveImage(im_name);

		//merge the point clouds and register to top sensor local frame
		std::vector<mavs::sensor::lidar::labeled_point> registered_points, merged_points, top_points, left_points, right_points, top_registered_labeled;
		top_points = top.GetLabeledPoints(); // in sensor coordinates
		left_points = left.GetRegisteredLabeledPoints();
		right_points = right.GetRegisteredLabeledPoints();
		registered_points = pc_analyzer.AppendClouds(right_points, left_points);
		top_registered_labeled = top.GetRegisteredLabeledPoints();
		registered_points = pc_analyzer.AppendClouds(top_registered_labeled, registered_points);
		mavs::Pose top_pose = top.GetPose();
		left_points = pc_analyzer.TransformCloud(left_points, top_pose.position, top_pose.quaternion);
		right_points = pc_analyzer.TransformCloud(right_points, top_pose.position, top_pose.quaternion);
		merged_points = pc_analyzer.AppendClouds(right_points, left_points);
		merged_points = pc_analyzer.AppendClouds(merged_points, top_points);
		
		// analyze the merged point cloud and save file
		std::string npy_name;
		npy_name = "cloud";
		npy_name.append(mavs::utils::ToString(i,4));
		std::string cname = npy_name;
		std::string pcd_name = npy_name;
		cname.append(".txt");
		pcd_name.append(".pcd");
		pc_analyzer.WriteLabeledCloudToText(merged_points,cname);
		pc_analyzer.WriteCloudToPcd(registered_points, pcd_name);
		pc_analyzer.ProjectAndSave(merged_points,npy_name, (mavs::sensor::lidar::Lidar *)&top);
		std::cout << "Number of merged points = " << merged_points.size()<<std::endl;
	}

#else
	std::cout << "This example only works if the code is built " <<
		"with Embree enabled." << std::endl;
#endif
	return 0;
}

