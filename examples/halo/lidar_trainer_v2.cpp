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
* \file lidar_trainer_v2.cpp
*
* Generates labeled lidar data 
*
* Usage: >./lidar_trainer_v2 (sensors) (disp) (hotstart_num) 
*
* where "sensors" is an optional keyword that specifies which sensor or sensors to run. 
* If not set on command line, defaults to "top"
* Options are:
* "top"   - top lidar sensor only
* "right" - right lidar sensor only
* "left"  - left lidar sensor only
* "front" - right and left lidar sensors
* "all"   - top, left, and right lidar senors.
*
* "disp" is the optional display parameter. 
* if greater than 1, a scene rendering will be shown
*
* "hotstart_num" is an optional parameter that will restart the batch sim
* at a given frame. 
*
* \author Chris Goodin
*
* \date 2/15/2019
*/
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>

#include <glm/gtx/rotate_vector.hpp>

#include <mavs_core/terrain_generator/random_scene.h>
#include <mavs_core/data_path.h>
#include <mavs_core/pose_readers/anvel_vprp_reader.h>
#include <mavs_core/math/utils.h>

#include <sensors/mavs_sensors.h>
#include <sensors/lidar/lidar_tools.h>
//#include <sensors/camera/simple_camera.h>
#include <sensors/camera/nir_camera.h>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif 

#ifdef USE_OMP
#include <omp.h>
#endif

int main(int argc, char *argv[]) {
#ifdef USE_EMBREE
  int hotstart_num = 1;
	bool display = false;
	int disp_int = 0;
	if (argc > 2)disp_int = atoi(argv[2]);
	if (disp_int > 0)display = true;
  if (argc>3)hotstart_num = atoi(argv[3]);
	std::string sens_to_run = "top";
	if (argc > 1) {
		std::string to_run(argv[1]);
		sens_to_run = to_run;
	}
	bool run_top = false;
	bool run_left = false;
	bool run_right = false;
	if (sens_to_run == "top") {
		run_top = true;
	}
	else if (sens_to_run == "left") {
		run_left = true;
	}
	else if (sens_to_run == "right") {
		run_right = true;
	}
	else if (sens_to_run == "front") {
		run_left = true;
		run_right = true;
	}
	else if (sens_to_run == "all") {
		run_left = true;
		run_right = true;
		run_top = true;
	}
	else {
		std::cerr << "ERROR: Sensors option not recognized!" << std::endl;
		std::cerr << "Must be \"all\", \"top\", \"left\", \"right\", or \"front\"" << std::endl;
		return 1;
	}

	mavs::sensor::lidar::LidarTools pc_analyzer;
	//if (display)pc_analyzer.DisplayLidarImage();
	mavs::MavsDataPath mavs_data_path;
	std::string eco_path = mavs_data_path.GetPath() + "/ecosystem_files/";

	std::ofstream logfile("logfile.txt",std::ofstream::app);
	logfile << "image_num pose_num terrain_roughness rotation_angle" <<std::endl;
	int image_num = 0;

	mavs::sensor::lidar::MEight top_lidar, left_lidar, right_lidar;
	left_lidar.SetName("LeftLidar");
	right_lidar.SetName("RightLidar");
	top_lidar.SetName("TopLidar");
	float angle = 25.0f;  //65.0;
	float to_rot = (float)(mavs::kDegToRad*angle);
	float fix_angle = (float)(mavs::kDegToRad*34.7);
	glm::vec3 z_hat(0.0, 0.0, 1.0);
	glm::vec3 left_pos(2.087, 0.779, 0.189);
	glm::vec3 right_pos(2.087, -0.779, 0.189);
	glm::vec3 left_y(sin(-fix_angle), cos(-fix_angle), 0.0);
	glm::vec3 right_y(sin(fix_angle), cos(fix_angle), 0.0);
	glm::quat left_ang = glm::angleAxis(to_rot, left_y);
	glm::quat right_ang = glm::angleAxis(to_rot, right_y);
	left_lidar.SetRelativePose(left_pos, left_ang);
	right_lidar.SetRelativePose(right_pos, right_ang);
	glm::dvec3 top_pos(-2.18756, 0.0, 1.830);
	glm::dquat top_ang(1.0, 0.0, 0.0, 0.0);
	top_lidar.SetRelativePose(top_pos, top_ang);

	float min_roughness = 0.0f;
	float max_roughness = 12.0f;
	//float roughness_step = 2.0f;
	float roughness_step = 0.5f;
	float terrain_roughness = max_roughness; // min_roughness;
	//while (terrain_roughness <= max_roughness) {
	while (terrain_roughness >= min_roughness) {
		std::vector<mavs::terraingen::RandomSceneInputs> inputs;
		mavs::terraingen::RandomSceneInputs forest_input;
		//mavs::terraingen::RandomSceneInputs desert_input;
		//mavs::terraingen::RandomSceneInputs meadow_input;
		forest_input.terrain_width = 100.0f;
		forest_input.terrain_length = 100.0f;
		forest_input.lo_mag = terrain_roughness;
		forest_input.hi_mag = 0.0f;
		forest_input.mesh_resolution = 1.0f;
		//forest_input.trail_width = 2.0f;
		forest_input.trail_width = 4.0f;
		forest_input.wheelbase = 1.25f;
		forest_input.track_width = 0.6f;
		forest_input.path_type = "Loop"; 
		forest_input.basename = "forest";
		//forest_input.plant_density = mavs::math::rand_in_range(0.1f, 0.6f);
		//forest_input.plant_density = mavs::math::rand_in_range(0.4f, 0.85f);
		forest_input.plant_density = mavs::math::rand_in_range(0.95f, 2.0f);
		forest_input.output_directory = ".";
		//forest_input.eco_file = eco_path + "american_southeast_forest.json";
		forest_input.eco_file = eco_path + "american_pine_forest.json";
		//meadow
		/*meadow_input = forest_input;
		meadow_input.basename = "meadow";
		meadow_input.eco_file = eco_path + "american_southeast_meadow.json";
		//desert
		desert_input = forest_input;
		desert_input.basename = "desert";
		desert_input.eco_file = eco_path + "american_southwest_desert.json";*/
		//add inputs to list
		//inputs.push_back(meadow_input);
		inputs.push_back(forest_input);
		//inputs.push_back(desert_input);

		double dt = 0.1;
		for (int si = 0; si < (int)inputs.size(); si++) {
			// Create a scene
			mavs::terraingen::RandomScene random_scene;
			random_scene.SetInputs(inputs[si]);
			random_scene.Create();

			std::string scene_file(inputs[si].basename);
			scene_file = scene_file + "_scene.json";

			std::string path_file(inputs[si].basename);
			path_file = path_file + "_path.vprp";

			mavs::raytracer::embree::EmbreeTracer scene;
			scene.Load(scene_file);
			scene.TurnOnLabeling();
			//create the pose structure and load
			mavs::AnvelVprpReader pose_reader;
			std::vector<mavs::Pose> poses = pose_reader.Load(path_file, 2);
			mavs::environment::Environment env;
			env.SetTurbidity(4.0);
			env.SetDateTime(2000, 6, 1, 12, 0, 0, 6);
			env.SetRaytracer(&scene);
			env.SetCloudCover(0.2f);

			//create the cameras
			//glm::vec3 cam_offset(0.0f, 0.0f, 0.75f);
			//glm::quat cam_orient(1.0f, 0.0f, 0.0f, 0.0f);
			//mavs::sensor::camera::NirCamera lowres_camera;
			mavs::sensor::camera::RccbCamera lowres_camera;
			lowres_camera.Initialize(224, 224, 0.0035f, 0.0035f, 0.0035f);
			lowres_camera.SetGamma(0.5f);
			lowres_camera.SetAntiAliasing("simple");
			lowres_camera.SetPixelSampleFactor(1);
			lowres_camera.SetRelativePose(top_pos, top_ang);
			for (int i = 0; i < (int)poses.size(); i++) {
				std::cout << "." << std::flush;
				image_num++;
				if (image_num >= hotstart_num) {
					logfile << image_num << " " << i << " " << terrain_roughness <<std::endl;
					std::string outbase = mavs::utils::ToString(image_num, 6);
					//camera takes about 1/10 the time of each lidar
					if (display) {
						lowres_camera.SetPose(poses[i].position, poses[i].quaternion);
						lowres_camera.Update(&env, dt);
						std::string cam_labels = outbase;
						std::string cam_csv = outbase;
						std::string cam_raw = outbase;
						cam_labels.append("_camera_segment.bmp");
						cam_csv.append("_camera_labels.csv");
						cam_raw.append("_camera_raw.bmp");
						lowres_camera.AnnotateFrame(&env, true);
						lowres_camera.SaveSegmentedImage(cam_labels);
						lowres_camera.SaveSemanticAnnotationsCsv(cam_csv);
						lowres_camera.SaveImage(cam_raw);
						lowres_camera.Display();
					}

					if (run_left) {
						left_lidar.SetPose(poses[i].position, poses[i].quaternion);
						left_lidar.Update(&env, dt);
						left_lidar.AnnotateFrame(&env, true);
						std::string left_out = outbase;
						left_out.append("_left");
						std::vector<mavs::sensor::lidar::labeled_point> points = left_lidar.GetLabeledPoints();
						pc_analyzer.AnalyzeCloud(points, left_out, image_num, (mavs::sensor::lidar::Lidar *)&left_lidar);
					}
					if (run_right) {
						right_lidar.SetPose(poses[i].position, poses[i].quaternion);
						right_lidar.Update(&env, dt);
						right_lidar.AnnotateFrame(&env, true);
						std::string right_out = outbase;
						right_out.append("_right");
						std::vector<mavs::sensor::lidar::labeled_point> points = right_lidar.GetLabeledPoints();
						pc_analyzer.AnalyzeCloud(points, right_out, image_num, (mavs::sensor::lidar::Lidar *)&right_lidar);
					}
					if(run_top) {
						top_lidar.SetPose(poses[i].position, poses[i].quaternion);
						top_lidar.Update(&env, dt);
						if (display) {
							top_lidar.Display();
						}
						top_lidar.AnnotateFrame(&env, true);
						std::string top_out = outbase;
						top_out.append("_top");
						std::vector<mavs::sensor::lidar::labeled_point> points = top_lidar.GetLabeledPoints();
						pc_analyzer.AnalyzeCloud(points, top_out, image_num, (mavs::sensor::lidar::Lidar *)&top_lidar);
					}

					//if (display) {
						
					//}
				} //if imagnum>= hotstart
			} //loop over poses
		} //loop over scenes
		//terrain_roughness += roughness_step;
		terrain_roughness -= roughness_step;
	} //loop over roughnesses

	logfile.close();
#endif
	return 0;
}

