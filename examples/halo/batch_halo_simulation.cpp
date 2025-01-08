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
* \file batch_halo_simulation.cpp
*
* Demonstrates a MAVS batch simulation for generating labeled training data
*
* Usage: >./batch_halo_simulation (hotstart_num) 
*
* where hotstart_num is an optional parameter that will restart the batch sim
* at a given frame. 
*
* There are 10,725 frames of output, with each frame saving 
* multiple image, lidar, and annotation files.
*
* \author Chris Goodin
*
* \date 10/4/2018
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
#include <sensors/camera/rgb_camera.h>
#include <sensors/camera/spherical_camera.h>
#include <sensors/lidar/lidar_tools.h>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif 

int main(int argc, char *argv[]) {
#ifdef USE_EMBREE
  int hotstart_num = 1;
  if (argc>1)hotstart_num = atoi(argv[1]);

	mavs::MavsDataPath mavs_data_path;
	std::string eco_path = mavs_data_path.GetPath() + "/ecosystem_files/";

	std::ofstream logfile("logfile.txt",std::ofstream::app);
	logfile << "image_num pose_num hour turbidity terrain_roughness" 
		<<std::endl;

	mavs::sensor::lidar::LidarTools pc_analyzer;
	mavs::sensor::lidar::MEight top_lidar, left_lidar, right_lidar;
	glm::vec3 top_offset(0.0f, 0.0f, 1.33f);
	glm::quat top_orient(1.0f, 0.0f, 0.0f, 0.0f);
	top_lidar.SetRelativePose(top_offset, top_orient);
	float angle = 25.0f;  //65.0;
	float to_rot = (float)(mavs::kDegToRad*angle);
	float fix_angle = (float)(mavs::kDegToRad*34.7);
	glm::vec3 y_hat(0.0, 1.0, 0.0);
	glm::vec3 z_hat(0.0, 0.0, 1.0);
	glm::vec3 left_pos(2.087, 0.779, 0.189);
	glm::vec3 right_pos(2.087, -0.779, 0.189);
	glm::vec3 left_y(sin(-fix_angle),cos(-fix_angle),0.0); 
	glm::vec3 right_y(sin(fix_angle), cos(fix_angle), 0.0); 
	glm::quat left_ang = glm::angleAxis(to_rot, left_y)*glm::angleAxis(-fix_angle, z_hat);
	glm::quat right_ang = glm::angleAxis(to_rot, right_y)*glm::angleAxis(fix_angle, z_hat);
	left_lidar.SetRelativePose(left_pos, left_ang);
	right_lidar.SetRelativePose(right_pos, right_ang);

	float min_roughness = 0.0;
	float min_turbidity = 2.0;
	int min_time = 9;

	int image_num = 0;
	float terrain_roughness = min_roughness;
	while (terrain_roughness <= 20.0) {

		std::vector<mavs::terraingen::RandomSceneInputs> inputs;
		mavs::terraingen::RandomSceneInputs forest_input;
		mavs::terraingen::RandomSceneInputs desert_input;
		mavs::terraingen::RandomSceneInputs meadow_input;
		forest_input.terrain_width = 50.0f;
		forest_input.terrain_length = 50.0f;
		forest_input.lo_mag = terrain_roughness;
		forest_input.hi_mag = 0.0f;
		forest_input.mesh_resolution = 1.0f;
		forest_input.trail_width = 2.0f;
		forest_input.wheelbase = 1.25f;
		forest_input.track_width = 0.6f;
		forest_input.path_type = "Valleys";
		forest_input.basename = "forest";
		forest_input.plant_density = 1.0f;
		forest_input.output_directory = ".";
		forest_input.eco_file = eco_path + "american_southeast_forest.json";
		//meadow
		meadow_input = forest_input;
		meadow_input.basename = "meadow";
		meadow_input.eco_file = eco_path + "american_southeast_meadow.json";
		//desert
		desert_input = forest_input;
		desert_input.basename = "desert";
		desert_input.eco_file = eco_path + "american_southwest_desert.json";
		//add inputs to list
		inputs.push_back(meadow_input);
		inputs.push_back(forest_input);
		inputs.push_back(desert_input);


		double dt = 0.5;
		for (int i = 0; i < (int)inputs.size(); i++) {
			// Create a scene
			mavs::terraingen::RandomScene random_scene;
			random_scene.SetInputs(inputs[i]);
			random_scene.Create();

			std::string scene_file(inputs[i].basename);
			scene_file = scene_file + "_scene.json";

			std::string path_file(inputs[i].basename);
			path_file = path_file + "_path.vprp";

			mavs::raytracer::embree::EmbreeTracer scene;
			scene.Load(scene_file);

			//create the pose structure and load
			mavs::AnvelVprpReader pose_reader;
			std::vector<mavs::Pose> poses = pose_reader.Load(path_file, 5);
			
			int hour = min_time;
			while (hour <= 17) {
				double turbidity = min_turbidity;
				while (turbidity <= 10.0) {
					//create the environment
					mavs::environment::Environment env;
					env.SetTurbidity(turbidity);
					env.SetDateTime(2000, 6, 1, hour, 0, 0, 6);
					env.SetRaytracer(&scene);

					//create the cameras
					glm::vec3 cam_offset(0.0f, 0.0f, 0.75f);
					glm::quat cam_orient(1.0f, 0.0f, 0.0f, 0.0f);
					mavs::sensor::camera::RgbCamera lowres_camera;
					lowres_camera.Initialize(224, 224, 0.0035f, 0.0035f, 0.0035f);
					lowres_camera.SetEnvironmentProperties(&env);
					lowres_camera.SetGamma(0.5f);
					lowres_camera.SetAntiAliasing("oversampled");
					lowres_camera.SetPixelSampleFactor(3);
					lowres_camera.SetRelativePose(cam_offset, cam_orient);
					mavs::sensor::camera::RgbCamera hires_camera;
					hires_camera.Initialize(1620, 1080, 0.0225f, 0.015f, 0.009f);
					hires_camera.SetEnvironmentProperties(&env);
					hires_camera.SetGamma(0.5);
					hires_camera.SetAntiAliasing("oversampled");
					hires_camera.SetPixelSampleFactor(4);
					hires_camera.SetRelativePose(cam_offset, cam_orient);
					mavs::sensor::camera::SphericalCamera spherical_camera;
					spherical_camera.SetRelativePose(top_offset,top_orient);

					for (int i = 0; i < (int)poses.size(); i++) {
						image_num++;
						if (image_num>=hotstart_num){
							logfile << image_num << " " << i << " " << hour << " " << turbidity << " " << terrain_roughness << std::endl;
							std::string outbase = mavs::utils::ToString(image_num, 6);

							lowres_camera.SetPose(poses[i].position, poses[i].quaternion);
							lowres_camera.Update(&env, dt);

							hires_camera.SetPose(poses[i].position, poses[i].quaternion);
							hires_camera.Update(&env, dt);

							spherical_camera.SetPose(poses[i].position, poses[i].quaternion);
							spherical_camera.Update(&env, dt);

							if (turbidity == min_turbidity && hour == min_time) {
								left_lidar.SetPose(poses[i].position, poses[i].quaternion);
								left_lidar.Update(&env, dt);
								right_lidar.SetPose(poses[i].position, poses[i].quaternion);
								right_lidar.Update(&env, dt);
								top_lidar.SetPose(poses[i].position, poses[i].quaternion);
								top_lidar.Update(&env, dt);

								top_lidar.AnnotateFrame(&env, true);
								left_lidar.AnnotateFrame(&env, true);
								right_lidar.AnnotateFrame(&env, true);

								std::vector<mavs::sensor::lidar::labeled_point> top_points = top_lidar.GetLabeledPoints();
								std::vector<mavs::sensor::lidar::labeled_point> left_points = left_lidar.GetLabeledPoints();
								std::vector<mavs::sensor::lidar::labeled_point> right_points = right_lidar.GetLabeledPoints();
								pc_analyzer.AnalyzeCloud(top_points, "top", image_num, (mavs::sensor::lidar::Lidar *)&top_lidar);
								pc_analyzer.AnalyzeCloud(left_points, "left", image_num, (mavs::sensor::lidar::Lidar *)&left_lidar);
								pc_analyzer.AnalyzeCloud(right_points, "right", image_num, (mavs::sensor::lidar::Lidar *)&right_lidar);

								//left_lidar.Display();
								//right_lidar.Display();
								//top_lidar.Display();

							}

							//hires_camera.Display();
							//lowres_camera.Display();
						
							std::string hires_out = outbase;
							std::string lowres_out = outbase;
							std::string sph_out = outbase;
							std::string hi_labels = outbase;
							std::string lo_labels = outbase;
							std::string hi_csv = outbase;
							std::string lo_csv = outbase;

							hi_labels.append("_high_segment.bmp");
							hi_csv.append("_high_labels.csv");
							hires_camera.AnnotateFrame(&env, true);
							hires_camera.SaveSegmentedImage(hi_labels);
							hires_camera.SaveSemanticAnnotationsCsv(hi_csv);

							lo_labels.append("_low_segment.bmp");
							lo_csv.append("_low_labels.csv");
							lowres_camera.AnnotateFrame(&env, true);
							lowres_camera.SaveSegmentedImage(lo_labels);
							lowres_camera.SaveSemanticAnnotationsCsv(lo_csv);

							hires_out.append("_high.bmp");
							lowres_out.append("_low.bmp");
							sph_out.append("_spherical.bmp");
							hires_camera.SaveImage(hires_out);
							lowres_camera.SaveImage(lowres_out);
							spherical_camera.SaveImage(sph_out);
						}
					}
					
					turbidity += 2.0;
				}
				hour += 2;
			}
		}

		terrain_roughness += 5.0f;
	}

	logfile.close();
#endif
	return 0;
}

