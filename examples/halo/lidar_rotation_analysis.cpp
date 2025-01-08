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
* \file lidar_rotation_analysis.cpp
*
* Generates labeled lidar data for different rotations
*
* Usage: >./lidar_rotation_analysis (display) (hotstart_num) 
*
* where display is an optional parameter that will show a simple 
* rendering of the scene if set >0 
* and hotstart_num is an optional parameter that will restart 
* the batch sim at a given frame. 
*
* \author Chris Goodin
*
* \date 1/28/2019
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
#include <sensors/camera/simple_camera.h>

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
	if (argc > 1)disp_int = atoi(argv[1]);
	if (disp_int > 0)display = true;
  if (argc>2)hotstart_num = atoi(argv[2]);

	mavs::MavsDataPath mavs_data_path;
	std::string eco_path = mavs_data_path.GetPath() + "/ecosystem_files/";

	std::ofstream logfile("logfile.txt",std::ofstream::app);
	logfile << "image_num pose_num terrain_roughness rotation_angle" <<std::endl;

	float lidar_rotation_angle = -90.0; // 25.0f is the current value;
	int image_num = 0;
	while (lidar_rotation_angle <= 90.0) {
		std::cout << "Doing lidar angle " << lidar_rotation_angle << std::endl;
		mavs::sensor::lidar::MEight top_lidar, left_lidar, right_lidar;
		left_lidar.SetName("LeftLidar");
		left_lidar.SetName("RightLidar");
		float to_rot = (float)(mavs::kDegToRad*lidar_rotation_angle);
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

		float min_roughness = 0.0;
		float max_roughness = 0.0; //20.0;
		float terrain_roughness = min_roughness;
		while (terrain_roughness <= max_roughness) {
			//std::cout << "Doing terrain rougness " << terrain_roughness << std::endl;
			std::vector<mavs::terraingen::RandomSceneInputs> inputs;
			mavs::terraingen::RandomSceneInputs forest_input;
			mavs::terraingen::RandomSceneInputs desert_input;
			mavs::terraingen::RandomSceneInputs meadow_input;
			forest_input.terrain_width = 75.0f;
			forest_input.terrain_length = 75.0f;
			forest_input.lo_mag = terrain_roughness;
			forest_input.hi_mag = 0.0f;
			forest_input.mesh_resolution = 1.0f;
			forest_input.trail_width = 2.0f;
			forest_input.wheelbase = 1.25f;
			forest_input.track_width = 0.6f;
			forest_input.path_type = "Valleys";
			forest_input.basename = "forest";
			forest_input.plant_density = mavs::math::rand_in_range(0.1f, 0.6f);
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

			double dt = 0.1;
			for (int i = 0; i < (int)inputs.size(); i++) {
				//std::cout << "Doing scene input " << i << " of " << inputs.size() << std::endl;
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
  				scene.TurnOnLabeling();
  				scene.TurnOnSpectral();
  				//scene.TurnOffSurfaceTextures();
				scene.TurnOnSurfaceTextures();
				//create the pose structure and load
				mavs::AnvelVprpReader pose_reader;
				std::vector<mavs::Pose> poses = pose_reader.Load(path_file, 5);
				mavs::environment::Environment env;
				env.SetTurbidity(4.0);
				env.SetDateTime(2000, 6, 1, 12, 0, 0, 6);
				env.SetRaytracer(&scene);

				//create the cameras
				glm::vec3 cam_offset(0.0f, 0.0f, 0.75f);
				glm::quat cam_orient(1.0f, 0.0f, 0.0f, 0.0f);
				mavs::sensor::camera::SimpleCamera lowres_camera;
				lowres_camera.Initialize(384, 384, 0.0035f, 0.0035f, 0.0035f);
				lowres_camera.SetGamma(0.85f);
				lowres_camera.SetAntiAliasing("oversampled");
				lowres_camera.SetPixelSampleFactor(1);
				lowres_camera.SetRelativePose(cam_offset, cam_orient);
				//std::cout << "Looping over " << poses.size() << " poses " << std::endl;
				for (int i = 0; i < (int)poses.size(); i++) {
					//std::cout << "Doing pose " << i << " of " << poses.size() << " "<<image_num<<std::endl;
					std::cout << "." << std::flush;
					image_num++;
					if (image_num >= hotstart_num) {
						logfile << image_num << " " << i << " " << terrain_roughness << " "<<lidar_rotation_angle<<std::endl;
						std::string outbase = mavs::utils::ToString(image_num, 6);

						//camera takes about 1/10 the time of each lidar
						//std::cout << "Camera..." << std::endl;
						if (display) {
							lowres_camera.SetPose(poses[i].position, poses[i].quaternion);
							lowres_camera.Update(&env, dt);
						}
						//std::cout << "Left lidar..." << std::endl;
						left_lidar.SetPose(poses[i].position, poses[i].quaternion);
						left_lidar.Update(&env, dt);
						left_lidar.AnnotateFrame(&env, true);
						//std::cout << "Right lidar..." << std::endl;
						right_lidar.SetPose(poses[i].position, poses[i].quaternion);
						right_lidar.Update(&env, dt);
						right_lidar.AnnotateFrame(&env, true);

						//top_lidar.SetPose(poses[i].position, poses[i].quaternion);
						//top_lidar.Update(&env, dt);
						//top_lidar.AnnotateFrame(&env, true);

						//std::cout << "Saving ..." << std::endl;
						std::string left_out = outbase;
						std::string right_out = outbase;

						left_out.append("_left.txt");
						right_out.append("_right.txt");

						left_lidar.WriteLabeledRegisteredPointsToText(left_out);
						right_lidar.WriteLabeledRegisteredPointsToText(right_out);
						//std::cout << "Displaying " << std::endl;
						if (display) {
							//left_lidar.Display();
							//right_lidar.Display();
							//left_lidar.DisplayLidarCamera(&env);
							//right_lidar.DisplayLidarCamera(&env);
							lowres_camera.Display();
							//top_lidar.Display();
						}
						//std::cout << "Done with pose number " << i << std::endl;
					} //if imagnum>= hotstart
				} //loop over poses
			} //loop over scenes
			terrain_roughness += 5.0f;
		} //loop over roughnesses
		lidar_rotation_angle += 1.0;
	} //loop over lidar rotation angle

	logfile.close();
#endif
	return 0;
}

