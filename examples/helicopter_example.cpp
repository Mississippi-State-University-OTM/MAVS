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
* \file helicopter_example.cpp
*
* Fly a MAVS uav 
*
* Usage: >./helicopter_example heli_input_file.json
*
* heli_input_file.json is a MAVS helicopter file, examples of which can
* be found in mavs/data/vehicles/helicopters.
*
* \author Chris Goodin
*
* \date 11/18/2024
*/
#include "vehicles/helicopter/mavs_helicopter.h"
#include <sensors/mavs_sensors.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/plotting/mavs_plotting.h>
#ifdef USE_OMP
#include <omp.h>
#endif
#include <raytracers/embree_tracer/embree_tracer.h>

int main(int argc, char *argv[]) {
#ifndef USE_EMBREE
	std::cerr << "ERROR, MUST HAVE EMBREE ENABLED TO RUN THIS EXAMPLE! " << std::endl;
	exit(12);
#endif
	if (argc < 3) {
		std::cerr << "ERROR, must provide a scene file and Helicopter input file as argument" << std::endl;
		return 1;
	}

	std::string scene_file(argv[1]);
	std::string heli_input_file(argv[2]);

	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	
	mavs::environment::Environment env;
	env.SetTurbidity(5.0);
	env.SetFog(0.0f);
	env.SetDateTime(2023, 9, 1, 8, 0, 0, 6);
	env.SetRaytracer(&scene);

	int nx = 960;
	int ny = nx;
	float zlo = -90.0;
	float zhi = -75.0;
	cimg_library::CImg<float> map_img;
	glm::vec2 map_ll(-250.0f, -250.0f);
	glm::vec2 map_ur(250.0f, 250.0f);
	float pixdim = (map_ur.x - map_ll.x) / (float)nx;
	map_img.assign(nx, ny, 1, 1, zhi);
	cimg_library::CImgDisplay disp;
	disp.assign(map_img);

	glm::vec3 sensor_offset(-15.0f, -15.0f, 1.5f);
	glm::quat sensor_orient(cosf(0.5f*0.7071f), 0.0f, 0.0f, sinf(0.5f*0.7071f));
	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(nx, (int)((640.0/960.0)*nx), 0.00525f, 0.0035f, 0.0035f);
	camera.SetRelativePose(sensor_offset, sensor_orient);
	camera.SetName("camera");
	camera.SetElectronics(0.75f, 1.0f);

	mavs::sensor::lidar::OusterOS2 front_lidar, rear_lidar;
	glm::vec3 front_offset(2.0f, 0.0f, -1.5f);
	glm::quat front_orient(cosf(-0.025f * mavs::kPi), 0.0f, sinf(-0.025f * mavs::kPi), 0.0f);
	front_lidar.SetRelativePose(front_offset, front_orient);
	glm::vec3 rear_offset(-2.0f, 0.0f, -1.5f);
	glm::quat rear_orient(cosf(-0.025f * mavs::kPi), 0.0f, sinf(0.025f * mavs::kPi), 0.0f);
	rear_lidar.SetRelativePose(rear_offset, rear_orient);

	mavs::vehicle::MavsHelicopter heli;
	glm::vec3 start_pos(10.0f, 10.0f, 0.0f);
	float zstart = scene.GetSurfaceHeight(start_pos.x, start_pos.y);
	start_pos.z = zstart + 1.0f;
	glm::quat start_ori(1.0f, 0.0f, 0.0f, 0.0f);
	heli.Load(heli_input_file, &env, start_pos, start_ori);

	float current_time = 0.0f;
	float dt = 1.0f / 100.0f;
	int nsteps = 0;

	while (camera.DisplayOpen() || nsteps == 0) {
		double t0 = omp_get_wtime();
		double throttle = 0.0;
		double steering = 0.0;
		double braking = 0.0;
		std::vector<bool> driving_commands = camera.GetKeyCommands();
		if (driving_commands[0]) {throttle = 1.0;}else if (driving_commands[1]) {braking = 1.0;}
		if (driving_commands[2]) {steering = 1.0;}else if (driving_commands[3]) {steering = -1.0;}

		if (camera.GetDisplay()->is_keyH()) { heli.SetHover(); }
		else if (camera.GetDisplay()->is_keyI()) { heli.IncreaseLift(dt); }
		else if (camera.GetDisplay()->is_keyK()) { heli.DecreaseLift(dt); }

		heli.Update(&env, (float)throttle, (float)steering, (float)braking, dt);

		mavs::VehicleState veh_state = heli.GetState();

		if (std::isinf(veh_state.pose.position.x) || std::isnan(veh_state.pose.position.x)) break;

		if (nsteps % 5 == 0) {
			camera.Update(&env, 5 * dt);
			camera.SetPose(veh_state);
			std::vector<glm::vec4> all_points = front_lidar.GetRegisteredPointsXYZI();
			std::vector<glm::vec4> rear_points = rear_lidar.GetRegisteredPointsXYZI();
			all_points.insert(all_points.end(), rear_points.begin(), rear_points.end());
			camera.AddLidarPointsToImage(all_points);
			camera.Display();
			std::string outfile = mavs::utils::ToString(nsteps, 5) + "_image.bmp";
			if (!heli.GetRp3dActive()) {
				camera.SaveImage(outfile);
				for (int i = 0; i < all_points.size(); i++) {
					int cx = (int)floor((all_points[i].x - map_ll.x) / pixdim);
					int cy = (int)floor((all_points[i].y - map_ll.y) / pixdim);
					if (cx >= 0 && cx < nx && cy >= 0 && cy < ny) {
						if (all_points[i].z < map_img(cx, cy)) {
							float val = std::min(255.0,255.0 * ((all_points[i].z - zlo) / (zhi - zlo)));
							map_img.draw_point(cx, cy, (float*)&val);
						}
					}
				}
				std::string mapout = mavs::utils::ToString(nsteps, 5) + "_map.bmp";
				map_img.save(mapout.c_str());
				disp = map_img;
			}
		}
		if (nsteps % 10 == 0) {
			front_lidar.Update(&env, 10*dt);
			front_lidar.SetPose(veh_state);
			rear_lidar.Update(&env, 10 * dt);
			rear_lidar.SetPose(veh_state);
		}
		nsteps++;

		while (omp_get_wtime() - t0 < dt) {}
	}

	return 0;
}
