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
* \file mavs_benchmark.cpp
*
* Run a benchmark test on rendering speed
*
* Usage: >./scene_viewer scene_file.json (seed) (nir) (timers)
*
* \author Chris Goodin
*
* \date 7/30/2025
*/
// c++ headers
#include <iostream>
#include <stdlib.h>
#include <omp.h>
// mavs headers
#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <sensors/mavs_sensors.h>
#include <raytracers/embree_tracer/embree_tracer.h>

mavs::raytracer::embree::EmbreeTracer scene;
mavs::environment::Environment env;
mavs::sensor::camera::RgbCamera camera;
mavs::sensor::lidar::OusterOS2 os2;
mavs::vehicle::Rp3dVehicle veh;

int main(int argc, char* argv[]) {
	std::cout << "Loading benchmark simulation..." << std::endl;
	mavs::MavsDataPath mdp;
	std::string mavs_data_path = mdp.GetPath();
	std::string scene_file = mavs_data_path+"/scenes/cavs_pg_full_withVeg.json";
	std::string vehic_file = mavs_data_path+"/vehicles/rp3d_vehicles/hmmwv.json";
	
	scene.Load(scene_file);
	
	env.SetTurbidity(9.0);
	env.SetFog(0.0f);
	env.SetDateTime(2023, 9, 1, 8, 0, 0, 6);
	env.SetRaytracer(&scene);

	camera.SetEnvironmentProperties(&env);
	camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
	camera.SetRelativePose(glm::vec3(-10.0f, 0.0f, 2.5f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	camera.SetName("camera");
;
	camera.SetElectronics(0.65f, 1.0f);
	
	os2.SetRelativePose(glm::vec3(0.0f, 0.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

	veh.Load(vehic_file);
	veh.SetPosition(5.0f, 0.0f, 1.0f);
	veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);

	float current_time = 0.0f;
	float dt = 0.01f;
	int nsteps = 0;

	std::cout << "Running benchmark simulation, please wait..." << std::endl;

	double t0 = omp_get_wtime();
	while (current_time<20.0f){
		veh.Update(&env, 0.4f, 0.25f, 0.0f, dt);

		mavs::VehicleState veh_state = veh.GetState();

		if (nsteps % 10 == 0) {
			os2.SetPose(veh_state);
			os2.Update(&env, 0.1f);
		}

		if (nsteps % 3 == 0) {
			glm::vec3 look_to = veh.GetLookTo();
			float heading = 0.5f * (atan2f(look_to.y, look_to.x));
			camera.SetPose(veh.GetPosition(), glm::quat(cosf(heading), 0.0f, 0.0f, sinf(heading)));
			camera.Update(&env, 3 * dt);
		}
		
		nsteps++;
		current_time += dt;
	}

	double t1 = omp_get_wtime();
	std::cout << current_time << " of simulation completed in " << (t1 - t0) << " seconds of wall time with " << omp_get_max_threads() << " threads." << std::endl;

	return 0;
}
