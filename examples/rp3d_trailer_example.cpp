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
* \file rp3d_trailer_example.cpp
*
* Drive a MAVS veh-trailer implemented in RP3D
*
* Usage: >./rp3d_trailer_example
*
* \author Chris Goodin
*
* \date 2/6/2025
*/
#include <iostream>
#include <stdlib.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <sensors/mavs_sensors.h>
#include <tinyfiledialogs.h>
#include <raytracers/embree_tracer/embree_tracer.h>

float throttle = 0.0f; float braking = 0.0f; float steering = 0.0f;
mavs::sensor::camera::RgbCamera camera;
void UpdateDrivingCommands();

int main(int argc, char *argv[]) {
	mavs::MavsDataPath mdp;
	std::string mavs_data_path = mdp.GetPath();
	std::string scene_file = mavs_data_path + "/scenes/cube_scene.json";
	std::string vehic_file = mavs_data_path + "/vehicles/rp3d_vehicles/m915a5_with_trailer.json";

	mavs::environment::Environment env;
	env.SetTurbidity(9.0);
	env.SetFog(0.0f);
	env.SetDateTime(2023, 9, 1, 8, 0, 0, 6);

	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	env.SetRaytracer(&scene);

	glm::vec3 sensor_offset(-3.0f, -10.0f, 1.5f);
	glm::quat sensor_orient(0.7071f, 0.0f, 0.0, 0.7071f);
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(640, 360, 0.00622222222f, 0.0035f, 0.0035f);
	camera.SetRelativePose(sensor_offset, sensor_orient);
	camera.SetElectronics(0.85f, 1.0f);
	
	mavs::vehicle::Rp3dVehicle veh;
	veh.Load(vehic_file);
	veh.SetPosition(0.0f,0.0f, 0.0f);
	veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
	veh.SetHitchPoint(-1.66f, 0.0f, 0.23f - 0.10075f);

	float dt = 1.0f / 100.0f;
	int nrender = int((1.0f / 30.0f) / dt);
	int nsteps = 0;

;
	while((camera.DisplayOpen() || nsteps==0) && nsteps<8640000){
		
		UpdateDrivingCommands();

		veh.Update(&env, (float)throttle, (float)steering, (float)braking, dt);

		mavs::VehicleState veh_state = veh.GetState();

		if (std::isinf(veh_state.pose.position.x) || std::isnan(veh_state.pose.position.x)) break;
		camera.SetPose(veh_state);

		if (nsteps % nrender == 0) {
			camera.Update(&env, 4 * dt);
			camera.Display();
		}
		
		nsteps++;
	}

	return 0;
}

void UpdateDrivingCommands() {
	std::vector<bool> driving_commands;
	driving_commands = camera.GetKeyCommands();
	float step = 0.025f;
	if (driving_commands[0]) {
		throttle += step;
		braking = 0.0f;
	}
	else if (driving_commands[1]) {
		braking += 2*step;
		throttle = 0.0f;
	}
	else {
		throttle *= 0.9f;
	}
	if (driving_commands[2]) {
		steering += 0.5*step;
	}
	else if (driving_commands[3]) {
		steering -= 0.5*step;
	}
	else {
		steering *= 0.9f;
	}
	steering = std::max(-1.0f, std::min(1.0f, steering));
	braking = std::max(0.0f, std::min(1.0f, braking));
	throttle = std::max(0.0f, std::min(1.0f, throttle));
}