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
* \file imu_example.cpp
*
* Drive a MAVS vehicle implemented in RP3D
*
* Usage: >./imu_example mavs_scene_file.json rp3d_vehicle_file.json surface_type cone_index pose_log_freq
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
*
* rp3d_vehicle_file.json examples can be found in mavs/data/vehicles/rp3d_vehicles
*
* Press "L" to start/stop logging poses and "P" to save the logged poses to a .vprp file.
*
* \author Chris Goodin
*
* \date 9/11/2019
*/
#include <iostream>
#include <stdlib.h>
#include <mavs_core/math/utils.h>
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <sensors/mavs_sensors.h>

#ifdef USE_OMP
#include <omp.h>
#endif
#include <raytracers/embree_tracer/embree_tracer.h>

static void UpdateVehicle(mavs::vehicle::Rp3dVehicle* veh, mavs::sensor::camera::Camera* camera, mavs::environment::Environment* env, float dt);

int main(int argc, char *argv[]) {

	if (argc < 2) {
		std::cerr << "ERROR, must provide scene file and vehicle file as arguments" << std::endl;
		return 1;
	}

	std::string scene_file(argv[1]);
	std::string vehic_file(argv[2]);

	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);

	mavs::environment::Environment env;
	env.SetTurbidity(9.0);
	env.SetFog(0.0f);
	env.SetDateTime(2023, 9, 1, 8, 0, 0, 6);
	env.SetRaytracer(&scene);

	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
	camera.SetRelativePose(glm::vec3(-10.0f, 0.0f, 1.5f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	camera.SetName("camera");
	camera.SetPose(glm::vec3(0.0f, 0.0f, 1.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	camera.SetElectronics(0.75f, 1.0f);

	
	
	mavs::vehicle::Rp3dVehicle veh;
	veh.Load(vehic_file);
	float zstart = scene.GetSurfaceHeight(0.0f, 0.0f);
	veh.SetPosition(0.0f,0.0f, zstart + 0.25f);
	veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);

	float dt = 1.0f / 100.0f;

	mavs::sensor::imu::Imu imu;
	imu.SetSampleRate(1.0f / dt);

	int nsteps = 0;
	float heading = 0.0f;
	while(camera.DisplayOpen() || nsteps==0){

		UpdateVehicle(&veh, &camera, &env, dt);
		auto state = veh.GetState();
		//std::cout << sqrtf(state.twist.linear.x * state.twist.linear.x + state.twist.linear.y * state.twist.linear.y) << " " << state.twist.angular.z << std::endl;
		heading += 5000.0f*state.twist.angular.z;
		glm::vec3 lt = veh.GetLookTo();
		std::cout << "Heading = " << heading << " "<<atan2f(lt.y, lt.x) << std::endl;
		imu.SetPose(veh.GetState());
		imu.Update(&env, dt);
		glm::vec3 ang_vel = imu.GetAngularVelocity();
		glm::vec3 lin_acc = imu.GetAcceleration();
		//glm::quat ori = imu.GetDeadReckoningOrientation();
		//std::cout << ang_vel.x << " " << ang_vel.y << " " << ang_vel.z << " " << lin_acc.x << " " << lin_acc.y << " " << lin_acc.z << " " << ori.w << " " << ori.x << " " << ori.y << " " << ori.z << std::endl;
		if (nsteps % 4 == 0) {
			glm::vec3 look_to = veh.GetLookTo();
			float heading = 0.5f * (atan2f(look_to.y, look_to.x));
			camera.SetPose(veh.GetPosition(), glm::quat(cosf(heading), 0.0f, 0.0f, sinf(heading)));
			camera.Update(&env, 4 * dt);
			camera.Display();
			
		}
		
		nsteps++;

	}

	return 0;
}

static void UpdateVehicle(mavs::vehicle::Rp3dVehicle *veh, mavs::sensor::camera::Camera *camera, mavs::environment::Environment *env, float dt) {
	double throttle = 0.0;
	double steering = 0.0;
	double braking = 0.0;
	 std::vector<bool> driving_commands = camera->GetKeyCommands();
	if (driving_commands[0]) {
		throttle = 1.0;
	}
	else if (driving_commands[1]) {
		braking = 1.0;
	}
	if (driving_commands[2]) {
		steering = 1.0;
	}
	else if (driving_commands[3]) {
		steering = -1.0;
	}

	veh->Update(env, (float)throttle, (float)steering, (float)braking, dt);
}