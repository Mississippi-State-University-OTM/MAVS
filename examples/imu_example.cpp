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
#include <mavs_core/plotting/mavs_plotting.h>

#ifdef USE_OMP
#include <omp.h>
#endif
#include <raytracers/embree_tracer/embree_tracer.h>

mavs::environment::Environment env;
mavs::sensor::camera::RgbCamera camera;
mavs::raytracer::embree::EmbreeTracer scene;
mavs::vehicle::Rp3dVehicle veh;
mavs::sensor::imu::ImuSimple imu;

mavs::utils::Mplot gyr_plot;
mavs::utils::Mplot acc_plot;
mavs::utils::Mplot mag_plot;
std::vector<float> tp, omega_z, acc_x, mag_z;

static void UpdateVehicle(float dt);

int main(int argc, char *argv[]) {

	if (argc < 2) {
		std::cerr << "ERROR, must provide scene file and vehicle file as arguments" << std::endl;
		return 1;
	}

	std::string scene_file(argv[1]);
	std::string vehic_file(argv[2]);

	scene.Load(scene_file);

	env.SetTurbidity(9.0);
	env.SetFog(0.0f);
	env.SetDateTime(2023, 9, 1, 8, 0, 0, 6);
	env.SetRaytracer(&scene);

	camera.SetEnvironmentProperties(&env);
	camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
	camera.SetRelativePose(glm::vec3(-10.0f, 0.0f, 1.5f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	camera.SetName("camera");
	camera.SetPose(glm::vec3(0.0f, 0.0f, 1.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	camera.SetElectronics(0.75f, 1.0f);

	veh.Load(vehic_file);
	float zstart = scene.GetSurfaceHeight(0.0f, 0.0f);
	veh.SetPosition(0.0f,0.0f, zstart + 0.25f);
	veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);

	float dt = 1.0f / 100.0f;

	imu.SetGyroNoise(0.0f, 0.03f);
	imu.SetAccelerometerNoise(0.0f, 0.1f);
	imu.SetMagnetometerNoise(0.0f, 0.01f);

	acc_plot.SetTrajectoryXSize(10.0f);
	acc_plot.TurnOnAxis();
	acc_plot.SetTitle("Accelerometer Longitudinal Acceleration");
	gyr_plot.SetTrajectoryXSize(10.0f); // display up to 5 seconds at a time
	gyr_plot.TurnOnAxis();
	gyr_plot.SetTitle("Gyroscope Angular Velocity");

	int nsteps = 0;
	float elapsed_time = 0.0f;
	while(camera.DisplayOpen() || nsteps==0){
		// update vehicle
		UpdateVehicle(dt);

		// update imu
		mavs::VehicleState veh_state = veh.GetState();
		imu.SetPose(veh_state);
		imu.Update(&env, dt);
		glm::quat ori = imu.GetDeadReckoningOrientation();
		glm::vec3 angvel = imu.GetAngularVelocity();
		glm::vec3 linacc = imu.GetAcceleration();
		std::cout << "[" << ori.w << ", " << ori.x << ", " << ori.y << ", " << ori.z << "], [" << linacc.x << ", " << linacc.y << ", " << linacc.z << "], [" << angvel.x << ", " << angvel.y << ", " << angvel.z << "]" << std::endl;
		//double roll, pitch, yaw;
		//mavs::math::QuatToEulerAngle(ori, pitch, roll, yaw);

		// update individual mems plots
		tp.push_back(elapsed_time);
		omega_z.push_back(imu.GetAngularVelocity().z);
		acc_x.push_back(imu.GetAcceleration().x);
		gyr_plot.PlotTrajectory(tp, omega_z);
		acc_plot.PlotTrajectory(tp, acc_x);

		// update the camera
		if (nsteps % 4 == 0) {
			glm::vec3 look_to = veh.GetLookTo();
			float heading = 0.5f * (atan2f(look_to.y, look_to.x));
			camera.SetPose(veh.GetPosition(), glm::quat(cosf(heading), 0.0f, 0.0f, sinf(heading)));
			camera.Update(&env, 4 * dt);
			camera.Display();
			
		}
		
		nsteps++;
		elapsed_time += dt;
	}

	return 0;
}

static void UpdateVehicle(float dt) {
	double throttle = 0.0;
	double steering = 0.0;
	double braking = 0.0;
	 std::vector<bool> driving_commands = camera.GetKeyCommands();
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

	veh.Update(&env, (float)throttle, (float)steering, (float)braking, dt);
}