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
* \file slam_example.cpp
*
* Usage: >./slam_example scenfile.json
* where scenefile.json examples can be found in 
* mavs/data/scenes
*
* Gives an example of dead reckoning using the IMU
* Also creates an M8 point cloud that could be used
* for a SLAM algorithm. A camera output is rendered
* for debugging purposes.
* 
* Saves an output file, "vehicle_log.txt" that has
* 5 columns:
* time, true.x, true.y, reckoned.x, reckoned.y
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <iostream>
#include <fstream>
//#include "vehicles/car/full_car.h"
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif
#include <raytracers/simple_tracer/simple_tracer.h>
#include <sensors/mavs_sensors.h>
#include <mavs_core/environment/environment.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>

int main (int argc, char *argv[]){

	mavs::vehicle::Rp3dVehicle vehicle;
	mavs::MavsDataPath dp;
	std::string veh_file = dp.GetPath() + "/vehicles/rp3d_vehicles/forester_2017_rp3d.json";
	vehicle.Load(veh_file);

	//----- Create/load the scene/environment for the simulation -----------
	mavs::environment::Environment env;
	env.SetLocalOrigin(32.6526, 90.8779, 73.152); //Vicksburg, MS
	bool scene_loaded = false;
#ifdef USE_EMBREE
	if (argc<2) {
		std::cerr << "Usage: ./slam_example scenefile.json" << std::endl;
		return 1;
	}
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	scene_loaded = true;
	env.SetRaytracer(&scene);
	
#else
	std::cout << scene_loaded << std::endl;
	if (!scene_loaded) {
	//	std::cout << "Creating the scene " << std::endl;
		mavs::raytracer::SimpleTracer scene;
		mavs::raytracer::Aabb ground(0.0f, 0.0f, 0.0f, 1.0E6f, 1.0E6f, 0.01f);
		ground.SetColor(0.24f, 0.48f, 0.2f);
		scene.AddPrimitive(ground);
		int nboxes = 50;
		std::vector<mavs::raytracer::Aabb> box;
		box.resize(nboxes);
		std::vector<glm::vec2> box_locations;
		box_locations.resize(nboxes);
		float range = 100.0f;
		for (int i = 0; i < nboxes; i++) {
			float x = mavs::math::rand_in_range(-range, range);
			float y = mavs::math::rand_in_range(-range, range);
			double sx = mavs::math::rand_in_range(1.0, 2.0);
			double sy = mavs::math::rand_in_range(1.0, 2.0);
			box_locations[i].x = x;
			box_locations[i].y = y;
			box[i].SetPosition(x, y, 2.0f);
			box[i].SetSize(sx, sy, 4.0f);
			float red = mavs::math::rand_in_range(0.25, 1.0);
			float blue = mavs::math::rand_in_range(0.25, 1.0);
			float green = mavs::math::rand_in_range(0.25, 1.0);
			box[i].SetColor(red, green, blue);
			scene.AddPrimitive(box[i]);
		}
		env.SetRaytracer(&scene);
	}
#endif
	//---- Done with scene/environment -----------------------

	//---- Create the sensors for the simulation -------------
	mavs::sensor::imu::Imu imu;
	imu.SetSampleRate(100.0f);
	imu.GetGyro()->SetConstantBias(glm::vec3(0.0f, 0.0f, 0.0f));
	mavs::sensor::lidar::MEight m8;
	mavs::sensor::camera::RgbCamera cam;
	cam.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	cam.SetPixelSampleFactor(1);
	//---- Done with sensor creation ---------------------------

	// Parameters for simulation timing
	float time = 0.0f;
	float dt = 1.0E-3f;
	int nsteps = 0;

	// variables for vehicle control
	float throttle = 0.0f;
	float steering = 0.5f;
	float desired_speed = 5.0f; //m/s
	float k_control = 0.1f;

	// Dead-reckoning solution variables
	glm::vec2 reckoned_pos(0.0f,0.0f);
	glm::vec2 reckoned_vel(0.0f, 0.0f);
	glm::vec2 look_to(1.0f, 0.0f);
	float reckoned_heading = 0.0f;

	std::ofstream fout("vehicle_log.txt");
	while (time < 15.0f) {
		//update the vehicle
		vehicle.Update(&env, throttle, steering, 0.0f, dt);

		//Get the vehicle state, in global coordinates
		mavs::VehicleState state = vehicle.GetState();

		//calculate desired throttle for next time step
		float speed = (float)sqrt(state.twist.linear.x*state.twist.linear.x + state.twist.linear.y*state.twist.linear.y);
		float dspeed = speed - desired_speed;
		if (dspeed < 0.0f) {
			throttle = throttle - k_control*dspeed;
		}
		else {
			throttle = throttle - k_control*dspeed;
		}
		throttle = std::max(std::min(throttle, 1.0f),0.0f);

		//do the IMU, lidar, and camera simulation
		if (nsteps % 10 == 0) {
			imu.SetPose(state);
			imu.Update(&env, dt);
		}
		if (nsteps % 100 == 0) {
			m8.SetPose(state);
			m8.Update(&env, dt);
		}
		if (nsteps % 30 == 0) {
			cam.SetPose(state);
			cam.Update(&env, dt);
		}

		// get acceleration and angular velocity in sensor coordinates
		glm::vec3 accel = imu.GetAcceleration();
		glm::vec3 rotvel = imu.GetAngularVelocity();
		
		//Dead-reckoning solution, no SLAM
		reckoned_heading = reckoned_heading + rotvel.z*dt;
		glm::vec2 look_to(cos(reckoned_heading), sin(reckoned_heading));
		glm::vec2 look_side(-sin(reckoned_heading), cos(reckoned_heading));
		glm::vec2 a = accel.x*look_to + accel.y*look_side;
		reckoned_pos = reckoned_pos + dt * reckoned_vel + 0.5f*a*dt*dt;
		reckoned_vel = reckoned_vel + dt*a;

		//log data
		fout << time << " "<<state.pose.position.x<<" "<<state.pose.position.y<<" "<<reckoned_pos.x<<" "<<reckoned_pos.y<<std::endl;

		//display camera and lidar
		cam.Display();
		m8.Display();

		if (nsteps%100==0)std::cout << "Simulation at time " << time << std::endl;

		//update timing
		time += dt;
		nsteps++;
	}
	fout.close();
  return 0;
}

