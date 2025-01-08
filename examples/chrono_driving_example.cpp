/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
/**
* \file chrono_driving_example.cpp
*
* Drive a MAVS vehicle [implemented in Chrono] with the W-A-S-D keys
*
* Usage: >./chrono_driving_example mavs_scene_file.json chrono_vehicle_file.json
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
*
* chrono_vehicle_file.json examples can be found in mavs/data/vehicles/chrono_inputs
*
* \author Chris Goodin
*
* \date 3/12/2020
*/
#include <iostream>
#include <stdlib.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>
#ifdef Bool
#undef Bool
#endif
#include <vehicles/chrono/chrono_wheeled_json.h>
#include <sensors/mavs_sensors.h>
#ifdef USE_OMP
#include <omp.h>
#endif
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif

// Function to grab driving commands from a camera, see definition below main()
static void GetDrivingCommand(mavs::sensor::camera::RgbCamera *camera, double dt, double &throttle, double &braking, double &steering);

int main(int argc, char *argv[]) {
#ifndef USE_EMBREE
	std::cerr << "The Chrono Driving Example can only be run with Embree enabled" << std::endl;
	return 1;
#endif

#ifndef USE_CHRONO
	std::cerr << "The Chrono Driving Example can only be run with Chrono enabled" << std::endl;
	return 2;
#endif
	// Define the input files, which are all in the MAVS repo
	mavs::MavsDataPath dp;
	std::string scene_file = dp.GetPath();
	scene_file.append("/scenes/cube_scene.json");
	std::string vehic_file = dp.GetPath();
	vehic_file.append("/vehicles/chrono_inputs/hmmwv.json");
	std::string anim_file("vehicles/hmmwv/hmmwv_centered.obj");

	// Create the environment and raytracer, add the vehicle actor
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	mavs::environment::Environment env;
	env.SetRaytracer(&scene);
	env.AddActor(anim_file, false, false, false, glm::vec3(0.0f, 0.0f, -0.6f), glm::vec3(1.0, 1.0, 1.0));

	// Create the sensor
	glm::vec3 sensor_offset(-10.0f, 0.0f, 1.5f);
	glm::quat sensor_orient(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 position(0.0f, 0.0f, 1.0f);
	glm::quat orient(1.0f, 0.0f, 0.0f, 0.0f);
	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(384, 384, 0.0035f, 0.0035f, 0.0035f);
	camera.SetRelativePose(sensor_offset, sensor_orient);
	camera.SetName("camera");
	camera.SetPose(position, orient);
	camera.SetElectronics(0.5f, 1.0f);

	// Create and load the vehicle, set the initial pose
	mavs::vehicle::ChronoWheeledJson veh;
	veh.Load(vehic_file);
	veh.SetPosition(0.0f, 0.0f, 2.5f);
	veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
	
	// Create the variables that will be used to grab driving commands
	double throttle = 0.0;
	double steering = 0.0;
	double braking = 0.0;

	// Start the simulation
	float dt = 1.0f / 30.0f;
	while (true) {
		// grab driving commands from the camera window
		GetDrivingCommand(&camera, dt, throttle, braking, steering);

		// update the vehicle state
		veh.Update(&env, (float)throttle, (float)steering, (float)braking, dt);

		// Get the vehicle state and move the actor
		mavs::VehicleState veh_state = veh.GetState();
		env.SetActorPosition(0, veh_state.pose.position, veh_state.pose.quaternion);

		// update the camera frame
		camera.SetPose(veh_state);
		camera.Update(&env, dt);
		camera.Display();
	}
	
	return 0;
}

static void GetDrivingCommand(mavs::sensor::camera::RgbCamera *camera, double dt, double &throttle, double &braking, double &steering){
	std::vector<bool> driving_commands;
	driving_commands = camera->GetKeyCommands();
	if (driving_commands[0]) {
		throttle += dt;
		braking = 0.0;
	}
	else if (driving_commands[1]) {
		braking = 1.0;
		throttle = 0.0;
	}
	if (driving_commands[2]) {
		steering += 1.5*dt;
	}
	else if (driving_commands[3]) {
		steering -= 1.5*dt;
	}
	else {
		steering = steering * 0.9;
		throttle = throttle * 0.9;
		braking = 0.0;
	}
}