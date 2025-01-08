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
* \file multi_vehicle_example.cpp
*
* Demonstrates how to use MAVS MPI framework.
* to simulate multiple vehicles.
*
* Usage: >./multi_vehicle_example mavs_scene_file.json 
*
* or, in the case where the code was built with MPI enabled
*
* >mpiexec -np 8 ./multi_vehicle_example mavs_scene_file.json 
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
*
* When using the MPI version of the code, this example requires 
* 8 processors to run.
*
* \date 10/5/2018
*/
#include <simulation/simulation.h>
//#include "vehicles/car/full_car.h"
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <sensors/gps/gps.h>
#include <sensors/camera/rgb_camera.h>
#include <drivers/simple_waypoint_follower/simple_waypoint_follower.h>
#include <mavs_core/data_path.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#else
#include <raytracers/simple_tracer/simple_tracer.h>
#endif

int main(int argc, char *argv[]) {

	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI  
	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
#endif

	double max_sim_time = 100.0;

	int group_number = 1;
#ifdef USE_MPI    
	MPI_Comm group_comm;
	if (myid % 2 == 0) group_number = 0;
	MPI_Comm_split(MPI_COMM_WORLD, group_number, myid, &group_comm);
#endif

	mavs::environment::Environment env;
	env.SetLocalOrigin(32.6526, 90.8779, 73.152); //Vicksburg, MS
#ifdef USE_EMBREE
	if (argc < 2) {
		std::cerr << "ERROR: Must specify scene file as command line input \n";
		return 1;
	}
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	if (myid == 0)std::cout << "Loading " << scene_file << std::endl;
	scene.Load(scene_file);
#else
	mavs::raytracer::SimpleTracer scene;
	mavs::raytracer::Sphere sred;
	sred.SetPosition(0, 0, 10);
	sred.SetColor(1.0, 0.25, 0.25);
	sred.SetRadius(5.0);
	scene.AddPrimitive(sred);
	mavs::raytracer::Aabb box, box2;
	box.SetSize(1.0E6, 1.0E6, 0.01);
	box.SetColor(0.25, 1.0, 0.25);
	box.SetPosition(0, 0, 5.0);
	scene.AddPrimitive(box);
	box2.SetPosition(0.0, 4, 3);
	box2.SetSize(10.0, 10.0, 2.0);
	box2.SetColor(0.7, 0.7, 0.25);
	scene.AddPrimitive(box2);
#endif

	env.SetRaytracer(&scene);

	mavs::Simulation veh1;
	veh1.SetMaxSimTime(max_sim_time);
	veh1.SetEnvironment(env);

	mavs::driver::SimpleWaypointFollower driver1;
	mavs::Path path1;
	mavs::PoseStamped waypoint;
	waypoint.pose.position.x = 50.0f;
	waypoint.pose.position.y = 0.0f;
	waypoint.pose.position.z = 0.0f;
	path1.poses.push_back(waypoint);
	waypoint.pose.position.x = 150.0f;
	path1.poses.push_back(waypoint);
	driver1.SetPath(path1);
	veh1.AddDriver(driver1, 1);

	mavs::sensor::gps::Gps gps1;
	veh1.AddSensor(gps1, 1);

	glm::vec3 sensor_offset(0.0f, 0.0f, 1.0f);
	glm::quat sensor_orient(1.0f, 0.0f, 0.0f, 0.0f);
	mavs::sensor::camera::RgbCamera camera1;
	camera1.SetEnvironmentProperties(&env);
	camera1.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	camera1.SetRelativePose(sensor_offset, sensor_orient);
	camera1.SetTimeStep(0.05f);
	camera1.SetName("camera1");
	veh1.AddSensor(camera1, 1);

	mavs::vehicle::Rp3dVehicle vehic1;
	mavs::MavsDataPath dp;
	std::string veh_file = dp.GetPath() + "/vehicles/rp3d_vehicles/forester_2017_rp3d.json";
	vehic1.Load(veh_file);
	vehic1.SetPosition(2.5, 0.0, 0.0);
	vehic1.SetOrientation(1.0, 0.0, 0.0, 0.0);
	veh1.AddVehicle(vehic1, 1);

	veh1.DisplaySensors();

#ifdef USE_MPI  
	veh1.CreateGroups(group_comm);
#endif

	mavs::Simulation veh2;
	veh2.SetMaxSimTime(max_sim_time);
	veh2.SetEnvironment(env);

	mavs::driver::SimpleWaypointFollower driver2;
	mavs::Path path2;
	waypoint.pose.position.x = 5.0;
	waypoint.pose.position.y = 50.0;
	waypoint.pose.position.z = 0.0;
	path2.poses.push_back(waypoint);
	waypoint.pose.position.x = 150.0;
	path2.poses.push_back(waypoint);
	driver2.SetPath(path2);
	veh2.AddDriver(driver2, 1);

	mavs::sensor::gps::Gps gps2;
	veh2.AddSensor(gps2, 1);

	mavs::sensor::camera::RgbCamera camera2;
	camera2.SetEnvironmentProperties(&env);
	camera2.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	camera2.SetRelativePose(sensor_offset, sensor_orient);
	camera2.SetTimeStep(0.05f);
	camera2.SetName("camera2");
	veh2.AddSensor(camera2, 1);

	mavs::vehicle::Rp3dVehicle vehic2;
	vehic2.SetPosition(2.5, 2.5, 0.0);
	vehic2.Load(veh_file);
	vehic2.SetOrientation(1.0, 0.0, 0.0, 0.0);
	veh2.AddVehicle(vehic2, 1);

	veh2.DisplaySensors();

#ifdef USE_MPI  
	veh2.CreateGroups(group_comm);
#endif

	double dt = 0.001;
	double sim_time = 0.0;
	int nsteps = 0;
	while (!veh1.Complete() && !veh2.Complete()) {
		mavs::VehicleState state1, state2;
#ifdef USE_MPI
		if (group_number == 0)veh1.Update(dt);
		if (group_number == 1)veh2.Update(dt);
#else
		veh1.UpdateSerial(dt);
		veh2.UpdateSerial(dt);
#endif
		state1 = veh1.GetVehicleState();
		state2 = veh2.GetVehicleState();
		
		if (myid == 0 && nsteps%1000==0) {
			std::cout << "Simulation time: " << sim_time << std::endl;
			std::cout << "Vehicle 1 position (" << state1.pose.position.x << ", " << state1.pose.position.y << ")" << std::endl;
			std::cout << "Vehicle 2 position (" << state2.pose.position.x << ", " << state2.pose.position.y << ")" << std::endl;
			std::cout << std::endl;
		}
		nsteps++;
		sim_time += dt;
	}
	if (myid == 0) {
		std::cout << "Simulation complete" << std::endl;
	}

#ifdef USE_MPI    
	MPI_Finalize();
#endif

	return 0;
}
