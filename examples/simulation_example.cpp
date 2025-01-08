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
* \file simulation_example.cpp
*
* Demonstrates a MAVS simulation.
*
* Usage: >./simulation_example mavs_scene_file.json (lidar_num)
*
* or, in the case where the code was built with MPI enabled
*
* >mpiexec -np 6 ./simulation_example mavs_scene_file.json (lidar_num)
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
*
* When running with MPI, requires at least 6 processors. One each
* for the vehicle, driver, camera, lidar, compass, and gps.
* 
* lidar_num is optional and specifies the type of lidar sensor
*
* 291 = SICK LMS291
*
* 8 = M8
*
* 16 = Velodyne VLP 16
*
* 32 = Velodyne HDL32E
*
* 64 = Velocyne HDL64E
*
* The default LIDAR is 291
*
* The simulation will load the scene and attempt to
* navigate the vehicle from the point (-45,-45) to 
* (45,45) in the scene using the A* driver.
* 
* \author Chris Goodin
*
* \date 10/5/2018
*/
#include <iostream>
#include <stdlib.h>

//mavs simulation
#include <simulation/simulation.h>

//mavs vehicles
//#include "vehicles/car/full_car.h"
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <mavs_core/data_path.h>

// mavs driver
#include <drivers/simple_path_planner/simple_path_planner.h>

//mavs sensors
#include <sensors/mavs_sensors.h>
/*#include <sensors/camera/simple_camera.h>
#include <sensors/camera/rgb_camera.h>
#include <sensors/compass/compass.h>
#include <sensors/gps/gps.h>
#include <sensors/lidar/lms_291.h>
#include <sensors/lidar/hdl64e.h>
#include <sensors/lidar/hdl32e.h>
#include <sensors/lidar/vlp16.h>
#include <sensors/lidar/m8.h>*/

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#else
#include <raytracers/simple_tracer/simple_tracer.h>
#endif

int main(int argc, char *argv[]) {
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	mavs::environment::Environment env;
	double origin_lat = 32.6526;
	double origin_lon = 90.8779;
	double elevation = 73.152;
	env.SetLocalOrigin(origin_lat, origin_lon, elevation); //Vicksburg, MS
	env.SetDateTime(2004, 6, 5, 15, 0, 0, 6);

#ifdef USE_EMBREE
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	if (myid == 0)std::cout << "Loaded scene with " << scene.GetNumberTrianglesLoaded()
		<< " triangles. " << std::endl;
#else
	mavs::raytracer::SimpleTracer scene;
	mavs::raytracer::Aabb floor, north, south, east, west, obstacle;
	floor.SetSize(1.0E6f, 1.0E6f, 0.01f);
	floor.SetColor(0.25f, 1.0f, 0.25f);
	floor.SetPosition(0.0f, 0.0f, 5.0f);
	scene.AddPrimitive(floor);
	east.SetPosition(55.0f, 0.0f, 0.0f);
	east.SetSize(0.1f, 120.0f, 10.0f);
	east.SetColor(0.7f, 0.7f, 0.25f);
	scene.AddPrimitive(east);
	west.SetPosition(-55.0f, 0.0f, 0.0f);
	west.SetSize(0.1f, 120.0f, 10.0f);
	west.SetColor(0.7f, 0.7f, 0.25f);
	scene.AddPrimitive(west);
	north.SetPosition(0.0f, 55.0f, 0.0f);
	north.SetSize(120.0f, 0.1f, 10.0f);
	north.SetColor(0.7f, 0.7f, 0.25f);
	scene.AddPrimitive(north);
	south.SetPosition(0.0f, -55.0f, 0.0f);
	south.SetSize(120.0f, 0.1f, 10.0f);
	south.SetColor(0.7f, 0.7f, 0.25f);
	scene.AddPrimitive(south);
	obstacle.SetPosition(0.0f, 20.0f, 0.0f);
	obstacle.SetSize(20.0f, 20.0f, 10.0f);
	obstacle.SetColor(0.7f, 0.7f, 0.25f);
	scene.AddPrimitive(obstacle);
#endif

	env.SetRaytracer(&scene);

	mavs::Simulation sim;
	sim.SetMaxSimTime(10.0);
	sim.SetEnvironment(env);

	mavs::driver::SimplePathPlanner driver;
	driver.SolvePotential();
	driver.SetOrigin(origin_lat, origin_lon);
	driver.SetGoalEnu(45, 45);
	driver.SetTimeStep(0.1);
	driver.SetMapResolution(1.5);
	sim.AddDriver(driver, 1);

	glm::vec3 sensor_offset(0.0f, 0.0f, 1.0f);
	glm::quat sensor_orient(1.0f, 0.0f, 0.0f, 0.0f);

	mavs::sensor::gps::Gps gps;
	gps.SetTimeStep(0.25);
	gps.SetType("differential");
	gps.SetRelativePose(sensor_offset, sensor_orient);
	sim.AddSensor(gps, 1);

	mavs::sensor::compass::Compass compass;
	compass.SetRmsErrorDegrees(0.0);
	compass.SetTimeStep(0.0025);
	compass.SetRelativePose(sensor_offset, sensor_orient);
	sim.AddSensor(compass, 1);

	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	camera.SetRelativePose(sensor_offset, sensor_orient);
	camera.SetTimeStep(0.05f);
	camera.SetName("camera");
	sim.AddSensor(camera, 1);


	int sens_num = 291;
	if (argc > 2)sens_num = sens_num = atoi(argv[2]);
	int num_lidar_procs = 1;
	mavs::sensor::lidar::Lms291_S05 s05;
	mavs::sensor::lidar::Vlp16 vlp;
	mavs::sensor::lidar::Hdl32E hdl32;
	mavs::sensor::lidar::Hdl64E hdl64;
	mavs::sensor::lidar::MEight m8;
	if (sens_num == 64) {
		hdl64.SetRotationRate(10.0);
		hdl64.SetRelativePose(sensor_offset, sensor_orient);
		hdl64.SetName("HDL-64E");
		sim.AddSensor(hdl64, num_lidar_procs);
	}
	else if (sens_num == 16) {
		vlp.SetRotationRate(10.0);
		vlp.SetRelativePose(sensor_offset, sensor_orient);
		vlp.SetName("VLP-16");
		sim.AddSensor(vlp, num_lidar_procs);
	}
	else if (sens_num == 32) {
		hdl32.SetTimeStep(0.1);
		hdl32.SetRelativePose(sensor_offset, sensor_orient);
		hdl32.SetName("HDL-32E");
		sim.AddSensor(hdl32, num_lidar_procs);
	}
	else if (sens_num == 8) {
		m8.SetRotationRate(10.0);
		m8.SetRelativePose(sensor_offset, sensor_orient);
		m8.SetName("M8");
		sim.AddSensor(m8, num_lidar_procs);
	}
	else {
		s05.SetTimeStep(1.0 / 50.0);
		s05.SetRelativePose(sensor_offset, sensor_orient);
		s05.SetName("SICK-LMS291");
		sim.AddSensor(s05, num_lidar_procs);
	}
	
	mavs::vehicle::Rp3dVehicle veh;
	veh.SetPosition(-45.0f, -45.0f, 2.5f);
	veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
	mavs::MavsDataPath dp;
	std::string veh_file = dp.GetPath() + "/vehicles/rp3d_vehicles/forester_2017_rp3d.json";
	veh.Load(veh_file);
	veh.SetTimeStep(1.0/30);
	//veh.SetCgHeight(2.5);

	sim.AddVehicle(veh, 1);
#ifdef USE_MPI 
	sim.CreateGroups(MPI_COMM_WORLD);
#endif
	sim.DisplaySensors();
	float current_time = 0.0f;
	float dt = 0.01f;
	int nsteps = 0;
	sim.SetMaxSimTime(100.0);
#ifdef USE_MPI  
	MPI_Barrier(MPI_COMM_WORLD);
#endif  
	if (myid == 0)std::cout << "Running simulation" << std::endl;
	double wall_time = 0.0;
	while (!sim.Complete()) {
#ifdef USE_MPI    
		double t1 = MPI_Wtime();
#endif    
#ifdef USE_MPI
		sim.Update(dt);
#else
		sim.UpdateSerial(dt);
#endif
		current_time += dt;
		nsteps++;
#ifdef USE_MPI    
		wall_time += MPI_Wtime() - t1;
		if (myid == 0 && nsteps%100 == 0) {
			std::cout << "Simulation at time " << sim.GetSimTime() << " " <<
				wall_time << " " << wall_time / sim.GetSimTime() << std::endl;
		}
#endif
	}
	if (myid == 0)
		std::cout << "Simulation completed successfully" << std::endl;

#ifdef USE_MPI
	MPI_Finalize();
#endif  
	return 0;
}
