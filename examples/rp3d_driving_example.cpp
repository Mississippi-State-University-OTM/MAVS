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
* \file rp3d_driving_example.cpp
*
* Drive a MAVS vehicle implemented in RP3D
*
* Usage: >./rp3d_driving_example mavs_scene_file.json rp3d_vehicle_file.json surface_type cone_index pose_log_freq
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
#include <tinyfiledialogs.h>
#ifdef USE_OMP
#include <omp.h>
#endif
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif

int main(int argc, char *argv[]) {
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif
#ifdef USE_EMBREE
	if (argc < 2) {
		std::cerr << "ERROR, must provide scene file and vehicle file as arguments" << std::endl;
		return 1;
	}
	std::string scene_file(argv[1]);
	std::string vehic_file(argv[2]);

	std::string surface_type = "dry"; // "clay"; // "wet"; // "sand";
	float RCI = 6894.76*250.0f;
	if (argc > 3) {
		surface_type = std::string(argv[3]);
	}
	if (argc > 4) {
		RCI = 6894.76f*(float)atof(argv[4]);
	}
	float pose_log_freq = 0.0f;
	if (argc > 5) {
		pose_log_freq = (float)atof(argv[5]);
	}
	std::cout << "Scene File = " << scene_file << std::endl;
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	std::cout << "Loaded scene with " << scene.GetNumberTrianglesLoaded() << " triangles. " << std::endl;
	std::cout << "Vehicle file = " << vehic_file << std::endl;
	std::cout << "Surface type and strength = " << surface_type << " " << RCI/ 6894.76f << std::endl;
#else
	std::cerr << "The Driving Example can only be run with Embree enabled" << std::endl;
	return 0;
#endif

	mavs::environment::Environment env;
	env.SetGlobalSurfaceProperties(surface_type, RCI);
	env.SetTurbidity(9.0);
	env.SetFog(0.0f);
	env.SetDateTime(2023, 9, 1, 8, 0, 0, 6);
	env.SetRaytracer(&scene);

	glm::vec3 sensor_offset(-10.0f, 0.0f, 1.5f);
	glm::quat sensor_orient(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 position(0.0f, 0.0f, 1.0f);
	glm::quat orient(1.0f, 0.0f, 0.0f, 0.0f);
	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
	camera.SetRelativePose(sensor_offset, sensor_orient);
	camera.SetName("camera");
	camera.SetAntiAliasing("oversampled");
	camera.SetPixelSampleFactor(3);
	camera.SetPose(position, orient);
	camera.SetElectronics(0.65f, 1.0f);
	
	mavs::vehicle::Rp3dVehicle veh;
	veh.Load(vehic_file);
	float zstart = scene.GetSurfaceHeight(0.0f, 0.0f);
	veh.SetPosition(0.0f,0.0f, zstart + 0.25f);
	veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
	//veh.SetPosition(20.0f, -20.0f, 2.5f);
	//veh.SetOrientation(0.7071f, 0.0f, 0.0f, 0.7071f);
	float current_time = 0.0f;
	float dt = 1.0f / 100.0f;
	mavs::Waypoints logged_wp;
	int pose_log_steps = (int)ceil((1.0f/ dt)/pose_log_freq);
	bool logging = false;
	bool save_images = false;
	std::string image_output_folder;
	int nsteps = 0;
	std::vector<bool> driving_commands;

	//if (myid == 0)camera.Display();
	while(camera.DisplayOpen() || nsteps==0){
#ifdef USE_OMP
		double t1 = omp_get_wtime();
#endif
		current_time += dt;

		double throttle = 0.0;
		double steering = 0.0;
		double braking = 0.0;
		driving_commands = camera.GetKeyCommands();
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

		if (camera.GetDisplay()->is_keyL()) {
			logging = !logging;
			if (logging) { std::cout << "Logging on. " << std::endl; }
			else { std::cout << "Logging off. " << std::endl; }
		}
		if (camera.GetDisplay()->is_keyP()) {
			logged_wp.SaveAsVprp("logged.vprp");
			break;
		}
	
		if (camera.GetDisplay()->is_keyR()) {
			save_images = !save_images;
			if (save_images && image_output_folder.size() == 0) {
				char const * folder_name;
				folder_name = tinyfd_selectFolderDialog("Select folder to save", "~/Desktop");
				if (folder_name) {
					image_output_folder = std::string(folder_name);
				}
			}
		}

		veh.Update(&env, (float)throttle, (float)steering, (float)braking, dt);
		
		mavs::VehicleState veh_state = veh.GetState();

		if (std::isinf(veh_state.pose.position.x) || std::isnan(veh_state.pose.position.x)) break;

		glm::vec3 look_to = veh.GetLookTo();
		float heading = 0.5f*(atan2f(look_to.y, look_to.x));
		camera.SetPose(veh.GetPosition(), glm::quat(cosf(heading), 0.0f, 0.0f, sinf(heading)));

		if (nsteps%pose_log_steps == 0 && logging) {
			logged_wp.AddPoint(glm::vec2(veh_state.pose.position.x, veh_state.pose.position.y));
		}

		if (nsteps%4==0)camera.Update(&env, 4*dt);
		if (myid == 0) {
			camera.Display();
			if (save_images) {
				std::string imname = image_output_folder + "/frame_" + mavs::utils::ToString(nsteps, 5) + "_img.bmp";
				camera.SaveImage(imname);
			}
		}
		nsteps++;

#ifdef USE_OMP
		double dwall = omp_get_wtime() - t1;
		if (dwall < dt) {
			int msleep = (int)(1000 * (dt - dwall));
			mavs::utils::sleep_milliseconds(msleep);
		}
#endif
	}

#ifdef USE_MPI
	MPI_Finalize();
#endif
	return 0;
}
