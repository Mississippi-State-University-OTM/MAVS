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
* \file pathtrace_example.cpp
*
* An example MAVS pathtraced camera sensor
*
* Usage: >./pathrace_example
*
* \author Chris Goodin
*
* \date 3/24/2020
*/
#include <iostream>
#include <sensors/camera/camera_models.h>
#include <mavs_core/terrain_generator/random_scene.h>
#include <mavs_core/data_path.h>
#include "vehicles/controllers/pure_pursuit_controller.h"
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif
#ifdef USE_OMP
#include <omp.h>
#endif

static float DistToGoal(glm::vec3 p,glm::vec2 g) {
	glm::vec2 pp(p.x, p.y);
	float d = glm::length(pp - g);
	return d;
}

static void MoveHeadlights(mavs::environment::Environment* env, mavs::vehicle::Vehicle* veh, float front_offset, float headlight_width, int left_id, int right_id) {
	glm::vec3 veh_lt = veh->GetLookTo();
	glm::vec3 veh_ls = veh->GetLookSide();
	glm::vec3 veh_lu = veh->GetLookUp();
	glm::vec3 veh_pos = veh->GetPosition();
	glm::vec3 left_light_pos = veh_pos + front_offset * veh_lt + 0.5f*headlight_width*veh_ls;
	glm::vec3 right_light_pos = veh_pos + front_offset * veh_lt - 0.5f*headlight_width*veh_ls;
	env->MoveLight(left_id, left_light_pos, veh_lt);
	env->MoveLight(right_id, right_light_pos, veh_lt);
}

glm::ivec2 AddHeadlightsToVehicle(mavs::environment::Environment* env, mavs::vehicle::Vehicle* veh, float front_offset, float headlight_width) {
	mavs::environment::Light headlight;
	headlight.type = 2; //spotlight
	headlight.angle = (float)(mavs::kDegToRad * 30.0f);
	headlight.color = glm::vec3(6.0f, 5.0f, 2.5f);
	headlight.decay = 0.25f;
	glm::ivec2 headlight_ids;
	headlight_ids[0] = env->AddLight(headlight); //left
	headlight_ids[1] = env->AddLight(headlight); //right
	MoveHeadlights(env, veh, front_offset, headlight_width, headlight_ids[0], headlight_ids[1]);
	return headlight_ids;
}

int main (int argc, char *argv[]){
#ifndef USE_EMBREE
	std::cerr << "ERROR: This example is only available when building with Embree turned on." << std::endl;
	return 1;
#endif

  int myid = 0;
  int numprocs = 1;
#ifdef USE_MPI  
  int ierr = MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD,&numprocs);
  MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif
  
	bool display = false;
	int start_frame = 0;
	if (argc > 1) {
		start_frame = atoi(argv[1]);
	}

	//mavs::sensor::camera::MachineVisionPathTraced camera(15,10,0.55f);
	mavs::sensor::camera::HDPathTraced camera(150,10,0.55f);
	//mavs::sensor::camera::MachineVision camera;
#ifdef USE_MPI  
	camera.SetComm(MPI_COMM_WORLD);
#endif
	camera.SetRelativePose(glm::vec3(-7.5f, 0.0f, 0.65f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	camera.SetGain(1.0f);
	camera.SetGamma(0.45f);
//std::cout<<"Creating scene"<<std::endl;
	mavs::terraingen::RandomSceneInputs input;
	input.terrain_width = 100.0f;
	input.terrain_length = 100.0f;
	input.lo_mag = 5.0f;
	input.hi_mag = 0.25f;
	input.mesh_resolution = 1.0; // 0.125f;
	input.trail_width = 6.5f;
	input.wheelbase = 1.8f;
	input.track_width = 0.3f;
	input.path_type = "Ridges"; // std::string(path_type);
	input.basename = "scene"; // std::string(basename);
	input.plant_density = 0.75f; // 0.25f; // 0.55f;
	input.output_directory = ".";
	mavs::MavsDataPath mavs_data_path;
	std::string eco_path = mavs_data_path.GetPath() + "/ecosystem_files/";
	input.eco_file = eco_path + "american_southeast_forest_brushy.json";
	mavs::terraingen::RandomScene random_scene;
	random_scene.SetInputs(input);
	//if (myid == 0) {
	random_scene.Create();
	//}
	//std::cout<<myid<<" done creating scene "<<std::endl;
#ifdef USE_MPI
	MPI_Barrier(MPI_COMM_WORLD);
#endif
	//std::cout<<"Loading scene "<<std::endl;
	std::string scene_file(input.basename);
	scene_file = scene_file + "_scene.json";
	std::string path_file(input.basename);
	path_file = path_file + "_path.vprp";
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	//std::cout<<myid<<" done loading scene "<<std::endl;
	mavs::environment::Environment env;
        env.SetRaytracer(&scene);
	env.SetDateTime(2020, 3, 24, 12, 0, 0, 6);
	env.SetTurbidity(4.0);
	env.SetWind(2.0f, 3.0f);
	float fog_scale = 1.0f*0.025f*5.0f*1.0E-3f;
	bool is_night = false;

	mavs::Waypoints mavs_path;
	mavs_path.CreateFromAnvelVprp(path_file);
	std::vector<glm::vec2> path;
	for (int i = 0; i < mavs_path.NumWaypoints(); i++) {
		glm::vec2 wp = mavs_path.GetWaypoint(i);
		path.push_back(wp);
	}
	mavs::vehicle::PurePursuitController controller;
	controller.SetDesiredPath(path);
	controller.SetDesiredSpeed(5.0f); //# m / s
	controller.SetSteeringParam(2.5f);
	controller.SetWheelbase(3.1f); // # meters
	controller.SetMaxSteering(0.615f);  // # radians

	std::string vehicle_file = mavs_data_path.GetPath() + "/vehicles/rp3d_vehicles/forester_2017_rp3d.json";
	mavs::vehicle::Rp3dVehicle veh;
	veh.Load(vehicle_file);
	veh.SetPosition(path[0].x, path[0].y, scene.GetSurfaceHeight(path[0].x, path[0].y) + 1.0f);
	double heading = atan2(path[1].y - path[0].y, path[1].x - path[0].x);
	veh.SetOrientation((float)cos(0.5*heading), 0.0f, 0.0f, (float)sin(0.5*heading));
	glm::ivec2 hl_id = AddHeadlightsToVehicle(&env, &veh, 2.0f, 2.0f);

	float dt = 0.01f;
	float elapsed_time = 0.0f;
	int i = 0;
	float dist_to_goal = 1000.f;
	//std::cout<<"Startin simulation lop"<<std::endl;
#ifdef USE_MPI
	MPI_Barrier(MPI_COMM_WORLD);
#endif
#ifdef USE_MPI    
    double t_start = MPI_Wtime();
#endif  
	while (elapsed_time<=30.0f && dist_to_goal>3.0f){
		if (elapsed_time > 24) { //# snow
			env.SetDateTime(2020, 3, 24, 18, 0, 0, 6);
			env.SetCloudCover(50.0f);
			env.SetTurbidity(8.0f);
			env.SetRainRate(0.0f);
			env.SetSnowRate(15.0f);
			env.SetFog(fog_scale*10.0f);
			is_night = false;
		}
		else if (elapsed_time > 18) { //#rain
			env.SetDateTime(2020, 3, 24, 18, 0, 0, 6);
			env.SetCloudCover(100.0f);
			env.SetTurbidity(9.0f);
			env.SetRainRate(5.0f);
			env.SetSnowRate(0.0f);
			env.SetFog(fog_scale*10.0f);
			is_night = false;
		}
		else if (elapsed_time > 12) { //# fog
			env.SetDateTime(2020, 3, 24, 12, 0, 0, 6);
			env.SetCloudCover(0.0f);
			env.SetTurbidity(7.0f);
			env.SetRainRate(0.0f);
			env.SetSnowRate(0.0f);
			env.SetFog(fog_scale*65.0f);
			is_night = false;
		}
		else if (elapsed_time > 6) { //# night
			env.SetDateTime(2020, 3, 24, 3, 0, 0, 6);
			env.SetCloudCover(00.0f);
			env.SetTurbidity(2.0f);
			env.SetRainRate(0.0f);
			env.SetSnowRate(0.0f);
			env.SetFog(fog_scale*0.0f);
			is_night = true;
		}

#ifdef USE_MPI    
    double t1 = MPI_Wtime();
#endif    
#ifdef USE_OMP
		double t1 = omp_get_wtime();
#endif
		glm::vec3 lt = veh.GetLookTo();
		heading = atan2(lt.y, lt.x);
		mavs::VehicleState state = veh.GetState();
		dist_to_goal = DistToGoal(state.pose.position, path.back());
		float velocity = (float)sqrt(state.twist.linear.x*state.twist.linear.x + state.twist.linear.y*state.twist.linear.y);
		controller.SetVehicleState(veh.GetPosition().x, veh.GetPosition().y, velocity, heading);

		float throttle = 0.0f;
		float steering = 0.0f;
		float braking = 1.0f;
		controller.GetDrivingCommand(throttle, steering, braking, dt);

		veh.Update(&env, throttle, steering, braking, dt);
		env.AdvanceParticleSystems(dt);
		if (i % 10 == 0 && i>=start_frame) {
			env.SetActorPosition(0, state.pose.position, state.pose.quaternion);
			MoveHeadlights(&env, &veh, 2.0f, 2.0f, hl_id[0], hl_id[1]);
			camera.SetPose(state.pose.position, state.pose.quaternion);
			camera.Update(&env, 0.1);
			if (myid == 0) {
#ifdef USE_MPI
				std::cout << "FPS = " << 1.0 / (MPI_Wtime() - t1) << " at " << elapsed_time << std::endl;
#endif
#ifdef USE_OMP
				std::cout << "FPS = " << 1.0 / (omp_get_wtime() - t1) << " at " << elapsed_time << std::endl;
#endif
				if (display)camera.Display();
				std::string fname = "image_" + mavs::utils::ToString(i, 3) + ".bmp";
				camera.SaveImage(fname);
			}
		}
		elapsed_time += dt;
		i++;
  }
#ifdef USE_MPI    
    double t_end = MPI_Wtime();
    std::cout<<"Total simulation time = "<<t_end-t_start<<std::endl;
#endif  
#ifdef USE_MPI
  MPI_Finalize();
#endif
  return 0;
}

