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
* \file utest_mavs_animation.cpp
*
* Unit test to evaluate mavs animations
*
* Usage: >./utest_mavs_animation 
*
* Ctrl+C to stop the simulation
*
* \author Chris Goodin
*
* \date 7/29/2018
*/
#ifdef USE_EMBREE
#include <iostream>
#include <sensors/mavs_sensors.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include <mavs_core/data_path.h>

int main (int argc, char *argv[]){
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	mavs::sensor::camera::RgbCamera camera;
	camera.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	mavs::sensor::lidar::MEight lidar;
	mavs::environment::Environment env;

	mavs::MavsDataPath mavs_data_path;
	std::string data_path = mavs_data_path.GetPath();
	std::string scene_file(data_path+"/scenes/cube_scene.json");
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	scene.TurnOffLabeling();
	scene.TurnOffSpectral();
	scene.TurnOnSurfaceTextures();
	env.SetRaytracer(&scene);

	mavs::raytracer::Animation anim;
	anim.SetPathToMeshes(data_path + "/scenes/meshes/animations/GenericWalk");
	anim.LoadFrameList(data_path + "/scenes/meshes/animations/GenericWalk/walk_frames.txt");
	anim.SetBehavior("wander");
	anim.SetFrameRate(30.0f);
	anim.SetMeshScale(0.01f);
	anim.SetSpeed(2.0f);
	int anim_id = scene.AddAnimation(anim);
	env.SetAnimationPosition(anim_id, 0.0f, 0.0f, 0.0f);

	//glm::vec3 position(-200.0, -25.0, 100.0);
	//glm::quat orientation(1.0, 0.0, 0.0, 0.0);
	glm::vec3 position(-10.0f, 0.0f, 2.0f);
	glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);

	float dt = 1.0f / 10.0f; // 30.0f;
	while(true){
		env.AdvanceTime(dt);
		glm::vec3 anim_pos = env.GetAnimationPosition(0);
		glm::vec3 lt = anim_pos - position;
		lt = lt / glm::length(lt);
		float theta = atan2(lt.y, lt.x);
		orientation.w = (float)cos(theta*0.5f);
		orientation.z = (float)sin(theta*0.5f);
		camera.SetPose(position, orientation);
		camera.Update(&env, 0.1);
		if (myid==0)camera.Display();
		lidar.SetPose(position, orientation);
		lidar.Update(&env, 0.1);
		if (myid==0)lidar.Display();
	}
#ifdef USE_MPI
	MPI_Finalize();
#endif
  return 0;
}

#endif
