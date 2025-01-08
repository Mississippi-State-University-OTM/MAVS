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
* \file scene_viewer.cpp
*
* View a scene and move the camera through it with the arrow keys.
*
* Usage: >./scene_viewer scene_file.json (seed) (nir) (timers)
*
* scene_file.json is a MAVS scene file, examples found
* in mavs/data/scenes.
* 
* seed is an optional integer argument. If positive, this will seed
* any randomly placed objects in the scene
*
* nir is an optional integer argument. If it is greater than 0, 
* a near-infrared camera will be used to view the scene.
*
* timers is an optional argument. If set >0, then the frame
* rate will be printed to the terminal
*
* Fly the camera around the scene with the W-A-S-D keys.
* Page Up & Page Down to move up and down.
* Arrow keys rotate the view (left,right,up,down)
* Home and End keys rotate the roll of the camera
* Close the view window to finish the program.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/

#include <iostream>
#include <mavs_core/math/utils.h>
#include <sensors/camera/simple_camera.h>
#include <sensors/camera/camera_models.h>
#include <sensors/camera/nir_camera.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif
#include <omp.h>

int main(int argc, char *argv[]) {
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI  
	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
#endif

	if (argc<2) {
		std::cerr << "Usage: ./scene_viewer scenefile.json (seed) (use_nir) (timers)" << std::endl;
		return 1;
	}

	int seed = -1;
	if (argc>2) {
		int ss = atoi(argv[2]);
		if (ss > 0) seed = ss;
	}
	bool use_nir = false;
	if (argc>3) {
		int nir = atoi(argv[3]);
		if (nir > 0) use_nir = true;
	}
	bool use_timers = false;
	if (argc>4) {
		int timers = atoi(argv[4]);
		if (timers > 0) use_timers = true;
	}

#ifdef USE_EMBREE
	mavs::environment::Environment env;
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	if (seed > 0) {
		scene.Load(scene_file, (unsigned int) seed);
	}
	else {
		scene.Load(scene_file);
	}
	env.SetRaytracer(&scene);

	glm::dvec3 position(0.0, 0.0, 1.0);
	//glm::dvec3 position(25.0, 25.0, 1.0);
	//glm::dvec3 position(-25.0, -25.0, 1.0);
	glm::dquat orientation(1.0, 0.0, 0.0, 0.0);
	//glm::dquat orientation(0.9238795, 0.0, 0.0, 0.38268);
	//glm::dquat orientation(0.38268, 0.0, 0.0, -0.9238795);

	mavs::sensor::camera::HD1080 hd_cam;
	hd_cam.SetEnvironmentProperties(&env);

	if (use_nir) {
		mavs::sensor::camera::NirCamera nir_camera;
		nir_camera.FreePose();
		nir_camera.SetPose(position, orientation);
		nir_camera.Update(&env, 0.1);
		if (myid==0)nir_camera.Display();
		//while (nir_camera.DisplayOpen()) {
		while (true) {
			nir_camera.Update(&env, 0.1);
			if(myid==0)nir_camera.Display();
		}
	}
	else {
		mavs::sensor::camera::SimpleCamera free_camera;
		free_camera.FreePose();
		free_camera.SetPose(position, orientation);
		free_camera.Update(&env, 0.1);
		if (myid==0) free_camera.Display();
#ifdef USE_MPI
		while (true) {
#else
		while(free_camera.DisplayOpen()){
#endif
#ifdef USE_OMP
			double t0 = omp_get_wtime();
#endif
			free_camera.Update(&env, 0.1);
#ifdef USE_OMP
			double t1 = omp_get_wtime();
			if (use_timers) {
				std::cout << "Frame rate = " << (1.0 / (t1 - t0)) << std::endl;
			}
#endif
			if (myid == 0) {
				free_camera.Display();
				if (free_camera.GrabFrame()) {
					mavs::Pose currpose = free_camera.GetPose();
					hd_cam.SetPose(currpose.position, currpose.quaternion);
					hd_cam.Update(&env, 0.1);
					std::string ofname = mavs::utils::GetSaveFileName();
					hd_cam.SaveImage(ofname);
					free_camera.UnsetFrameGrab();
				}
				if (free_camera.GetDisplay()->is_keyP()) {
					mavs::Pose currpose = free_camera.GetPose();
					std::cout << currpose.position.x << " " << currpose.position.y << " " << currpose.position.z << std::endl;
				}
			}
		}
	}

#endif

#ifdef USE_MPI    
	MPI_Finalize();
#endif
	return 0;
}


