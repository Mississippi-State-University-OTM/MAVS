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
* \file path_tracer_render.cpp
*
* Example of using the path tracer to render the scene
*
* Usage: >./path_tracer_render scene_file.json (num_samples) (ray_depth) (rr_val) (sky_on)
*
* scene_file.json is a MAVS scene file, examples found in mavs/data/scenes.
*
* num_samples is an optional integer argument that specifies how many rays to shoot per pixel.
* 
* ray_depth is an optional integer argument that specifies the maximum number of times a ray can be reflected.
* 
* rr_val is the russian-roulette cutoff paramter for ray depth
* 
* sky_on turns off the sky if less than 1, turns it on if greater than 1
*
* Fly the camera around the scene with the W-A-S-D keys.
* Page Up & Page Down to move up and down.
* Arrow keys rotate the view (left,right,up,down)
* Home and End keys rotate the roll of the camera
* Close the view window to finish the program.
* Press "R" to render an image from the current pose with the path tracer.
*
* \author Chris Goodin
*
* \date 10/31/2019
*/
#ifdef USE_OMP
#include <omp.h>
#endif
#include <iostream>
#include <sensors/camera/path_tracer.h>
#include <sensors/camera/simple_camera.h>
#include <mavs_core/math/utils.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif

int main(int argc, char *argv[]) {

#ifndef USE_EMBREE
	std::cerr << "This example only works with embree enabled" << std::endl;
	return 1;
#endif

	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	if (argc < 2) {
		std::cerr << "Usage: ./scene_viewer scenefile.json (num_samples) (ray_depth)" << std::endl;
		return 1;
	}

	int num_samples = 10;
	if (argc > 2) {
		num_samples = atoi(argv[2]);
	}

	int ray_depth = 5;
	if (argc > 3) {
		ray_depth = atoi(argv[3]);
	}

	float rr_val = 0.66f;
	if (argc > 4) {
		rr_val = (float)atof(argv[4]);
	}

	int sky_on = 1;
	if (argc > 5) {
		sky_on = atoi(argv[5]);
	}

	mavs::environment::Environment env;
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);

	env.SetRaytracer(&scene);
	if (sky_on < 1) {
		env.TurnOffSky();
	}
	else {
		env.TurnOnSky();
	}
	// turn on Rain
	//env.SetRainRate(5.0f);
	//env.SetWind(1.0f, 1.0f);

	// Add lights to the environment
	/*
	mavs::environment::Light left_headlight, right_headlight;
	left_headlight.position = glm::vec3(1.5f, 0.68f, 0.85f);
	left_headlight.color = glm::vec3(255.0f, 255.0f, 192.0f);
	left_headlight.decay = 1.5f;
	left_headlight.direction = glm::vec3(0.7071f, 0.0f, 0.7071f);
	left_headlight.type = 2; //spotlight
	left_headlight.radius = 0.1f;
	left_headlight.angle = 45.0f*(float)mavs::kDegToRad;
	right_headlight = left_headlight;
	right_headlight.position = glm::vec3(1.5f, -0.68f, 0.85f);
	env.SetDateTime(2019, 11, 8, 2, 0, 0, 6);
	env.AddLight(left_headlight);
	env.AddLight(right_headlight);
	*/

	mavs::environment::Light spotlight;
	spotlight.position = glm::vec3(0.4f, -3.0f, 0.8f);
	//spotlight.color = glm::vec3(255.0f, 255.0f, 192.0f);
	//spotlight.color = glm::vec3(64.0f, 64.0f, 48.0f);
	spotlight.color = glm::vec3(25.0f, 25.0f, 25.0f);
	//spotlight.color = glm::vec3(1.0f, 1.0f, 0.75f);
	spotlight.decay = 0.5f; // 1.5f;
	spotlight.direction = glm::vec3(0.0f, 1.0f, 0.0f);
	spotlight.type = 2; //spotlight
	spotlight.radius = 0.5f; //0.1f;
	spotlight.angle = 15.0f*(float)mavs::kDegToRad;
	env.AddLight(spotlight);

	glm::dvec3 position(0.0f, -3.7f, 1.0f);
	glm::dquat orientation(0.7071f, 0.0f, 0.0f, 0.7071f);

	mavs::sensor::camera::PathTracerCamera hd_cam;
	//hd_cam.Initialize(128, 128, 0.025f, 0.025f, 0.035f);
	//hd_cam.Initialize(384, 384, 0.025f, 0.025f, 0.035f);
	hd_cam.Initialize(512, 512, 0.025f, 0.025f, 0.035f);
	//hd_cam.Initialize(1024, 1024, 0.025f, 0.025f, 0.035f);
	//hd_cam.Initialize(810, 540, 0.0225f, 0.015f, 0.009f);
	//hd_cam.Initialize(1620, 1080, 0.0225f, 0.015f, 0.009f);

	hd_cam.SetNumIterations(num_samples);
	hd_cam.SetMaxDepth(ray_depth);
	hd_cam.SetRRVal(rr_val);
	hd_cam.SetExposureTime(1.0f / 500.0f);
	hd_cam.TurnOffPixelSmoothing();
	//hd_cam.SetGamma(2.0f);

	mavs::sensor::camera::SimpleCamera free_camera;
	free_camera.Initialize(384, 384, 0.025f, 0.025f, 0.035f);
	free_camera.FreePose();
	free_camera.SetPose(position, orientation);
	free_camera.Update(&env, 0.1);
	if (myid==0)free_camera.Display();

	while (free_camera.DisplayOpen()) {

		free_camera.Update(&env, 0.1);
		if (myid==0)free_camera.Display();

		if (free_camera.GrabFrame()) {
			mavs::Pose currpose = free_camera.GetPose();
			hd_cam.SetPose(currpose.position, currpose.quaternion);
#ifdef USE_OMP
			double t1 = omp_get_wtime();
#endif
			hd_cam.Update(&env, 0.1);
#ifdef USE_OMP
			std::cout << (omp_get_wtime() - t1) << std::endl;
#endif
			if (myid == 0) {
				hd_cam.Display();
				std::string ofname = mavs::utils::GetSaveFileName();
				hd_cam.SaveImage(ofname);
				free_camera.UnsetFrameGrab();
			}
		}

	}
#ifdef USE_MPI
	MPI_Finalize();
#endif
	return 0;
}


