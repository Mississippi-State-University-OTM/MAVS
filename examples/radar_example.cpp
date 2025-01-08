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
* \file radar_example.cpp
*
* Demonstrates a MAVS radar simulation.
*
* Usage: >./radar_example mavs_scene_file.json 
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
*
* The simulation will save two files, camera.bmp and radar.bmp
* camera.bmp shows the view from the position of the radar.
* radar.bmp shows the radar output with detected targets in yellow.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <sensors/radar/radar.h>
#include <sensors/camera/rgb_camera.h>

#include <iostream>

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

#ifdef USE_EMBREE
	if (argc < 2) {
		std::cerr << "Usage: ./radar_example scenefile.json " << std::endl;
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
	box.SetSize(1.0E6f, 1.0E6f, 0.01f);
	box.SetColor(0.25f, 1.0f, 0.25f);
	box.SetPosition(0.0f, 0.0f, 5.0f);
	scene.AddPrimitive(box);
	box2.SetPosition(0.0f, 4.0f, 3.0f);
	box2.SetSize(10.0f, 10.0f, 2.0f);
	box2.SetColor(0.7f, 0.7f, 0.25f);
	scene.AddPrimitive(box2);
#endif

	env.SetRaytracer(&scene); 

	mavs::sensor::radar::Radar radar;
	float hfov_deg = 60.0f;
	float vfov_deg = 1.0f; // 2.5f;
	float samp_res_deg = 0.5f;
	radar.Initialize(hfov_deg, vfov_deg, samp_res_deg);
	radar.SetMaxRange(250.0f);
	radar.SetSampleResolution(0.1f);

	mavs::sensor::camera::RgbCamera camera;
	camera.Initialize(640, 360, 0.006222222f, 0.0035f, 0.0035f);

	glm::dvec3 position(-10.0, 0.0, 1.0);
	glm::dquat orientation(1.0, 0.0, 0.0, 0.0);

#ifdef USE_MPI  
	radar.SetComm(MPI_COMM_WORLD);
	MPI_Barrier(MPI_COMM_WORLD);
	double t1 = MPI_Wtime();
#endif
	if (myid == 0)std::cout << "Done loading, scanning" << std::endl;

	radar.SetPose(position, orientation);
	camera.SetPose(position, orientation);
	radar.Update(&env, 0.1f);
	camera.Update(&env, 0.1f);
	camera.FreePose();
	camera.Display();
	int ndetected = 0;
	while (camera.DisplayOpen()) {
		mavs::Pose currpose = camera.GetPose();
		radar.SetPose(currpose.position, currpose.quaternion);
		radar.Update(&env, 0.1f);
		camera.Update(&env, 0.1f);
		if (myid == 0) {
			radar.Display();
			camera.Display();
			std::vector<mavs::RadarTarget> targets =  radar.GetDetectedTargets();
			
			if ((int)targets.size() != ndetected) {
				std::cout << "Detected " << targets.size() << " targets." << std::endl;
				for (int tj = 0; tj < (int)targets.size(); tj++) {
					std::cout << tj << " " << targets[tj].position_x << " " << targets[tj].position_y << " " << targets[tj].width << std::endl;
				}
				ndetected = (int)targets.size();
			}
			
			//radar.AnnotateFrame(&env, true);
			//radar.SaveAnnotation();
			//radar.SaveImage("radar.bmp");
			//camera.SaveImage("camera.bmp");
		}
	}

#ifdef USE_MPI 
	MPI_Finalize();
#endif
	return 0;
}

