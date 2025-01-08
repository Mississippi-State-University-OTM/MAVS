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
* \file utest_mavs_lidar.cpp
*
* Unit test to evaluate the mavs lidar sensor
*
* Usage: >./utest_mavs_lidar
*
* If working correctly, should show a simple scene
* with a red sphere, a green floor, and a yellow box. Smoke will rise
* up from the floor in front of the sphere. A top-down rendering of 
* lidar scan will appear in a second window.
*
* Press Ctrl+C to kill the simulation.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <iostream>

#include <raytracers/simple_tracer/simple_tracer.h>
//#include "sensors/lidar/lidar.h"
#include <sensors/lidar/vlp16.h>
#include <sensors/lidar/os2.h>
#include <sensors/camera/rgb_camera.h>
#include "mavs_core/environment/particle_system/particle_system.h"

#ifdef USE_OMP
#include <omp.h>
#endif

int main (int argc, char *argv[]){
  int myid = 0;
  int numprocs = 1;
#ifdef USE_MPI 
  int ierr = MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD,&numprocs);
  MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif
  
  mavs::raytracer::SimpleTracer scene;
	scene.CreateTestScene();
	/*mavs::Material shiny_red;
	shiny_red.kd = glm::vec3(0.7, 0.15, 0.15);
	shiny_red.ks = glm::vec3(0.3, 0.3, 0.3);
	shiny_red.ns = 30.0;
	mavs::raytracer::Aabb box;
	box.SetSize(0.01f, 1.0E6f, 1.0E6f);
	box.SetColor(0.25f, 1.0f, 0.25f);
	//box.SetPosition(10.0f, 0.0f, 0.0f);
	box.SetMaterial(shiny_red);
	scene.AddPrimitive(box);*/

  mavs::environment::Environment env;
  env.SetRaytracer(&scene);

	//add a particle system to the scene
	mavs::environment::ParticleSystem smoke_psys;
	smoke_psys.SetLifetime(3.0f);
	smoke_psys.SetInitialVelocity(0.0f, 0.0f, 5.0f);
	smoke_psys.SetGravityFactor(0.1f);
	smoke_psys.SetExpansionRate(0.5f);
	smoke_psys.SetInitialRadius(0.5f);
	smoke_psys.SetTransparency(0.5f);
	smoke_psys.SetVelocityRandomization(1.5f, 1.5f, 0.5f);
	smoke_psys.SetInitialColor(0.75f, 0.75f, 0.75f);
	glm::vec3 smoke_center(-5.0f, -2.0f, 5.0f);
	float smoke_radius = 3.0f;
	float smoke_rate = 100.0f;
	smoke_psys.SetSource(smoke_center, smoke_radius, smoke_rate);
	env.AddParticleSystem(smoke_psys);

  glm::vec3 position(-25.0f, 0.0f, 10.0f);
  glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);

  //mavs::sensor::lidar::Lidar *lidar = new mavs::sensor::lidar::Vlp16;
	//mavs::sensor::lidar::Lidar *lidar = new mavs::sensor::lidar::OusterOS2;
	mavs::sensor::camera::RgbCamera camera;
	mavs::sensor::lidar::Lidar *lidar = new mavs::sensor::lidar::Vlp16;
	lidar->SetHorizontalBlankingRangeDegrees(-180.0f, -90.0f);
#ifdef USE_MPI  
  lidar->SetComm(MPI_COMM_WORLD);
	camera.SetComm(MPI_COMM_WORLD);
#endif

	
	//float hres = 360.0f / 2048.0f;
	//float vres = 90.0f / 512.0f;
  //lidar->SetScanPattern(-180.0f, 180.0f-hres, hres, -45.0f, 45.0f-vres, vres);

	float dt = 0.1f;
	while (true) {
	//for (int tt=0;tt<1;tt++){
#ifdef USE_MPI  
		double t1 = MPI_Wtime();
#elif USE_OMP
		double t1 = omp_get_wtime();
#endif  
		env.AdvanceTime(dt);
		camera.SetPose(position, orientation);
		camera.Update(&env, dt);
		camera.Display();
		lidar->SetPose(position, orientation);
		lidar->SetVelocity(1.0f, 0.0f, 0.0f);
		lidar->Update(&env, dt);
		//mavs::PointCloud cloud = lidar.GetRosPointCloud();
		lidar->Display();
		if (myid == 0) {
#ifdef USE_MPI    
			std::cout << "Scan time = " << MPI_Wtime() - t1 << std::endl;
#elif USE_OMP
			std::cout << "Scan time = " << omp_get_wtime() - t1 << std::endl;
#endif
		}
	}
	if (myid == 0) {
		lidar->WritePointsToText("lidar_points.txt");
		camera.SaveImage("utest_lidar_image.bmp");
		lidar->WriteProjectedLidarImage("lidar_projected.bmp");
	}
#ifdef USE_MPI  
  MPI_Finalize();
#endif
  
  return 0;
}

