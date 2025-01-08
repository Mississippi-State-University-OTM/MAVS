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
* \file utest_mavs_snow.cpp
*
* Unit test to evaluate the mavs snow renderer
*
* Usage: >./utest_mavs_snow
*
* \author Chris Goodin
*
* \date 6/7/2019
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <mavs_core/environment/environment.h>
#include <raytracers/simple_tracer/simple_tracer.h>
#include <sensors/camera/rgb_camera.h>
#include <iostream>

int main(int argc, char *argv[]){
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	float snow_rate = 10.0f;
	if (argc > 1)snow_rate = (float)atof(argv[1]);

	mavs::environment::Environment env;
	env.SetTemperature(-10.0f);
	env.SetSnowRate(snow_rate);
	env.SetCloudCover(0.65f);
	env.SetTurbidity(8.0f);
	env.SetSnowAccumulation(1.0f);

	mavs::raytracer::SimpleTracer scene;
	scene.CreateTestScene();
	//scene.CreateForest();

	env.SetRaytracer(&scene);

	mavs::sensor::camera::RgbCamera cam;

	cam.SetEnvironmentProperties(&env);

	//cam.SetPose(glm::vec3(-35.0f, 0.0f, 1.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	cam.SetPose(glm::vec3(-35.0f, 0.0f, 10.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

	float dt = 0.03f;
	while (true) {
		env.AdvanceTime(dt);
		cam.Update(&env, dt);
		if (myid==0)cam.Display();
	}
	//cam.SaveImage("snowing.bmp");
#ifdef USE_MPI
	MPI_Finalize();
#endif
  return 0;
}
