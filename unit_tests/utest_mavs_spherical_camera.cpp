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
* \file utest_mavs_spherical_camera.cpp
*
* Unit test to evaluate the mavs spherical camera.
*
* Usage: >./utest_mavs_spherical_camera 
*
* Result is a rendering of a spherical projection of the input scene.
* Two files will be saved, "simple_render.bmp" and "spherical_projection.bmp"
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <iostream>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#else
#include <raytracers/simple_tracer/simple_tracer.h>
#endif

#include "raytracers/material.h"
#include <sensors/camera/spherical_camera.h>
#include <sensors/camera/simple_camera.h>
#include <mavs_core/data_path.h>
#include <mavs_core/terrain_generator/random_scene.h>

#ifdef USE_OMP
#include <omp.h>
#endif

int main(int argc, char *argv[]) {
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	mavs::sensor::camera::SphericalCamera camera;
	mavs::sensor::camera::SimpleCamera simple;
#ifdef USE_MPI  
	camera.SetComm(MPI_COMM_WORLD);
#endif

#ifdef USE_EMBREE
	mavs::MavsDataPath mavs_data_path;
	std::string data_path = mavs_data_path.GetPath();
	std::string scene_file(data_path + "/scenes/cube_scene.json");
	mavs::raytracer::embree::EmbreeTracer scene;
	if (myid == 0)std::cout << "Loading " << scene_file << std::endl;
	scene.Load(scene_file);
#else
	mavs::Material shiny_red;
	shiny_red.kd = glm::vec3(0.7f, 0.15f, 0.15f);
	shiny_red.ks = glm::vec3(0.3f, 0.3f, 0.3f);
	shiny_red.ns = 30.0f;
	mavs::raytracer::SimpleTracer scene;
	mavs::raytracer::Sphere sred;
	sred.SetPosition(1.0f, -2.0f, 10.0f);
	//sred.SetColor(1.0,0.25,0.25);
	sred.SetMaterial(shiny_red);
	sred.SetRadius(5.0f);
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

	mavs::environment::Environment env;

	env.SetRaytracer(&scene);

	glm::vec3 position(-25.0f, 0.0f, 2.0f);
	glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);

#ifdef USE_MPI
	double t1 = MPI_Wtime();
#elif USE_OMP
	double t1 = omp_get_wtime();
#endif

	camera.SetPose(position, orientation);
	camera.Update(&env, 0.1);
	camera.Display();

	simple.SetPose(position, orientation);
	simple.Update(&env, 0.1);
	simple.Display();

	if (myid == 0) {
		camera.SaveImage("spherical_projection.bmp");
		simple.SaveImage("simple_render.bmp");
	}
#ifdef USE_MPI
	MPI_Finalize();
#endif
	return 0;
}

