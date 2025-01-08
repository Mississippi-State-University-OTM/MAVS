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
* \file utest_mavs_pathtrace.cpp
*
* Unit test to evaluate the mavs pathtracer camera model model
*
* Usage: >./utest_mavs_pathtrace camera_res
*
* camera_res is an optional argument to specify the resolution of the camera
*
* \author Chris Goodin
*
* \date 2/3/2020
*/
#include <iostream>
#include <raytracers/simple_tracer/simple_tracer.h>
#include <sensors/camera/path_tracer.h>
#ifdef USE_OMP
#include <omp.h>
#endif

mavs::raytracer::SimpleTracer CreateSphereScene() {
	mavs::raytracer::SimpleTracer scene;
	glm::vec3 red(0.7f, 0.15f, 0.15f);
	glm::vec3 green(0.25f, 1.0f, 0.25f);
	glm::vec3 yellow(0.9f, 0.9f, 0.1f);
	glm::vec3 blue(0.1f, 0.1f, 0.85f);
	mavs::Material shiny_red, flat_blue, flat_yellow, ground;
	shiny_red.kd = red;
	shiny_red.ks = glm::vec3(0.3f, 0.3f, 0.3f);
	shiny_red.ns = 30.0f;
	shiny_red.ni = 1.5f;
	flat_blue.kd = blue;
	flat_blue.ks = glm::vec3(0.0f, 0.0f, 0.0f);
	flat_blue.ns = 1.0f;
	flat_blue.ni = 1.75f;
	flat_yellow.kd = yellow;
	flat_yellow.ks = glm::vec3(0.0f, 0.0f, 0.0f);
	flat_yellow.ns = 1.0f;
	flat_yellow.ni = 5.0f;
	ground.kd = green;
	ground.ks = glm::vec3(0.0f, 0.0f, 0.0f);
	ground.ns = 1.0f;
	ground.ni = 1000.0f;

	mavs::raytracer::Sphere sred, syellow, sblue;
	sred.SetPosition(1.0f, -2.0f, 10.0f);
	sred.SetMaterial(shiny_red);
	sred.SetRadius(3.0f);
	sred.SetColor(red.x, red.y, red.z);
	scene.AddPrimitive(sred);

	syellow.SetPosition(0.0f, 4.0f, 3.0f);
	syellow.SetRadius(7.0f);
	syellow.SetColor(yellow.x, yellow.y, yellow.z);
	syellow.SetMaterial(flat_yellow);
	scene.AddPrimitive(syellow);

	sblue.SetPosition(-2.0f, 0.0f, 6.0f);
	sblue.SetRadius(2.0f);
	sblue.SetColor(blue.x, blue.y, blue.z);
	sblue.SetMaterial(flat_blue);
	scene.AddPrimitive(sblue);

	mavs::raytracer::Aabb floor;
	floor.SetSize(1.0E6f, 1.0E6f, 0.01f);
	floor.SetColor(green.x, green.y, green.z);
	floor.SetPosition(0.0f, 0.0f, 5.0f);
	floor.SetMaterial(ground);
	scene.AddPrimitive(floor);

	return scene;
}

int main(int argc, char *argv[]) {

	mavs::raytracer::SimpleTracer scene = CreateSphereScene();

	int pix = 384;
	if (argc > 1) {
		pix = atoi(argv[1]);
	}
	int num_samples = 500;
	int ray_depth = 15;
	float rr_val = 0.55f;

	mavs::sensor::camera::PathTracerCamera hd_cam;
	hd_cam.Initialize(pix, pix, 0.025f, 0.025f, 0.035f);
	hd_cam.SetNumIterations(num_samples);
	hd_cam.SetMaxDepth(ray_depth);
	hd_cam.SetRRVal(rr_val);
	hd_cam.SetExposureTime(1.0f / 500.0f);
	hd_cam.TurnOnPixelSmoothing();

	mavs::environment::Environment env;
	env.SetRaytracer(&scene);

	glm::dvec3 position(-25.0f, 0.0, 5.0f);
	glm::dquat orientation(1.0, 0.0, 0.0, 0.0);

#ifdef USE_OMP
	double t1 = omp_get_wtime();
#endif

	hd_cam.SetPose(position, orientation);
	hd_cam.Update(&env, 0.03f);

#ifdef USE_OMP
	std::cout << "Render time = " << omp_get_wtime() - t1 << std::endl;
#endif    

	hd_cam.Display();
	hd_cam.SaveImage("pathtrace_utest_output.bmp");

	return 0;
}

