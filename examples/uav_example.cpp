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
* \file uav_example.cpp
*
* Fly a MAVS uav 
*
* Usage: >./uav_example mavs_uav_file.json
*
* mavs_scene_file.json is a MAVS UAV file, examples of which can
* be found in mavs/data/vehicles/uav.
*
* \author Chris Goodin
*
* \date 11/18/24
*/
//#include <iostream>
//#include <stdlib.h>
//#include <mavs_core/math/utils.h>
#include "vehicles/uav/uav_sim.h"
//#include <sensors/mavs_sensors.h>
#ifdef USE_OMP
#include <omp.h>
#endif
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif

int main(int argc, char *argv[]) {
#ifndef USE_EMBREE
	std::cerr << "ERROR, MUST HAVE EMBREE ENABLED TO RUN THIS EXAMPLE! " << std::endl;
	exit(12);
#endif
	if (argc < 2) {
		std::cerr << "ERROR, must provide UAV input file as argument" << std::endl;
		return 1;
	}
	std::string uav_input_file(argv[1]);

	mavs::vehicle::UavSim uav_sim;
	mavs::raytracer::embree::EmbreeTracer scene;
	mavs::environment::Environment env;
	uav_sim.LoadSimulation(uav_input_file, &scene, &env);
	uav_sim.LoadAnimations(&scene, &env);
	uav_sim.SetRenderDebug(true);
	uav_sim.SetControllerActive(false); // fly manually
	
	float dt = 0.01f;
	while (uav_sim.IsActive()) {
		double t0 = omp_get_wtime();
		uav_sim.Update(&env, dt);
		while ((omp_get_wtime() - t0) < dt) {}
	}

	return 0;
}
