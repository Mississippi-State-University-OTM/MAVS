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
* \file create_random_scene.cpp
*
* Create a random scene. Dialogs will ask for input.
*
* Usage: >./create_random_scene
*
* \date 1/25/2021
*/
#include <stdlib.h>     
#include <time.h>
#include <mavs_core/terrain_generator/random_scene.h>
#include <mavs_core/data_path.h>
#include <mavs_core/environment/environment.h>
#include <sensors/camera/rgb_camera.h>
#include <tinyfiledialogs.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif
#include <ctime>
#ifdef Bool
#undef Bool
#endif

int main (int argc, char *argv[]){
#ifndef USE_EMBREE
	std::cerr << "ERROR - This example only works if Embree is built" << std::endl;
	return 0;
#endif
	// seed the random number generator 
	// so we don't get the same surface over and over
	srand(time(NULL));

	const char *width_input = tinyfd_inputBox("Terrain Width","Select terrain width in meters", "100");
	float w = (float)std::atof(width_input);
	const char *length_input = tinyfd_inputBox("Terrain Length", "Select terrain length in meters", "100");
	float l = (float)std::atof(length_input);
	const char *roughness_input = tinyfd_inputBox("Roughness", "Select terrain RMS roughness in meters (0-0.1)", "0.025");
	float hm = (float)std::atof(roughness_input);

	mavs::terraingen::RandomSceneInputs input;
	input.terrain_width = w;
	input.terrain_length = l;
	input.hi_mag = hm;

	std::string basename = "width_" + std::to_string((int)w) + "_length_" + std::to_string((int)l) + "_rough_" + std::to_string((int)(1000*hm));

	input.lo_mag = 0.0f; 
	input.mesh_resolution = 0.15f; 
	input.trail_width = 0.0f; 
	input.wheelbase = 0.0f; 
	input.track_width = 0.0f; 
	input.path_type = std::string("loop");
	input.basename = basename; 
	input.plant_density = 0.0f; 
	input.surface_rough_type = "variable";

	input.output_directory = std::string("./"); 
	mavs::MavsDataPath mavs_data_path;
	std::string eco_path = mavs_data_path.GetPath() + "/ecosystem_files/";
	input.eco_file = eco_path + std::string("american_southeast_forest_brushy.json");

	mavs::terraingen::RandomScene random_scene;
	random_scene.SetInputs(input);
	random_scene.Create();

	std::string scene_file(input.basename);
	scene_file = input.output_directory + "/" + scene_file + "_scene.json";

	std::string path_file(input.basename);
	path_file = path_file + "_path.vprp";

	mavs::raytracer::embree::EmbreeTracer scene; 
	scene.Load(scene_file);

	mavs::environment::Environment env;
	env.SetRaytracer(&scene);

	mavs::sensor::camera::RgbCamera camera;
	camera.FreePose();

	glm::vec3 position(0.0f, 0.0f, 3.0f);
	glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);
	camera.SetPose(position, orientation);
	camera.Update(&env, 0.1);
	camera.Display();

	while (camera.DisplayOpen()) {
		camera.Update(&env, 0.1);
		camera.Display();
	}

  return 0;
}

