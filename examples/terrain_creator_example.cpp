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
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "sensors/mavs_sensors.h"
#include "mavs_core/terrain_generator/terrain_elevation_functions.h"
#include <raytracers/embree_tracer/embree_tracer.h>

int main(int argc, char* argv[]) {



	mavs::MavsDataPath mdp;
	std::string mavs_data_path = mdp.GetPath();

	mavs::terraingen::TerrainCreator terrain_creator;

	float top_width = 12.0f; // GetFloatParam("terrain_feature.top_width", 12.0f);
	float ditch_width = 6.0f; // GetFloatParam("terrain_feature.bottom_width", 6.0f);
	float ditch_depth = 2.0f; // GetFloatParam("terrain_feature.depth", 2.0f);
	float ditch_location = 20.0f; // GetFloatParam("terrain_feature.location", 20.0f);
	//terrain_creator_.AddTrapezoid(6.0f, 12.0f, 2.0f, 20.0f);
	terrain_creator.AddTrapezoid(ditch_width, top_width, ditch_depth, ditch_location);

	mavs::terraingen::VegPolygon veg_poly_right, veg_poly_left;
	veg_poly_right.polygon.push_back(glm::vec2(-20.0f, -20.0f));
	veg_poly_right.polygon.push_back(glm::vec2(-20.0f, -10.0f));
	veg_poly_right.polygon.push_back(glm::vec2(195.0f, -10.0f));
	veg_poly_right.polygon.push_back(glm::vec2(195.0f, -20.0f));
	veg_poly_right.number = 75;
	veg_poly_right.scale_low = 0.25f;
	veg_poly_right.scale_high = 0.5f;
	veg_poly_right.meshfile = mavs_data_path + "/scenes/meshes/vegetation/pine_tree/pine_tree.obj";
	terrain_creator.AddMeshes(veg_poly_right);
	veg_poly_left.polygon.push_back(glm::vec2(-20.0f, 10.0f));
	veg_poly_left.polygon.push_back(glm::vec2(-20.0f, 20.0f));
	veg_poly_left.polygon.push_back(glm::vec2(195.0f, 20.0f));
	veg_poly_left.polygon.push_back(glm::vec2(195.0f, 10.0f));
	veg_poly_left.number = 75;
	veg_poly_left.scale_low = 0.25f;
	veg_poly_left.scale_high = 0.5f;
	veg_poly_left.meshfile = mavs_data_path + "/scenes/meshes/vegetation/pine_tree/pine_tree.obj";
	terrain_creator.AddMeshes(veg_poly_left);

	terrain_creator.CreateTerrain(-25.0f, -25.0f, 200.0f, 25.0f, 0.5f);
	mavs::raytracer::embree::EmbreeTracer* scene_ptr = terrain_creator.GetScenePointer();
	scene_ptr->TurnOffLabeling();

	mavs::environment::Environment env;
	env.SetRaytracer(scene_ptr);

	glm::dvec3 position(0.0, 0.0, 1.0);
	glm::dquat orientation(1.0, 0.0, 0.0, 0.0);
	mavs::sensor::camera::SimpleCamera free_camera;
	free_camera.FreePose();
	free_camera.SetPose(position, orientation);
	free_camera.Update(&env, 0.1);
	free_camera.Display();

	while (free_camera.DisplayOpen()) {

		free_camera.Update(&env, 0.1);

		free_camera.Display();

	}

	return 0;
}


