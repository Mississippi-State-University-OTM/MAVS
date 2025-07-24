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
* \file camera_soiling_example.cpp
*
* Fly the camera around the scene with the W-A-S-D keys.
* Page Up & Page Down to move up and down.
* Arrow keys rotate the view (left,right,up,down)
* Home and End keys rotate the roll of the camera
* Close the view window to finish the program.
*
* Usage: >./camera_soiling_example mavs_scene_file.json
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
*
* \author Chris Goodin
*
* \date 7/24/2025
*/

#include <sensors/mavs_sensors.h>
#include <sensors/camera/camera_soiling.h>
#include <iostream>
#include <raytracers/embree_tracer/embree_tracer.h>

int main(int argc, char* argv[]) {

	// check that the scene file is supplied on the command line
	if (argc < 2) {
		std::cerr << "Usage: ./camera_soiling_example scenefile.json" << std::endl;
		return 1;
	}

	// create and load the scene 
	mavs::raytracer::embree::EmbreeTracer scene;
	std::string scene_file(argv[1]);
	scene.Load(scene_file);

	// create the camera and set parameters
	mavs::sensor::camera::RgbCamera camera;
	camera.FreePose(); // move the camera around with WASD, arrows, and PageUp/PageDown
	camera.SetPose(glm::vec3(0.0f, 0.0f, 3.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	camera.Display();

	// create the environment and set the parameters
	mavs::environment::Environment env;
	env.SetRainRate(15.0f);
	env.SetCloudCover(0.85f);
	env.SetFog(0.01f);
	env.SetWind(2.0f, 1.0f);
	env.SetRaytracer(&scene);

	// create the camera soiling class
	mavs::sensor::CameraSoiling soiling;

	float dt = 0.025f; // seconds
	int nframes = 0;

	// start the sim loop
	while (camera.DisplayOpen()) {
		// update the camera
		camera.Update(&env, dt);

		// add raindrops
		soiling.AddRaindropsToCamera(&env, &camera, dt);

		// add mud (there are no masks at first)
		soiling.AddMudToCamera(&env, &camera, dt);

		// display the updated camera image
		camera.Display();

		// increment the frames and add another mud mask every 100 time steps
		nframes++;
		if (nframes % 100 == 0)soiling.AddMudMask();
	}

	return 0;
}

