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
* \file free_camera_example.cpp
*
* Fly the camera around the scene with the W-A-S-D keys.
* Page Up & Page Down to move up and down.
* Arrow keys rotate the view (left,right,up,down)
* Home and End keys rotate the roll of the camera
* Close the view window to finish the program.
*
* Usage: >./free_camera_example mavs_scene_file.json 
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
* 
* \author Chris Goodin
*
* \date 8/16/2018
*/

#include <sensors/mavs_sensors.h>

#include <iostream>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#else
#include <raytracers/simple_tracer/simple_tracer.h>
#endif

int main(int argc, char *argv[]) {

	mavs::sensor::camera::RgbCamera camera;
	//mavs::sensor::camera::Phantom4Camera camera;
	//mavs::sensor::camera::Phantom4CameraPathTracedLowRes camera(100, 10, 0.55f);

	camera.FreePose();
	camera.SetRaindropsOnLens(false);
	mavs::environment::Environment env;

#ifdef USE_EMBREE
	if (argc < 2) {
		std::cerr << "Usage: ./free_camera_example scenefile.json envfile.json" << std::endl;
		return 1;
	}
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
#else
	mavs::raytracer::SimpleTracer scene;
	scene.CreateTestScene();
#endif

	if (argc > 2) {
		std::string env_file(argv[2]);
		env.Load(env_file);
	}

  env.SetRaytracer(&scene);
  glm::vec3 position(0.0f, 0.0f, 3.0f);
  glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);
	//glm::vec3 position(0.0f, 0.0f, 350.0f);
	//glm::quat orientation(0.7071f, 0.0f, 0.7071f, 0.0f);
	camera.SetPose(position, orientation);

	while (true){
		camera.Update(&env,0.1);
		camera.Display(); 
  }
  
  return 0;
}

