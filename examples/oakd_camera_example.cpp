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
* \file camera_example.cpp
*
* An example MAVS Oak-D depth camera sensor
*
* Usage: >./oakd_camera_example scene.json
*
* scene.json examples found in mavs/data/scenes
*
* \author Chris Goodin
*
* \date 9/11/2024
*/
#include <sensors/camera/oak_d_camera.h>
#include <iostream>
#include <raytracers/embree_tracer/embree_tracer.h>

int main(int argc, char* argv[]) {
	if (argc < 2)std::cerr << "Usage: ./camera_example scenefile.json" << std::endl;

	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);

	mavs::environment::Environment env;
	env.SetRaytracer(&scene);

	mavs::sensor::camera::OakDCamera camera;
	camera.SetMaxDepth(4.0f);
	camera.FreePose();
	camera.SetDisplayType("both");

	camera.SetPose(glm::dvec3(0.0f, -2.25f, 1.25f), glm::quat(0.70710678f, 0.0f, 0.0f, 0.70710678f));
	camera.Update(&env, 0.1);
	camera.Display();

	while (camera.DisplayOpen()) {
		camera.Update(&env, 0.1);
		mavs::Pose pose = camera.GetPose();
		//float depth_at_center = camera.GetCmDepthAtPixel(camera.GetDepthImageWidth() / 2, camera.GetDepthImageHeight() / 2);
		camera.Display();
	}
	return 0;
}

