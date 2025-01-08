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
* \file sky_example.cpp
*
* Renders the day and night sky over 24 hours using the fisheye camera model
*
* Usage: >./sky_example
*
* \author Chris Goodin
*
* \date 10/15/2018
*/

#include <iostream>
#include <sensors/mavs_sensors.h>
#include <mavs_core/math/utils.h>
#include <raytracers/simple_tracer/simple_tracer.h>

int main (int argc, char *argv[]){
  
  mavs::sensor::camera::FisheyeCamera camera;
	camera.Initialize(1024, 1024, 0.025f, 0.025f, 0.008f);
	camera.SetPixelSampleFactor(3);
  mavs::environment::Environment env;
	double origin_lat = 32.0;
	double origin_lon = 90.0;
	double elevation = 73.152;
	env.SetLocalOrigin(origin_lat, origin_lon, elevation); //Vicksburg, MS
	int hour = 0;
	int minute = 0;
	env.SetDateTime(2004, 6, 5, hour, minute, 0, 6);

  mavs::raytracer::SimpleTracer scene;
	mavs::raytracer::Aabb ground; 
  ground.SetSize(1.0E6f, 1.0E6f, 0.01f);
  ground.SetColor(0.25f,1.0f,0.25f);
  ground.SetPosition(0.0f,0.0f,0.0f);
  scene.AddPrimitive(ground);

  env.SetRaytracer(&scene);
  camera.SetEnvironmentProperties(&env);	
  glm::vec3 position(0.0f, 0.0f, 1.0f);
  glm::quat orientation(0.707107f, 0.0f, -0.707107f, 0.0f);
	camera.SetPose(position, orientation);

	int elapsed_time = 0;
	int secs_per_day = 86400;
	int minstep = 3;
	int dt = minstep*60;
	while (elapsed_time<secs_per_day){
    camera.Update(&env,0.1);
    camera.Display();
		std::string fname = "sky_"+mavs::utils::ToString(hour,2)+mavs::utils::ToString(minute,2)+".bmp";
		env.SetDateTime(2004, 6, 5, hour, minute, 0, 6);
		camera.SetEnvironmentProperties(&env);
		minute = minute + minstep;
		if (minute >= 60) {
			hour = hour + 1;
			minute = 0;
		}
		elapsed_time += dt;
		camera.SaveImage(fname);
  }
  
  return 0;
}

