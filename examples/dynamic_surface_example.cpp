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
* \file dynamic_surface_example.cpp
*
* Creates terrains / scenes programattically and drives a vehicle in them
*
* Usage: >./dynamic_surface_example rp3d_vehicle_file.json 
*
* rp3d_vehicle_file.json examples can be found in mavs/data/vehicles/rp3d_vehicles
*
* \author Chris Goodin
*
* \date 6/25/2025
*/
#include <mavs_core/terrain_generator/terrain_elevation_functions.h>
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <sensors/camera/rgb_camera.h>

int main(int argc, char *argv[]) {
	
	// check that the vehicle command line arg is provided
	if (argc < 2) {
		std::cerr << "ERROR, must provide a vehicle file as argument" << std::endl;
		return 1;
	}
	std::string vehic_file(argv[1]);
	bool render = true;
	if (argc > 2) render = bool(atoi(argv[2]));

	// create the different types of terrains for testing and add them to a vector
	std::vector<mavs::terraingen::TerrainCreator> terrains;
	terrains.resize(2);
	// Add a terrain with a positive and negative hole obstacle
	terrains[0].AddHole(20.0f, 0.0f, 2.0f, 15.0f, 1.0f);
	terrains[0].AddHole(40.0f, 0.0f, -3.0f, 1.0f, 1000.0f);
	terrains[0].CreateTerrain(-25.0f, -25.0f, 200.0f, 25.0f, 0.2f);
	// Create a terrain with a positive and negative trapezoidal obstacle
	terrains[1].AddTrapezoid(6.0f, 12.0f, 2.0f, 20.0f);
	terrains[1].AddTrapezoid(6.0f, 12.0f, -2.0f, 30.0f);
	terrains[1].CreateTerrain(-25.0f, -25.0f, 200.0f, 25.0f, 0.5f);

	// loop over all the terrain types
	for (int i = 0; i < (int)terrains.size(); i++) {
		
		// Get a pointer to the created scene
		mavs::raytracer::embree::EmbreeTracer *scene = terrains[i].GetScenePointer();

		// create the environment and add the scene to it
		mavs::environment::Environment env;
		env.SetRaytracer(scene);
		env.SetDateTime(2025, 6, 25, 19, 0, 0, 6); // 7 PM so we have long shadows

		// create the camera and set it's propoerties
		mavs::sensor::camera::RgbCamera camera;
		camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
		camera.SetRelativePose(glm::vec3(-12.0f, 0.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
		camera.SetElectronics(0.75f, 1.0f);

		// load the vehicle and set it's initial position
		mavs::vehicle::Rp3dVehicle veh;
		veh.Load(vehic_file);
		veh.SetPosition(0.0f, 0.0f, scene->GetSurfaceHeight(0.0f, 0.0f) + 0.25f);
		veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);

		// initiate the main sim loop
		float elapsed_time = 0.0f;
		float dt = 1.0f / 100.0f;
		int nsteps = 0;
		while (elapsed_time<15.0f){
			// constant throttle, steering, and braking
			float throttle = 0.3f;
			float steering = 0.0f;
			float braking = 0.0f;

			// update the vehicle
			veh.Update(&env, throttle, steering, braking, dt);

			// update the vehicle every 4 steps (25 Hz)
			if (nsteps % 4 == 0 && render) {
				glm::vec3 look_to = veh.GetLookTo();
				float yaw = atan2f(look_to.y, look_to.x);
				glm::quat ori(cosf(0.5 * yaw), 0.0f, 0.0f, sin(0.5f * yaw));
				glm::vec3 pos = veh.GetPosition();
				camera.SetPose(pos, ori);
				camera.Update(&env, 4 * dt);
				camera.Display();
			}

			// update the loop counters
			nsteps++;
			elapsed_time += dt;
		} // sim loop
	} // loop over terrain types
	
	return 0;
}
