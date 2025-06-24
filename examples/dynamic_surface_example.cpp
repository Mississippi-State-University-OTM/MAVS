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
* Drive a MAVS vehicle implemented in RP3D
*
* Usage: >./dynamic_surface_example rp3d_vehicle_file.json 
*
* rp3d_vehicle_file.json examples can be found in mavs/data/vehicles/rp3d_vehicles
*
* \author Chris Goodin
*
* \date 6/25/2025
*/
#include <iostream>
#include <stdlib.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>
#include <mavs_core/terrain_generator/heightmap.h>
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <sensors/mavs_sensors.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif

mavs::raytracer::embree::EmbreeTracer CreateScene() {
	mavs::MavsDataPath mdp;
	std::string mavs_data_path = mdp.GetPath();
	mavs::terraingen::HeightMap heightmap;
	int nx = 1000;
	int ny = 1000;
	heightmap.Resize(nx, ny);
	heightmap.SetCorners(-500.0f, -500.0f, 500.0f, 500.0f);
	heightmap.SetResolution(1.0f);
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			heightmap.SetHeight(i, j, 0.0f);
}
	}
	std::string file_path = mavs_data_path + "/scenes/meshes/";
	mavs::raytracer::Mesh surf_mesh = heightmap.GetAsMesh();
	mavs::raytracer::embree::EmbreeTracer scene;
	glm::mat3x4 rot_scale = scene.GetAffineIdentity();

	scene.SetLayeredSurfaceMesh(surf_mesh, rot_scale);
	scene.SetSurfaceMesh(surf_mesh, rot_scale);
	std::string layer_file = file_path + "surface_textures/road_surfaces.json";
	mavs::raytracer::LayeredSurface layers;
	layers.LoadSurfaceTextures(file_path, layer_file);
	scene.AddLayeredSurface(layers);
	scene.LoadSemanticLabels(file_path + "labels.json");
	scene.SetLabelsLoaded(true);
	scene.CommitScene();
	scene.SetLoaded(true);
	scene.SetFilePath(file_path);
	return scene;
}


int main(int argc, char *argv[]) {
#ifndef USE_EMBREE
	std::cerr << "The Driving Example can only be run with Embree enabled" << std::endl;
	return 0;
#endif
	if (argc < 2) {
		std::cerr << "ERROR, must provide scene file and vehicle file as arguments" << std::endl;
		return 1;
	}
	std::string vehic_file(argv[1]);

	mavs::raytracer::embree::EmbreeTracer scene = CreateScene();
	

	mavs::environment::Environment env;
	env.SetRaytracer(&scene);

	glm::vec3 sensor_offset(-10.0f, 0.0f, 1.5f);
	glm::quat sensor_orient(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 position(0.0f, 0.0f, 1.0f);
	glm::quat orient(1.0f, 0.0f, 0.0f, 0.0f);
	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
	camera.SetRelativePose(sensor_offset, sensor_orient);
	camera.SetName("camera");
	//camera.SetAntiAliasing("oversampled");
	//camera.SetPixelSampleFactor(3);
	camera.SetPose(position, orient);
	camera.SetElectronics(0.65f, 1.0f);
	
	mavs::vehicle::Rp3dVehicle veh;
	veh.Load(vehic_file);
	float zstart = scene.GetSurfaceHeight(0.0f, 0.0f);
	veh.SetPosition(0.0f,0.0f, zstart + 0.25f);
	veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
	//veh.SetPosition(20.0f, -20.0f, 2.5f);
	//veh.SetOrientation(0.7071f, 0.0f, 0.0f, 0.7071f);

	float dt = 1.0f / 100.0f;
	int nsteps = 0;

	while(camera.DisplayOpen() || nsteps==0){

		float throttle = 0.3f;
		float steering = 0.0f;
		float braking = 0.0f;
		
		veh.Update(&env, throttle, steering, braking, dt);
		
		if (nsteps % 4 == 0) {
			mavs::VehicleState veh_state = veh.GetState();
			camera.SetPose(veh_state);
			camera.Update(&env, 4 * dt);
			camera.Display();
		}
		nsteps++;
	}

	return 0;
}
