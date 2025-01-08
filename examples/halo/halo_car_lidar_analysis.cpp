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
* \file halo_car_lidar_analysis.cpp
*
* Programatically analyzes lidar of the Halo vehicle
*
* Usage: >./halo_car_lidar_analysis
* 
* \author Chris Goodin
*
* \date 11/13/2018
*/
#include <iostream>
#include <vector>
#include "simulation/halo/halo_car.h"
#include <glm/glm.hpp>

int main(int argc, char *argv[]) {

	std::vector<std::string> scenes;
	scenes.push_back("rough_0_allveg_scene.json");
	scenes.push_back("rough_0_halfveg_scene.json");
	scenes.push_back("rough_0_noveg_scene.json");
	scenes.push_back("rough_50_allveg_scene.json");
	scenes.push_back("rough_50_halfveg_scene.json");
	scenes.push_back("rough_50_noveg_scene.json");
	scenes.push_back("rough_25_allveg_scene.json");
	scenes.push_back("rough_25_halfveg_scene.json");
	scenes.push_back("rough_25_noveg_scene.json");

	glm::vec3 init_pos(-20.0f, -20.0f, 0.0f);
	glm::quat init_ori(0.92388f, 0.0f, 0.0f, 0.38268f);

	for (int s = 0; s < (int)scenes.size(); s++) {
		mavs::halo::HaloCar halo_car;
		halo_car.SetInitialVehiclePose(init_pos, init_ori);
		halo_car.SetVehicleGoal(0.0f, 0.0f);
		halo_car.TurnOnSensor(7);
		halo_car.TurnOnSensor(8);
		halo_car.TurnOnSensor(9);
		std::string basename = scenes[s].substr(0, scenes[s].size() - 5);
		halo_car.SetWriteCombinedPointCloud(basename,5.0f);

		halo_car.LoadScene(scenes[s]);
		
		halo_car.Run();
	}
	return 0;
}
