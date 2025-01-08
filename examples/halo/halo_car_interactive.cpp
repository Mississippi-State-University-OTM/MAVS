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
* \file halo_car_interactive.cpp
*
* Drive the halo car through the scene and record data, etc.
*
* Usage: >./halo_car_example mavs_scene_file.json 
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
* 
* \author Chris Goodin
*
* \date 11/2/2018
*/
#include <iostream>

#include "simulation/halo/halo_car.h"

int main(int argc, char *argv[]) {

	mavs::halo::HaloCar halo_car;
	halo_car.SetGpsFrameRate(20.0f);
	halo_car.SetCanBusRate(20.0f);
	halo_car.SetLidarFrameRate(5.0);

	if (argc < 2) {
		halo_car.LoadScene();
	}
	else {
		std::string scene_file(argv[1]);
		halo_car.LoadScene(scene_file);
	}

	if (argc > 2) {
		std::string chrono_inputs(argv[2]);
		halo_car.UseChronoVehicle(chrono_inputs);
	}

	//halo_car.AddActor("hmmwv");

	halo_car.RunInteractive();
 
	return 0;
}
