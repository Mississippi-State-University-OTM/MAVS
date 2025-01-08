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
* \file dis_datagen.cpp
*
* Generate SLAM input data for the DIS project
*
* Usage: >./dis_datagen
*
* \author Chris Goodin
*
* \date 11/26/2018
*/
#include<cstdlib>
#include<ctime>

#include <iostream>
#include <string>

#include <mavs_core/terrain_generator/random_scene.h>
#include <mavs_core/data_path.h>
#include <mavs_core/pose_readers/anvel_vprp_reader.h>
#include <mavs_core/math/utils.h>

#include "simulation/halo/halo_car.h"

int main(int argc, char *argv[]) {
#ifdef USE_EMBREE
	std::vector<std::string> scenefiles,prefixes;
	scenefiles.push_back("/scenes/dis_forest_sparse_scene.json");
	scenefiles.push_back("/scenes/dis_forest_medium_scene.json");
	scenefiles.push_back("/scenes/dis_forest_dense_scene.json");
	prefixes.push_back("sparse_");
	prefixes.push_back("medium_");
	prefixes.push_back("dense_");
	for (int s = 0; s < 3; s++) {
		mavs::MavsDataPath mavs_data_path;
		//std::string scene_file = mavs_data_path.GetPath() + "/scenes/desert_dis_scene.json";
		std::string scene_file = scenefiles[s];
		//std::string scene_file = mavs_data_path.GetPath() + "/scenes/cube_scene.json";
		std::string path_file = mavs_data_path.GetPath() + "/waypoints/Ball_DIS.vprp";
		//create the pose structure and load
		mavs::AnvelVprpReader pose_reader;
		std::vector<mavs::Pose> poses = pose_reader.Load(path_file, 1);

		mavs::halo::HaloCar halo_car;
		halo_car.SetOutfilePrefix(prefixes[s]);
		halo_car.LoadScene((mavs_data_path.GetPath()+scene_file));
		halo_car.LoadPoses(path_file);
		halo_car.SetUsePoses(true);
		halo_car.SetInitialVehiclePose(poses[0].position, poses[0].quaternion);
		halo_car.WriteSlamOutput();
		//halo_car.TurnOnSensor(1);
		//halo_car.SetImageFrameRate(0.5f);
		halo_car.TurnOnLogging("./");
		halo_car.SetDisplay(false);
		halo_car.Run();

	}
#endif
	return 0;
}

