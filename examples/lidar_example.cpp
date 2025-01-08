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
* \file lidar_example.cpp
*
* Demonstrates a MAVS lidar simulation.
*
* Usage: >./lidar_example mavs_scene_file.json (lidar_num) (rain_rate)
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
*
* lidar_num is optional and specifies the type of lidar sensor
*
* 291 = SICK LMS291
*
* 8 = M8
*
* 16 = Velodyne VLP 16
*
* 32 = Velodyne HDL32E
*
* 64 = Velocyne HDL64E
*
* The default LIDAR is 291
*
* rain_rate is an optional parameter and specifies the rain rate in mm/h
* Typical values are 2.5-25.0
*
* The simulation will save several files
*
* scene_stats.txt - specifies the number of triangles in the scene
*
* lidar_output.bmp - A top-down rendering of the point cloud
*
* lidar_sensor00001_annotated.txt A space-delimited column file of the point cloud (x,y,z,i,label_num)
*
* lidar_sensor00001_annotated.csv A comma-delimited file listing the objects that were detected in the point cloud and their extent
*
* \date 10/4/2018
*/
#include <sensors/lidar/lms_291.h>
#include <sensors/lidar/hdl64e.h>
#include <sensors/lidar/hdl32e.h>
#include <sensors/lidar/vlp16.h>
#include <sensors/lidar/m8.h>

#include <iostream>
#include <algorithm>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#else
#include <raytracers/simple_tracer/simple_tracer.h>
#endif

int main(int argc, char *argv[]) {
	int myid = 0;
	int numprocs = 1;
#ifdef USE_MPI    
	int ierr = MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	mavs::environment::Environment env;

#ifdef USE_EMBREE
	if (argc < 2) {
		std::cerr << "Usage: ./lidar_example scenefile.json " << std::endl;
		return 1;
	}
	std::string scene_file(argv[1]);
	mavs::raytracer::embree::EmbreeTracer scene;
	if (myid == 0)std::cout << "Loading " << scene_file << std::endl;
	scene.Load(scene_file);
	scene.TurnOnLabeling();
	scene.TurnOnSpectral();
	scene.TurnOffSurfaceTextures();
#else
	mavs::raytracer::SimpleTracer scene;
	mavs::raytracer::Sphere sred;
	sred.SetPosition(0, 0, 10);
	sred.SetColor(1.0, 0.25, 0.25);
	sred.SetRadius(5.0);
	scene.AddPrimitive(sred);
	mavs::raytracer::Aabb box, box2;
	box.SetSize(1.0E6f, 1.0E6f, 0.01f);
	box.SetColor(0.25f, 1.0f, 0.25f);
	box.SetPosition(0.0f, 0.0f, 5.0f);
	scene.AddPrimitive(box);
	box2.SetPosition(0.0f, 4.0f, 3.0f);
	box2.SetSize(10.0f, 10.0f, 2.0f);
	box2.SetColor(0.7f, 0.7f, 0.25f);
	scene.AddPrimitive(box2);
#endif
	int disp = 1;
	//will display sensor data
	/*if (argc>2){
		disp = atoi(argv[2]);
		if(myid==0 && disp>0) std::cout<<"Displaying sensors..."<<std::endl;
	}
	*/
	float rain_rate = 0.0f;
	if (argc > 3) {
		rain_rate = (float)atof(argv[3]);
		//if (myid == 0 && disp>0) std::cout << "Displaying sensors..." << std::endl;
	}

	env.SetRaytracer(&scene);
	env.SetRainRate(rain_rate);

	double rot_rate = 5.0;
	double dt = 1.0 / rot_rate;
	int sens_num = 291;
	if (argc > 2)sens_num = sens_num = atoi(argv[2]);
	mavs::sensor::lidar::Lms291_S05 s05;
	mavs::sensor::lidar::Vlp16 vlp;
	mavs::sensor::lidar::Hdl32E hdl32;
	mavs::sensor::lidar::Hdl64E hdl64;
	mavs::sensor::lidar::MEight m8;
	mavs::sensor::lidar::Lidar *lidar;
	if (sens_num == 64) {
		hdl64.SetRotationRate(5.0);
		dt = 0.2;
		lidar = &hdl64;
	}
	else if (sens_num == 16) {
		vlp.SetRotationRate(5.0);
		dt = 0.2;
		lidar = &vlp;
	}
	else if (sens_num == 32) {
		hdl32.SetTimeStep(0.2);
		dt = 0.2;
		lidar = &hdl32;
	}
	else if (sens_num == 8) {
		m8.SetRotationRate(10);
		dt = 0.1;
		lidar = &m8;
	}
	else {
		s05.SetTimeStep(1.0 / 50.0);
		dt = 0.02;
		lidar = &s05;
	}
	double elapsed_time = 0.0;
#ifdef USE_MPI    
	lidar->SetComm(MPI_COMM_WORLD);
#endif  
	if (disp <= 0)lidar->LogData("_lidar_points");
	glm::dvec3 position(0.0, 0.0, 1.0);
	glm::dquat orientation(1.0, 0.0, 0.0, 0.0);

#ifdef USE_MPI  
	MPI_Barrier(MPI_COMM_WORLD);
	double t1 = MPI_Wtime();
#endif
	if (myid == 0)std::cout << "Done loading, scanning" << std::endl;
	for (int i = 0; i < 5; i++) {
		//for (int i=0;i<35;i++){
		//for (int i = 0; i<1; i++) {
		lidar->SetPose(position, orientation);
		lidar->Update(&env, dt);
		if (myid == 0) {
#ifdef USE_MPI
			double walltime = MPI_Wtime() - t1;
			if (elapsed_time != 0.0)std::cout << "Timing: " << elapsed_time << " " << walltime << " " <<
				walltime / elapsed_time << std::endl;
#endif
			if (disp > 0)lidar->Display();
		}
		//elapsed_time += dt;
		//position.x = position.x + 2.5;
	}
	//lidar->WritePointsToImage("lidar_output.bmp");
	//lidar->WritePointsToText("lidar_output.txt");

	std::vector<float> distances = lidar->GetReturnedDistances();
	float max_dist = 0.0;
	for (int i = 0; i < (int)distances.size(); i++) {
		if (distances[i] > max_dist)max_dist = distances[i];
	}
	std::cout << "Maximum measured distance = " << max_dist << std::endl;
	lidar->AnnotateFrame(&env, true);
	lidar->SaveAnnotation();
	lidar->WritePcdWithLabels("labeled_lidar.pcd");
	//mavs::sensor::lidar::LidarTools pc_analyzer;
	//std::vector<mavs::sensor::lidar::labeled_point> points = lidar->GetLabeledPoints();
	//pc_analyzer.AnalyzeCloud(points, "labeled_lidar.pcd", 0, lidar);

#ifdef USE_MPI 
	MPI_Finalize();
#endif
	return 0;
}

