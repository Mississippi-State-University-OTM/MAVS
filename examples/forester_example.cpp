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
* \file forester_example.cpp
*
* An example sensor placement analysis with MAVS
*
* Usage: >./forester_example sensor_inputs.json
*
* sensor_inputs.json is specific to this executable, an
* example be found in mavs/data/sims/misc_sims/forester_sim.json.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <sensors/lidar/m8.h>
#include <sensors/camera/simple_camera.h>
#include <sensors/camera/rgb_camera.h>

#include <iostream>
#include <fstream>

#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#else
#include <raytracers/simple_tracer/simple_tracer.h>
#endif

#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>

#include <glm/gtx/quaternion.hpp>

std::string scene_file;
double min_ang,max_ang,ang_step;
double min_dist,max_dist,dist_step;
bool disp;

bool LoadInputFile(std::string input_file){
  FILE* fp = fopen(input_file.c_str(),"rb");
  char readBuffer[65536];
  rapidjson::FileReadStream is(fp,readBuffer,sizeof(readBuffer));
  rapidjson::Document d;
  d.ParseStream(is);
  fclose(fp);
  mavs::MavsDataPath mavs_data_path;
  //std::string data_path = mavs_data_path.GetPath();
  
  if (d.HasMember("Scene File")){
    std::string scene_file_in = d["Scene File"].GetString();
    scene_file = mavs_data_path.GetPath();
    scene_file.append("/scenes/");
    scene_file.append(scene_file_in);
  }
  else {
    return false;
  }

  if (d.HasMember("Minimum Angle (degrees)")){
    min_ang = d["Minimum Angle (degrees)"].GetDouble();
  }
  else {
    return false;
  }
  if (d.HasMember("Maximum Angle (degrees)")){
    max_ang = d["Maximum Angle (degrees)"].GetDouble();
  }
  else {
    return false;
  }
  if (d.HasMember("Angle Step (degrees)")){
    ang_step = d["Angle Step (degrees)"].GetDouble();
  }
  else {
    return false;
  }
  if (d.HasMember("Display Sensors")){
    disp = d["Display Sensors"].GetBool();
  }
  else {
    return false;
  }
  return true;
}

int main (int argc, char *argv[]){
  int myid = 0;
  int numprocs = 1;
#ifdef USE_MPI    
  int ierr = MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD,&numprocs);
  MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif
  
  if (argc<2){
    std::cerr<<"Usage: ./forester_example input.json "<<std::endl;
    return 1;
  }

  std::string input(argv[1]);
  bool loaded = LoadInputFile(input);
  if (!loaded){
    if (myid==0)
      std::cout<<"Failed to read json input, using defaults."<<std::endl;
    disp = false;
    scene_file="forester_scene_trees.json";
    min_ang = -90;
    max_ang = 90;
    ang_step = 5.0;
    min_dist = 6;
    max_dist = 58;
    dist_step = 2;
  }
  
  mavs::environment::Environment env;
  env.SetDateTime(2018,6,14,1,34,0,6);
  
#ifdef USE_EMBREE
  mavs::raytracer::embree::EmbreeTracer scene;
  if (myid==0)std::cout<<"Loading "<<scene_file<<std::endl;
  scene.Load(scene_file);
#else
  mavs::raytracer::SimpleTracer scene;
  mavs::raytracer::Aabb ground,obstacle,left_wall,right_wall;
  ground.SetSize(1.0E6f, 1.0E6f, 0.01f);
  ground.SetColor(0.25f,1.0f,0.25f);
  ground.SetPosition(0.0f,0.0f,5.0f);
  scene.AddPrimitive(ground);
  //put an obstacle at LIDAR height 175 meters away
  obstacle.SetPosition(175.0f,0.0f,0.5f);
  obstacle.SetSize(0.5f,0.5f,1.0f);
  obstacle.SetColor(0.7f,0.25f,0.25f);
  scene.AddPrimitive(obstacle);
  left_wall.SetPosition(100.0f,3.7f,0.0f);
  left_wall.SetSize(200.0f,0.1f,3.0f);
  left_wall.SetColor(0.25f,0.25f,0.25f);
  scene.AddPrimitive(left_wall);
  right_wall.SetPosition(100.0f,-3.7f,0.0f);
  right_wall.SetSize(200.0f,0.1f,3.0f);
  right_wall.SetColor(0.25f,0.25f,0.25f);
  scene.AddPrimitive(right_wall);
#endif

  env.SetRaytracer(&scene);

  float rot_rate = 10.0;
  float dt = 1.0f/rot_rate;
 
  mavs::sensor::lidar::MEight left,right;
  left.SetNoiseCutoff(0.0f);
  right.SetNoiseCutoff(0.0f);
#ifdef USE_MPI    
  left.SetComm(MPI_COMM_WORLD);
  right.SetComm(MPI_COMM_WORLD);
#endif  
  left.SetName("Left Lidar");
  right.SetName("Right Lidar");
  
  //mavs::sensor::camera::SimpleCamera topcam, leftcam,rightcam;
  mavs::sensor::camera::RgbCamera topcam,leftcam,rightcam;
  topcam.SetEnvironmentProperties(&env);
  leftcam.SetEnvironmentProperties(&env);
  rightcam.SetEnvironmentProperties(&env);
#ifdef USE_MPI    
  topcam.SetComm(MPI_COMM_WORLD);
  leftcam.SetComm(MPI_COMM_WORLD);
  rightcam.SetComm(MPI_COMM_WORLD);
#endif  
  topcam.SetName("Top Camera");
  leftcam.SetName("Left Camera");
  rightcam.SetName("Right Camera");

  double px = -5.0;
  
  std::ofstream log_out;
  log_out.open("summary.txt");
  log_out<<"angle distance num_points"<<std::endl;
  glm::dvec3 x_hat(1,0,0);
  glm::dvec3 z_hat(0,0,1);
  double angle = min_ang; 
  while (angle <= max_ang){
    double fix_angle = mavs::kDegToRad*34.7;
    glm::dvec3 top_pos(-2.18756, 0.0, 1.830);
    glm::dquat top_ang(1.0, 0.0, 0.0, 0.0);
    glm::dvec3 left_pos(-0.1, 0.779,0.689);
    glm::dquat left_ang = glm::angleAxis(-mavs::kDegToRad*angle,x_hat)*
      glm::angleAxis(-fix_angle,z_hat);
    glm::dvec3 right_pos(-0.1,-0.779,0.689);
    glm::dquat right_ang=glm::angleAxis(mavs::kDegToRad*angle,x_hat)*
      glm::angleAxis(fix_angle,z_hat);
#ifdef USE_MPI      
    MPI_Barrier(MPI_COMM_WORLD);
    double t1 = MPI_Wtime();
#endif    
    //double px = 175-max_dist;
    //while (px<175-min_dist){
      top_pos.x += px;
      left_pos.x += px;
      right_pos.x += px;
      right.SetPose(right_pos,right_ang);
      right.Update(&env,dt);
      left.SetPose(left_pos,left_ang);
      left.Update(&env,dt);
      if (disp){
	topcam.SetPose(top_pos,top_ang);
	topcam.Update(&env,dt);
	rightcam.SetPose(right_pos,right_ang);
	rightcam.Update(&env,dt);
	leftcam.SetPose(left_pos,left_ang);
	leftcam.Update(&env,dt);
      }
      if(myid==0){
#ifdef USE_MPI  	
	double walltime = MPI_Wtime()-t1;
#endif
	std::vector<glm::vec3> left_points = left.GetRegisteredPoints();
	std::vector<glm::vec3> right_points = right.GetRegisteredPoints();
	/*
	std::vector<glm::vec3> left_saved,right_saved;
	for (int i=0;i<left_points.size();i++){
	  if (left_points[i].x>170 && left_points[i].x<180 &&
	      left_points[i].y>-3 && left_points[i].y<3 &&
	      left_points[i].z>0.1){
	    left_saved.push_back(left_points[i]);
	  }
	}
	for (int i=0;i<right_points.size();i++){
	  if (right_points[i].x>170 && right_points[i].x<180 &&
	      right_points[i].y>-3 && right_points[i].y<3 &&
	      right_points[i].z>0.1){
	    right_saved.push_back(right_points[i]);
	  }
	}
	int num_points_on_target = left_saved.size()+right_saved.size();
	log_out<<angle<<" "<<175-px<<" "<<num_points_on_target<<std::endl;
	std::string out_name = mavs::utils::ToString(175-px) + "_a" +
	  mavs::utils::ToString(angle)+"_points.txt";
	if (num_points_on_target>0){
	  std::ofstream fout;
	  fout.open(out_name.c_str());
	  for (int i=0;i<right_saved.size();i++){
	    if (right_saved[i].x>170 && right_saved[i].x<180 &&
		right_saved[i].y>-3 && right_saved[i].y<3 &&
		right_saved[i].z>0.1){
	      fout<<right_saved[i].x<<" "<<right_saved[i].y<<" "<<
		right_saved[i].z<<" "<<1<<std::endl;
	    }
	  }
	  for (int i=0;i<left_saved.size();i++){
	    if (left_saved[i].x>170 && left_saved[i].x<180 &&
		left_saved[i].y>-3 && left_saved[i].y<3 &&
		left_saved[i].z>0.1){
	      fout<<left_saved[i].x<<" "<<left_saved[i].y<<" "<<
		left_saved[i].z<<" "<<2<<std::endl;
	    }
	  }
	  fout.close();
	}
	*/
	if (disp){
	  topcam.Display();
	  right.Display();
	  rightcam.Display();
	  left.Display();
	  leftcam.Display();
	}
	std::string left_lidar_fname,right_lidar_fname,top_cam_fname;
	left_lidar_fname = "left_angle";
	left_lidar_fname.append(mavs::utils::ToString(angle));
	left_lidar_fname.append(".txt");
	right_lidar_fname = "right_angle";
	right_lidar_fname.append(mavs::utils::ToString(angle));
	right_lidar_fname.append(".txt");
	top_cam_fname = "top_cam_angle";
	top_cam_fname.append(mavs::utils::ToString(angle));
	top_cam_fname.append(".bmp");
	left.WritePointsToText(left_lidar_fname);
	right.WritePointsToText(right_lidar_fname);
	topcam.SaveImage(top_cam_fname);
      }
      //px += dist_step;
      //}
      if (disp){
	if (myid==0){
	  std::cout<<"Press enter to continue:";
	  std::cin.ignore();
	}
      }
    angle += ang_step;
  }

  log_out.close();
#ifdef USE_MPI  
  MPI_Finalize();
#endif  
  return 0;
}

