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
* \file forester_example_simple.cpp
*
* An example sensor placement analysis with MAVS
*
* Usage: >./forester_example_simple sensor_inputs.json
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

#include <iostream>
#include <fstream>

#include <raytracers/simple_tracer/simple_tracer.h>

#include <sensors/lidar/m8.h>
#include <sensors/camera/simple_camera.h>
#include <sensors/camera/rgb_camera.h>

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
  if (d.HasMember("Minimum Distance (meters)")){
    min_dist = d["Minimum Distance (meters)"].GetDouble();
  }
  else {
    return false;
  }
  if (d.HasMember("Maximum Distance (meters)")){
    max_dist = d["Maximum Distance (meters)"].GetDouble();
  }
  else {
    return false;
  }
  if (d.HasMember("Distance Step (meters)")){
    dist_step = d["Distance Step (meters)"].GetDouble();
  }
  else{
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

struct Zone{
  double x_lo;
  double y_lo;
  double x_hi;
  double y_hi;
};

struct Coverage{
  int num_points;
  float percent_covered;
  float map[100][100];
  void WriteMap(std::string input_file){
    std::ofstream fout;
    fout.open(input_file.c_str());
    for (int i=0;i<100;i++){
      for (int j=0;j<100;j++){
	fout<<i<<" "<<j<<" "<<map[i][j]<<std::endl;
      }
    }
    fout.close();
  }
};

Coverage GetPointsInZone(std::vector<glm::vec3> &points, Zone zone){
  int num = 0;
  Coverage cov;
  float filled_zones[100][100];
  for (int i=0;i<100;i++){
    for (int j=0;j<100;j++){
      filled_zones[i][j]=0.0f;
      cov.map[i][j]=0.0f;
    }
  }
  float x_step = (float)(zone.x_hi-zone.x_lo)/100.0f;
  float y_step = (float)(zone.y_hi-zone.y_lo)/100.0f;

  for (int i=0;i<(int)points.size();i++){
    if (points[i].x>zone.x_lo && points[i].x<zone.x_hi &&
	points[i].y>zone.y_lo && points[i].y<zone.y_hi){
      num = num + 1;
      int zi = (int)(floor((points[i].x-zone.x_lo)/x_step));
      int zj = (int)(floor((points[i].y-zone.y_lo)/y_step));
      filled_zones[zi][zj]=1.0f;
      cov.map[zi][zj] += 1.0f;
    }
  }
  
  float pix_area = x_step*y_step;
  float filled_sum = 0.0;
  for (int i=0;i<100;i++){
    for (int j=0;j<100;j++){
      filled_sum += filled_zones[i][j];
      cov.map[i][j] = cov.map[i][j]/pix_area;
    }
  }
  
  cov.num_points = num;
  cov.percent_covered = 100.0f*filled_sum/(100.0f*100.0f);
  return cov;
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
    disp = true;
    scene_file="/home/cgoodin/cavs_shared/data/scenes/forester_scene_wall.json";
    min_ang = -90;
    max_ang = 90;
    ang_step = 5.0;
    min_dist = 6;
    max_dist = 58;
    dist_step = 2;
  }

  std::vector<Zone> zones;
  zones.resize(4);
  zones[0].x_lo = 1.001734; zones[0].y_lo = -1.1;
  zones[0].x_hi = 2.257577; zones[0].y_hi = 1.1;
  zones[1].x_lo = zones[0].x_hi; zones[1].y_lo = -1.1;
  zones[1].x_hi = 4.013238; zones[1].y_hi = 1.1;
  zones[2].x_lo = zones[1].x_hi; zones[2].y_lo = -1.1;
  zones[2].x_hi = 6.270815; zones[2].y_hi = 1.1;
  zones[3].x_lo = zones[2].x_hi; zones[3].y_lo = -1.1;
  zones[3].x_hi = 9.03031; zones[3].y_hi = 1.1;
  
  mavs::environment::Environment env;

  mavs::raytracer::SimpleTracer scene;
  mavs::raytracer::Aabb ground; //,obstacle,left_wall,right_wall;
  ground.SetSize(1.0E6f, 1.0E6f, 0.01f);
  ground.SetColor(0.1f,0.5f,0.25f);
  ground.SetPosition(0.0f,0.0f,-0.01f);
  scene.AddPrimitive(ground);
  //put an obstacle at LIDAR height 175 meters away
  /*obstacle.SetPosition(175.0,0,0.5);
  obstacle.SetSize(0.5,0.5,1.0);
  obstacle.SetColor(0.7,0.25,0.25);
  scene.AddPrimitive(&obstacle);
  left_wall.SetPosition(100.0,3.7,0);
  left_wall.SetSize(200,0.1,3.0);
  left_wall.SetColor(0.25,0.25,0.25);
  scene.AddPrimitive(&left_wall);
  right_wall.SetPosition(100.0,-3.7,0);
  right_wall.SetSize(200,0.1,3.0);
  right_wall.SetColor(0.25,0.25,0.25);*/

  env.SetRaytracer(&scene);

  double rot_rate = 10.0;
  double dt = 1.0/rot_rate;
 
  mavs::sensor::lidar::MEight left,right;
  left.SetNoiseCutoff(0.0);
  right.SetNoiseCutoff(0.0);
#ifdef USE_MPI    
  left.SetComm(MPI_COMM_WORLD);
  right.SetComm(MPI_COMM_WORLD);
#endif
  left.SetName("Left Lidar");
  right.SetName("Right Lidar");
  
  mavs::sensor::camera::SimpleCamera topcam, leftcam,rightcam;
#ifdef USE_MPI    
  topcam.SetComm(MPI_COMM_WORLD);
  leftcam.SetComm(MPI_COMM_WORLD);
  rightcam.SetComm(MPI_COMM_WORLD);
#endif  
  topcam.SetName("Top Camera");
  leftcam.SetName("Left Camera");
  rightcam.SetName("Right Camera");

  std::ofstream log_out;
  log_out.open("summary.txt");
  log_out<<"angle Zone1 Zone2 Zone3 Zone4 Zone1_norm "<<
    "Zone2_norm Zone3_norm Zone4_norm Zone1_cov Zone2_cov "<<
    "Zone3_cov Zone4_cov"<<std::endl;
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
      std::vector<glm::vec3> all_points = left_points;
      for (int p=0;p<(int)right_points.size();p++){
	all_points.push_back(right_points[p]);
      }
      std::vector<Coverage> num_per_zone;
      num_per_zone.resize(4);
      for (int z =0;z<4;z++){
	num_per_zone[z] = GetPointsInZone(all_points,zones[z]);
	std::string fname = "angle";
	fname.append(mavs::utils::ToString(angle));
	fname.append("_zone");
	fname.append(mavs::utils::ToString(z));
	fname.append(".txt");
	num_per_zone[z].WriteMap(fname);
      }
      log_out<<angle<<" "<<
	num_per_zone[0].num_points<<" "<<
	num_per_zone[1].num_points<<" "<<
	num_per_zone[2].num_points<<" "<<
	num_per_zone[3].num_points<<" "<<
	0.25*num_per_zone[0].num_points*pow(zones[0].x_hi-zones[0].x_lo,2)
	     <<" "<<
	0.25*num_per_zone[1].num_points*pow(zones[1].x_hi-zones[1].x_lo,2)
	     <<" "<<
        0.25*num_per_zone[2].num_points*pow(zones[2].x_hi-zones[2].x_lo,2)
	     <<" "<<
	0.25*num_per_zone[3].num_points*pow(zones[3].x_hi-zones[3].x_lo,2)
	     <<" "<<
	num_per_zone[0].percent_covered<<" "<<
	num_per_zone[1].percent_covered<<" "<<
	num_per_zone[2].percent_covered<<" "<<
	num_per_zone[3].percent_covered<<" "<<
	std::endl;
      if (disp){
	topcam.Display();
	right.Display();
	rightcam.Display();
	left.Display();
	leftcam.Display();
      }
    } // if myid ==0
    if (disp){
      if (myid==0){
	std::cout<<"Finished with angle "<<angle<<
	  ", press Enter to continue.";
	std::cin.ignore();
      }
#ifdef USE_MPI        
      MPI_Barrier(MPI_COMM_WORLD);
#endif      
    }
    else{
      if (myid==0)std::cout<< (100*(angle-min_ang)/(max_ang-min_ang))<<
		    "% complete"<<std::endl;
    }
    angle += ang_step;
  }

  log_out.close();
#ifdef USE_MPI  
  MPI_Finalize();
#endif  
  return 0;
}

