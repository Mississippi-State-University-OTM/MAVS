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
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <drivers/simple_path_planner/simple_path_planner.h>

#ifdef USE_MPI
#include <mpi.h>
#endif

#include <iostream>
#include <fstream>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <mavs_core/math/constants.h>
#include <mavs_core/math/utils.h>
#include <sensors/compass/compass.h>

namespace mavs{
namespace driver{

SimplePathPlanner::SimplePathPlanner(){
  speed_ = 2.0; //m/s
  map_resolution_ = 1.5;
  tolerance_ = 1.5*1.5; //meters
  local_sim_time_ = 0.0;
  updated_ = false;
  current_command_.throttle = 0.0;
  current_command_.steering = 0.0;
  first_call_ = true;
  map_size_ = 200.0; //meters
  max_nav_height_ = 1.5;
  solve_potential_ = false;

  current_waypoint_ = 0;

  PoseStamped p;
  p.pose.position.x = 20.0;
  p.pose.position.y = 20.0;
  path_.poses.push_back(p);
}

void SimplePathPlanner::Load(std::string input_file){
  FILE* fp = fopen(input_file.c_str(),"rb");
  char readBuffer[65536];
  rapidjson::FileReadStream is(fp,readBuffer,sizeof(readBuffer));
  rapidjson::Document d;
  d.ParseStream(is);
  fclose(fp);

  if (d.HasMember("Solve Potential")){
    bool solve = d["Solve Potential"].GetBool();
    if (solve)SolvePotential();
  }
  
  if (d.HasMember("Map Resolution")){
    double mapres = d["Map Resolution"].GetDouble();
    SetMapResolution(mapres);
    tolerance_ = 1.5*map_resolution_;
  }

  if (d.HasMember("Waypoints")){
    int num_waypoints = d["Waypoints"].Capacity();
    if (num_waypoints>0){
      path_.poses.clear();
      for (int i=0; i<num_waypoints;i++){
	PoseStamped p;
	p.pose.position.x = d["Waypoints"][i][0].GetDouble();
	p.pose.position.y = d["Waypoints"][i][1].GetDouble();
	path_.poses.push_back(p);
      }
      goal_.x = path_.poses[0].pose.position.x;
      goal_.y = path_.poses[0].pose.position.y;
    }
  }
}
#ifdef USE_MPI  
void SimplePathPlanner::PublishData(int root,MPI_Comm broadcast_to){
  MPI_Bcast(&updated_,1,MPI_LOGICAL,root,broadcast_to);
  MPI_Bcast(&complete_,1,MPI_LOGICAL,root,broadcast_to);
  if (updated_){
    MPI_Bcast(&current_command_.steering,1,MPI_DOUBLE,root,broadcast_to);
    MPI_Bcast(&current_command_.throttle,1,MPI_DOUBLE,root,broadcast_to);
    updated_ = false;
  }
}
#endif
void SimplePathPlanner::GetSensorData(std::vector<sensor::Sensor* >
				      &sensors){

  for (int i=0;i<(int)sensors.size();i++){
    if (sensors[i]->GetType()=="gps"){
      gps_ = (sensor::gps::Gps*)sensors[i]->clone();
    }
    if (sensors[i]->GetType()=="lidar"){
      lidar_ = (sensor::lidar::Lidar*)sensors[i]->clone();
    }
    if (sensors[i]->GetType()=="compass"){
      compass_ = (sensor::compass::Compass*)sensors[i]->clone();
    }
  }
  
  heading_ = compass_->GetHeading();
  NavSatStatus status = gps_->GetRosNavSatStatus();
  if (status.status>=0){
    NavSatFix fix = gps_->GetRosNavSatFix();
    last_position_ = position_;
    glm::dvec3 enu = gps_->GetRecieverPositionENU();
    position_.x = enu.x;
    position_.y = enu.y;
    glm::dvec3 vel = gps_->GetRecieverVelocityENU();
    velocity_.x = vel.x;
    velocity_.y = vel.y;
  }
}

MapIndex SimplePathPlanner::PointToIndex(glm::dvec2 p){
  MapIndex c;
  c.i = (int)((p.x-map_corner_.x)/map_resolution_);
  c.j = (int)((p.y-map_corner_.y)/map_resolution_);
  return c;
}

glm::dvec2 SimplePathPlanner::IndexToPoint(MapIndex c){
  glm::dvec2 p;
  p.x = (c.i+0.5)*map_resolution_ + map_corner_.x;
  p.y = (c.j+0.5)*map_resolution_ + map_corner_.y;
  return p;
}

void SimplePathPlanner::FillMap(){
  if (first_call_){ //initialize map
    double minpx = position_.x;
    double maxpx = position_.x;
    double minpy = position_.y;
    double maxpy = position_.y;
    for (int i=0;i<(int)path_.poses.size();i++){
      if (path_.poses[i].pose.position.x<minpx)
	minpx = path_.poses[i].pose.position.x;
      if (path_.poses[i].pose.position.x>maxpx)
	maxpx = path_.poses[i].pose.position.x;
      if (path_.poses[i].pose.position.y<minpy)
	minpy = path_.poses[i].pose.position.y;
      if (path_.poses[i].pose.position.y>maxpy)
	maxpy = path_.poses[i].pose.position.y;
    }

    glm::dvec2 map_center(0.5*(minpx+maxpx),0.5*(minpy+maxpy));
    double dim = sqrt( (maxpx-minpx)*(maxpx-minpx)+(maxpy-minpy)*(maxpy-minpy));
    double xlow = map_center.x - dim;
    double xhigh = map_center.x + dim;
    double ylow = map_center.y - dim;
    double yhigh = map_center.y + dim;
    map_corner_.x = xlow; map_corner_.y = ylow;
    nx_ = ((int)ceil( (xhigh-xlow)/map_resolution_));
    ny_ = ((int)ceil( (yhigh-ylow)/map_resolution_));
    //Astar map is column-major
    map_.AllocateMap(ny_,nx_,0);
    first_call_ = false;
  }
  std::vector<glm::vec3> registered_points = lidar_->GetRegisteredPoints();
  if (lidar_->IsPlanar()){
    for (int i=0;i<(int)registered_points.size(); i++){
      if (lidar_->GetIntensity(i)>0.0f){
	glm::vec2 p(registered_points[i].x,registered_points[i].y);
	MapIndex c = PointToIndex(p);
	if (c.i>=0 && c.i<nx_ && c.j>=0 && c.j<ny_){
	  map_.SetMapValue(c.i,c.j,100);
	}
      }
    }
  }
  else{
     for (int i=0;i<(int)registered_points.size(); i++){
       if (lidar_->GetIntensity(i)>0.0f){
	 glm::vec2 p(registered_points[i].x,registered_points[i].y);
	 MapIndex c = PointToIndex(p);
	 if (c.i>=0 && c.i<nx_ && c.j>=0 && c.j<ny_){
	   int cost = (int)(100.0*registered_points[i].z/max_nav_height_);
	   if (cost<10) cost = 0;
	   if (cost>100) cost = 100;
	   if (cost>map_.GetMapValue(c.i,c.j)){
	     map_.SetMapValue(c.i,c.j,cost);
	   }
	 }
       }
     }
  }
  MapIndex goal = PointToIndex(goal_);
  MapIndex pos = PointToIndex(position_);
  map_.SetGoal(goal.i,goal.j);
  map_.SetStart(pos.i,pos.j);
}
  
void SimplePathPlanner::Update(std::vector<sensor::Sensor*> &sensors, 
			       double dt){
  image_filled_ = false;
  GetSensorData(sensors);

  //check to see if we have reached the waypoint
  if (glm::length(position_-goal_)<tolerance_) {
    current_waypoint_++;
  }
  //check to see if we have reached the last waypoint
  if (current_waypoint_>= (int)path_.poses.size()){
    complete_ = true;
  }
  else {
    goal_ = path_.poses[current_waypoint_].pose.position;
  }
  
  if (local_sim_time_ < 2*local_time_step_){
    current_command_.steering = 0.0;
    current_command_.throttle = 0.0;
    local_sim_time_ += local_sim_time_ + local_time_step_;
    return;
  }

  FillMap();

  bool path_found;
  if (solve_potential_){
    path_found = map_.SolvePotential();
  }
  else{
    path_found = map_.Solve(); 
  }
  if (!path_found)std::cout<<"Could not find path to goal."<<std::endl;
  std::vector<MapIndex> path = map_.GetPath();
  glm::dvec2 next_goal(0.0,0.0);
  if (path.size()>1)next_goal = IndexToPoint(path[1]);
  
  double speed = length(velocity_);
  glm::dvec2 direction = velocity_;
  if (speed>0.0)direction = direction/speed;
  
  glm::dvec2 to_goal = next_goal - position_;
  double dist_to_goal = length(to_goal);
  if (dist_to_goal>0)to_goal = to_goal/dist_to_goal;

  double desired_yaw = atan2(to_goal.y, to_goal.x);
  double max_steer_angle = 0.436; //25 degrees
  current_command_.steering = math::clamp((desired_yaw - heading_)/
					  max_steer_angle,-1.0,1.0);
  
  double scale = math::clamp(speed/speed_,0.0,1.0);

  current_command_.throttle = 0.5*(cos(kPi*scale)+1.0);

  if (log_data_){
    WriteMap(utils::ToString(local_sim_time_)+log_file_name_);
  }
  
  local_sim_time_ += local_time_step_;
  updated_ = true;

}

void SimplePathPlanner::FillImage(){
  if (first_call_){
    image_.assign(512,512,1,3,0);
  }
  else {
    int green[3],yellow[3], blue[3], white[3], black[3];
    black[0] = 0; black[1] = 0; black[2] = 0;
    white[0] = 255; white[1] = 255; white[2]=255;
    green[0] = 0; green[1]=255; green[2] = 0;
    yellow[0] = 255; yellow[1]=255; yellow[2] = 0;
    blue[0] = 0; blue[1] = 0; blue[2] = 255;
    image_.assign(map_.GetMapWidth(),map_.GetMapHeight(),1,3,0);
    for (int i=0;i<map_.GetMapWidth();i++){
      for (int j=0;j<map_.GetMapHeight();j++){
	if(map_.GetMapValue(i,j)>0){
	  int mapcol[3];
	  mapcol[0] = (int)(255*( (1.0*map_.GetMapValue(i,j))/100.0));
	  mapcol[1] = 0; 
	  mapcol[2] = (int)(255*( (1.0*map_.GetMapValue(i,j))/100.0));
	  image_.draw_point(i,j,(int*)&mapcol);
	}
      }
    }
     
    std::vector<MapIndex> path = map_.GetPath();
    for (int n=0;n<(int)path.size();n++){
      image_.draw_point(path[n].i,path[n].j,(int*)&blue);
    }

    MapIndex g = PointToIndex(goal_);
    MapIndex p = PointToIndex(position_);
    image_.draw_point(g.i,g.j,(int*)&green);
    image_.draw_point(p.i,p.j,(int*)&yellow);

    image_.mirror("y");
    image_.resize(512,512,1,3,1);
  }
  image_filled_ = true;
}

void SimplePathPlanner::WriteMap(std::string fname){
  FillImage();
  image_.save(fname.c_str());
}

void SimplePathPlanner::Display(){
  if (!image_filled_)FillImage();
  if (first_call_)disp_.assign(512,512,"Cost Map",1);
  if (comm_rank_ ==0){
    if (!first_call_){
      disp_.display(image_);
    }
  }
}

} //namespace driver
} //namespace map
