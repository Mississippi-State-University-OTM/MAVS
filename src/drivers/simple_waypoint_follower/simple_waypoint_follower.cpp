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

#include <drivers/simple_waypoint_follower/simple_waypoint_follower.h>
#ifdef USE_MPI
#include >mpi.h>
#endif
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <mavs_core/math/constants.h>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace driver{

SimpleWaypointFollower::SimpleWaypointFollower(){
  speed_ = 2.0; //m/s
  current_waypoint_ = 0;
  tolerance_ = 1.0; //meters
  local_time_ = 0.0;
}

/*
void SimpleWaypointFollower::Load(std::string input_file){
  FILE* fp = fopen(input_file.c_str(),"rb");
  char readBuffer[65536];
  rapidjson::FileReadStream is(fp,readBuffer,sizeof(readBuffer));
  rapidjson::Document d;
  d.ParseStream(is);
  fclose(fp);

  if (d.HasMember("Waypoints")){
    int num_waypoints = d["Waypoints"].Capacity();
    if (num_waypoints>0){
      path_.poses.clear();
      for (int i=0;i<num_waypoints;i++){
	mavs::PoseStamped p;
	p.pose.position.x = d["Waypoints"][i][0].GetDouble();
	p.pose.position.y = d["Waypoints"][i][1].GetDouble();
	path_.poses.push_back(p);
      }
    }
  }

}
*/
  
#ifdef USE_MPI  
void SimpleWaypointFollower::PublishData(int root,MPI_Comm broadcast_to){
  MPI_Bcast(&current_command_.steering,1,MPI_DOUBLE,root,broadcast_to);
  MPI_Bcast(&current_command_.throttle,1,MPI_DOUBLE,root,broadcast_to);
  MPI_Bcast(&complete_,1,MPI_LOGICAL,root,broadcast_to);
}
#endif
  
void SimpleWaypointFollower::GetSensorData(std::vector<sensor::Sensor* >
					   &sensors){
  NavSatFix fix;
  for (int i=0;i<(int)sensors.size();i++){
    if (sensors[i]->GetType()=="gps"){
      gps_ = (sensor::gps::Gps*)sensors[i]->clone();
      fix = gps_->GetRosNavSatFix();
    }
  }
  
  glm::dvec3 enu = gps_->GetRecieverPositionENU();
  position_.x = enu.x;
  position_.y = enu.y;
  glm::dvec3 vel = gps_->GetRecieverVelocityENU();
  velocity_ = vel;
}

bool SimpleWaypointFollower::Complete(){
  return complete_;
}
 
void SimpleWaypointFollower::Update(std::vector<sensor::Sensor*> &sensors, 
				    double dt){
  GetSensorData(sensors);

  if (path_.NumWaypoints()<=0){
    complete_ = true;
    return;
  }

  local_time_ += dt;
  if (local_time_<=dt){
    current_command_.throttle = 0.0;
    current_command_.steering = 0.0;
    return;
  }

  double speed = length(velocity_);
  glm::dvec2 direction = velocity_;
  if (speed>0.0)direction = direction/speed;
  
 //glm::dvec2 goal(path_.poses[current_waypoint_].pose.position.x,
	//	 path_.poses[current_waypoint_].pose.position.y);
	glm::dvec2 goal = path_.GetWaypoint(current_waypoint_);
  glm::dvec2 to_goal = goal-position_;
  double dist_to_goal = length(to_goal);

  to_goal = to_goal/dist_to_goal;

  double yaw = atan2(direction.y, direction.x);
  double desired_yaw = atan2(to_goal.y, to_goal.x);

  double max_steer_angle = 0.2; //12 degrees
  current_command_.steering = math::clamp((desired_yaw - yaw)/
					  max_steer_angle,-1.0,1.0);
  
  double scale = math::clamp(speed/speed_,0.0,1.0);

  current_command_.throttle = 0.5*(cos(kPi*scale)+1.0);

  if (dist_to_goal<tolerance_) {
    current_waypoint_++;
  }
  if (current_waypoint_>= (int)path_.NumWaypoints()){
    complete_ = true;
  }  

  local_sim_time_ += local_time_step_;
}


} //namespace mavs
} //namespace driver
