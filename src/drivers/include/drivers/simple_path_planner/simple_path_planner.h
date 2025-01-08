/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
/**
 * \class SimplePathPlanner
 * 
 * Example that uses LIDAR data to create an occupancy grid / cost map
 * and do simple path planning  using A*
 *
 * \author Chris Goodin
 *
 * \date 1/22/2018
 */
#ifndef SIMPLE_PATH_PLANNER_H
#define SIMPLE_PATH_PLANNER_H

#include <glm/glm.hpp>

#include <drivers/driver.h>
#include <drivers/simple_path_planner/astar.h>
#include <sensors/gps/gps.h>
#include <sensors/lidar/lidar.h>
#include <sensors/compass/compass.h>

#include <CImg.h>

namespace mavs{
namespace driver{

/** 
 * Example of driver that uses LIDAR data to create cost  map
 */
class SimplePathPlanner : public Driver {
 public:
  /// Create the waypoint follower
  SimplePathPlanner();

  /// Update the waypoint follower based on the current vehicle state
  void Update(std::vector<sensor::Sensor*> &sensors, double dt);

#ifdef USE_MPI
  /// Publish the current driving command to the communicator
  void PublishData(int root, MPI_Comm broadcast_to);
#endif
  
  /// Set the goal point in lat/long
  void SetGoalLatLong(double latitude, double longitude){
    coordinate_system::LLA goal;
    goal.latitude = latitude;
    goal.longitude = longitude;
    goal.altitude = 0.0;
    coordinate_system::ENU enu = coord_converter_.LLA2ENU(goal);
    goal_.x = enu.x;
    goal_.y = enu.y;
  }

  /// Set the goal in local ENU coordinates
  void SetGoalEnu(double easting, double northing){
    goal_.x = easting;
    goal_.y = northing;
  }

  /// Set the desired speed, in m/s
  void SetSpeed(double speed) {speed_ = speed;}

  /// Set the desired resolution of the grid used in the A* calcualtion (meters)
  void SetMapResolution(double res){map_resolution_ = res;}

  /// Write the current map to a file
  void WriteMap(std::string fname);

  /// Set the map size in meters
  void SetMapSize(double size){map_size_ = size;}

  /// Displays the current A* map
  void Display();

  /**
   * Set the maximum navigable obstacle height, in meters
   \param h Obstacle height in meters
   **/
  void SetMaxNavHeight(double h){
    max_nav_height_ = h;
  }

  /// Call this to use potential path planner instead of A*
  void SolvePotential(){solve_potential_ = true;}

  /// Override load function from communicator base class
  void Load(std::string input_file);
  
 private:
  double speed_;
  double tolerance_;
  double map_resolution_;
  double max_nav_height_;
  mavs::Path path_;
  glm::dvec2 goal_;
  int current_waypoint_;
  
  glm::dvec2 position_;
  glm::dvec2 last_position_;
  glm::dvec2 velocity_;
  double heading_;
 
  sensor::gps::Gps *gps_;
  sensor::lidar::Lidar *lidar_;
  sensor::compass::Compass *compass_;

  void GetSensorData(std::vector<sensor::Sensor*> &sensors);

  // Methods and data for displaying cost map
  void FillImage();
  cimg_library::CImg<int> image_; 
  cimg_library::CImgDisplay disp_;
  bool image_filled_;
  
  //Methods and data related to A* map
  void FillMap();
  //bool SolveMap();
  //std::vector<glm::vec3> PointsToSlope(std::vector<glm::vec3> &points);
  MapIndex PointToIndex(glm::dvec2 p);
  glm::dvec2 IndexToPoint(MapIndex c);
  Astar map_;
  glm::dvec2 map_corner_;
  bool first_call_;
  int nx_, ny_;
  double map_size_;
  bool solve_potential_;
};

} //namespace driver
} //namespace mavs

#endif
