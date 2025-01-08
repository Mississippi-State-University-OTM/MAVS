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
 * \class SimpleWaypointFollower
 * 
 * As a very simple example of a mavs::driver implementation, this class
 * follows a user defined waypoint path using only GPS. 
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */
#ifndef SIMPLE_WAYPOINT_FOLLOWER_H
#define SIMPLE_WAYPOINT_FOLLOWER_H

#include <glm/glm.hpp>

#include <drivers/driver.h>

#include <sensors/gps/gps.h>

namespace mavs{
namespace driver{

/** 
 * This class is provided only as an example of a driver implementation.
 * It should not be used for any real calculations of autonomous performance.
 */
class SimpleWaypointFollower : public Driver {
 public:
  /// Create the waypoint follower
  SimpleWaypointFollower();

  /// Update the waypoint follower based on the current vehicle state
  void Update(std::vector<sensor::Sensor*> &sensors, double dt);
#ifdef USE_MPI
  /// Publish the current driving command to the communicator
  void PublishData(int root, MPI_Comm broadcast_to);
#endif
  /// Set the path to follow, in local ENU coordinates.
  void SetPath(mavs::Path path){path_.CreateFromPathMsg(path);}

  /// Set the desired speed, in m/s
  void SetSpeed(double speed) {speed_ = speed;}

  /// Override the virtual driver virtual function
  bool Complete();

  /// Override the virtual load function for the coumminicator.
	void Load(std::string input_file) { path_.Load(input_file); }
  
 private:
  //mavs::Path path_;
	 Waypoints path_;
  double speed_;
  double tolerance_;
  int current_waypoint_;
 
  glm::dvec2 position_;
  glm::dvec2 last_position_;
  glm::dvec2 velocity_;
 
  sensor::gps::Gps *gps_;
 
  void GetSensorData(std::vector<sensor::Sensor*> &sensors);

  double local_time_;
};

} //namespace driver
} //namespace mavs

#endif
