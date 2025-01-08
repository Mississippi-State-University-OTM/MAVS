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
 * \class Satellite
 * 
 * Initializes the position and velocity of a GPS satellite based on broadcast
 * ephemeris data. Moves the satellite in a constant-radius, circular orbit. 
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */

#ifndef SATELLITE_H_
#define SATELLITE_H_
#include <glm/vec3.hpp>
#include <rinex.h>

namespace mavs{
namespace sensor{
namespace gps{

class Satellite{
 public: 
  ///Create a satelllite.
  Satellite();

  ///Move the satellite along its orbit over time interval dt (seconds)
  void Update(double dt);

  /** 
   * Initialize the position and velocity of a satellite from the broadcast
   * ephemeris data. 
   */
  void Init(NGSrinex::PRNBlock &prb);

  ///Returns the current satellite position in ECEF coordinates.
  glm::dvec3 GetPosition(){return position_;}

  /// Prints the current satellite state to stdout
  void PrintState();

  ///Sets the initial position of the satellite, in ECEF coordinates
  void SetPosition(double x, double y, double z){
    position_.x = x;
    position_.y = y;
    position_.z = z;
  }

  ///Sets the initial velocity of the satellite, in ECEF coordinates
  void SetVelocity(double x, double y, double z){
    velocity_.x = x;
    velocity_.y = y;
    velocity_.z = z;
  }

  ///Defines the plane the satellite orbits in by its normal, in ECEV coords.
  void SetNormal(double x, double y, double z){
    orbit_normal_.x = x;
    orbit_normal_.y = y;
    orbit_normal_.z = z;
  }

  ///Sets the satellite orbit radius, in meters
  void SetOrbitRadius(double r){orbit_radius_=r;}

  ///Set the orbital (angular) velocity of the satellite
  void SetOrbitVelocity(double v){orbit_velocity_ = v;}

  ///Set the clock time of the satellite
  void SetSatelliteTime(double t){satellite_time_ = t;}
  
 private:
  glm::dvec3 position_;
  glm::dvec3 velocity_;
  glm::dvec3 orbit_normal_;
  double orbit_radius_;
  double orbit_velocity_;
  double satellite_time_;
};

} //namespace gps
} //namespace sensor
} //namespace mavs

#endif
