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
 * \class Gps
 * 
 * Simulates a digital compass
 *
 * \author Chris Goodin
 *
 * \date 1/24/2018
 */

#ifndef COMPASS_H
#define COMPASS_H

#include <glm/glm.hpp>

#include <sensors/sensor.h>
#include <mavs_core/math/constants.h>

#ifdef USE_MPI
#include <mpi.h>
#endif

namespace mavs{
namespace sensor{
namespace compass{

class Compass : public Sensor {
 public:
  /// Create the simulated digital compass
  Compass();

  /// Compass destructor
  ~Compass();
  
  Compass(const Compass &comp){
    rms_error_degrees_ = comp.rms_error_degrees_;
    rms_error_radians_ = comp.rms_error_radians_;
    current_heading_ = comp.current_heading_;
  }

  /** 
   * Update the Compass sensor.
   * \param env The current environment.
   * \param dt The amount of time to simulate (seconds).
   */
  void Update(environment::Environment *env, double dt);

  /**
   * Get the heading (radians CCW from east)
   * calculated by the GPS magnetometer, if it has one.
   * 0 = east, 90 = north, -90 = south, 180 = west
   */
  double GetHeading();
#ifdef USE_MPI
  /** 
   * Publish the current heading to the simulation 
   */
  void PublishData(int root, MPI_Comm broadcast_to);
#endif  
  /// make a deep copy
  virtual Compass* clone() const{
    return new Compass(*this);
  }

  void SetRmsErrorDegrees(double err){
    rms_error_degrees_ = err;
    rms_error_radians_ = kDegToRad*err;
  }

 private:
  double rms_error_degrees_;
  double rms_error_radians_;
  double current_heading_;
 
};

} //namespace compass
} //namespace sensor
} //namespace mavs

#endif
