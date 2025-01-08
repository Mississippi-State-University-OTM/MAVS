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
 * \class Driver
 *
 * The base class for drivers, autonomous and otherwise.
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */

#ifndef DRIVER_H
#define DRIVER_H
#include <iostream>
#include <mavs_core/communicator.h>
#include <mavs_core/messages.h>
#include <sensors/sensor.h>
#include <mavs_core/coordinate_systems/coord_conversions.h>
namespace mavs{
namespace driver{

class Driver : public Communicator {
 public: 
  Driver() : complete_(false) {};
  
  /**
   * The update routine for either a simulated human driver, who is assumed
   * to have perfect knowledge of the current vehicle state, or an 
   * autonomous driver, who only knows about sensor information.
   * \param sensors List of sensors the autonomous driver will use.
   * \param dt The time step (seconds) to update.
   */
  virtual void Update(std::vector<sensor::Sensor*> &sensors, double dt){}

  /**
   * The driver must track the progress of the simulation and set 
   * complete_=true when the stop condition is met.
   */
  virtual bool Complete(){return complete_;}

  /// Returns the current commanded throttle, from 0 to 1.
  virtual double GetThrottle(){return current_command_.throttle;}

  /// Returns the current commanded steering angle, in radians.
  virtual double GetSteering(){return current_command_.steering;}

  /** 
   * Sets the local origin for the driver for any path following or obstacle
   * avoidance maneuvers.
   * \param latitude Latitude (decimal degrees) of the local origin
   * \param longitude Longitude (decimal degrees) of the local origin
   */
  void SetOrigin(double latitude, double longitude){
    local_origin_.latitude = latitude;
    local_origin_.longitude = longitude;
    local_origin_.altitude = 0.0;
    coord_converter_.SetLocalOrigin(local_origin_);
  }

 protected:
  mavs::DrivingCommand current_command_;
  bool complete_;

  coordinate_system::LLA local_origin_;
  coordinate_system::CoordinateConverter coord_converter_;
};

} //namespace mavs
} //namespace driver

#endif
