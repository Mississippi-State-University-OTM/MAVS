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
#ifndef M8_
#define M8_

#include <sensors/lidar/lidar.h>

namespace mavs{
namespace sensor{
namespace lidar{

/**
 * \class M8
 *
 * Quanergy M8 sensor.
 *
 * \author Chris Goodin
 *
 * \date 2/23/2018
 */
class MEight : public Lidar{
 public:
  /// Constructor
  MEight();

  MEight(float rot_rate){SetRotationRate(rot_rate);}
 
  /// Destructor
  ~MEight() {
	  CloseDisplay();
  }

  /// Lidar copy contstructor
  MEight(const MEight &s){
    position_ = s.position_;
    velocity_ = s.velocity_;
    offset_ = s.offset_;
    orientation_ = s.orientation_;
    relative_orientation_ = s.relative_orientation_;
    look_to_ = s.look_to_;
    look_side_ = s.look_side_;
    look_up_ = s.look_up_;
    distances_ = s.distances_;
    intensities_ = s.intensities_;
    points_ = s.points_;
    registered_points_ = s.registered_points_;
    max_range_ = s.max_range_;
    min_range_ = s.min_range_;
    rotation_rate_ = s.rotation_rate_;
    recharge_dt_ = s.recharge_dt_;
    mode_ = s.mode_;
    cutoff_len_ = s.cutoff_len_;
    is_planar_ = s.is_planar_;
  }
  
  /**
   * Set the rotation rate of the sensor in Hz. 10 Hz reccommended for M8.
   * \param rot_hz Rotation rate (hz) of the sensor, [5,20].
   */
  void SetRotationRate(float rot_hz);
  
};
  
}//namespace lidar
} //namespace sensor
} //namespace mavs

#endif
