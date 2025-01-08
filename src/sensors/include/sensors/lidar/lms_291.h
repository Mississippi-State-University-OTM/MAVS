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
#ifndef LMS_291_S05
#define LMS_291_S05

#include <sensors/lidar/planar_lidar.h>

namespace mavs{
namespace sensor{
namespace lidar {

/**
 * \class Lms291_S05
 *
 * Simulates a SICK LMS291-S05 sensor. See LMS user manual for details
 *
 * \author Chris Goodin
 *
 * \date 1/25/2018
 */
class Lms291_S05 : public PlanarLidar {
 public:
  /// Sensor constructor
  Lms291_S05();

  /// Destructor
  ~Lms291_S05() {
	  CloseDisplay();
  }

  /// copy contstructor
  Lms291_S05(const Lms291_S05 &s){
    position_ = s.position_;
    velocity_ = s.velocity_;
    offset_ = s.offset_;
    orientation_ = s.orientation_;
    relative_orientation_ = s.relative_orientation_;
    registered_points_ = s.registered_points_;
    look_to_ = s.look_to_;
    look_side_ = s.look_side_;
    look_up_ = s.look_up_;
    distances_ = s.distances_;
    intensities_ = s.intensities_;
    points_ = s.points_;
    max_range_ = s.max_range_;
    min_range_ = s.min_range_;
    rotation_rate_ = s.rotation_rate_;
    recharge_dt_ = s.recharge_dt_;
		repitition_rate_ = s.repitition_rate_;
    mode_ = s.mode_;
    cutoff_len_ = s.cutoff_len_;
    is_planar_ = s.is_planar_;
    angle_min_ = s.angle_min_;
    angle_max_ = s.angle_max_;
    angle_increment_ = s.angle_increment_;
  }

  /// Puts the sensor in the 1-degree resolution mode.
  void OneDegreeResolution();

  /// Puts the sensor in 0.5-degree resolution mode.
  void HalfDegreeResolution();

  /// Puts the sensor in 0.25 degree resolution mode.
  void QuarterDegreeResolution();

  /// make a deep copy
  virtual Lms291_S05* clone() const{
    return new Lms291_S05(*this);
  }

};

} //namespace lidar
} //namespace sensor
} //namespace mavs

#endif
