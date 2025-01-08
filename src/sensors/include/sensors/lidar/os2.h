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
#ifndef OUSTER_OS2
#define OUSTER_OS2

#include <sensors/lidar/lidar.h>

namespace mavs{
namespace sensor{
namespace lidar{

/**
 * \class OusterOS2
 *
 * Ouster OS2 sensor, see
 * https://www.ouster.io/product-os2/
 *
 * \author Chris Goodin
 *
 * \date 1/22/2019
 */
class OusterOS2 : public Lidar{
 public:
  /// OusterOS2 Constructor
  OusterOS2();

  /**
	 * Conctructor which specifies rotation rate and mode
	 * \param rot_hz Rotation rate (hz) of the sensor, for the OS2, must be 10 Hz or 20 Hz
	 * \param mode Horizontal resolution per scan, 512, 1024, or 2048. If mode=2048, then rot_hz must be 10
	 */
  OusterOS2(float rot_rate, int mode){SetRotationRate(rot_rate, mode);}
  
  /// Destructor
  ~OusterOS2() {
	  CloseDisplay();
  }

  /// OusterOS2 copy contstructor
  OusterOS2(const OusterOS2 &s){
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
   * Set the rotation rate of the sensor in Hz.
   * \param rot_hz Rotation rate (hz) of the sensor, for the OS2, must be 10 Hz or 20 Hz
   * \param mode Horizontal resolution per scan, 512, 1024, or 2048. If mode=2048, then rot_hz must be 10
   */
  void SetRotationRate(float rot_hz, int mode);

  void WriteOs2Pcd(std::string fname);

  void WriteOs2LabeledPcd(std::string fname);
};

}//namespace lidar
} //namespace sensor
} //namespace mavs

#endif
