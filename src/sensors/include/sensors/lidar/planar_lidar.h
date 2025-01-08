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
#ifndef PLANAR_LIDAR
#define PLANAR_LIDAR

#include <sensors/lidar/lidar.h>

namespace mavs{
namespace sensor{
namespace lidar{

/**
 * \class PlanarLidar
 *
 * Base class for a 2D scanning lidar, contains information needed for the
 * Ros-style LaserScan message
 *
 * \author Chris Goodin
 *
 * \date 1/25/2018
 */
class PlanarLidar : public Lidar{
 public:
  PlanarLidar(){}

  /// Destructor
  ~PlanarLidar() {
	  CloseDisplay();
  }

  //copy contstructor
  PlanarLidar(const PlanarLidar &s){
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
  
  /**
   * Returns a 2D laser scan data message 
   */
  LaserScan GetLaserScan();

  /// make a deep copy
  virtual PlanarLidar* clone() const{
    return new PlanarLidar(*this);
  }

	/**
	* Set the scan properties of the lidar
	* \param ang_min The miniumum scan angle
	* \param ang_max The maximum scan angle
	* \param ang_res The scan resolution
	*/
	void SetScanProperties(float ang_min, float ang_max, float ang_res);

 protected:
  float angle_min_;
  float angle_max_;
  float angle_increment_;
	float repitition_rate_;
};

}// namespace lidar
} // namespace sensor
} // namespace mavs

#endif
