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
#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <sensors/lidar/planar_lidar.h>

namespace mavs{
namespace sensor{

/**
 * \class ObjectDetector
 *
 * Notional detector that returns a list of obstacles.
 * Inherits from the planar lidar base class.
 * Extracts a list of objects from a planar lidar scan
 *
 * \author Chris Goodin
 *
 * \date 1/17/2019
 */
class ObjectDetector : public lidar::PlanarLidar{
 public:

	 /// Create the detector
	 ObjectDetector();

	 ///Inherited from base sensor class to update the detector
	 void Update(environment::Environment *env, double dt);

	 /// Return a list of obstacles
	 std::vector<Obstacle> GetObstacles();

 private:


};

} // namespace sensor
} // namespace mavs

#endif
