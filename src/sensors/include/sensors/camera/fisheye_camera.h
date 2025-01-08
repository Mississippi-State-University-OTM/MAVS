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
#ifndef FISHEYE_H
#define FISHEYE_H

#include <sensors/camera/rgb_camera.h>

namespace mavs {
namespace sensor {
namespace camera {

class FisheyeCamera : public RgbCamera {
public:
	/// Fisheye camera constructor
	FisheyeCamera();

	///Inherited sensor update method from the sensor base class
	void Update(environment::Environment *env, double dt);

	/**
	* Inherited from Camera base class.
	* Convert a point in world coordinates to pixel coordinates
	* returns true if point is in frame, false if it is not
	* \param point_world Point in world coordinates
	* \param pixel Calculated point in pixel coordinates
	*/
	bool WorldToPixel(glm::vec3 point_world, glm::ivec2 &pixel);

private:
	/**
	* Distort a point in the image plane according to the fisheye projectino
	* All units are in meters (not pixels)
	* \param pixel The undistorted pixel position in the image plane,
	* plus the z component as the distorted angle from the principal axis
	*/
	glm::vec3 Distort(glm::vec2 pixel);
};

} //namespace camera
} //namespace sensor
} //namespace mavs

#endif
