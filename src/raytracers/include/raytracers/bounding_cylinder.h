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
#ifndef BOUNDING_CYLINDER_H
#define BOUNDING_CYLINDER_H

#include <glm/glm.hpp>

namespace mavs {
namespace raytracer {

class BoundingCylinder {
public:
	BoundingCylinder();

	BoundingCylinder(glm::vec3 base_center, float radius, float height);

	BoundingCylinder(float bsx, float bsy, float bsz,
		float radius, float height);

	void Transform(glm::mat3x4 aff_rot);

	glm::vec3 GetCenter() { return center_; }

	glm::vec3 GetBaseCenter() { return base_center_; }


private:
	glm::vec3 base_center_;
	float radius_;
	float height_;
	glm::vec3 center_;
};

} //namespace raytracer 
} //namespace mavs
#endif