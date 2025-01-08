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
* \class BoundingBox
*
* A class for manipulating 3D, axis aligned bounding boxes
*
* \author Chris Goodin
*
* \date 7/12/2018
*/
#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H
#include <vector>
#include <glm/glm.hpp>

namespace mavs {
namespace raytracer {

class BoundingBox {
public:
	/// Create a blank bounding box
	BoundingBox();

	/**
	* Create a bounding box and define the corners
	* \param ll x-y-z vector with coordinates for the lower left corner
	* \param ur x-y-z vector with coordinates for the upper right corner
	*/
	BoundingBox(glm::vec3 ll, glm::vec3 ur);

	/**
	* Create a bounding box and define the corners
	* \param llx x-component of the lower left corner
	* \param lly y-component of the lower left corner
	* \param llz z-component of the lower left corner
	* \param urx x-component of the upper right corner
	* \param ury y-component of the upper right corner
	* \param urz z-component of the upper right corner
	*/
	BoundingBox(float llx, float lly, float llz,
		float urx, float ury, float urz);

	/**
	* Transform the bounding box and recalculate the corners
	* \param aff_rot The affine rotation/translation/scale transformation matrix
	*/
	void Transform(glm::mat3x4 aff_rot);

	/// Return the center of the box
	glm::vec3 GetCenter() { return center_; }

	/// Return the lower left corner of the box
	glm::vec3 GetLowerLeft() { return lower_left_; }

	/// Return the upper right corner of the box
	glm::vec3 GetUpperRight() { return upper_right_; }

	/// Return a list containing all 8 corners of the box
	std::vector<glm::vec3> GetCorners() {return corners_;}

	/// Print the lower left and upper right corners of the box to stdout
	void Print();

private:
	void CalcPoints();
	glm::vec3 lower_left_;
	glm::vec3 upper_right_;
	glm::vec3 center_;
	std::vector<glm::vec3> corners_;
};

} //namespace raytracer 
} //namespace mavs
#endif