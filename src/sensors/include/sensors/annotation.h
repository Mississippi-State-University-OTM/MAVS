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
* \class Annotations
*
* Class for adding segmentation and
* annotations to images
*
* \author Daniel Carruth
* \author Chris Goodin
*
* \date 7/5/2018
*/

#ifndef ANNOTATION_H
#define ANNOTATION_H

#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace mavs {
namespace sensor {
/**
* The annotation class provides methods for documenting
* annotations for camera, lidar, and radar data. 
*/
class Annotation {
 public:
	/// Constructor for a blank annotation
	Annotation();

	/** 
	* Create a camera annotation
	* \param name The name of the object / annotation.
	* \param min_x The minimum horizontal pixel coordinate of the annotation in the image.
	* \param min_y The minimum vertical pixel coordinate of the annotation in the image.
	* \param max_x The maximum horizontal pixel coordinate of the annotation in the image.
	* \param max_y The maximum vertical pixel coordinate of the annotation in the image.
	*/
  Annotation(std::string name, int min_x, int min_y, int max_x, int max_y);

	/**
	* Create a camera annotation for semantically labeled images
	* \param name The name of the object / annotation.
	* \param r The red color of the semantic label.
	* \param g The green color of the semantic label.
	* \param b The blue color of the semantic label.
	*/
	Annotation(std::string name, int r, int g, int b);

	/**
	* Create a point cloud annotation based on the location of the point
	* \param name The name of the object / annotation.
	* \param p The x-y-z location of the point
	* \param class_num The number of the semantic class
	*/
	Annotation(std::string name, glm::vec3 p, int class_num);

	/// Get the name of the annotation
	std::string GetName() { return name_; }

	/// Get the number of the annotation classification
	int GetClassNum() { return class_number_; }

	/// Get the lower left corner of the image annotation 
	glm::ivec2 GetLLPixel() { return pix_ll_; }

	/// Get the upper right corner of the image annotation 
	glm::ivec2 GetURPixel() { return pix_ur_; }

	/// Get the lower left corner of the point cloud annotation 
	glm::vec3 GetLLCorner() { return ll_; }

	/// Get the upper right corner of the point cloud annotation 
	glm::vec3 GetURCorner() { return ur_; }

	/// Get the color of the sematic label of the annotation
	glm::ivec3 GetColor() { return color_; }

	/**
	* Set the lower left corner of the image annotation
	* \param i Horizontal pixel coordinate of the annotation
	* \param j Vertical pixel coordinate of the annotation
	*/
	void SetLLPixel(int i, int j) {
		pix_ll_.x = i;
		pix_ll_.y = j;
	}

	/**
	* Set the upper right corner of the image annotation
	* \param i Horizontal pixel coordinate of the annotation
	* \param j Vertical pixel coordinate of the annotation
	*/
	void SetURPixel(int i, int j) {
		pix_ur_.x = i;
		pix_ur_.y = j;
	}

	/**
	* Set the lower left corner of the point cloud annotation
	* \param x X / east coordinate of the annotation
	* \param y Y / north coordinate of the annotation
	* \param z Z / vertical coordinate of the annotation
	*/
	void SetLLCorner(float x, float y, float z) {
		ll_ = glm::vec3(x, y, z);
	}

	/**
	* Set the lower left corner of the point cloud annotation
	* \param v vector coordinate of the lower left corner
	*/
	void SetLLCorner(glm::vec3 v) {
		ll_ = v;
	}

	/**
	* Set the top upper right corner of the point cloud annotation
	* \param x X / east coordinate of the annotation
	* \param y Y / north coordinate of the annotation
	* \param z Z / vertical coordinate of the annotation
	*/
	void SetURCorner(float x, float y, float z) {
		ur_ = glm::vec3(x, y, z);
	}

	/**
	* Set the upper right corner of the point cloud annotation
	* \param v vector coordinate of the upper right corner
	*/
	void SetURCorner(glm::vec3 v) {
		ur_ = v;
	}

	/**
	* Set the orientation of the annotation
	* \param q Orientation to set
	*/
	void SetOrientation(glm::quat q) {
		orientation_ = q;
	}

	/// Return the orientation of the annotated object
	glm::quat GetOrientation() {
		return orientation_;
	}

private:
	std::string name_;
	glm::ivec2 pix_ll_;
	glm::ivec2 pix_ur_;
	glm::vec3 ll_;
	glm::vec3 ur_;
	int class_number_;
	glm::ivec3 color_;
	glm::quat orientation_;
};

} //namespace sensor
} //namespace mavs

#endif