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
 * \class Primitive
 *
 * Base class for primitives like sphere and bounding box in the simple ray 
 * tracer. 
 *
 * \author Chris Goodin
 *
 * \date 12/18/2017
 */
#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <glm/glm.hpp>
#include <raytracers/raytracer.h>
#include <raytracers/material.h>

namespace mavs{
namespace raytracer{
    
class Primitive {
 public:
  /**
   * Virtual method for primitive intersections. 
   \param origin Origin of the intersecting ray.
   \param direction Direction of the intersecting ray.
   */
  virtual Intersection GetIntersection(glm::vec3 origin, glm::vec3 direction){
    Intersection inter;
    inter.dist = -1;
    return inter;
  }

  /**
   * Sets the geometric center of the primitive to (x,y,z).
   */
  void SetPosition(float x, float y, float z){
    position_.x = x;
    position_.y = y;
    position_.z = z;
  }

	/**
	* Get the geometric center of the primitive to (x,y,z).
	*/
	glm::vec3 GetPosition() {
		return position_;
	}

  /**
   * Sets the reflectance of the primitive in RGB reflectance. Values should 
   * range from 0 to 1.0
   */
  void SetColor(float r, float g, float b){
    color_.x = r;
    color_.y = g;
    color_.z = b;
		material_.kd = color_;
  }
  
	/**
	* Set the material of the primitive
	* \param mat The material to use
	*/
	void SetMaterial(Material &mat) { 
		material_ = mat; 
		color_ = mat.kd;
	}

	/// Return the primitive material
	Material GetMaterial() { return material_; }

	/// Return the name of the primitive
	std::string GetName() { return name_; }

 protected:
  glm::vec3 position_;
  glm::vec3 color_;
	Material material_;
	std::string name_;
};

}// namespace raytracer
}//namespace mavs
#endif
