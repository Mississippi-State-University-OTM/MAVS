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
 * \class Aabb
 *
 * Primitive axis-aligned bounding box object for the simple raytracer. 
 * Defined by three size dimensions.
 *
 * \author Chris Goodin
 *
 * \date 12/18/2017
 */
#ifndef AABB_H
#define AABB_H

#include <raytracers/simple_tracer/primitive.h>

namespace mavs{
namespace raytracer{

struct Triangle{
  glm::vec3 vertex0;
  glm::vec3 vertex1;
  glm::vec3 vertex2;
  glm::vec3 normal;
  void CalculateNormal(){
    normal = glm::cross(vertex2-vertex0,vertex1-vertex0);
    normal = glm::normalize(normal);
  }
};
  
class Aabb : public Primitive{
 public:
  Aabb();
  Aabb(glm::vec3 position, glm::vec3 dimensions);
  Aabb(float px, float py, float pz, float sx, float sy, float sz);
  ~Aabb();

  ///Overwritten virtual method from the Primitive base class
  Intersection GetIntersection(glm::vec3 origin, glm::vec3 direction);

  ///Set the size dimensions in (x,y,z), meters
  void SetSize(double xdim, double ydim, double zdim);
  
 private:
  glm::dvec3 bounds_[2];
  std::vector<Triangle> triangles_;
  
};

} //namespace primitive
} //namespace ray-tracer
 
#endif
