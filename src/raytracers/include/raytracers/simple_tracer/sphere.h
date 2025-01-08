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
 * \class Sphere
 *
 * Primitive sphere object for the simple raytracer. Defined by a radius.
 *
 * \author Chris Goodin
 *
 * \date 12/18/2017
 */
#ifndef SPHERE_H
#define SPHERE_H

#include <raytracers/simple_tracer/primitive.h>

namespace mavs{
namespace raytracer{
  
class Sphere : public Primitive {
 public:
  Sphere();
  ~Sphere();
  
  Intersection GetIntersection(glm::vec3 origin, glm::vec3 direction);

  void SetRadius(float r){
    radius_ = r;
    r2_ = r*r;
  }
  
 protected:
  float radius_;
  float r2_;

};

} //namespace raytracer
} //namespace mavs
 
#endif
