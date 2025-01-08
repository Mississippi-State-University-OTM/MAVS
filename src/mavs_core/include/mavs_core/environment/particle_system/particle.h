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
 * \class Particle
 *
 * The particle class is used in simulations of particle systems.
 * Particle objects hold basic information about the particle as well
 * as a fast intersection calculation. 
 *
 * \author Chris Goodin
 *
 * \date 5/22/2018
 */
#ifndef PARTICLE_H
#define PARTICLE_H

#include <glm/glm.hpp>

namespace mavs{
namespace environment{
  
class Particle{
 public:
  /// Create a particle
  Particle();

  /**
   * Determine ray intersection with a particle. If there is an intersection, 
   * the. Function returns a glm::vec2. The first component of the vec2 is
   * a float in the range 0 to 1, quantifying the
   * distance the ray passes from the center of the particle, scaled by the
   * particle radius. The second is the range from the camera to the surface
   * of the particle.
   * \param orig Origin of the ray, in world coordinates.
   * \param dir Normalized direction of the ray, in world ENU coordinates.
   */
  glm::vec2 GetIntersection(glm::vec3 orig, glm::vec3 dir);

  /// Position of the particle in world ENU coordiates
  glm::vec3 position_;

  /// Velocity of the particle (m/s) in world ENU coordinates.
  glm::vec3 velocity_;

  /// Color of the particle in RGB reflectance (0,1)
  glm::vec3 color_;

  /// Transparency of the particle (0,1). 0=opaque, 1=clear
  float transparency_;

  /// Age of the particle in seconds
  float age_;

  /// Radius of the particle in meters
  float radius_;
};

} // namespace environemnt
} // namespace mavs
 
#endif
