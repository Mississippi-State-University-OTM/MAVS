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
 * \class ParticleSystem
 *
 * The particle system class can be used to simulate the motion and 
 * appearance of features such as dust, smoke, and steam. 
 *
 * \author Chris Goodin
 *
 * \date 5/22/2018
 */
#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include <vector>

#include <glm/glm.hpp>

#include <mavs_core/environment/particle_system/particle.h>

namespace mavs{
namespace environment{

class ParticleSystem{
 public:
  /// Construct an empty particle system.
  ParticleSystem();

  /**
   * Advance the particle system. Will move the particles, expand them, and
   * delete old particles.
   * \param dt The length of the time step in seconds.
   */
  void Advance(float dt);

  /**
   * Add particles at random locations inside a sphere.
   * \param center The center of the sphere in world coordinates.
   * \param radius The radius of the sphere in meters.
   * \param num_particles The number of particles to add.
   */
  void AddParticlesInSphere(glm::vec3 center, float radius, int num_particles);

  /**
   * Set the maximum lifetime of the particle, in seconds. Particles will be
   * deleted after they have existed past their lifetime.
   * \param life Lifetime of the particle in seconds.
   */
  void SetLifetime(float life){lifetime_ = life;}

  /**
   * Scale gravity by a constant factor. Will result in particles having a 
   * floating-like appearance.
   * \param gfac Factor by which to scale gravity, (0,1).
   */
  void SetGravityFactor(float gfac){gravity_fac_ = gfac;}

  /**
   * Set the expansion rate of the particles. Simulates diffusion.
   * \param rate Increase in radius of the particle with time, in m/s
   */
  void SetExpansionRate(float rate){expansion_rate_ = rate;}

  /**
   * Set the initial particle radius.
   * \param rad Radius of the particle in meters
   */
  void SetInitialRadius(float rad){init_rad_ = rad;}

  /**
   * Define the initial color of the dust in RGB reflectance.
   * \param r Red band reflectance, (0,1)
   * \param g Green band reflectance (0,1)
   * \param b Blue band reflectance (0,1)
   */
  void SetInitialColor(float r, float g, float b){
    init_color_ = glm::vec3(r,g,b);
  }

  /**
   * Set the transparency of the particles
   * \param trans Transparency from 0 to 1. 0 = opaque, 1 = clear
   */
  void SetTransparency(float trans){trans_ = trans;}

  /**
   * If this is called, the particles will slowly fade out over the course
   * of their lifetime, rather than disappearing abruptly.
   */
  void SetParticleFadeOut(){fade_ = true;}

  /**
   * If this is set, the particles will abruptly disappear at the end of their
   * life, rather than fading out.
   */
  void UnSetParticleFadeOut(){fade_ = false;}

  /**
   * Set the randomization in the initial velocity field in the area 
   * where the particles are created.
   * \param vx Variation in velocity the global-x direction, m/s
   * \param vy Variation in velocity in the global-y direction, m/s
   * \param vz Variation in velocity in the global-z direction, m/s
   */
  void SetVelocityRandomization(float vx, float vy, float vz){
    vel_rand_fac_ = glm::vec3(vx,vy,vz);
  }

  /**
   * Set the velocity field in the area where the particles are created.
   * \param vx Initial velocity in the global-x direction, m/s
   * \param vy Initial velocity in the global-y direction, m/s
   * \param vz Initial velocity in the global-z direction, m/s
   */
  void SetInitialVelocity(float vx, float vy, float vz){
    initial_vel_ = glm::vec3(vx,vy,vz);
  }

  /// Returns a pointer to the array of particles.
  Particle* GetParticles(){
    if (particles_.size()>0){
      return &particles_[0];
    }
    else {
      return NULL;
    }
  }

  /// Returns the number of active particles in the system.
  size_t GetNumParticles(){ return particles_.size(); }

  /**
   * Add a particle source to the system.
   * \param center The location of the source in world ENU coordinates.
   * \param radius The radius of the source in meters
   * \param pps The rate of particle injection, in particles per second.
   */
  void SetSource(glm::vec3 center, float radius, float pps){
    source_center_ = center;
    source_radius_ = radius;
    source_rate_ = pps;
  }

  void Smoke();

  void Dust();
  
  void SetSourceCenter(glm::vec3 center) {
	  source_center_ = center;
  }
	bool RayIntersectsSystem(glm::vec3 orig, glm::vec3 direction);

 private:
  glm::vec3 initial_vel_;
  float lifetime_;
  float gravity_fac_;
  float expansion_rate_;
  float init_rad_;
  glm::vec3 init_color_;
  float trans_;
  bool fade_;
  glm::vec3 vel_rand_fac_;

  glm::vec3 source_center_;
  float source_radius_;
  float source_rate_;

	glm::vec3 sys_ll;
	glm::vec3 sys_ur;
  
  std::vector<Particle> particles_;
};

} // namespace environemnt
} // namespace mavs

#endif
