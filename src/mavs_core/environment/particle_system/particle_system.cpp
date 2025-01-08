/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <mavs_core/environment/particle_system/particle_system.h>

#include <iostream>
#include <vector>
#include <string>

#include <glm/glm.hpp>
#include <CImg.h>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace environment{

static const glm::vec3 gravity(0.0,0.0,-9.8);

ParticleSystem::ParticleSystem(){
  lifetime_ = 1.0; //seconds
  gravity_fac_ = 0.5;
  init_rad_ = 0.0;
  expansion_rate_ = 0.0;
  trans_ = 0.85f;
  vel_rand_fac_ = glm::vec3(0.0,0.0,0.0);
  fade_ = true;
}

bool ParticleSystem::RayIntersectsSystem(glm::vec3 orig, glm::vec3 dir) {
	// see https://www.scratchapixel.com/lessons/3d-basic-rendering/
	// minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
	float tmin = (sys_ll.x - orig.x) / dir.x;
	float tmax = (sys_ur.x - orig.x) / dir.x;

	if (tmin > tmax) std::swap(tmin, tmax);

	float tymin = (sys_ll.y - orig.y) / dir.y;
	float tymax = (sys_ur.y - orig.y) / dir.y;

	if (tymin > tymax) std::swap(tymin, tymax);

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	float tzmin = (sys_ll.z - orig.z) / dir.z;
	float tzmax = (sys_ur.z - orig.z) / dir.z;

	if (tzmin > tzmax) std::swap(tzmin, tzmax);

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;

	if (tzmin > tmin)
		tmin = tzmin;

	if (tzmax < tmax)
		tmax = tzmax;

	return true;
}

void ParticleSystem::Advance(float dt){
  int num_to_erase = 0;
  float to_add = source_rate_ * dt;
  int num_to_add = (int)(to_add);
  if (num_to_add < 1) {
	  float testval = mavs::math::rand_in_range(0.0f, 1.0f);
	  if (testval < to_add)num_to_add = 1;
  }
  AddParticlesInSphere(source_center_,source_radius_,num_to_add);
  for (int i=0; i<(int)particles_.size();i++){
    particles_[i].age_ += dt;
    particles_[i].position_ += particles_[i].velocity_*dt;
    particles_[i].velocity_ += dt*gravity*gravity_fac_;
    if (fade_){
      float x = (particles_[i].age_/lifetime_);
      float tx = (1.0f-trans_)*(x) + trans_;
      particles_[i].transparency_ = tx;
    }
    particles_[i].radius_ += expansion_rate_*dt;
  }

  std::vector< Particle >::iterator it = particles_.begin();
  while(it != particles_.end()) {
    Particle p = *it;
    if(p.age_ >= lifetime_) {     
      it = particles_.erase(it);
    }
		else {
			if (p.position_.x < sys_ll.x)sys_ll.x = p.position_.x;
			if (p.position_.y < sys_ll.y)sys_ll.y = p.position_.y;
			if (p.position_.z < sys_ll.z)sys_ll.z = p.position_.z;
			if (p.position_.x > sys_ur.x)sys_ur.x = p.position_.x;
			if (p.position_.y > sys_ur.y)sys_ur.y = p.position_.y;
			if (p.position_.z > sys_ur.z)sys_ur.z = p.position_.z;
			++it;

		}
  }
}

void ParticleSystem::AddParticlesInSphere(glm::vec3 center, float radius,
					  int num_particles){
  for (int i = 0; i<num_particles; i++){
    Particle p;
    p.radius_ = init_rad_;
    p.color_ = init_color_;
    glm::vec3 randvel(math::rand_in_range(-vel_rand_fac_.x,vel_rand_fac_.x),
		      math::rand_in_range(-vel_rand_fac_.y,vel_rand_fac_.y),
		      math::rand_in_range(-vel_rand_fac_.z,vel_rand_fac_.z));
    p.velocity_ = initial_vel_ + randvel;
    p.position_.x = center.x + math::rand_in_range(-radius,radius);
    p.position_.y = center.y + math::rand_in_range(-radius,radius);
    p.position_.z = center.z + math::rand_in_range(-radius,radius);
    p.transparency_ = trans_;
    particles_.push_back(p);
  }
}

void ParticleSystem::Smoke() {
	SetLifetime(3.0f);
	SetInitialVelocity(0.0f, 0.0f, 5.0f);
	SetGravityFactor(0.1f);
	SetExpansionRate(0.5f);
	SetInitialRadius(0.5f);
	SetTransparency(0.5f);
	SetVelocityRandomization(1.5f, 1.5f, 0.5f);
	SetInitialColor(0.75f, 0.75f, 0.75f);
	float source_rate = 100.0f;
	float source_radius = 3.0f;
	SetSource(glm::vec3(0.0f, 0.0f, 1.0f), source_radius, source_rate);
}
void ParticleSystem::Dust() {
	//SetLifetime(3.0f);
	//SetGravityFactor(0.5f);
	//SetExpansionRate(2.0f);
	//SetInitialRadius(0.5f);
	//SetTransparency(0.75f);
	SetLifetime(9.0f);
	SetGravityFactor(0.25f);
	SetExpansionRate(4.0f);
	SetInitialRadius(0.5f);
	SetTransparency(0.95f);
	SetVelocityRandomization(0.5f, 0.5f, 0.5f);
	SetInitialColor((144.0f/255.0f), (118.0f/255.0f), (84.0f/255.0f));
	float source_rate = 50.0f; // 20.0f;
	float source_radius = 0.5f;
	SetSource(glm::vec3(0.0f, 0.0f, 1.0f), source_radius, source_rate);
}
  
} // namespace environemnt
} // namespace mavs
