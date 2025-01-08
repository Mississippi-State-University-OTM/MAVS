#ifndef RADIAL_SPRING_TIRE_H
#define RADIAL_SPRING_TIRE_H
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
#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <glm/glm.hpp>
#include <mavs_core/math/constants.h>
#include <mavs_core/environment/environment.h>
#undef None

namespace mavs {
namespace vehicle {
namespace radial_spring {


/// Class for an individial spring element in the radial spring tire model
class Spring {
public:
	/// Create a spring with 0 deflection
	Spring() {
		deflection_ = 0.0f;
		theta_ = 0.0f;
	}

	/// Copy constructor
	Spring(const Spring &spring) {
		deflection_ = spring.deflection_;
		theta_ = spring.theta_;
		direction_ = spring.direction_;
		position_ =  spring.position_; 
	}

	/**
	* Set the radial orientation ofthe spring in the tire frame
	* \param t The angle in radians
	*/
	void SetTheta(float t) {
		theta_ = t;
		direction_ = glm::vec3(cos(t), 0.0f, sin(t));
	}

	/// Get the "look to" of the spring in world coordinates
	glm::vec3 GetDirection() { return direction_; }

	/// Get the current deflection of the spring in meters
	float GetDeflection() { return deflection_; }

	/// Get the position of the spring in world coordinates
	glm::vec3 GetPosition() { return position_; }

	/// Set the position of the spring in world coordinates
	void SetPosition(glm::vec3 pos) { position_ = pos; }

	/// Set the "look to" of the spring in world coordinates
	void SetDirection(glm::vec3 dir) { direction_ = dir; }

	/// Set the deflection of the spring in meters.
	void SetDeflection(float def) { deflection_ = def; }

private:
	float deflection_;
	float theta_;
	glm::vec3 direction_;
	glm::vec3 position_;
};

/// A slice is a lateral division of the tire - a collection of springs.
struct Slice {
	/// Springs in the slice
	std::vector<Spring> springs;
	/// Lateral offset of the slice from the tire center, in meters
	float offset;
	/// Position of the center of the slice, in world coordinates
	glm::vec3 position;
};

/**
* \class RadialTire
*
* Class for a radial spring tire model for calculating normal forces
* 3D version of the radial spring tire model, using equivalent volume instead of area
*
* Based on the paper:
* DENNY C. DAVIS (1975) A Radial-Spring Terrain-Enveloping Tire Model,
* Vehicle System Dynamics, 4:1, 55-69
*
* \author Chris Goodin
*
* \date 1/4/2020
*/
class Tire {
public:
	/// Create a tire
	Tire();

	/// Copy constructor
	Tire(const Tire &tire) {
		undeflected_radius_ = tire.undeflected_radius_;
		spring_constant_ = tire.spring_constant_;
		section_width_ = tire.section_width_;
		orientation_ = tire.orientation_;
		tire_position_ = tire.tire_position_;
		slices_ = tire.slices_;
		dtheta_ = tire.dtheta_;
		dslice_ = tire.dslice_;
		nsprings_ = tire.nsprings_;
		nslices_ = tire.nslices_;
		initialized_ = tire.initialized_;
		current_equivalent_deflection_ = tire.current_equivalent_deflection_;
	}

	/**
	* Initialize the tire.
	* \param sw The width of the tire in meters
	* \param ud_r The undeflected radius of the tire in meters
	* \param k The effective spring constant of the tire, in pascals
	* \param dtheta_degrees The resolution of the angular discretizatio of the springs, in degrees
	* \param num_slices The number of lateral slices comprising the tire
	*/
	void Initialize(float sw, float ud_r, float k, float dtheta_degrees, int num_slices);

	/**
	* Get the current normal force, given the current state of the tire. 
	* The position and orientaiton of the tire should be updated before calling this method.
	* \param env Pointer to a MAVS environment
	*/
	float GetNormalForce(environment::Environment *env);

	/**
	* Set the position of the tire in world coordinates
	* \param pos The position vector of the tire
	*/
	void SetPosition(glm::vec3 pos) { tire_position_ = pos; }

	/**
	* Set the orientation of the tire (as a 3x3 matrix) in the world frame
	* \param ori The orientation matrix of the tire
	*/
	void SetOrientation(glm::mat3 ori) { orientation_ = ori; }

	/// Get the undeflected radius of the tire (meters)
	float GetUndeflectedRadius() { return undeflected_radius_; }

	/// Get the calcuated equivalent deflection (meters)
	float GetCurrentEquivalentDeflection() { return current_equivalent_deflection_; }

	/// Returns true of the tire has been initialized, false if not.
	bool IsInitialized() { return initialized_; }

	/// Print the state of the springs to stdout.
	void PrintSprings();

private:
	// tire properties / model inputs
	float undeflected_radius_;
	float spring_constant_;
	float section_width_;

	// input parameters from the vehicle dynamics
	glm::mat3 orientation_;
	glm::vec3 tire_position_;

	// data structures for holding calculation variables
	std::vector<Slice> slices_;
	float dtheta_;
	float dslice_;
	int nsprings_;
	int nslices_;
	bool initialized_;

	// state variables updated by the simulation
	float current_equivalent_deflection_;
};

} // namespace radial_spring
}// namespace vehicle
}// namespace mavs

#endif