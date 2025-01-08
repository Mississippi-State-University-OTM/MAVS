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
* \class Fog
*
* See
* Biri, Michelin, and Arques. 
* "Real-Time Animation of Realistic Fog"
* Thirteenth Eurographics Workship on Rendering (2002)
*
* \date 7/15/2019
*/
#ifndef FOG_H
#define FOG_H

#include <FastNoise.h>
#include <glm/glm.hpp>

namespace mavs {
namespace environment {

class Fog {
public:
	/**
	*/
	Fog();

	/**
	* Get spatially varying fog, can be quite slow
	* because it reqiures an integration over the ray path
	* \param p0 First endpoint of the ray in world coodinates
	* \param p1 Second endpoint of the ray in world coodinates
	*/
	float GetK(glm::vec3 &p0, glm::vec3 &p1);

	/**
	* Get spatially univorm value for k
	*/
	float GetK() { return k_; }

	/**
	* Set the value for the fog scattering coefficient 
	* \param k Scattering coefficient
	*/
	void SetK(float k) { k_ = k; }

private:
	FastNoise fog_spatial_lo_,fog_spatial_hi_;
	float k_;
};

} // namespace environment
} // namespace mavs

#endif