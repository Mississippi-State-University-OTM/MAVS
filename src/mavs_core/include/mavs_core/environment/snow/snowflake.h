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
* \class Snowflake
*
* The Snowflake class implements the initialization of 
* a single snowflake for addition to a snow simulation.
* 
* See:
* Saltvik, I., Elster, A. C., & Nagel, H. R. (2006, June). 
* Parallel methods for real-time visualization of snow. 
* In International Workshop on Applied Parallel Computing (pp. 218-227). 
* Springer, Berlin, Heidelberg.
* Chapter 4
*
* \author Chris Goodin
*
* \date 6/7/2019
*/
#ifndef SNOWFLAKE_H
#define SNOWFLAKE_H

#include <glm/glm.hpp>

namespace mavs {
namespace environment {

class Snowflake {
public:
	/**
	* Create a snowflake
	* \param temperature The current temperature in degrees celsius
	*/
	Snowflake(float temperature);

	/**
	* Set the position of the snowflake
	* \param pos The position of the snowflake
	*/
	void SetPosition(glm::vec3 pos) {
		position_ = pos;
	}

	/**
	* Set the position of the snowflake
	* \param x The x position of the snowflake
	* \param y The y position of the snowflake
	* \param z The z position of the snowflake
	*/
	void SetPosition(float x, float y, float z) {
		position_ = glm::vec3(x, y, z);
	}

	/**
	* Get the position of the snowflake
	*/
	glm::vec3 GetPosition() {
		return position_;
	}

	/**
	* Get the radius of the snowflake
	*/
	float GetRadius() {
		return radius_;
	}

	/**
	* Update the position of the snowflake according
	* to the equations in Saltvik 2006
	* \param wind The wind speed in m/s
	* \param dt The time step in seconds
	*/
	void UpdatePosition(glm::vec3 wind, float dt);

	/**
	* Update the position of the snowflake according
	* to the equations in Saltvik 2006
	* \param dt The time step in seconds
	*/
	void UpdatePosition(float dt);

private:
	glm::vec3 position_;
	glm::vec3 velocity_;
	float diameter_;
	float radius_;
	float mass_;
	float density_;
	float rotational_radius_;
	float rotational_velocity_;
	float rotfac_;
	float terminal_velocity_;
	float terminal_velocity_squared_;
	bool wet_;
	float age_;
	float g_;
	glm::vec3 gvec_;
};

} // namespace environment
} // namespace mavs

#endif