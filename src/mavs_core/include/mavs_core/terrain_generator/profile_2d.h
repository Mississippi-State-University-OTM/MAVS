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
* \file profile_2d.h
*
* Functions for creating and manipulating 2D profiles
*
* \author Chris Goodin
*
* \date 3/18/2019
*/

#ifndef MAVS_PROFILE_H
#define MAVS_PROFILE_H
#include <vector>
#include <random>
#include <string>
#include <glm/glm.hpp>


namespace mavs {
namespace terraingen {

class Profile {
public:
	/// Create an empty profile
	Profile();

	/**
	* Create a Gaussian profile of a given length, RMS, autocorrelation length, and resolution
	* \param length The requested length (meters) of the terrain. Will be modified to meet Nyquist critera
	* \param rms The requested RMS in meters.
	* \param acl The requested autocorrelation length (meters) of the profile
	* \param res The requested resolution of the profile (meters), modifed to meet Nyquist criteria
	*/
	void GenerateProfile(float &length, float rms, float acl, float &res);

	/// Return the current profile
	std::vector<glm::vec2> GetProfile() {
		return profile_;
	}

	/**
	* Calculate the RMS height and RMS slope of an existing profile
	* \param profile The input profile
	* \param rms The output calculated RMS height
	* \param rms_slope The output calculated RMS slope
	*/
	void GetRmsOfProfile(float &rms, float &rms_slope);

	/**
	* Return the minimum elevation value in a profile
	* \param profile The input profile
	*/
	float GetMinimumOfProfile();

	/**
	* Write the current profile to a text file
	* \param fname The name of the text file to save the profile to
	*/
	void WriteProfile(std::string fname);

	/**
	* Get the closest intersection of a ray with the profile. 
	* Returns -1.0 if there's no intersection
	* \param origin The origin of the ray
	* \param direction The direction of the ray
	*/
	float GetClosestIntersection(glm::vec2 origin, glm::vec2 direction);

private:
	std::vector<glm::vec2> profile_;
	std::default_random_engine generator_;
};
} //namespace terraingen
} //namespace mavs
#endif
