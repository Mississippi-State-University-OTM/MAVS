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
 * \class LensDrop
 *
 * Class for a water droplet on a lens
 * 
 * \author Chris Goodin
 *
 * \date 5/30/2019
 */

#ifndef LENS_DROP_H
#define LENS_DROP_H
#include <vector>
#include <FastNoise.h>
#include <glm/glm.hpp>

namespace mavs{
namespace sensor{
namespace camera{

class LensDrop {
public:
	/// Constuctor
	LensDrop();

	/// Print drop properties
	void Print();

	/**
	* Tell if a pixel is covered by the drop
	\param pi The pixel horizontal coordinate
	\param pj The pixel vertical coordinate
	*/
	bool PixelInDrop(int pi, int pj);

	/**
	* Get the radius of the drop in meters
	*/
	float GetRadius() { return radius_; }

	/**
	* Get the age of the drop in seconds
	*/
	float GetAge() { return age_; }

	/**
	* Get the maximum lifetime of the drop in seconds
	*/
	float GetLifetime() { return lifetime_; }

	/**
	* Get the center of the drop on the lens frame in pixel coordinates
	*/
	glm::ivec2 GetCenterPixels() { return center_pixels_; }

	/**
	* Get the radius of the drop in pixels
	*/
	int GetRadiusPixels() { return radius_pixels_; }

	/**
	* Set the radius of the drop in pixels
	\param r The desired radius in pixels
	*/
	void SetRadiusPixels(int r) { radius_pixels_ = r; }

	/**
	* Set the radius of the drop in meters
	\param r The desired radius in meters
	*/
	void SetRadius(float r) { radius_ = r; }

	/**
	* Set the age of the drop in seconds
	* \param age The desired age of the drop in seconds
	*/
	void SetAge(float age) { age_ = age; }

	/**
	* Set the maximum lifetim of the drop in seconds
	* \param lt The lifetime of the drop in seconds
	*/
	void SetLifetime(float lt) { lifetime_ = lt; }

	/** 
	* Set the color of the drop in RGB
	* \param r The red color
	* \param g The green color
	* \param b The blue color
	*/
	void SetColor(float r, float g, float b) { color_ = glm::vec3(r, g, b); }

	/**
	* Set the drop center in pixel coordinates
	* \param i The horizontal coordinate
	* \param j The vertical coordinate
	*/
	void SetCenterPixels(int i, int j) { center_pixels_ = glm::ivec2(i, j); }

private:
	float radius_;
	int radius_pixels_;
	float age_;
	float lifetime_;
	glm::vec3 color_;
	glm::ivec2 center_pixels_;
	FastNoise shape_noise_;
};

} //namespace camera
} //namespace sensor
} //namespace mavs

#endif
