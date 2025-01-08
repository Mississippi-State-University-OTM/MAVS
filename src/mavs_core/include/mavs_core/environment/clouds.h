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
* \class Clouds
*
* \author Chris Goodin
*
* \date 5/22/2019
*/

#ifndef MAVS_CLOUDS_H
#define MAVS_CLOUDS_H

#include <glm/glm.hpp>
#include <FastNoise.h>
#include <CImg.h>

namespace mavs {
namespace environment {

class Clouds {
public:
	//Clouds constructor
	Clouds();

	~Clouds() {}

	float GetCloudDensity(glm::vec3 direction);

	void ShowCloudImage();

	void WriteHemisphere();

	void SetCloudCoverFraction(float frac) {
		if (frac < 0.0f)frac = 0.0f;
		if (frac > 1.0f)frac = 1.0f;
		thresh_ = 1.0f -frac;
		intens_range_ = 1.0f - thresh_;
	}

	float GetCloudCoverFraction() {
		return intens_range_;
	}

	glm::vec3 GetCloudColor(glm::vec3 direction); // { return cloud_color_; }

	void SetCloudColor(float r, float g, float b) {
		cloud_color_ = glm::vec3(r, g, b);
	}

	void SetCloudColor(glm::vec3 col) { cloud_color_ = col; }

private:
	FastNoise cloud_noise_low_;
	FastNoise cloud_noise_high_;
	cimg_library::CImgDisplay disp_;
	cimg_library::CImg<float> image_;
	float thresh_;
	float intens_range_;
	glm::vec3 cloud_color_;

}; //class Clouds

} //namespace environment
} //namespace mavs

#endif

