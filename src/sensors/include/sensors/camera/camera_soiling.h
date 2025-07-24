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
 * \class CameraSoiling
 *
 * Class for adding various soiling properties
 * 
 * \author Chris Goodin
 *
 * \date 7/24/2025
 */

#ifndef CAMERA_SOILING_H
#define CAMERA_SOILING_H
#include <vector>
#include <sensors/camera/camera.h>
#include <sensors/camera/lens_drop.h>
#include <FastNoise.h>

namespace mavs{
namespace sensor{

class MudMask {
public:
	MudMask();

	void SetMaskFrequency(float freq);

	glm::vec3 GetColor() const { return mud_color_; }

	float GetAlpha(int x, int y);

private:
	glm::vec3 mud_color_;
	FastNoise mud_noise_;
};

class CameraSoiling {
public:
	/// Constuctor
	CameraSoiling();

	void AddRaindropsToCamera(mavs::environment::Environment* env, camera::Camera* cam, float dt);

	void AddMudToCamera(mavs::environment::Environment* env, camera::Camera* cam, float dt);

	void AddMudMask() { MudMask mask; mud_masks_.push_back(mask); }

	int GetNumMudMasks() { return (int)mud_masks_.size(); }
private:

	std::vector<camera::LensDrop> droplets_;
	std::vector<MudMask> mud_masks_;
};


} //namespace sensor
} //namespace mavs

#endif
