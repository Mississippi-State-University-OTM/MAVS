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

/// MudMask class for Camera
class MudMask {
public:
	/// Create a mud mask
	MudMask();

	/// Set the spatial frequency of the masin in 1/meters
	void SetMaskFrequency(float freq);

	/// Get the color of the mask
	glm::vec3 GetColor() const { return mud_color_; }

	/// Get the alpha map by location
	float GetAlpha(int x, int y);

	/// Set the mud color with 0-255 float RGB
	void SetColor(float r, float g, float b) { mud_color_ = glm::vec3(r, g, b); }

private:
	glm::vec3 mud_color_;
	FastNoise mud_noise_;
};

/// Camera soiling class that can apply either raindrops or mud to the camera lens
class CameraSoiling {
public:
	/// Camera soiling constuctor
	CameraSoiling();

	/// Add raindrops to the camera lens
	void AddRaindropsToCamera(mavs::environment::Environment* env, camera::Camera* cam, float dt);

	/// Add mud to the camera lens using all the existing masks
	void AddMudToCamera(mavs::environment::Environment* env, camera::Camera* cam, float dt);

	/// Increase the number of mud masks. Default is zero
	void AddMudMask() { MudMask mask; mud_masks_.push_back(mask); }

	/// Get the total number of current mud masks
	int GetNumMudMasks() { return (int)mud_masks_.size(); }

	/// Clear away all the mud
	void ClearMudMasks() { mud_masks_.clear(); }
private:

	std::vector<camera::LensDrop> droplets_;
	std::vector<MudMask> mud_masks_;
};


} //namespace sensor
} //namespace mavs

#endif
