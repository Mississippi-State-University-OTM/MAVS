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
* \file distortion_model.h
*
* Implements a distortion model similar to the one
* in the Matlab Camera Calibration Toolbox published on 
* http://www.vision.caltech.edu/bouguetj/calib_doc/
* The input parameters for the model can be measured with the
* toolbox
*
* \author Chris Goodin
*
* \date 6/21/2018
*/


#ifndef DISTORTION_MODEL_H
#define DISTORTION_MODEL_H

#include <vector>
#include <glm/glm.hpp>

namespace mavs {
namespace sensor {
namespace camera {

	/**
	* \class DistortionModel
	*
	* Example pixel coordinates of a 3x3 image
	*     -------------------
	*     | 0,0 | 1,0 | 2,0 |
	*     -------------------
	*     | 1,0 | 1,1 | 2,1 |
	*     -------------------
	*     | 2,0 | 1,2 | 2,2 |
	*     -------------------
	*/
class DistortionModel {
 public:
	 /// Constructor
	DistortionModel();

	/// Destructor
	~DistortionModel();

	/// Copy constructor
	DistortionModel(const DistortionModel &dm);

	/**
	* Get distorted coordinates of a pixel in undistorted
	* coordinates.
	* \param p_undistorted Undistorted pixel location in
	* the image plane in units of pixels from upper left.
	*/
	glm::vec2 Distort(glm::vec2 p_undistorted);

	/**
	* Get undistorted coordinates of a pixel in distorted
	* coordinates.
	* \param p_distorted Distorted pixel location in
	* the image plane in units of pixels from upper left.
	*/
	glm::vec2 Undistort(glm::vec2 p_distorted);

	/**
	* Transform from pixel coordinates to meters offset
	* Example meter coordinates of a 2x2 image
	*     --------------------
	*     | -hx,hy  | hx,hy  | 
	*     ----------C---------
	*     | -hx,-hy | hx,-hy | 
	*     --------------------
	* where hx = (half the horizontal dimension of the
	* focal plane in meters), hy = (half the vertical
	* dimension of the focal plane in meters), and 
	* C = (0,0)=(center of the focal plane. 
	* \param pixel The input pixel (in pixels) to be
	* transfromed.
	*/
	glm::vec2 PixelToMeters(glm::vec2 pixel);

	/**
	* Transform from pixel coordinates from meters offset
	* to pixels, see the pixel coordinate frame in class def
	* Example meter coordinate frame of a 2x2 image
	*     --------------------
	*     | -hx,hy  | hx,hy  |
	*     ----------C---------
	*     | -hx,-hy | hx,-hy |
	*     --------------------
	* where hx = (half the horizontal dimension of the
	* focal plane in meters), hy = (half the vertical
	* dimension of the focal plane in meters), and
	* C = (0,0)=(center of the focal plane.
	* \param pixel The input pixel (in meters) to be
	* transfromed..
	*/
	glm::vec2 MetersToPixels(glm::vec2 pixel);

	/**
	* Set the nominal (ideal) focal length of the camera.
	* Used to convert from pixel to meter coordinates and back.
	* \param f Ideal focal length of the camera in meters
	*/
	void SetNominalFocalLength(float f);

	/**
	* Set the distortion parameters of the model
	* Parameters can be obtained with measurements from the Matlab
	* Camera Calibration Toolbox
	* \param cc The center of the image plane, in pixels
	* \param fc The horizontal and vertical focal length of the camera,
	* in pixels
	* \param alpha_c The skew coefficient of the camera
	* \param kc The radial and tangential distortion coefficients of the 
	* camera
	*/
	void SetDistortionParameters(glm::vec2 cc, glm::vec2 fc, 
		float alpha_c, std::vector<float> kc);

 private:
	 // radial and tangential distortion coefficients
	std::vector<float> kc_;

	// principal coordinate point in pixels
	glm::vec2 cc_;

	//horizontal and vertical focal length in pixels
	glm::vec2 fc_;

	//the nominal (ideal) focal length in meters
	float fnom_;
	// conversion cofficient (m/pixel) from pixels to meters, horizontal
	float hscale_;
	// conversion cofficient (m/pixel) from pixels to meters, vertical
	float vscale_;

	//skew coefficient
	float alpha_c_;
};

} //namespace camera
} // namespace sensor
} //namespace mavs

#endif