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
 * \class OrthoViewer
 *
 * Class to render a top down version of the scene. 
 * Uses orthographic projection.
 *
 * \author Chris Goodin
 *
 * \date 5/11/2020
 */

#ifndef ORTHO_VIEWER_H
#define ORTHO_VIEWER_H
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <raytracers/raytracer.h>
#include <mavs_core/pose_readers/waypoints.h>
#include <mavs_core/environment/environment.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include <sensors/camera/ortho_camera.h>

namespace mavs{

class OrthoViewer{
 public:
	 /**
	 * Create an ortho-viewer.
	 */
  OrthoViewer();

	/**
	* Destroy an ortho-viewer.
	*/
  ~OrthoViewer();

	/**
	* Render the top-down ortho-view.
	* \param env Pointer to a MAVS environment.
	*/
  void Update(mavs::environment::Environment *env);

	/**
	* Render the top-down ortho-view with vehicle position overlaid.
	* \param env Pointer to a MAVS environment.
	* \param waypoints List x-y waypoints in ENU coordinates.
	*/
  void Update(mavs::environment::Environment *env, std::vector<glm::vec2> waypoints);

	/**
	* Save the current top-down frame to an image file.
	* \param fname Full path to the file to be saved.
	*/
	void SaveImage(std::string fname);

	/**
	* Display the current top down frame to screen.
	*/
	void Display() { disp_ = cropped_image_; }

	/**
	* Get the buffer of floats of the current top down image.
	*/
	float * GetImageBuffer() { return cropped_image_.data(); }

	/**
	* Get the size of the float buffer containing the image data.
	*/
	int GetImageBufferSize() { return (int)cropped_image_.size(); }

private:
	mavs::sensor::camera::OrthoCamera cam_;
	std::vector<glm::ivec2> waypoints_;
	std::vector<glm::ivec2> vehicle_path_;
	glm::vec2 ll_;
	float pixres_;
	bool initialized_;
	cimg_library::CImg<float> cropped_image_;
	cimg_library::CImgDisplay disp_;
};

}//namespace mavs

#endif
