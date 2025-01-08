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
* \class Rp3dVehicleViewer
*
* Class for quickly, easily viewing a MAVS vehicle to check the alignment of the mesh and physics.
*
* \author Chris Goodin
*
* \date 5/7/2020
*/
#ifndef RP3D_VEHICLE_VIEWER_H_
#define RP3D_VEHICLE_VIEWER_H_
#include <CImg.h>
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <sensors/camera/ortho_camera.h>
#include <mavs_core/environment/environment.h>
#include <raytracers/embree_tracer/embree_tracer.h>

namespace mavs {
class Rp3dVehicleViewer {
public:
	/**
	* Create and RP3D vehicle viewer.
	*/
	Rp3dVehicleViewer();

	/**
	* Load the vehicle to view.
	* \param vehfile Full path to the vehicle json file to load.
	*/
	void LoadVehicle(std::string vehfile);

	/**
	* Update the current vehicle view frames, without display.
	* \param show_debug Set to true to render the RP3D physics objects over the vehicle mesh.
	*/
	void Update(bool show_debug);

	/**
	* Display the current vehicle view frames.
	* \param show_debug Set to true to render the RP3D physics objects over the vehicle mesh.
	*/
	void Display(bool show_debug);

	/**
	* Get the buffer of floats of the current side-view image.
	*/
	float * GetSideBuffer() { return side_image_.data(); }

	/**
	* Get the buffer of floats of the current front-view image.
	*/
	float * GetFrontBuffer() { return front_image_.data(); }

	/**
	* Get the size of buffer of floats of the current side-view image.
	*/
	int GetSideBufferSize() { return (int)side_image_.size(); }

	/**
	* Get the size of buffer of floats of the current front-view image.
	*/
	int GetFrontBufferSize() { return (int)front_image_.size(); }

	/**
	* Write the front image to a file.
	* \param fname The name of the file to write, with image type extension.
	*/
	void WriteFrontImage(std::string fname) { front_image_.save(fname.c_str()); }

	/**
	* Write the side image to a file.
	* \param fname The name of the file to write, with image type extension.
	*/
	void WriteSideImage(std::string fname) { side_image_.save(fname.c_str()); }

private:
	void FillDisplay(bool show_debug);
	int npix_;
	int np2_;
	cimg_library::CImg<float> front_image_;
	cimg_library::CImg<float> side_image_;
	cimg_library::CImgDisplay front_disp_;
	cimg_library::CImgDisplay side_disp_;
	mavs::vehicle::Rp3dVehicle vehicle_;
	mavs::sensor::camera::OrthoCamera front_camera_, side_camera_;
	mavs::environment::Environment env_;
	mavs::raytracer::embree::EmbreeTracer scene_;
	bool disp_filled_;
};
} // namespace MAVS

#endif
