/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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

	bool IsOpen(); 
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
