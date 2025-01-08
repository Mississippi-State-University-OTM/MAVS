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
* \file utest_mavs_camera_distortion.cpp
*
* Unit test to evaluate the mavs camera distortion model
*
* Usage: >./utest_mavs_camera_distortion
*
* Creates a distortion model, then prints the undistorted
* and distorted positions of those pixels. 
*
* Correct output is: 
*
* Undistorted.u Undistorted.v Distorted.u Distorted.c
*
* 0 0 23.3065 18.3965
*
* 0 483 23.6183 466.037
*
* 603 0 581.996 18.3743
*
* 603 483 582.06 466.059
*
* 302 242 303.22 242.83
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <iostream>
#include "sensors/camera/distortion_model.h"


int main(int argc, char *argv[]) {
	
	//create a camera system
	int num_horizontal_pix = 604;
	int num_vertical_pix = 484;
	float focal_array_width = 0.003215f;
	float focal_array_height = 0.002576f;
	float focal_length = 0.0035f;
	float xpixdim = focal_array_width / num_horizontal_pix;
	float ypixdim = focal_array_height / num_vertical_pix;
	float half_horizontal_dim = 0.5f*num_horizontal_pix;
	float half_vertical_dim = 0.5f*num_vertical_pix;

	//create a distortion model
	mavs::sensor::camera::DistortionModel dm;
	glm::vec2 cc, fc;
	std::vector<float> kc;
	kc.resize(5, 0.0f);
	fc.x = 657.30f;
	fc.y = 657.74f;
	cc.x = 302.72f;
	cc.y = 242.33f;
	float alpha_c = 0.00042f;
	kc[0] = -0.25349f;
	kc[1] = 0.11868f;
	kc[2] = -0.00028f;
	kc[3] = 0.00005f;
	dm.SetDistortionParameters(cc, fc, alpha_c, kc);
	dm.SetNominalFocalLength(focal_length);

	//Define a several pixels, in this case the top left corner
	std::vector<glm::vec2> pixels;
	pixels.resize(5);
	pixels[0] = glm::vec2(0, 0);
	pixels[1] = glm::vec2(0, num_vertical_pix - 1);
	pixels[2] = glm::vec2(num_horizontal_pix - 1, 0);
	pixels[3] = glm::vec2(num_horizontal_pix - 1, num_vertical_pix - 1);
	pixels[4] = glm::vec2(half_horizontal_dim, half_vertical_dim);

	std::cout << "Undistorted.u" << " " << "Undistorted.v" << " " <<
		"Distorted.u" << " " << "Distorted.c" << std::endl;
	for (int i = 0; i < (int)pixels.size(); i++) {
		// get the location of the pixel in pixel plane coordinates
		glm::vec2 meters(
			(pixels[i].x - half_horizontal_dim + 0.5f)*xpixdim,
			(pixels[i].y - half_vertical_dim + 0.5f)*ypixdim);
		meters = meters / focal_length;

		//distort it
		glm::vec2 xd = dm.Distort(meters);
		
		//print the results
		std::cout << pixels[i].x << " " << pixels[i].y << " " <<
			xd.x << " " << xd.y << std::endl;
	}
	return 0;
}

