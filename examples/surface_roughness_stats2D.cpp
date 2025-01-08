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
* \file surface_roughness_stats2D.cpp
*
* Application to generate statistics from a simplified surface model
*
* Usage: >./surface_roughness_stats2D (rms) (acl) (num_samples) (length)
* where
* rms is the requested rms of the terrain (default=0.05 meters)
* acl is the requested autocorrealation length of the terrain (default=0.6 meters)
* num_samples is the number of simulations to run (default=1000)
* length is the requested length of the terrain (default=115 meters)
*
* \date 3/11/2019
*/

#include <vector>
#include <iostream>
#include <fstream>
#include <glm/glm.hpp>
#include "mavs_core/math/segment.h"
#include <mavs_core/math/constants.h>
#include <mavs_core/math/utils.h>
#include "mavs_core/math/histogram.h"
#include "mavs_core/terrain_generator/profile_2d.h"

int main(int argc, char * argv[]) {

	// Set parameters of simulation
	float sensor_height = 2.0f;
	float res = 0.03f; 
	float rms = 0.05f; // typically 0.005 - 0.127 meters
	float acl = 0.6f;
	int num_samples = 1000;
	float length = (float)(sensor_height / tan(mavs::kDegToRad*1.0)) + 1.0f; 
	if (argc > 1) {
		rms = (float)atof(argv[1]);
	}
	if (argc > 2) {
		acl = (float)atof(argv[2]);
	}
	if (argc > 3) {
		num_samples = atoi(argv[3]);
	}
	if (argc > 4) {
		float req_length = (float)atof(argv[4]);
		if (req_length > length)length = req_length;
	}
	
	glm::vec2 sensor_origin(0.1f, sensor_height);

	//create the terrain ray vectors
	std::vector<glm::vec2> rays;
	rays.resize(91);
	std::vector<float> min_heights;
	min_heights.resize(91, 1000.0);
	for (int i = 0; i < rays.size(); i++) {
		double theta = i * mavs::kDegToRad;
		rays[i].x = (float)cos(theta);
		rays[i].y = -(float)sin(theta);
	}

	// profile will be regenerated at each estep
	mavs::terraingen::Profile profile;
	for (int n = 0; n < num_samples; n++) {
		// Create a new profile
		profile.GenerateProfile(length, rms, acl, res);
		std::vector<glm::vec2> surface = profile.GetProfile();
		if (n == 0) {
			float rms_calc, rms_slope;
			profile.GetRmsOfProfile(rms_calc, rms_slope);
			profile.WriteProfile("profile.txt");
		}
		// do the "ray tracing"
		for (int i = 0; i < rays.size(); i++) {
			float closest = profile.GetClosestIntersection(sensor_origin, rays[i]);
			if (closest > 0.0f ) {
				float h = (sensor_origin.y - closest * fabs(rays[i].y));
				if (h < min_heights[i]) {
					min_heights[i] = h;
				}
			}
		}

	} // loop over num samples
	
	// calculate the rms and acl from the simulated data
	float r = (float)(0.5*rms*sqrt(2.0) / acl);
	float sfac = (float)(-0.77*(1.0f + log10((float)num_samples)));
	float sigma_calc = (float)(min_heights[80] / (sfac));
	float r_calc = (float)(-2.0*(mavs::kDegToRad*2.0) / log(1.0 - min_heights[2] / (sfac*sigma_calc)));
	float acl_calc = (float)(0.5*rms*sqrt(2.0) / r_calc);
	std::cout << "RMS calculated from data: " << sigma_calc << ", true = " << rms << std::endl;
	std::cout << "Autocorrelation length calculated from data: " << acl_calc << ", true = " << acl << std::endl;

	std::ofstream fout("output.txt");
	for (int i = 1; i < min_heights.size()-1; i++) {
		float theta = (float)mavs::kDegToRad*(90-i);
		float ct = (float)(cos(theta)/sin(theta));
		float lambda = (float)((r / (sqrt(mavs::k2Pi)*ct))*exp(-ct * ct / (2 * r*r)) - 0.5*erfc(ct/(sqrt(2)*r)));
		float p_scaled = (float)(2.0*pow(1.0 / (float)num_samples, 1.0 / (lambda + 1.0)));
		float erfarg = (float)fabs(p_scaled - 1.0);
		float z_inv = (float)(-sqrt(2.0)*rms*mavs::math::erfinv(erfarg));
		float z_inv_simple = -0.77f*(float)((log10((float)num_samples) + 1.0f)*rms*(1.0 - exp(-(2.0*mavs::kDegToRad*i)/r)));
		fout << i << " " << min_heights[i] << " "<<z_inv<<" "<<z_inv_simple<<std::endl;
	}
	fout.close();

	return 0;
}