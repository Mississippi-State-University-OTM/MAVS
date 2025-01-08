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
#include <mavs_core/terrain_generator/random_surface.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <random>
#include <mavs_core/math/utils.h>
#include <FastNoise.h>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>

namespace mavs {
namespace terraingen {

RandomSurface::RandomSurface() {
	color_ = glm::vec3(0.38f, 0.25f, 0.06f);
	float size = 100.0f;
	hifreq_feature_len_ = 2.0f;
	hifreq_feature_mag_ = 0.2f;
	lofreq_feature_len_ = 50.0f;
	lofreq_feature_mag_ = 0.0f;
	texture_pixres_ = 0.001f;
	max_image_dim_ = 8192;
	textured_ = false;
	trail_set_ = false;
}

RandomSurface::~RandomSurface() {

}

void RandomSurface::GenerateHeightmapWithGap(float llx, float lly, float urx, float ury, float res, float gh, float gw, float g_theta) {
	SetDimensions(llx, lly, urx, ury, res);
	FastNoise hifreq_noise;
	float hi_wl = 1.25f*(float)kPi*hifreq_feature_len_;
	hifreq_noise.SetFrequency(1.0 / hi_wl);

	int nx = (int)heightmap_.GetHorizontalDim();
	int ny = (int)heightmap_.GetVerticalDim();
	float tt = tanf(g_theta);
	float x0 = 0.0f; 
	float x1 = gh * tt;
	float x2 = gw + gh * tt;
	float x3 = gw + 2.0f*gh * tt;
	float m = 1.0f / tanf(g_theta);
	float b = -(2.0f*gh + gw / tt);

	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			glm::vec2 p = heightmap_.IndexToCoordinate(i, j);
			float z = 0.0f;
			if (p.x > x0 && p.x <= x1){
				z = -m * p.x;
			}
			else if (p.x > x1 && p.x <= x2) {
				z = -gh;
			}
			else if (p.x > x2 && p.x <= x3) {
				z = m * p.x + b;
			}
			z += hifreq_feature_mag_ * hifreq_noise.GetPerlin(p.x, p.y);
			heightmap_.SetHeight(i, j, z);
		}
	}
} // Generate heightmap with gap

void RandomSurface::GenerateGaussianHeightMap(float llx, float lly, float urx, float ury, float res) {
	unsigned t = (unsigned)std::time(NULL);
	std::default_random_engine generator(t);
	float rms = hifreq_feature_mag_;
	float acl = hifreq_feature_len_;
	std::normal_distribution<float> distribution(0.0f, rms);
	float lx = urx - llx;
	float ly = ury - lly;
	//int nx = (int)(2.0f * lx / acl) + 1; // nyquist criteria
	//int ny = (int)(2.0f * ly / acl) + 1; // nyquist criteria
	int nx = (int)(0.5f * lx / acl) + 1; 
	int ny = (int)(0.5f * ly / acl) + 1;
	float res_nyq = std::min(lx / ((float)nx),ly/((float)ny));
	res = std::min(res, res_nyq);
	SetDimensions(llx, lly, urx, ury, res);
	nx = (int)heightmap_.GetHorizontalDim();
	ny = (int)heightmap_.GetVerticalDim();


	//create initial gaussian height field
	std::vector<std::vector<float> > hm = mavs::utils::Allocate2DVector(nx, ny, 0.0f);
	float dx = 2.0f*acl;
	float rm = (float)exp(-dx * acl);
	float sig_m = (float)sqrt(rms*rms*(1.0 - rm * rm));
	hm[0][0] = 0.0f;
	for (int i = 0; i < nx; i++) {
		if (i != 0) {
			std::normal_distribution<float> disti(rm*hm[i-1][0], sig_m);
			hm[i][0] = disti(generator);
		}
		for (int j = 1; j < ny; j++) {
			std::normal_distribution<float> distj(rm*hm[i][j-1], sig_m);
			hm[i][j] = distj(generator);
		}
	}

	/*
	std::vector<std::vector<float> > hm = mavs::utils::Allocate2DVector(nx, ny, 0.0f);
	std::vector<std::vector<float> > temphm = mavs::utils::Allocate2DVector(nx, ny, 0.0f);
	for (int i = 0; i < nx; i++) {
		float x = (i + 0.5f)*res;
		for (int j = 0; j < ny; j++) {
			float y = (j + 0.5f)*res;
			temphm[i][j] = distribution(generator);
		}
	}

	//create low pass filter
	float acl2 = acl * acl;
	int goff = (int)(2.0*ceil(acl / res));
	int glen = 2 * goff + 1;
	float gscale = (float)(2.0*res / (sqrt(mavs::kPi)*acl));
	std::vector<std::vector<float>> g_lo = mavs::utils::Allocate2DVector(glen,glen,0.0f);
	//g_lo.resize(glo_len, 0.0f);
	for (int i = 0; i < glen; i++) {
		float x = res * (goff - i);
		for (int j = 0; j < glen; j++) {
			float y = res * (goff - j);
			g_lo[i][j] = (float)(gscale*exp(-0.5f*(x*x+y*y) / acl2));
		}

	}

	//apply low pass filter
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			for (int ii = 0; ii < glen; ii++) {
				int ip = i + goff - ii;
				while (ip < 0) {
					ip += nx;
				}
				while (ip >= nx) {
					ip -= nx;
				}
				for (int jj = 0; jj < glen; jj++) {
					int jp = j + goff - jj;
					while (jp < 0) {
						jp += nx;
					}
					while (jp >= ny) {
						jp -= ny;
					}
					hm[i][j] += temphm[ip][jp] * g_lo[ii][jj];
				} //jj
			} //ii
		} // j
	} //i 
	*/

	// set the heightmap and add low-frequency noise
	FastNoise lofreq_noise;
	lofreq_noise.SetFrequency(1.0 / lofreq_feature_len_);
	for (int i = 0; i < nx; i++) {
		float x = (i + 0.5f)*res;
		for (int j = 0; j < ny; j++) {
			float y = (j + 0.5f)*res;
			float z = hm[i][j]+(float)(lofreq_feature_mag_ * lofreq_noise.GetPerlin(x, y));
			heightmap_.SetHeight(i, j, z);
		}
	}
}

void RandomSurface::GenerateVariableRoughness(float llx, float lly, float urx, float ury, float res) {
	//feature lengths between 1 and 10 feet, 0.3 to 3 meters
	//are most relevant for mobility purposes
	FastNoise noise((int)std::time(NULL));
	noise.SetFrequency(1.0f);
	FastNoise  rough_scale((int)std::time(NULL));
	rough_scale.SetFrequency(0.01f);

	SetDimensions(llx, lly, urx, ury, res); //sets heightmap to 0

	int nx = (int)heightmap_.GetHorizontalDim();
	int ny = (int)heightmap_.GetVerticalDim();

	float max_wl = std::min(100.0f, 0.25f*std::min(urx - llx, ury - lly));
	float min_wl = 2.0f*res;
	int num_decades = 1;
	float wl = min_wl;
	while (wl <= max_wl) {
		wl = (float)pow(2.0, num_decades)*min_wl;
		num_decades = num_decades + 1;
	}
	num_decades--;

	// Populate arrays holding noise coefficients for use later
	std::vector<float> magnitudes; 
	std::vector<float> wavelengths; 
	for (int n=0;n<num_decades;n++) {
		float fac = (float)pow(2.0, n);
		magnitudes.push_back(hifreq_feature_mag_ * fac);
		wavelengths.push_back(min_wl * fac);
	}
	//std::ofstream fout;
	//fout.open("rms_truth.txt");
	for (int i = 0; i < nx; i++) {
		float x = llx + (i + 0.5f)*res;
		for (int j = 0; j < ny; j++) {
			float y = lly + (j + 0.5f)*res;
			float z = 0.0f;
			for (int n = 0; n < num_decades; n++) {
				float z0 = magnitudes[n] * (float)noise.GetPerlin(x / wavelengths[n], y / wavelengths[n]);
				z = z + z0; 
			}
			float scale_fac = 0.5f*(1.0f + (float)rough_scale.GetPerlin(x, y));
			z = scale_fac * z;
			//fout << x << " " << y << " " << scale_fac << " " << z << std::endl;
			heightmap_.SetHeight(i, j, z);
		}
	}
	//fout.close();
}

void RandomSurface::GenerateHeightMap(float llx, float lly, float urx, float ury, float res) {
	//feature lengths between 1 and 10 feet, 0.3 to 3 meters
	//are most relevant for mobility purposes
	FastNoise hifreq_noise;
	FastNoise lofreq_noise;
	float hi_wl = 1.25f*(float)kPi*hifreq_feature_len_;
	float hi_mag = 4.0f*hifreq_feature_mag_;
	hifreq_noise.SetFrequency(1.0 / hi_wl);
	lofreq_noise.SetFrequency(1.0 / lofreq_feature_len_);

	SetDimensions(llx, lly, urx, ury, res);

	int nx = (int)heightmap_.GetHorizontalDim(); 
	int ny = (int)heightmap_.GetVerticalDim(); 
	for (int i = 0; i < nx; i++) {
		float x = (i + 0.5f)*res;
		for (int j = 0; j < ny; j++) {
			float y = (j + 0.5f)*res;
			float z = (float)(hi_mag * hifreq_noise.GetPerlin(x, y) + lofreq_feature_mag_ * lofreq_noise.GetPerlin(x, y));
			heightmap_.SetHeight(i, j, z);
		}
	}
}

void RandomSurface::AddPotholeAtLocation(float x, float y, float pothole_rad, float pothole_depth) {
	glm::vec2 point(x, y);
	float res = heightmap_.GetResolution();
	glm::vec2 corner = heightmap_.GetLLCorner();
	int nr = (int)ceil(pothole_rad / res);
	int nx = (int)((point.x - corner.x) / res);
	int ny = (int)((point.y - corner.y) / res);
	for (int j = -nr; j <= nr; j++) {
		for (int k = -nr; k <= nr; k++) {
			float r = (float)sqrt(j*res*j*res + k * res*k*res);
			if (r < pothole_rad) {
				int jj = nx + j;
				int kk = ny + k;
				float z = heightmap_.GetCellHeight(jj, kk);
				float offset = (float)((pothole_depth*0.5f)*(cos(mavs::kPi*((r / pothole_rad) - 1.0f)) - 1.0f));
				z = z + offset;
				heightmap_.SetHeight(jj, kk, z);
			}
		}
	}
}

void RandomSurface::AddPotholes(float pothole_depth_in, float pothole_diameter, int num_holes, std::vector<glm::vec2> pothole_locations) {
	float length = (float)sqrt(heightmap_.GetArea());
	Waypoints *waypoints = trail_.GetPath();
	int nwp = (int)waypoints->NumWaypoints();
	std::ofstream fout("pothole_truth.txt");
	fout << "x y depth diameter" << std::endl;
	for (int i = 0; i < num_holes; i++) {
		glm::vec2 point;
		if (i >= pothole_locations.size()) {
			//pick a random point on the trail
			int wp = mavs::math::rand_in_range(0, nwp - 2);
			glm::vec2 tv = waypoints->GetWaypoint(wp + 1) - waypoints->GetWaypoint(wp);
			glm::vec2 point = waypoints->GetWaypoint(wp) + mavs::math::rand_in_range(0.0f, 1.0f)*tv;
			point.x += mavs::math::rand_in_range(-0.5f*trail_.GetTrailWidth(), 0.5f*trail_.GetTrailWidth());
			point.y += mavs::math::rand_in_range(-0.5f*trail_.GetTrailWidth(), 0.5f*trail_.GetTrailWidth());
		}
		else {
			point = pothole_locations[i];
		}
		// now create a pothole
		float pothole_rad = mavs::math::rand_in_range(0.45f*pothole_diameter,0.55f*pothole_diameter);
		float pothole_depth = mavs::math::rand_in_range(0.9f*pothole_depth_in, 1.1f*pothole_depth_in);
		fout << point.x << " " << point.y << " " << pothole_depth << " " << (2.0f*pothole_rad) << std::endl;
		AddPotholeAtLocation(point.x, point.y, pothole_rad, pothole_depth);
	}
	fout.close();
}

} //namespace terraingen
} //namespace mavs
