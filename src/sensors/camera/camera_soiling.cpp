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
#include <sensors/camera/camera_soiling.h>
#ifdef USE_OMP
#include <omp.h>
#endif
#include <mavs_core/math/utils.h>
#include <cstdlib>
#include <ctime>

namespace mavs {
namespace sensor {

/// MudMask class constructor
MudMask::MudMask() {
	// Default mud color is brown
	mud_color_ = glm::vec3(101.0f, 67.0f, 33.0f); 
	// seed the random number generator that defines the mask
	int seed = static_cast<int>(std::time(nullptr));
	mud_noise_.SetSeed(seed);
}

void MudMask::SetMaskFrequency(float freq) {
	// the spatial frequency of the mud mask, in 1/meters
	// should be roughly equal to 1/pixel_plane_width
	mud_noise_.SetFrequency(freq);
}

float MudMask::GetAlpha(int x, int y) {
	// Get the alpha value for the mask at a pixel location
	// Mask is defined by areas where Perlin noise exceeds a threshold
	float n_thresh = (float)mud_noise_.GetPerlin(x, y); // ranges from -0.5 to 0.5
	float alpha = 0.0f;
	if (n_thresh > 0.3f) {
		// alpha map is stronger in the center of the mask, falls off at edges
		alpha = powf(std::max(0.0f, std::min(1.0f, (n_thresh - 0.3f) / 0.2f)), 0.1f);
	}
	return alpha;
}


CameraSoiling::CameraSoiling() {}

// Add mud using the current mud masks
void CameraSoiling::AddMudToCamera(mavs::environment::Environment* env, camera::Camera* cam, float dt) {

	// Set the frequency of the mud masks
	for (int i = 0; i < (int)mud_masks_.size(); i++) {
		mud_masks_[i].SetMaskFrequency(2.0f / (cam->GetWidth()));
	}

	cimg_library::CImg<float> image = cam->GetCurrentImage();

	// loop over all pixels and add mud
	for (int y = 0; y < cam->GetHeight(); ++y) {
		for (int x = 0; x < cam->GetWidth(); ++x) {
			// loop over each mud mask
			for (int i = 0; i < (int)mud_masks_.size(); i++) {
				// get the alpha value for this mask at this pixel
				float alpha = mud_masks_[i].GetAlpha(x,y);
				// if the alpha >0.0, there is mud there
				if (alpha > 0.0f) { 
					glm::vec3 base_color(image(x, y, 0), image(x, y, 1), image(x, y, 2));
					glm::vec3 blended_color = (base_color * (1.0f - alpha) + mud_masks_[i].GetColor() * alpha);
					image.draw_point(x, y, (float*)&blended_color);
				}
			}
		}
	}
	cam->SetImage(image);
}

void CameraSoiling::AddRaindropsToCamera(mavs::environment::Environment* env, camera::Camera *cam, float dt){
	// get the rain rate in mm/h fro the evironment
	float rate = env->GetRainRate(); //mm/h
	// raindrop diameter, Feingold & Levin 1986, Eq 39
	float d_g = (0.75f * pow(rate, 0.21f))/1000.0f; // in meters
	
	//empirical model for calculating apparent rain color
	glm::vec3 skycol = env->GetAvgSkyRgb();
	float alpha = -2.0f * 0.0001f * rate;
	float raindrop_rho = 0.02f * (skycol.x + skycol.y + skycol.z) / 3.0f;
	glm::vec3 rain_col(raindrop_rho, raindrop_rho, raindrop_rho);

	// calculate the total number of raindrops that should be on the lens
	float lens_area = cam->GetLensArea();
	// some empiricism here
	float numfac = (float)(40000.0f*lens_area * rate);
	int num_raindrops = (int)(10.0f*(1.0 - exp(-0.1*numfac)));
	// number to add based on the number that are currently on the lens
	int num_to_add = num_raindrops - (int)droplets_.size();
	// create the raindrops to be added and add them to the list
	for (int i=0;i<num_to_add;i++) {
		camera::LensDrop drop;
		drop.SetRadius(0.2f*mavs::math::rand_in_range(0.75f*d_g, 1.1f*d_g));
		drop.SetRadiusPixels((int)(drop.GetRadius() / cam->GetHorizontalPixdim()));
		drop.SetCenterPixels(mavs::math::rand_in_range(0, cam->GetWidth()), mavs::math::rand_in_range(0, cam->GetHeight()));
		drop.SetColor(rain_col.x, rain_col.y, rain_col.z);
		drop.SetLifetime(mavs::math::rand_in_range(2.0f, 5.0f));
		droplets_.push_back(drop);
	}

	// Erase any raindrops that have exceeded their lifetime
	for (int d = 0; d < droplets_.size(); d++) {
		droplets_[d].SetAge(droplets_[d].GetAge() + (float)dt);
		if (droplets_[d].GetAge() > droplets_[d].GetLifetime()) {
			droplets_.erase(droplets_.begin() + d);
		}
	}
	
	// extract the current camera image, and creat the modified image
	cimg_library::CImg<float> image = cam->GetCurrentImage();
	cimg_library::CImg<float> new_image = cam->GetCurrentImage();

	// loop over the droplets and render them to the camera image
	for (int d = 0; d < droplets_.size(); d++) {
		glm::ivec2 c = droplets_[d].GetCenterPixels();
		int rp = droplets_[d].GetRadiusPixels();

#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic) 
#endif
		for (int i = c.x - rp; i <= c.x + rp; i++) {
			for (int j = c.y - rp; j <= c.y + rp; j++) {
				// check all the pixels in the drop
				if (i >= 0 && i < cam->GetWidth() && j >= 0 && j < cam->GetHeight()) {
					glm::vec2 pixel(i, j);
					glm::vec2 center((float)c.x, (float)c.y);
					float r = glm::length(pixel - center);
					if (droplets_[d].PixelInDrop(i,j) && r <= rp){
						//blur filter to simulate the effect of refraction through the drop
						glm::vec3 newcol(0.0f, 0.0f, 0.0f);
						int nfilt = 5;
						int nr = (nfilt - 1) / 2;
						for (int ii = -nr; ii <= nr; ii++) {
							int iii = ii + i;
							for (int jj = -nr; jj <= nr; jj++) {
								int jjj = jj + j;
								if (iii >= 0 && iii < cam->GetWidth() && jjj >= 0 && jjj < cam->GetHeight()) {
									newcol = newcol + glm::vec3(image(iii, jjj, 0), image(iii, jjj, 1), image(iii, jjj, 2));
								}
							}
						}
						// blending the blurred old pixels with the new pixel
						newcol = newcol / (0.8f*nfilt*nfilt);
						glm::vec3 oldcol = glm::vec3(image(i, j, 0), image(i, j, 1), image(i, j, 2));
						float s = 1.0f - exp(-((float)r / rp));
						newcol = (1.0f-s) * newcol + s*oldcol;
						new_image.draw_point(i, j, (float *)&newcol);
					}
				}
			}
		}
	}

	// Update the camera image with the new image
	cam->SetImage(new_image);
}

} //namespace sensor
} //namespace mavs
