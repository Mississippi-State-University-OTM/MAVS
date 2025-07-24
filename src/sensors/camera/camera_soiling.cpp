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

namespace mavs {
namespace sensor {

CameraSoiling::CameraSoiling() {

}

void CameraSoiling::AddRaindropsToCamera(mavs::environment::Environment* env, camera::Camera *cam, float dt){

	float rate = env->GetRainRate(); //mm/h
	// d_g = raindrop diameter, Feingold & Levin 1986, Eq 39
	float d_g = 0.75f * pow(rate, 0.21f); //mm
	d_g = d_g / 1000.0f; //meters

	//empirical model for calculating apparent rain color
	glm::vec3 skycol = env->GetAvgSkyRgb();
	float alpha = -2.0f * 0.0001f * rate;
	float raindrop_rho = 0.02f * (skycol.x + skycol.y + skycol.z) / 3.0f;
	glm::vec3 rain_col(raindrop_rho, raindrop_rho, raindrop_rho);

	float lens_area = cam->GetLensArea(); //horizontal_pixdim_* vertical_pixdim_;
	float numfac = (float)(lens_area * rate * 1.0E10);
	int num_raindrops = (int)(10.0f*(1.0 - exp(-0.1*numfac)));
	int num_to_add = num_raindrops - (int)droplets_.size();

	for (int i=0;i<num_to_add;i++) {
		camera::LensDrop drop;
		drop.SetRadius(0.2f*mavs::math::rand_in_range(0.75f*d_g, 1.1f*d_g));
		drop.SetRadiusPixels((int)(drop.GetRadius() / cam->GetHorizontalPixdim()));
		drop.SetCenterPixels(mavs::math::rand_in_range(0, cam->GetWidth()), mavs::math::rand_in_range(0, cam->GetHeight()));
		drop.SetColor(rain_col.x, rain_col.y, rain_col.z);
		drop.SetLifetime(mavs::math::rand_in_range(2.0f, 5.0f));
		droplets_.push_back(drop);
	}
	for (int d = 0; d < droplets_.size(); d++) {
		droplets_[d].SetAge(droplets_[d].GetAge() + (float)dt);
		if (droplets_[d].GetAge() > droplets_[d].GetLifetime()) {
			droplets_.erase(droplets_.begin() + d);
		}
	}

	cimg_library::CImg<float> image = cam->GetCurrentImage();
	cimg_library::CImg<float> new_image = cam->GetCurrentImage();
	for (int d = 0; d < droplets_.size(); d++) {
		glm::ivec2 c = droplets_[d].GetCenterPixels();
		int rp = droplets_[d].GetRadiusPixels();

#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic) 
#endif
		for (int i = c.x - rp; i <= c.x + rp; i++) {
			for (int j = c.y - rp; j <= c.y + rp; j++) {
				if (i >= 0 && i < cam->GetWidth() && j >= 0 && j < cam->GetHeight()) {
					glm::vec2 pixel(i, j);
					glm::vec2 center((float)c.x, (float)c.y);
					float r = glm::length(pixel - center);
					if (droplets_[d].PixelInDrop(i,j) && r <= rp){
						//blur filter
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
	cam->SetImage(new_image);
}

} //namespace sensor
} //namespace mavs
