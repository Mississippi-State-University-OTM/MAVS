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
#include <sensors/camera/add_rain_to_existing_image.h>
#include <vector>
#include <glm/glm.hpp>
#include <mavs_core/math/utils.h>
#include <sensors/camera/lens_drop.h>

namespace mavs {
namespace sensor {
namespace camera {

void AddRainToImage(cimg_library::CImg<float> &image_, float rate,  bool raindrops_on_lens) {
	AddRainToImage(image_, rate, 1.0f, raindrops_on_lens);
}

void AddRainToImage(cimg_library::CImg<float> &image_, float rate, float rho, bool raindrops_on_lens) {

	srand((unsigned int)time(NULL));
	float xwind = mavs::math::rand_in_range(-3.0f, 3.0f);
	float ywind = mavs::math::rand_in_range(-3.0f, 3.0f);

	glm::vec3 wind(xwind, ywind, 0.0f);
	//glm::vec3 skycol(204.0f, 227.0f, 229.0f);
	float mean = rho*(float)image_.mean();
	float dark_fac = (float)exp(-0.0227f*std::min(rate, 25.0f));
	glm::vec3 skycol(mean, mean, mean);
	glm::vec3 look_to_(1.0f, 0.0f, 0.0f);
	glm::vec3 look_up_(0.0f, 0.0f, 1.0f);
	glm::vec3 look_side_(0.0f, 1.0f, 0.0f);
	float focal_length_ = 0.0035f;
	float focal_array_width_ = 0.0035f;
	float focal_array_height_ = 0.0035f;
	float exposure_time_ = 0.001f;
	int num_horizontal_pix_ = image_.width();
	int num_vertical_pix_ = image_.height();
	float horizontal_pixdim_ = focal_array_width_ / (1.0f*num_horizontal_pix_);
	float vertical_pixdim_ = focal_array_height_ / (1.0f*num_vertical_pix_);


	std::vector< std::vector< float > > rain_mask = mavs::utils::Allocate2DVector(num_horizontal_pix_, num_vertical_pix_, 0.0f);
	//environmental factors
	//glm::vec3 skycol = env->GetAvgSkyRgb();
	//float rate = env->GetRainRate(); //mm/h
	//glm::vec3 wind(env->GetWind().x, env->GetWind().y, 0.0);

	//empirical stuff
	//float alpha = -2.0f*0.00003*rate; //rain absorption coeff, 1/m
	float alpha = -2.0f*0.00009f*rate;
	float max_alpha = 1000.0f*alpha;
	float raindrop_rho = 0.1f*mean; // 0.1f*(skycol.x + skycol.y + skycol.z);
	glm::vec3 rain_col  = glm::vec3(raindrop_rho, raindrop_rho, raindrop_rho);
	//std::cout << "Rain color = " << raindrop_rho << std::endl;
	//glm::vec3 rain_col_fac = rain_col;
	float min_pix_frac = 0.1f; //10% of pixel
	float n_t = 172.0f*pow(rate, 0.21f); // #/m^3
										 //Feingold & Levin 1986, Eq 39
	float d_g = 0.75f*pow(rate, 0.21f); //mm
	d_g = d_g / 1000.0f; //meters
	float vterm = 3.1654f*log(d_g) + 26.006f; //m/s
	wind.z = vterm;
	glm::vec2 wind_projected;
	wind_projected.x = glm::dot(look_side_, wind);
	wind_projected.y = glm::dot(look_up_, wind);

	// Geometry stuff
	//float range_max = focal_length_ * d_g*(2.0f/min_pix_frac) / (horizontal_pixdim_ + vertical_pixdim_);
	//float range_max = 15.0;
	float range_max = std::max(20.0f, 4500.0f*focal_length_);
	float width_mag = 0.5f*focal_array_width_ / focal_length_;
	float height_mag = 0.5f*focal_array_height_ / focal_length_;
	float dh = 2.0f*range_max * height_mag;
	float dw = 2.0f*range_max * width_mag;
	float rain_vol = range_max * dw*dh / 3.0f; //volume of view frustum
	int total_num_drops = (int)(n_t * rain_vol);
	float halfwidth = 0.5f*focal_array_width_;
	float halfheight = 0.5f*focal_array_height_;

#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
	for (int i = 0; i < total_num_drops; i++) {
		float z = mavs::math::rand_in_range(0.0f, range_max);
		float dx = width_mag * z;
		float dy = height_mag * z;
		float x = mavs::math::rand_in_range(-dx, dx);
		float y = mavs::math::rand_in_range(-dy, dy);
		float m = focal_length_ / z;
		x = m * x;
		y = m * y;
		int u = (int)((x + halfwidth) / horizontal_pixdim_);
		int v = (int)((y + halfheight) / vertical_pixdim_);
		//get the streak length and direction in pixels
		glm::vec2 l_streak(m * wind_projected.x*exposure_time_ / horizontal_pixdim_,
			m * wind_projected.y*exposure_time_ / vertical_pixdim_);
		float l_streak_mag = glm::length(l_streak);
		l_streak = l_streak / l_streak_mag;
		glm::vec2 l_pos(u, v);
		for (int ns = 0; ns <= (int)l_streak_mag; ns++) {
			glm::vec2 distorted_pos = l_pos;
			u = (int)distorted_pos.x;
			v = (int)distorted_pos.y;
			if (u >= 0 && u < num_horizontal_pix_ && v >= 0 && v < num_vertical_pix_) {

				rain_mask[u][v] = rain_mask[u][v] + raindrop_rho; // to_add;

			}
			else {
				break;
			}
			l_pos += l_streak;
		}
	}

	//calculate average and max intensity of image
	float intens_max = 0.0f;
	float intens_avg = 0.0f;
	int bfs = (int)(image_.height()*image_.width()); // image_buffer_.size();
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic) shared(intens_avg, intens_max)
#endif
													 //for (int n = 0; n < bfs; n++) {
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			//------ Light absorbed by the rain ------------------------

			//if (j<0.5*num_vertical_pix_) { //distance thingy?
			//float range = focal_length_ / ((focal_array_height_*j) / (1.0f* num_vertical_pix_));
			//float abs_fac = exp(alpha*range);
			for (int k = 0; k<3; k++)image_(i, j, k) = dark_fac * image_(i, j, k) + (1.0f - dark_fac)*rain_col[k];
			/*}
			else {
			float abs_fac = exp(max_alpha);
			for (int k = 0; k<3; k++)image_(i, j, k) = abs_fac * image_(i, j, k) + (1.0f - abs_fac)*rain_col_fac[k];
			}*/
			//---- end light absorbed by rain -----------------------------
			float intens = glm::length(image_(i, j));
			if (intens > intens_max)intens_max = intens;
			intens_avg += intens;
		}
	}
	intens_avg = intens_avg / (1.0f*bfs);
	float sf = (intens_max - intens_avg) / (intens_max*intens_max);

	//apply rain mask
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic) 
#endif
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			//int n = GetFlattenedIndex(i, j);
			//float intens = glm::length(image_buffer_[n]);
			float r = image_(i, j, 0);
			float g = image_(i, j, 1);
			float b = image_(i, j, 2);
			float intens = (float)sqrt(r*r + g * g + b * b);
			float scale_fac = (intens_max - intens)*sf;
			//image_buffer_[n] = image_buffer_[n] + rain_mask[i][j] *scale_fac;
			for (int k = 0; k < 3; k++) {
				image_(i, j, k) = image_(i, j, k) + rain_mask[i][j] * scale_fac;
			}
		}
	}

	std::vector<LensDrop> droplets;
	float dt = 1.0f / 30.0f;
	if (raindrops_on_lens) {
		float lens_area = horizontal_pixdim_ * vertical_pixdim_;
		float numfac = (float)(lens_area * rate * 1.0E10);
		int num_raindrops = (int)(10.0f*(1.0 - exp(-0.1*numfac)));
		int num_to_add = num_raindrops - (int)droplets.size();
		for (int i = 0; i<num_to_add; i++) {
			LensDrop drop;
			drop.SetRadius(0.2f*mavs::math::rand_in_range(0.75f*d_g, 1.1f*d_g));
			drop.SetRadiusPixels((int)(drop.GetRadius() / horizontal_pixdim_));
			drop.SetCenterPixels(mavs::math::rand_in_range(0, num_horizontal_pix_), mavs::math::rand_in_range(0, num_vertical_pix_));
			drop.SetColor(rain_col.x, rain_col.y, rain_col.z);
			drop.SetLifetime(mavs::math::rand_in_range(2.0f, 5.0f));
			droplets.push_back(drop);
		}
		for (int d = 0; d < droplets.size(); d++) {
			droplets[d].SetAge(droplets[d].GetAge() + (float)dt);
			if (droplets[d].GetAge() > droplets[d].GetLifetime()) {
				droplets.erase(droplets.begin() + d);
			}
		}
		cimg_library::CImg<float> new_image = image_;
		for (int d = 0; d < droplets.size(); d++) {
			glm::ivec2 c = droplets[d].GetCenterPixels();
			int rp = droplets[d].GetRadiusPixels();
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic) 
#endif
			for (int i = c.x - rp; i <= c.x + rp; i++) {
				for (int j = c.y - rp; j <= c.y + rp; j++) {
					if (i >= 0 && i < num_horizontal_pix_ && j >= 0 && j < num_vertical_pix_) {
						glm::vec2 pixel(i, j);
						glm::vec2 center((float)c.x, (float)c.y);
						float r = glm::length(pixel - center);
						if (droplets[d].PixelInDrop(i, j) && r <= rp) {
							//blur filter
							glm::vec3 newcol(0.0f, 0.0f, 0.0f);
							int nfilt = 5;
							int nr = (nfilt - 1) / 2;
							int nadded = 0;
							for (int ii = -nr; ii <= nr; ii++) {
								int iii = ii + i;
								for (int jj = -nr; jj <= nr; jj++) {
									int jjj = jj + j;
									if (iii >= 0 && iii < num_horizontal_pix_ && jjj >= 0 && jjj < num_vertical_pix_) {
										newcol = newcol + glm::vec3(image_(iii, jjj, 0), image_(iii, jjj, 1), image_(iii, jjj, 2));
										nadded = nadded + 1;
									}
								}
							}
							newcol = newcol / (float)(nadded);
							glm::vec3 oldcol = glm::vec3(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
							float s = 1.0f - exp(-((float)r / rp));
							newcol = (1.15f)*(1.0f - s) * newcol + s * oldcol;
							new_image.draw_point(i, j, (float *)&newcol);
						}
					}
				}
			}
		}
		image_ = new_image;
	}

} // add rain to image

} //namespace camera
} //namespace sensor
} //namespace mavs