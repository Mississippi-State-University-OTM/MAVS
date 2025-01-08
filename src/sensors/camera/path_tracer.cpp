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

#include <sensors/camera/path_tracer.h>
#include <iostream>
#include <numeric>
#include <limits>
#include <cmath>
#include <raytracers/fresnel.h>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace sensor {
namespace camera {

static float random() {
	float x = mavs::math::rand_in_range(0.0f, 1.0f);
	return x;
}

static float HenyeyGreenstein(float costheta, float g) {
	//see Real-time Scattering, E.R.S. Livermore, 2012 (page 29)
	float g2 = g * g;
	float d1 = 0.25f / (float)mavs::kPi;
	float d2 = (float)pow(1+g2-2.0f*g*costheta, 1.5f);
	float hg = (1.0f - g2) / (d1*d2);
	return hg;
}

PathTracerCamera::PathTracerCamera() {
	updated_ = false;
	local_sim_time_ = 0.0f;
	Initialize(512, 512, 0.0035f, 0.0035f, 0.0035f);
	gamma_ = 0.75f;
	gain_ = 1.0f;
	num_iter_ = 3;
	max_depth_ = 5;
	rr_val_ = 0.66f;
	fix_pixels_ = false;
	sun_color_ = glm::vec3(100.0f, 100.0f, 100.0f);
	camera_type_ = "pathtraced";

	rand_array_size_ = 1000000;
	randcount_ = 0;
	rand_array_.resize(rand_array_size_);
	for (int i = 0; i < (int)rand_array_size_; i++) {
		rand_array_[i] = random();
	}
}

PathTracerCamera::~PathTracerCamera() {}

glm::vec3 PathTracerCamera::OrientNormal(glm::vec3 normal, glm::vec3 direction) {
	if (glm::dot(normal, direction) < 0.0f) {
		return -1.0f*normal; // flip normal
	}
	else {
		return normal;
	}
}

glm::vec3 PathTracerCamera::f_func(glm::vec3 ks, float exp, glm::vec3 incoming, glm::vec3 outgoing, glm::vec3 normal) {
	float ndotwi = glm::dot(normal, incoming); //normal and incoming light
	glm::vec3 reflect_dir = (-1.0f*incoming) + normal * 2.0f * ndotwi;
	float rdotwo = glm::dot(reflect_dir, outgoing);
	if (rdotwo > 0.0f) {
		return (ks * (float)pow(rdotwo, exp));
	}
	else {
		return glm::vec3(0.0f, 0.0f, 0.0f);
	}
}

glm::vec3 PathTracerCamera::SampleHemisphere(float u1, float u2, float exp) {
	//Pass two numbers range[0, 1] and exponent
	float z = (float)pow(1.0f - u1, 1.0f / (exp + 1.0f));
	float phi = (float)mavs::k2Pi * u2; //Azimuth
	float theta = (float)sqrt(std::max(0.0f, 1.0f - z * z)); //Polar
	glm::vec3 p; // = Vector3D
	p.x = theta * cos(phi);
	p.y = theta * sin(phi);
	p.z = z;
	return p;
}

glm::vec4 PathTracerCamera::sample_f(float exp, glm::vec3 normal, glm::vec3 outgoing) {
	//perfect mirror reflection
	float ndotwo = glm::dot(normal, outgoing);
	glm::vec3 reflect_dir = (-1.0f*outgoing) + normal * 2.0f * ndotwo;
	//orthonormal basis
	glm::vec3 w = reflect_dir;
	glm::vec3 v = glm::cross(glm::vec3(0.00419f, 0.0078f, 1.0f), w);
	v = glm::normalize(v);
	glm::vec3 u = glm::cross(v, w);
	//random samples
	//float u1 = random(); //[0, 1]
	//float u2 = random(); //[0, 1]
	float u1 = rand_array_[randcount_%rand_array_size_]; //[0, 1]
	randcount_++;
	float u2 = rand_array_[randcount_%rand_array_size_];; //[0, 1]
	randcount_++;
	glm::vec3 p = SampleHemisphere(u1, u2, exp); //point in hemi
	glm::vec3 wi = (u * p.x) + (v * p.y) + (w * p.z); //linear projection
	if (glm::dot(normal, wi) < 0.0) { //if reflected direction is below surface
		wi = (u * -p.x) + (v * -p.y) + (w * p.z); //reflect it
	}
	//phong lobe
	float phong_lobe = pow(glm::dot(reflect_dir, wi), exp);
	float pdf = glm::dot(normal, wi) * phong_lobe;
	glm::vec4 wr(wi.x, wi.y, wi.z, pdf);
	return wr;
}

glm::vec3 PathTracerCamera::CalculateRainColor(glm::vec3 incolor, float distance, glm::vec3 direction, environment::Environment *env) {
	// see Physics-Based Rendering for Improving Robustness to Rain, SS Halder 2019, Eqns 1-2
	float L_ext = (float)exp(-0.312f*pow(env->GetRainRate(), 0.67f)*distance);
	float costheta = glm::dot(direction, env->GetSolarDirection());
	glm::vec3 A_in = HenyeyGreenstein(costheta, 0.5f)*sun_color_*(1.0f - L_ext);
	glm::vec3 outcolor = incolor * L_ext + A_in;
	return outcolor;
}

glm::vec3 PathTracerCamera::TraceLights(glm::vec3 hit_point, glm::vec3 direction, environment::Environment *env) {
	/*if (env->IsSkyOn() && env->GetNumLights() == 0) {
		// add the sun
		environment::Light sun;
		sun.type = 1; //
		sun.color = 0.5f*env->GetSunColor(); 
		// sun distance and radius are factor of a million
		// smaller than real to keep floating point arithmetic
		// more reasonable
		sun.position = 149.6E3f*env->GetSolarDirection();
		sun.radius = 695.51f;
		sun.decay = 0.0f;
		env->AddLight(sun);
	}*/

	glm::vec3 result(0.0f, 0.0f, 0.0f);
	if (env->GetNumLights() > 0) {
		//int light_to_sample = (int)(math::rand_in_range(0.0f, (float)100.0f*env->GetNumLights() - 100.0f)/100.0f);
		int light_to_sample = math::rand_in_range(0, (int)env->GetNumLights());
		environment::Light light = env->GetLight(light_to_sample);
		glm::vec3 rando(math::rand_in_range(-1.0f, 1.0f), math::rand_in_range(-1.0f, 1.0f), math::rand_in_range(-1.0f, 1.0f));
		rando = glm::normalize(rando);
		rando = light.radius*rando;
		glm::vec3 to_light = (light.position + rando) - hit_point;
		float light_dist = glm::length(to_light);
		to_light = to_light / light_dist;
		raytracer::Intersection light_inter = env->GetClosestIntersection(hit_point, to_light);
		if (light_inter.dist < 0.0f && light.type == 1) { // not blocked, point light
			result = result + light.color / (float)pow(light_dist, light.decay);
		}
		else if (light_inter.dist < 0.0f && light.type == 2) { // not blocked, spot light
			float ang = acos(glm::dot(to_light, -1.0f*light.direction));
			float x = ang / light.angle;
			result = result + (float)exp(-2.0f * x * x)*light.color / (float)pow(light_dist, light.decay);
		}
	}
	return result;
}

glm::vec3 PathTracerCamera::TraceRay(glm::vec3 origin, glm::vec3 direction, int depth, environment::Environment *env) {
	
	glm::vec3 result = glm::vec3(0.0f, 0.0f, 0.0f); //black
	if (depth > max_depth_) {
		return result;
	}

	raytracer::Intersection inter = env->GetClosestIntersection(origin, direction);

	if (inter.dist < 0.0f) { //No Hit
		glm::vec3 skycolor = env->GetSkyRgb(direction, pix_theta_);
		//if (env->IsRaining()) skycolor = CalculateRainColor(skycolor, std::numeric_limits<float>::infinity(), direction, env);

		return skycolor;
	}
	else {
		glm::vec3 hit_point = origin + 0.999999f*inter.dist*direction;

		result = result + TraceLights(hit_point, direction, env);

		//if (env->IsRaining()) result = CalculateRainColor(result, inter.dist, direction, env);

		glm::vec3 wo = -1.0f*direction;  //outgoing(towards camera)
		glm::vec3 normal = inter.normal;
		normal = OrientNormal(normal, wo); //make normal point in correct direction
		
		glm::vec4 shading_data = sample_f(inter.material.ns, normal, wo);
		glm::vec3 wi(shading_data[0], shading_data[1], shading_data[2]); //incoming direction
		float pdf = shading_data[3]; //pdf
		if (pdf <= 0.0f) pdf = 1.0;
		glm::vec3 f = f_func(inter.color, inter.material.ns, wi, wo, normal);
		/*
		// check if the ray is actually refracted
		if (inter.material.ni > 0.0f) {
			float theta_incident = (float)(mavs::kPi_2 - acos(glm::dot(normal, wo)));
			float theta_refracted = mavs::raytracer::GetFresnelTransmissionAngle(1.0f, inter.material.ni, theta_incident);
			float refl_frac = mavs::raytracer::GetFresnelReflectance(1.0f, inter.material.ni, theta_incident, theta_refracted);
			float reflect_test_val = mavs::math::rand_in_range(0.0f, 1.0f);
			if (reflect_test_val > refl_frac) {
				//wi = direction; // need to actually refract this
				glm::vec3 kr = glm::cross(inter.normal, direction);
				kr = glm::normalize(kr);
				wi = math::RodriguesRotation(direction, kr, (theta_incident - theta_refracted));
				pdf = 1.0f;
				f = inter.color;
				hit_point = origin + 1.000001f*inter.dist*direction;
			}
		}
		*/
		if (depth > 2) {
			if (mavs::math::rand_in_range(0.0f, 1.0f) < rr_val_) {
				return result;
			}
		}

		glm::vec3 next_col = TraceRay(hit_point, wi, depth + 1, env);
		result = result + f * next_col * glm::dot(wi, normal) / pdf;
		//Add emission
		result = result + inter.material.ke;
		result = result / rr_val_;
		//} // if not in solar direction
	}

	return result; //return final colour
}

void PathTracerCamera::Update(environment::Environment *env, double dt) {
	CheckFreq(dt);
	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;
	pix_theta_ = 2.0f*(float)atan(horizontal_pixdim_ / focal_length_);
	sun_color_ = env->GetSkyRgb(env->GetSolarDirection(), pix_theta_);

	randcount_ = 0;
	ZeroBuffer();

#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j); 
#ifdef USE_MPI
			if (n % comm_size_ == comm_rank_) {
#endif
			// Fill the range and label buffer
			glm::vec3 direction_one = look_to_f + (i - half_horizontal_dim_)*look_side_d + (j - half_vertical_dim_ )*look_up_d;
			direction_one = glm::normalize(direction_one);
			raytracer::Intersection base_inter = env->GetClosestIntersection(position_, direction_one);
			if (base_inter.dist > 0.0f) {
				range_buffer_[n] = base_inter.dist;
				segment_buffer_[n] = base_inter.object_id;
				label_buffer_[n] = base_inter.label;
			}

			glm::vec3 pix_color(0.0f, 0.0f, 0.0f);
			int ns_this = 0;
			float green_accum = 0.0f;
			float depth_accum_ = 0.0f;
			for (int s = 0; s < num_iter_; s++) {
				//glm::vec3 direction = look_to_f + (i - half_horizontal_dim_ + random())*look_side_d + (j - half_vertical_dim_ + random())*look_up_d;
				float xfr = rand_array_[randcount_%rand_array_size_];
				randcount_++;
				float yfr = rand_array_[randcount_%rand_array_size_];
				randcount_++;
				glm::vec3 direction = look_to_f + (i - half_horizontal_dim_ + xfr)*look_side_d + (j - half_vertical_dim_ + yfr)*look_up_d;
				direction = glm::normalize(direction);
				glm::vec3 this_color = TraceRay(position_, direction, 1, env);
				float cmag = glm::length(this_color);
				if (!std::isnan(this_color.x) && !std::isnan(this_color.y) && !std::isnan(this_color.z) &&
					!std::isinf(this_color.y) && !std::isinf(this_color.y) && !std::isinf(this_color.z) &&
					cmag<1000.0f) {
					for (int kk = 0; kk < 3; kk++) if (this_color[kk] < 0.0f) this_color[kk] = 0.0f;
					// stop the loop if the pixel is converged
					//if (s > 0 && s % 10 == 0 && !env->IsRaining()) {
					if (s > 0 && s % 10 == 0 ) {
						float old_avg = green_accum / ns_this;
						float new_avg = (green_accum + this_color.y) / (ns_this + 1.0f);
						float change = fabs(old_avg - new_avg) / old_avg;
						if (change < 1.0E-5) break;
					}
					green_accum += this_color.y;
					pix_color += this_color;
					ns_this++;
				}
			}
			if (ns_this > 0)pix_color = pix_color / (float)ns_this;
			//apply gamma compression and copy to image_ buffer
			for (int c = 0; c < 3; c++) {
				image_(i, j, c) = pow(pix_color[c], gamma_);
			}
#ifdef USE_MPI
		}
#endif
		} // j
	} //i


#ifdef USE_MPI  
	ReduceImageBuffer();
#endif

	if (fix_pixels_ && env->IsSkyOn()) {
		FixBadPixels();
	}

	//RenderParticleSystems(env);
	//if (env->IsRaining())PTMask(env,(float)dt); //doesn't work as well as default
	if (env->IsRaining()) {
		raindrops_on_lens_ = false;
		CreateRainMask(env, dt);
	}

	if (env->IsSnowing()) {
		RenderSnow(env, dt);
	}

	if (env->IsFoggy()) {
		RenderFog(env);
	}

	RenderParticleSystems(env);

	//ApplyElectronics();

	CopyBufferToImage();

	if (log_data_) {
		SaveImage(utils::ToString(local_sim_time_) + log_file_name_);
	}

	Normalize();

	local_sim_time_ += local_time_step_;
	updated_ = true;
}

void PathTracerCamera::Normalize() {
	float scale = 128.0f / (float)image_.mean();
	if (norm_type_ == "max")scale = 255.0f / (float)image_.max();
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			for (int k = 0; k < 3; k++) {
				image_(i, j, k) = image_(i, j, k)*scale;
				if (image_(i,j,k) < 0.0f) image_(i,j,k) = 0.0f;
				if (image_(i, j, k) > 255.0f) image_(i, j, k) = 255.0f;
			}
		}
	}
}


std::vector<float> PathTracerCamera::GetLocalDistribution(int i, int j, int win_size, bool use_center, bool use_weighted) {
	std::vector<float> distribution;
	for (int ii = std::max(0, i - win_size); ii < std::min(num_horizontal_pix_ - 1, i + win_size); ii++) {
		for (int jj = std::max(0, j - win_size); jj < std::min(num_vertical_pix_ - 1, j + win_size); jj++) {
			if (!(ii == i && jj == j) || use_center) {
				glm::vec3 p(image_(ii, jj, 0), image_(ii, jj, 1), image_(ii, jj, 2));
				distribution.push_back(glm::length(p));
			}
		}
	}
	return distribution;
}

glm::vec3 PathTracerCamera::GetLocalAverage(int i, int j, int win_size, bool use_center, bool use_weighted) {
	glm::vec3 local_avg(0.0f, 0.0f, 0.f);
	float nlocal = 0.0f;
	for (int ii = std::max(0, i - win_size); ii < std::min(num_horizontal_pix_ - 1, i + win_size); ii++) {
		for (int jj = std::max(0, j - win_size); jj < std::min(num_vertical_pix_ - 1, j + win_size); jj++) {
			if (!(ii == i && jj == j) || use_center) {
				glm::vec3 p(image_(ii, jj, 0), image_(ii, jj, 1), image_(ii, jj, 2));
				if (glm::length(p) != 0.0f) {
					float w = 1.0f;
					if (use_weighted) {
						w = glm::length((glm::vec2((float)(ii - i), (float)(jj - j))));
					}
					local_avg += w * p;
					nlocal += w;
				}
			}
		}
	}
	local_avg = local_avg / nlocal;
	return local_avg;
}

void PathTracerCamera::FixBadPixels() {
	cimg_library::CImg<float> fixed_image = image_;
	int win_size = 2;
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			std::vector<float> v = GetLocalDistribution(i, j, win_size, true, false);
			double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
			double mean = sum / v.size();

			double accum = 0.0;
			std::for_each(std::begin(v), std::end(v), [&](const double d) {
				accum += (d - mean) * (d - mean);
			});
			double stdev = sqrt(accum / (v.size() - 1));
			glm::vec3 col(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
			float p = glm::length(col);

			if (fabs(p - mean) > 2.5f*stdev) {
				glm::vec3 mu = GetLocalAverage(i, j, win_size, false, true);
				fixed_image.draw_point(i, j, (float *)&mu);
			}
			/*glm::vec3 mu = GetLocalAverage(i, j, win_size, false, true);
			glm::vec3 col(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
			if (glm::length(mu - col) > 10.0f) {
				fixed_image.draw_point(i, j, (float *)&mu);
			}*/
		}
	}
	image_ = fixed_image;
}

void PathTracerCamera::PTMask(mavs::environment::Environment *env, float dt) {
	
	std::vector< std::vector< float > > rain_mask = mavs::utils::Allocate2DVector(num_horizontal_pix_, num_vertical_pix_, 0.0f);
	//environmental factors
	float rate = env->GetRainRate(); //mm/h
	glm::vec3 wind(env->GetWind().x, env->GetWind().y, 0.0);

	//empirical stuff
	/*float alpha = -2.0f*0.0001f*rate;
	glm::vec3 skycol = env->GetAvgSkyRgb();
	float raindrop_rho = 0.02f*(skycol.x + skycol.y + skycol.z) / 3.0f;
	glm::vec3 rain_col(raindrop_rho, raindrop_rho, raindrop_rho);*/

	float mean = (float)image_.mean();
	float alpha = -0.00009f*rate;
	float raindrop_rho = 0.05f*mean;
	glm::vec3 rain_col = glm::vec3(raindrop_rho, raindrop_rho, raindrop_rho);

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
			if (use_distorted_) {
				glm::vec2 meters((l_pos.x - half_horizontal_dim_ + 0.5f)*horizontal_mag_scale_,
					(l_pos.y - half_vertical_dim_ + 0.5f)*vertical_mag_scale_);
				distorted_pos = dm_.Distort(meters);
			}
			u = (int)distorted_pos.x;
			v = (int)distorted_pos.y;
			if (u >= 0 && u < num_horizontal_pix_ && v >= 0 && v < num_vertical_pix_) {
				int n = GetFlattenedIndex(u, v);
				if (z < range_buffer_[n] || range_buffer_[n] == 0.0f) {
					rain_mask[u][v] = rain_mask[u][v] + raindrop_rho; // to_add;
				}
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
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			//------ Light absorbed by the rain ------------------------
			int n = GetFlattenedIndex(i, j);
			if (range_buffer_[n] > 0.0) {
				float r = std::min(1000.0f, range_buffer_[n]);
				float abs_fac = exp(alpha*r);
				for (int k = 0; k < 3; k++)image_(i, j, k) = abs_fac * image_(i, j, k) + (1.0f - abs_fac)*rain_col[k];
			}
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
			float r = image_(i, j, 0);
			float g = image_(i, j, 1);
			float b = image_(i, j, 2);
			float intens = (float)sqrt(r*r + g * g + b * b);
			float scale_fac = (intens_max - intens)*sf;
			for (int k = 0; k < 3; k++) {
				image_(i, j, k) = image_(i, j, k) + rain_mask[i][j] * scale_fac;
			}
		}
	}

	/*
	if (raindrops_on_lens_) {
		float lens_area = horizontal_pixdim_ * vertical_pixdim_;
		float numfac = (float)(lens_area * rate * 1.0E10);
		int num_raindrops = (int)(10.0f*(1.0 - exp(-0.1*numfac)));
		int num_to_add = num_raindrops - (int)droplets_.size();
		for (int i = 0; i<num_to_add; i++) {
			LensDrop drop;
			drop.SetRadius(0.2f*mavs::math::rand_in_range(0.75f*d_g, 1.1f*d_g));
			drop.SetRadiusPixels((int)(drop.GetRadius() / horizontal_pixdim_));
			drop.SetCenterPixels(mavs::math::rand_in_range(0, num_horizontal_pix_), mavs::math::rand_in_range(0, num_vertical_pix_));
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
		cimg_library::CImg<float> new_image = image_;
		for (int d = 0; d < droplets_.size(); d++) {
			glm::ivec2 c = droplets_[d].GetCenterPixels();
			int rp = droplets_[d].GetRadiusPixels();
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic) 
#endif
			for (int i = c.x - rp; i <= c.x + rp; i++) {
				for (int j = c.y - rp; j <= c.y + rp; j++) {
					if (i >= 0 && i < num_horizontal_pix_ && j >= 0 && j < num_vertical_pix_) {
						glm::vec2 pixel(i, j);
						glm::vec2 center((float)c.x, (float)c.y);
						float r = glm::length(pixel - center);
						if (droplets_[d].PixelInDrop(i, j) && r <= rp) {
							//blur filter
							glm::vec3 newcol(0.0f, 0.0f, 0.0f);
							int nfilt = 5;
							int nr = (nfilt - 1) / 2;
							for (int ii = -nr; ii <= nr; ii++) {
								int iii = ii + i;
								for (int jj = -nr; jj <= nr; jj++) {
									int jjj = jj + j;
									if (iii >= 0 && iii < num_horizontal_pix_ && jjj >= 0 && jjj < num_vertical_pix_) {
										newcol = newcol + glm::vec3(image_(iii, jjj, 0), image_(iii, jjj, 1), image_(iii, jjj, 2));
									}
								}
							}
							newcol = newcol / (0.8f*nfilt*nfilt);
							glm::vec3 oldcol = glm::vec3(image_(i, j, 0), image_(i, j, 1), image_(i, j, 2));
							float s = 1.0f - exp(-((float)r / rp));
							newcol = (1.0f - s) * newcol + s * oldcol;
							new_image.draw_point(i, j, (float *)&newcol);
						}
					}
				}
			}
		}
		image_ = new_image;
	} // raindrops on lens
	*/
	/*
	std::vector< std::vector< float > > rain_mask = mavs::utils::Allocate2DVector(num_horizontal_pix_, num_vertical_pix_, 0.0f);
	//environmental factors
	glm::vec3 skycol = env->GetAvgSkyRgb();
	float rate = env->GetRainRate(); //mm/h
	glm::vec3 wind(env->GetWind().x, env->GetWind().y, 0.0);

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
	float range_max = 8.0f*focal_length_ * ((d_g / std::max(horizontal_pixdim_, vertical_pixdim_)) - 1.0f);
	float width_mag = 0.5f*focal_array_width_ / focal_length_;
	float height_mag = 0.5f*focal_array_height_ / focal_length_;
	float dh = 2.0f*range_max * height_mag;
	float dw = 2.0f*range_max * width_mag;
	float rain_vol = range_max * dw*dh / 3.0f; //volume of view frustum
	int total_num_drops = (int)(n_t * rain_vol);
	float halfwidth = 0.5f*focal_array_width_;
	float halfheight = 0.5f*focal_array_height_;

	cimg_library::CImg<float> temp_image_ = image_;

#ifdef USE_OMP  
#pragma omp parallel for
#endif  
	for (int i = 0; i < total_num_drops; i++) {
		float z = mavs::math::rand_in_range(2.0f*focal_length_, range_max);
		float dx = width_mag * z;
		float dy = height_mag * z;
		float x = mavs::math::rand_in_range(-dx, dx);
		float y = mavs::math::rand_in_range(-dy, dy);
		float m = focal_length_ / z;
		x = m * x;
		y = m * y;
		float streak_width = (0.125f*d_g*m / (horizontal_pixdim_ + vertical_pixdim_));
		int sw = (int)streak_width;
		int u0 = (int)((x + halfwidth) / horizontal_pixdim_);
		int v0 = (int)((y + halfheight) / vertical_pixdim_);
		//get the streak length and direction in pixels
		glm::vec2 l_streak(m * wind_projected.x*exposure_time_ / horizontal_pixdim_,m * wind_projected.y*exposure_time_ / vertical_pixdim_);
		float l_streak_mag = glm::length(l_streak);
		l_streak = l_streak / l_streak_mag;
		glm::vec2 l_pos((float)u0, (float)v0);
		for (int ns = 0; ns <= (int)l_streak_mag; ns++) {
			int uu = (int)l_pos.x;
			int vv = (int)l_pos.y;  
			for (int swu = -sw; swu <= sw; swu++) {
				for (int swv = -sw; swv <= sw; swv++) {
					int u = uu + swu;
					int v = vv + swv;
					float dfac = (float)sqrt(swu*swu + swv * swv)*horizontal_pixdim_;
					if (dfac < streak_width) {
						if (u >= 0 && u < num_horizontal_pix_ && v >= 0 && v < num_vertical_pix_) {
							//calculate rain color here
							// see Physics-Based Rendering for Improving Robustness to Rain, SS Halder 2019, Eqn 4
							glm::vec3 local_avg = GetLocalAverage(u, v, 3, true, true);
							float s = dfac / streak_width;
							s = 4.0f*s * s;
							glm::vec3 new_color = 0.94f*local_avg + s*env->GetSunColor();
							temp_image_.draw_point(u, v, (float *)&new_color);
						}
					}
				}// swv loop
			} //swu loop
			l_pos += l_streak;
		}
	}
	image_ = temp_image_;
	*/
}

} //namespace camera
} //namespace sensor
} //namespace mavs
