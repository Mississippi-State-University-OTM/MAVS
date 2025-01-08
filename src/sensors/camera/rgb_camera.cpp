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
// MPI has to be included first or 
// the compiler chokes up
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <sensors/camera/rgb_camera.h>
#include <iostream>
#include <algorithm>
#include <ctime>
#include <limits>
#include <glm/glm.hpp>
#include <mavs_core/math/utils.h>
#include <raytracers/fresnel.h>
#ifdef USE_OMP
#include <omp.h>
#endif

namespace mavs {
namespace sensor {
namespace camera {

RgbCamera::RgbCamera() {
	updated_ = false;
	local_sim_time_ = 0.0f;
	Initialize(512, 512, 0.0035f, 0.0035f, 0.0035f);
	gamma_ = 0.75f;
	//gain_ = pow(255.0,1.0-gamma_);
	for (int i = 0; i < 3; i++) {
		suncolor_[i] = 255.0f;
		up_[i] = 0.0f;
	}
	up_.z = 1.0f;
	type_ = "camera";
	env_set_ = false;
	camera_type_ = "rgb";
	do_fast_lights_ = true;
	frame_count_ = 0;
	blur_on_ = false;
}

RgbCamera::~RgbCamera() {

}

void RgbCamera::ConvertImageToRccb() {
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = j + i * num_vertical_pix_;
			float new_green = 0.3f * image_(i, j, 0) + 0.59f * image_(i, j, 1) + 0.11f * image_(i, j, 2);
			image_(i, j, 1) = new_green;
		}
	}
}

void RgbCamera::SetElectronics(float gamma, float gain) {
	gamma_ = gamma;
	//gain_ = gain*pow(255.0,1-gamma);
}

void RgbCamera::SetEnvironmentProperties(environment::Environment* env) {
	//set solar properties
	sun_direction_ = env->GetSolarDirection();
	suncolor_ = env->GetSunColor();
	//skycolor_ = env->GetAvgSkyRgb();
		// ctg 12/8/22
	skycolor_ = 3.5f * env->GetAvgSkyRgb();
	groundcolor_ = env->GetAlbedoRgb();
	env_set_ = true;
}

RgbPixel RgbCamera::RenderPixel(environment::Environment* env, glm::vec3 direction) {
	RgbPixel intersection;
	intersection.object_id = -1;
	raytracer::Intersection inter = env->GetClosestIntersection(position_, direction);
	//raytracer::Intersection inter;
	//env->GetClosestIntersection(position_, direction, inter);
	glm::vec3 color = zero_vec3;
	float range = std::numeric_limits<float>::max();
	if (inter.dist > 0.0f) {
		intersection.object_id = inter.object_id;
		intersection.label = inter.label;
		range = inter.dist;
		glm::vec3 point = position_ + 0.999f * inter.dist * direction;
		glm::vec3 diffcol = zero_vec3;
		glm::vec3 speccol = zero_vec3;
		inter.normal = glm::normalize(inter.normal);
		//first check the sun
		bool shadowed = false;
		if (render_shadows_)shadowed = env->GetAnyIntersection(point, sun_direction_);

		if (!shadowed) {
			//lighting model is a modified phong model from
			// "Making Shaders  More Physically Plausible" by RR Lewis
			float costheta = glm::dot(inter.normal, sun_direction_);
			//make sure the normal is facing "out"
			if (costheta <= 0.0) {
				inter.normal = -1.0f * inter.normal;
				costheta = -costheta;
			}
			intersection.normal = inter.normal;
			diffcol = costheta * suncolor_ * inter.material.kd;
			// the following lines are phong-like specular reflectance
			//glm::vec3 spec_dir = 2*(costheta)*inter.normal - sun_direction_;
			//float specular = pow(math::clamp(glm::dot(spec_dir,sun_direction_),0.0f,1.0f),inter.material.ns);
			//blinn-like specular reflectance
			glm::vec3 H = (sun_direction_ - direction);
			H = glm::normalize(H);
			float specular = pow(std::max(glm::dot(inter.normal, H), 0.0f), inter.material.ns);
			speccol = specular * suncolor_ * inter.material.ks;
		}
		//now loop through the artificial lights
		for (int i = 0; i < (int)env->GetNumLights(); i++) {
			environment::Light light = env->GetLight(i);
			if (light.is_active) {
				glm::vec3 to_light = light.position - point;
				float dist_to_light = glm::length(to_light);
				if (dist_to_light < light.cutoff_distance) {
					to_light = to_light / dist_to_light;
					glm::vec3 offset_point = point + 1.01f * to_light;
					raytracer::Intersection linter =
						env->GetClosestIntersection(offset_point, to_light);
					if (linter.dist<0 || linter.dist>dist_to_light) {
						float falloff = pow(dist_to_light, light.decay);
						glm::vec3 intens = light.color / falloff;
						if (light.type == 2) { //spotlight;
							float ang = acosf(glm::dot(to_light, -1.0f * light.direction));
							//if (ang>light.angle)intens = 0.0f*light.color;
							float x = ang / light.angle;
							intens = (float)exp(-2 * x * x) * light.color;
						}
						float diffuse = glm::dot(inter.normal, to_light);
						if (diffuse <= 0.0) {
							inter.normal = -1.0f * inter.normal;
							diffuse = -diffuse;
						}
						diffcol = diffcol + diffuse * intens;
						glm::vec3 spec_dir = 2 * (diffuse)*inter.normal - to_light;
						float specular = pow(math::clamp(glm::dot(spec_dir, to_light),
							0.0f, 1.0f), inter.material.ns);
						speccol = speccol + specular * intens;
					}
				} // cutoff distance
			} // if do_light
		} // loop over the artificial lights

		//Make the normal be facing me
		float dir = glm::dot(inter.normal, direction);
		if (dir > 0.0) inter.normal = -1.0f * inter.normal;
		float theta = acosf(inter.normal.z);
		float a = 0.5f * sinf(theta);
		if (theta < kPi_2) a = 1 - a;
		glm::vec3 amblight = skycolor_ * (a + (1 - a) * groundcolor_);
		glm::vec3 total_lighting = (diffcol + speccol + (float)k1_Pi * amblight);
		color = inter.color * total_lighting;

		// check if we hit a transparent material
		if (inter.material.ni > 0.0) {
			float theta_incident = (float)(mavs::kPi_2 - acosf(glm::dot(inter.normal, -1.0f * direction)));
			float theta_refracted = mavs::raytracer::GetFresnelTransmissionAngle(1.0f, inter.material.ni, theta_incident);
			float refl_frac = mavs::raytracer::GetFresnelReflectance(1.0f, inter.material.ni, theta_incident, theta_refracted);
			raytracer::Intersection inter2;
			glm::vec3 kr = glm::cross(inter.normal, direction);
			kr = glm::normalize(kr);
			glm::vec3 new_dir = math::RodriguesRotation(direction, kr, (theta_incident - theta_refracted));
			glm::vec3 hit_point = position_ + 1.000001f * inter.dist * direction;
			env->GetClosestIntersection(hit_point, new_dir, inter2);
			glm::vec3 new_color;
			if (inter2.dist > 0.0f) {
				new_color = inter2.color * total_lighting;
			}
			else {
				new_color = env->GetSkyRgb(new_dir, pixel_solid_angle_);
			}

			color = refl_frac * color + (1.0f - refl_frac) * new_color;
		} // if transparent material

	}
	else {
		color = env->GetSkyRgb(direction, pixel_solid_angle_);
	}

	//apply gamma compression
	for (int i = 0; i < 3; i++) {
		color[i] = pow(std::max(color[i], 0.0f), gamma_);
	}

	//glm::vec4 intersection(color.x,color.y,color.z,range);
	intersection.color = color;
	intersection.distance = range;
	if (isnan(intersection.color.r) || std::isnan(intersection.color.g) || std::isnan(intersection.color.b) || std::isinf(intersection.color.r) || std::isinf(intersection.color.g) || std::isinf(intersection.color.b)) {
		intersection.color = glm::vec3(5.0f, 5.0f, 5.0f);
	}
	return intersection;
}

/*void RgbCamera::RenderParticleSystems(environment::Environment *env){
	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;

	environment::ParticleSystem *systems = env->GetParticleSystems();
	for (int ps=0; ps<(int)env->GetNumParticleSystems(); ps++){
	environment::Particle *particles = systems[ps].GetParticles();
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic)
#endif
	for (int p=0;p<(int)systems[ps].GetNumParticles();p++){
		float r = particles[p].radius_;
		glm::vec3 to_particle = particles[p].position_ - position_;
		float x = glm::dot(to_particle,look_side_);
		float y = glm::dot(to_particle,look_up_);
		float R = glm::dot(to_particle,look_to_);
		float M = focal_length_/R;
		float M_h = M/horizontal_pixdim_;
		float M_v = M/vertical_pixdim_;
		int px_l = (int)(half_horizontal_dim_ + floor(M_h*(x-r)));
		int px_h = (int)(half_horizontal_dim_ + ceil(M_h*(x+r)));
		int py_l = (int)(half_vertical_dim_ + floor(M_v*(y-r)));
		int py_h = (int)(half_vertical_dim_ + ceil(M_v*(y+r)));
		if (px_l<0) px_l = 0;
		if (px_l>=num_horizontal_pix_) px_l = num_horizontal_pix_-1;
		if (px_h<0) px_h = 0;
		if (px_h>=num_horizontal_pix_) px_h = num_horizontal_pix_-1;
		if (py_l<0) py_l = 0;
		if (py_l>=num_vertical_pix_) py_l = num_vertical_pix_-1;
		if (py_h<0) py_h = 0;
		if (py_h>=num_vertical_pix_) py_h = num_vertical_pix_-1;
		for (int u=px_l;u<=px_h;u++){
				for (int v =py_l;v<py_h;v++){
					int n = GetFlattenedIndex(u, v); // v + u * num_vertical_pix_;

					float ud = (float)u;
					float vd = (float)v;
					if (use_distorted_) {
						glm::vec2 meters((u - half_horizontal_dim_ + 0.5f)*horizontal_mag_scale_,
							(v - half_vertical_dim_ + 0.5f)*vertical_mag_scale_);
						glm::vec2 pu = dm_.Distort(meters);
						ud = pu.x;
						vd = pu.y;
					}

					glm::vec3 direction = (float)(ud - half_horizontal_dim_)*look_side_d +
							(float)(vd-half_vertical_dim_)*look_up_d + look_to_f;

					//glm::vec3 direction = (float)(u-half_horizontal_dim_)*look_side_d +
					//	(float)(v-half_vertical_dim_)*look_up_d + look_to_f;

					direction = direction/glm::length(direction);
					glm::vec2 d = particles[p].GetIntersection(position_, direction);
					if (d.y>0.0f && d.y<range_buffer_[n]){
						if (d.x<1.0f && d.x>0.0f){
							float s = 1.0f-d.x;
							float t = 1.0f-s*(1.0f-particles[p].transparency_);
							glm::vec3 col;
							int vert_index = num_vertical_pix_-v-1;
							for (int c = 0; c<3;c++){
								//col[c] = (1.0f-t)*suncolor_[c]*particles[p].color_[c] +t*image_buffer_[n][c];
								col[c] = (1.0f - t)*suncolor_[c] * particles[p].color_[c] + t * image_(u,v,c);
								image_(u, v, c) = col[c];
							}
							//image_buffer_[n] = col;
						}
					}
				}
		}
	}
	}
} // RenderParticleSystems
*/

void RgbCamera::ApplyElectronics(double dt) {

	int nf = (int)(1.0f / (float)dt);

	float mean = 0.0f;

	if (frame_count_ > 5) {
		for (int i = 0; i < num_horizontal_pix_; i++) {
			for (int j = 0; j < num_vertical_pix_; j++) {
				for (int k = 0; k < 3; k++) {
					mean += image_(i, j, k);
				}
			}
		}
		mean = mean / (1.0f * num_horizontal_pix_ * num_vertical_pix_ * 3);
	}
	else {
		mean = 5.0f;
		frame_count_++;
	}

	if (mean > 255.0f || mean < 0.0f || isnan(mean))mean = 5.0f;

	//if (!isnan(mean)){
	exposure_.push_back(mean);
	//}
	//else {
	//	exposure_.push_back(150.0f);
	//}

	if (exposure_.size() > nf)exposure_.erase(exposure_.begin());

	float avg = 0.0f;
	if (exposure_.size() > 0) {
		for (int i = 0; i < exposure_.size(); i++) {
			avg += exposure_[i];
		}
		avg = avg / (float)exposure_.size();
	}
	else {
		avg = 100.0f;
	}

	float mfac = 150.0f / avg;

	image_ = mfac * image_;

	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			for (int k = 0; k < 3; k++) {
				if (image_(i, j, k) > 255.0f) image_(i, j, k) = 255.0f;
				if (image_(i, j, k) < 0.0f) image_(i, j, k) = 0.0f;

			}
		}
	}
}

static glm::vec3 GetPixVal(int i, int j) {
	glm::vec3 p(1.0f * i, 1.0f * j, 0.0f);
	return p;
}

void RgbCamera::RenderLoopOversampled(environment::Environment* env) {
	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;
	float scale = (1.0f * pixel_sample_factor_);
	//note that looping over i-j is faster than manually
	// unrolling the loop
#ifdef USE_OMP  
#if defined(_WIN32) || defined(WIN32)
#pragma omp parallel for schedule(dynamic)
#else
#pragma omp parallel for schedule(dynamic) collapse(2)
#endif
#endif  
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j); //j + i * num_vertical_pix_;
#ifdef USE_MPI
			if ((n) % comm_size_ == comm_rank_) {
#endif
				float id = (float)i;
				float jd = (float)j;
				if (use_distorted_) {
					glm::vec2 meters((i - half_horizontal_dim_ + 0.5f) * horizontal_mag_scale_,
						(j - half_vertical_dim_ + 0.5f) * vertical_mag_scale_);
					glm::vec2 pu = dm_.Distort(meters);
					id = pu.x;
					jd = pu.y;
				}
				glm::vec3 col(0.0f, 0.0f, 0.0f);
				float dist_accum = 0.0f;
				float num_dist = 0.0f;
				int obj_id = -1;
				std::string label = "sky";
				for (int k = 0; k < pixel_sample_factor_; k++) {
					glm::vec3 direction;
					if (k == 0) {
						direction = look_to_f
							+ (id - half_horizontal_dim_ + 0.5f) * look_side_d
							+ (jd - half_vertical_dim_ + 0.5f) * look_up_d;
					}
					else {
						direction = look_to_f
							+ (id - half_horizontal_dim_ + mavs::math::rand_in_range(0.1f, 0.9f)) * look_side_d
							+ (jd - half_vertical_dim_ + mavs::math::rand_in_range(0.1f, 0.9f)) * look_up_d;
					}
					direction = glm::normalize(direction);
					RgbPixel p = RenderPixel(env, direction);
					col += p.color;
					if (p.object_id >= 0 || p.object_id == -99) {
						dist_accum += p.distance;
						num_dist += 1.0f;
						normal_buffer_[n] = p.normal;
					}
					obj_id = p.object_id;
					label = p.label;
				}
				//col = col / scale;
				if (num_dist > 0.0f) {
					dist_accum = dist_accum / num_dist;
				}
				else {
					dist_accum = 10000.0f;
				}
				//image_buffer_[n] = col/scale; // glm::vec3(col.x, col.y, col.z);
				for (int c = 0; c < 3; c++) image_(i, j, c) = col[c] / scale;
				range_buffer_[n] = dist_accum; // / scale;
				segment_buffer_[n] = obj_id;
				label_buffer_[n] = label;
#ifdef USE_MPI
			}
#endif
		}
	}
}

void RgbCamera::RenderLoopCorners(environment::Environment* env) {

	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;

	std::vector<glm::vec3> image_buffer_expanded;
	std::vector<float> range_buffer_expanded;
	std::vector<int> segment_buffer_expanded;
	std::vector<std::string> label_buffer_expanded;
	int exp_size = (num_horizontal_pix_ + 1) * (num_vertical_pix_ + 1);
	segment_buffer_expanded.resize(exp_size, -1);
	label_buffer_expanded.resize(exp_size, "sky");
	image_buffer_expanded.resize(exp_size, zero_vec3);
	range_buffer_expanded.resize(exp_size, 0.0f);

	//note that looping over i-j is faster than manually
	// unrolling the loop
#ifdef USE_OMP
#if defined(_WIN32) || defined(WIN32)
#pragma omp parallel for schedule(dynamic)
#else
#pragma omp parallel for schedule(dynamic) collapse(2)
#endif
#endif
	for (int i = 0; i < num_horizontal_pix_ + 1; i++) {
		for (int j = 0; j < num_vertical_pix_ + 1; j++) {
			int n = GetFlattenedIndex(i, j); // j + i * (num_vertical_pix_ + 1);
#ifdef USE_MPI
			if ((n) % comm_size_ == comm_rank_) {
#endif
				float id = (float)i;
				float jd = (float)j;
				if (use_distorted_) {
					glm::vec2 meters((i - half_horizontal_dim_) * horizontal_mag_scale_,
						(j - half_vertical_dim_) * vertical_mag_scale_);
					glm::vec2 pu = dm_.Distort(meters);
					id = pu.x;
					jd = pu.y;
				}

				glm::vec3 direction = look_to_f
					+ (id - half_horizontal_dim_) * look_side_d
					+ (jd - half_vertical_dim_) * look_up_d;

				direction = glm::normalize(direction);
				RgbPixel pix = RenderPixel(env, direction);
				image_buffer_expanded[n] = pix.color;
				range_buffer_expanded[n] = pix.distance;
				segment_buffer_expanded[n] = pix.object_id;
				label_buffer_expanded[n] = pix.label;
#ifdef USE_MPI
			}
#endif
		}
	}

#ifdef USE_MPI
	MPI_Allreduce(MPI_IN_PLACE, &image_buffer_expanded[0], exp_size * 3,
		MPI_FLOAT, MPI_SUM, comm_);
	MPI_Allreduce(MPI_IN_PLACE, &range_buffer_expanded[0], exp_size,
		MPI_FLOAT, MPI_SUM, comm_);
#endif

#ifdef USE_OMP
#if defined(_WIN32) || defined(WIN32)
#pragma omp parallel for schedule(dynamic)
#else
#pragma omp parallel for schedule(dynamic) collapse(2)
#endif
#endif
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j); // j + i * (num_vertical_pix_);
#ifdef USE_MPI
			if ((n) % comm_size_ == comm_rank_) {
#endif
				int n1 = j + i * (num_vertical_pix_ + 1);
				int n2 = (j + 1) + i * (num_vertical_pix_ + 1);
				int n3 = j + (i + 1) * (num_vertical_pix_ + 1);
				int n4 = (j + 1) + (i + 1) * (num_vertical_pix_ + 1);
				//calculate the average
				glm::vec3 col = 0.25f * (image_buffer_expanded[n1] +
					image_buffer_expanded[n2] + image_buffer_expanded[n3] +
					image_buffer_expanded[n4]);
				float dist = 0.25f * (range_buffer_expanded[n1] +
					range_buffer_expanded[n2] + range_buffer_expanded[n3] +
					range_buffer_expanded[n4]);
				glm::vec3 v1 = col - image_buffer_expanded[n1];
				glm::vec3 v2 = col - image_buffer_expanded[n2];
				glm::vec3 v3 = col - image_buffer_expanded[n3];
				glm::vec3 v4 = col - image_buffer_expanded[n4];
				float variance = 0.25f * (glm::dot(v1, v1) + glm::dot(v2, v2) + glm::dot(v3, v3) + glm::dot(v4, v4));
				if (variance > 1.0) {

					// if the pixel has high variance, cast four more rays down the middle.
					float id = (float)i;
					float jd = (float)j;
					std::vector<glm::vec2> raydirs;
					raydirs.resize(5);
					if (use_distorted_) {
						glm::vec2 m1((id - half_horizontal_dim_ + 0.25f) * horizontal_mag_scale_,
							(jd - half_vertical_dim_ + 0.25f) * vertical_mag_scale_);
						raydirs[0] = dm_.Distort(m1);
						glm::vec2 m2((id - half_horizontal_dim_ + 0.25f) * horizontal_mag_scale_,
							(jd - half_vertical_dim_ + 0.75f) * vertical_mag_scale_);
						raydirs[1] = dm_.Distort(m2);
						glm::vec2 m3((id - half_horizontal_dim_ + 0.75f) * horizontal_mag_scale_,
							(jd - half_vertical_dim_ + 0.25f) * vertical_mag_scale_);
						raydirs[2] = dm_.Distort(m3);
						glm::vec2 m4((id - half_horizontal_dim_ + 0.75f) * horizontal_mag_scale_,
							(jd - half_vertical_dim_ + 0.75f) * vertical_mag_scale_);
						raydirs[3] = dm_.Distort(m4);
						glm::vec2 m5((id - half_horizontal_dim_ + 0.5f) * horizontal_mag_scale_,
							(jd - half_vertical_dim_ + 0.5f) * vertical_mag_scale_);
						raydirs[4] = dm_.Distort(m5);
					}

					//glm::vec4 pix(0.0f, 0.0f, 0.0f, 0.0f);
					RgbPixel pix;
					for (int pn = 0; pn < (int)raydirs.size(); pn++) {
						glm::vec3 direction = look_to_f
							+ (raydirs[pn].x - half_horizontal_dim_) * look_side_d
							+ (raydirs[pn].y - half_vertical_dim_) * look_up_d;
						RgbPixel this_pix = RenderPixel(env, direction);
						normal_buffer_[n] = this_pix.normal;
						pix.color += this_pix.color;
						pix.distance += this_pix.distance;
						pix.object_id = this_pix.object_id;
						//pix = pix + RenderPixel(env, direction);
					}
					//glm::vec3 newcol(pix.x, pix.y, pix.z);
					//col = 0.5f*col + 0.1f*newcol;
					col = 0.5f * col + 0.1f * pix.color;
					//dist = 0.5f*dist + 0.1f*pix.w;
					dist = 0.5f * dist + 0.1f * pix.distance;
				}
				//image_buffer_[n] = col;
				for (int c = 0; c < 3; c++) image_(i, j, c) = col[c];
				range_buffer_[n] = dist;
				segment_buffer_[n] = segment_buffer_expanded[n1];
				label_buffer_[n] = label_buffer_expanded[n1];
#ifdef USE_MPI
			}
#endif
		}
	}
}

void RgbCamera::RenderLoopAdaptive(environment::Environment* env) {
	//define directions
	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;

	int bf = num_horizontal_pix_ * num_vertical_pix_;
	std::vector<glm::vec3> tempcol;
	std::vector<float> tempdist;
	tempcol.resize(bf, zero_vec3);
	tempdist.resize(bf, 0.0f);
	//note that looping over i-j is faster than manually
	// unrolling the loop
#ifdef USE_OMP  
#if defined(_WIN32) || defined(WIN32)
#pragma omp parallel for schedule(dynamic)
#else
#pragma omp parallel for schedule(dynamic) collapse(2)
#endif
#endif  
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j); //j + i * num_vertical_pix_;
#ifdef USE_MPI
			if ((n) % comm_size_ == comm_rank_) {
#endif
				float id = (float)i;
				float jd = (float)j;
				if (use_distorted_) {
					glm::vec2 meters((i - half_horizontal_dim_ + 0.5f) * horizontal_mag_scale_,
						(j - half_vertical_dim_ + 0.5f) * vertical_mag_scale_);
					glm::vec2 pu = dm_.Distort(meters);
					id = pu.x;
					jd = pu.y;
				}
				glm::vec3 direction = look_to_f
					+ (id - half_horizontal_dim_ + 0.5f) * look_side_d
					+ (jd - half_vertical_dim_ + 0.5f) * look_up_d;
				direction = glm::normalize(direction);
				//glm::vec4 pix = RenderPixel(env, direction);
				//tempcol[n] = glm::vec3(pix.x, pix.y, pix.z);
				//tempdist[n] = pix.w;
				RgbPixel pix = RenderPixel(env, direction);
				tempcol[n] = pix.color;
				if (pix.object_id >= 0 || pix.object_id == -99) {
					tempdist[n] = pix.distance;
					normal_buffer_[n] = pix.normal;
				}
				segment_buffer_[n] = pix.object_id;
				label_buffer_[n] = pix.label;
#ifdef USE_MPI
			}
#endif
		}
	}

#ifdef USE_MPI  
	MPI_Allreduce(MPI_IN_PLACE, image_.data(), bf * 3, MPI_FLOAT, MPI_MAX, comm_);
	MPI_Allreduce(MPI_IN_PLACE, &range_buffer_[0], bf,
		MPI_FLOAT, MPI_MAX, comm_);
#endif
	float sf = (float)pixel_sample_factor_;
	//2nd loop to do anti-aliasing
#ifdef USE_OMP  
#if defined(_WIN32) || defined(WIN32)
#pragma omp parallel for schedule(dynamic)
#else
#pragma omp parallel for schedule(dynamic) collapse(2)
#endif
#endif  
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j); // j + i * num_vertical_pix_;
			if (i > 0 && j > 0 && i < (num_horizontal_pix_ - 1) && j < (num_vertical_pix_ - 1)) {
#ifdef USE_MPI
				if ((n) % comm_size_ == comm_rank_) {
#endif
					//first calculate local variance, then decide wether to resample
					int n1 = GetFlattenedIndex(i, j); // j + i * (num_vertical_pix_);
					int n2 = GetFlattenedIndex(i, j + 1); // (j + 1) + i * (num_vertical_pix_);
					int n3 = GetFlattenedIndex(i + 1, j); // j + (i + 1) * (num_vertical_pix_);
					int n4 = GetFlattenedIndex(i + 1, j + 1);// (j + 1) + (i + 1) * (num_vertical_pix_);
					//calculate the average
					glm::vec3 col = 0.2f * (tempcol[n1] +
						tempcol[n2] + tempcol[n3] +
						tempcol[n4] + tempcol[n]);
					glm::vec3 v0 = col - tempcol[n];
					glm::vec3 v1 = col - tempcol[n1];
					glm::vec3 v2 = col - tempcol[n2];
					glm::vec3 v3 = col - tempcol[n3];
					glm::vec3 v4 = col - tempcol[n4];
					float variance = 0.2f * (glm::dot(v1, v1) + glm::dot(v2, v2) + glm::dot(v3, v3) + glm::dot(v4, v4));
					if (variance > 1.0) {
						float id = (float)i;
						float jd = (float)j;
						if (use_distorted_) {
							glm::vec2 meters((i - half_horizontal_dim_ + 0.5f) * horizontal_mag_scale_,
								(j - half_vertical_dim_ + 0.5f) * vertical_mag_scale_);
							glm::vec2 pu = dm_.Distort(meters);
							id = pu.x;
							jd = pu.y;
						}
						//glm::vec4 pix(tempcol[n].x, tempcol[n].y, tempcol[n].z, tempdist[n]);
						RgbPixel pix;
						pix.color = glm::vec3(tempcol[n].x, tempcol[n].y, tempcol[n].z);
						pix.distance = tempdist[n];
						float dist_sf = 1.0f;
						for (int k = 0; k < pixel_sample_factor_ - 1; k++) {
							glm::vec3 direction;
							direction = look_to_f
								+ (id - half_horizontal_dim_ + mavs::math::rand_in_range(0.1f, 0.9f)) * look_side_d
								+ (jd - half_vertical_dim_ + mavs::math::rand_in_range(0.1f, 0.9f)) * look_up_d;
							direction = glm::normalize(direction);
							//pix = pix + RenderPixel(env, direction);
							RgbPixel this_pix = RenderPixel(env, direction);
							pix.color += this_pix.color;
							if (this_pix.object_id >= 0) {
								pix.distance += this_pix.distance;
								dist_sf += 1.0f;
							}
						}
						pix.color = pix.color / sf;
						pix.distance = pix.distance / dist_sf;
						//image_buffer_[n] = pix.color; // glm::vec3(pix.x, pix.y, pix.z);
						for (int c = 0; c < 3; c++)image_(i, j, c) = pix.color[c];
						range_buffer_[n] = pix.distance; // pix.w;
					}
					else {
						//image_buffer_[n] = tempcol[n];
						for (int c = 0; c < 3; c++)image_(i, j, c) = tempcol[n][c];
						range_buffer_[n] = tempdist[n];
					}
			}
				else {
					//image_buffer_[n] = tempcol[n];
					for (int c = 0; c < 3; c++)image_(i, j, c) = tempcol[n][c];
					range_buffer_[n] = tempdist[n];
				}
#ifdef USE_MPI
			}
#endif
		}
	}
}

void RgbCamera::RenderLoopSimple(environment::Environment* env) {
	//define directions
	glm::vec3 look_to_f = focal_length_ * look_to_;
	glm::vec3 look_side_d = horizontal_pixdim_ * look_side_;
	glm::vec3 look_up_d = vertical_pixdim_ * look_up_;

	float i2 = half_horizontal_dim_ - 0.5f;
	float j2 = half_vertical_dim_ - 0.5f;
	//note that looping over i-j is faster than manually
	// unrolling the loop
/*#ifdef USE_OMP
#if defined(_WIN32) || defined(WIN32)
#pragma omp parallel for schedule(dynamic)
#else
#pragma omp parallel for schedule(dynamic) collapse(2)
#endif
#endif */
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic)
#endif
	for (int i = 0; i < num_horizontal_pix_; i++) {
		float ix = (i - i2) * horizontal_mag_scale_;
		float id = (float)i;
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = GetFlattenedIndex(i, j); // j + i * num_vertical_pix_;
#ifdef USE_MPI
			if ((n) % comm_size_ == comm_rank_) {
#endif

				float jd = (float)j;
				if (use_distorted_) {
					glm::vec2 meters(ix,
						(j - j2) * vertical_mag_scale_);
					glm::vec2 pu = dm_.Distort(meters);
					id = pu.x;
					jd = pu.y;
				}
				glm::vec3 direction = look_to_f
					+ (id - i2) * look_side_d
					+ (jd - j2) * look_up_d;
				direction = glm::normalize(direction);
				//glm::vec4 pix = RenderPixel(env, direction);
				RgbPixel pix = RenderPixel(env, direction);
				//image_buffer_[n] = pix.color; // glm::vec3(pix.x, pix.y, pix.z);
				for (int c = 0; c < 3; c++)image_(i, j, c) = pix.color[c];
				if (pix.object_id >= 0 || pix.object_id == -99) {
					range_buffer_[n] = pix.distance; // .w;
					normal_buffer_[n] = pix.normal;
				}
				segment_buffer_[n] = pix.object_id;
				label_buffer_[n] = pix.label;
#ifdef USE_MPI
			}
#endif
		}
	}
}

void RgbCamera::CheckLights(environment::Environment* env) {
	glm::vec3 max_v(horizontal_pixdim_ * num_horizontal_pix_, vertical_pixdim_ * num_vertical_pix_, focal_length_);
	max_v = max_v / glm::length(max_v);
	float max_theta = acosf(max_v.z);

	for (int i = 0; i < (int)env->GetNumLights(); i++) {
		glm::vec3 f_to_light = env->GetPointerToLight(i)->position - position_;
		float d_to_light = glm::length(f_to_light);
		f_to_light = f_to_light / d_to_light;
		if (do_fast_lights_) {
			float theta = acosf(glm::dot(look_to_, f_to_light));
			float dist = glm::length(f_to_light);

			if (theta < max_theta && d_to_light < 100.0f) {
				env->GetPointerToLight(i)->is_active = true;
			}
			else {
				env->GetPointerToLight(i)->is_active = false;
			}
		}
	}
}

void RgbCamera::RenderFrame(environment::Environment* env, double dt) {
	if (local_sim_time_ <= 0.0 || !env_set_)SetEnvironmentProperties(env);

	if (disp_is_free_)UpdatePoseKeyboard();

	ZeroBuffer();

	CheckLights(env);

	std::vector< cimg_library::CImg<float> > images;

	int nframes = 1;
	if (blur_on_) {
		float motion_dist = velocity_.length() * (float)dt;
		nframes = (int)ceilf((motion_dist / vertical_pixdim_) / 450.0f);
		images.resize(nframes, image_);
	}

	for (int nf = 0; nf < nframes; nf++) {
		position_ = position_ + (((float)nf / nframes) * (float)dt) * velocity_;
		
		if (anti_aliasing_type_ == "adaptive") {
			RenderLoopAdaptive(env);
		}
		/*
		else if (anti_aliasing_type_ == "corners") {
			RenderLoopCorners(env);
		}
		*/
		else if (anti_aliasing_type_ == "oversampled") {
			RenderLoopOversampled(env);
		}
		else {
			RenderLoopSimple(env);
		}

#ifdef USE_MPI  
		ReduceImageBuffer();
#endif

		RenderParticleSystems(env);

		if (env->IsRaining()) {
			CreateRainMask(env, dt);
		}

		if (env->IsSnowing()) {
			RenderSnow(env, dt);
		}

		if (env->IsFoggy()) {
			RenderFog(env);
		}
		if (blur_on_) images[nf] = image_;
	}

	if (blur_on_) {
		cimg_library::CImg<float> imsum;
		imsum.assign(num_horizontal_pix_, num_vertical_pix_, 1, 3, 0.0f);
		for (int nf = 0; nf < nframes; nf++) {
			imsum = imsum + images[nf];
		}
		image_ = imsum / 3.0;
	}

	ApplyElectronics(dt);
}
//update so that anti-alising is not done on every pixel, just those at edges
void RgbCamera::Update(environment::Environment* env, double dt) {
	CheckFreq(dt);

	RenderFrame(env, dt);

	AdjustSaturationAndTemperature(1.25f, 9500.0f);

	CopyBufferToImage();

	if (log_data_) {
		SaveImage(utils::ToString(local_sim_time_) + log_file_name_);
	}

	local_sim_time_ += local_time_step_;
	updated_ = true;
}

bool RgbCamera::WorldToPixel(glm::vec3 point_world, glm::ivec2& pixel) {
	// first put in the pixel reference frame. 
	glm::vec2 pixfloat;
	glm::vec3 v = point_world - position_;
	float z = glm::dot(v, look_to_);
	if (z <= 0.0f) {
		return false;
	}
	float x = glm::dot(v, look_side_);
	float y = glm::dot(v, look_up_);
	glm::vec2 p_meters_undistort(focal_length_ * x / z, focal_length_ * y / z);
	if (use_distorted_) {
		pixfloat = dm_.Distort(p_meters_undistort);
		pixfloat = glm::vec2(pixfloat.x / horizontal_pixdim_, pixfloat.y / vertical_pixdim_);
	}
	else {
		pixfloat = glm::vec2(p_meters_undistort.x / horizontal_pixdim_,
			p_meters_undistort.y / vertical_pixdim_);
	}
	pixfloat.x += half_horizontal_dim_;
	pixfloat.y += half_vertical_dim_;
	pixel = glm::ivec2((int)pixfloat.x, (int)pixfloat.y);

	return true;
}

} //namespace camera
} //namespace sensor
} //namespace mavs
