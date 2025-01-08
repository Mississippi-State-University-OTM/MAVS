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
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <sensors/lidar/lidar.h>
#include <interfaces/drive_px2.h>
#include <mavs_core/plotting/mavs_plotting.h>

#include <string>
#include <fstream>
#include <limits>
#include <sstream>
#include <iomanip>

#include <mavs_core/math/constants.h>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace sensor {
namespace lidar {

Lidar::Lidar() {
	Init(1);
	is_planar_ = false;
	vertical_readout_ = false;
	nsteps_ = 0;
	prefix_ = "./";
	alpha_rain_ = 0.0f;
	alpha_snow_ = 0.0f;
	rotation_rate_ = 10.0f;
	blanking_dist_ = 0.0f;
	display_width_ = 256;
	display_height_ = 256;
	wavelength_ = 0.905f;
	range_noise_meters_ = 0.0f;
	psys_errors_ = 0.0f;
	recharge_dt_ = 50.0E-6f;
	display_color_type_ = "color";
}

Lidar::Lidar(int mode) {
	Init(mode);
}

void Lidar::SetRangeNoise(float noise){
	range_noise_meters_ = noise;
	std::normal_distribution<float> dist(0.0f, range_noise_meters_);
	distribution_ = dist;
}

void Lidar::Init(int mode) {
	max_range_ = 100.0f; //meters
	min_range_ = 1.0f;
	cutoff_len_ = 1.0f;
	is_planar_ = true;
	glm::vec3 x(1.0f, 0.0f, 0.0f);
	beam_spot_points_.push_back(x);
	mode_ = mode; //strongest
	updated_ = false;
	local_sim_time_ = 0.0;
	divergence_ = 0.0;
	type_ = "lidar";
	first_display_ = true;
	image_filled_ = false;
	optical_depth_thresh_ = 0.7f;
	registration_complete_ = false;
}

void Lidar::Load(std::string input_file) {
	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	if (d.HasMember("Signal Cutoff")) {
		cutoff_len_ = d["Signal Cutoff"].GetFloat();
	}

	if (d.HasMember("Min Range")) {
		min_range_ = d["Min Range"].GetFloat();
	}

	if (d.HasMember("Max Range")) {
		max_range_ = d["Max Range"].GetFloat();
	}

	if (d.HasMember("Beam Properties")) {
		if (d["Beam Properties"].HasMember("Shape")) {
			std::string shape = d["Beam Properties"]["Shape"].GetString();
			float div_h = d["Beam Properties"]["Divergence"][0].GetFloat();
			float div_v = d["Beam Properties"]["Divergence"][1].GetFloat();
			if (shape == "ellipse") {
				if (d["Beam Properties"].HasMember("Divergence")) {
					SetBeamSpotEllipse(div_h, div_v);
				}
			}
			else if (shape == "circle") {
				//float div = d["Beam Properties"]["Divergence"].GetFloat();
				SetBeamSpotCircular(div_h);
			}
			else if (shape == "rectangle") {
				//float div_h = d["Beam Properties"]["Divergence"][0].GetFloat();
				//float div_v = d["Beam Properties"]["Divergence"][1].GetFloat();
				SetBeamSpotRectangular(div_h, div_v);
			}
		}
	}

	if (d.HasMember("Mode")) {
		mode_ = d["Mode"].GetInt();
	}
	if (d.HasMember("Scan Pattern")) {
		if (d["Scan Pattern"].HasMember("Horizontal Range") &&
			d["Scan Pattern"].HasMember("Horizontal Step")) {
			float h_min = d["Scan Pattern"]["Horizontal Range"][0].GetFloat();
			float h_max = d["Scan Pattern"]["Horizontal Range"][1].GetFloat();
			float h_step = d["Scan Pattern"]["Horizontal Step"].GetFloat();
			if (d["Scan Pattern"].HasMember("Vertical Range") &&
				d["Scan Pattern"].HasMember("Vertical Step")) {
				float v_min = d["Scan Pattern"]["Vertical Range"][0].GetFloat();
				float v_max = d["Scan Pattern"]["Vertical Range"][1].GetFloat();
				float v_step = d["Scan Pattern"]["Vertical Step"].GetFloat();
				SetScanPattern(h_min, h_max, h_step, v_min, v_max, v_step);
			}
			else {
				SetScanPattern(h_min, h_max, h_step);
			}
		}
	}
}

void Lidar::SetScanPattern(float hfov_low, float hfov_high, float hres) {
	SetScanPattern(hfov_low, hfov_high, hres, 0.0f, 0.0f, -1.0f);
}

void Lidar::SetScanPattern(float hfov_low, float hfov_high, float hres,
	float vfov_low, float vfov_high, float vres) {
	//inputs are in degrees
	int num_h_steps = 1;
	if (hres > 0.0) num_h_steps = (int)((hfov_high - hfov_low) / hres) + 1;
	int num_v_steps = 1;
	if (vres > 0.0) {
		num_v_steps = (int)(ceil((vfov_high - vfov_low) / vres)) + 1;
	}
	else {
		vfov_low = 0.0;
		vfov_high = 0.0;
	}

	SetScanSize(hfov_low, hfov_high, num_h_steps, vfov_low, vfov_high, num_v_steps);
}
void Lidar::SetScanSize(float hfov_low, float hfov_high, int num_h_steps,
	float vfov_low, float vfov_high, int num_v_steps) {

	scan_rotations_.resize(num_h_steps*num_v_steps);
	beam_properties_.resize(num_h_steps*num_v_steps);
	hfov_low = (float)kDegToRad*hfov_low;
	hfov_high = (float)kDegToRad*hfov_high;
	float hres = 0.0f;
	if (num_h_steps > 1)hres = (hfov_high - hfov_low) / (1.0f*num_h_steps - 1);
	vfov_low = (float)kDegToRad*vfov_low;
	vfov_high = (float)kDegToRad*vfov_high;
	float vres = 0.0f;
	if (num_v_steps > 1) {
		vres = (vfov_high - vfov_low) / (1.0f*num_v_steps - 1);
		is_planar_ = false;
	}
	else {
		angle_max_ = hfov_high;
		angle_min_ = hfov_low;
		angle_increment_ = hres;
	}

	//update some property variables
	pc_width_ = num_h_steps;
	pc_height_ = num_v_steps;
	vertical_fov_min_ = vfov_low;
	vertical_fov_max_ = vfov_high;
	vertical_res_ = vres;
	horizontal_res_ = hres;

	rendered_image_.assign(pc_width_, pc_height_, 1, 3, 0.0);

	int n = 0;
	if (vertical_readout_) {
		// assumes look_to = x, look_up = z. 
		for (int j = 0; j < num_v_steps; j++) {
			float omega = vfov_low + j * vres;
			for (int i = 0; i < num_h_steps; i++) {
				float alpha = hfov_low + i * hres;
				scan_rotations_[n][0][0] = cos(omega)*cos(alpha);
				scan_rotations_[n][0][1] = cos(omega)*sin(alpha);
				scan_rotations_[n][0][2] = sin(omega);
				scan_rotations_[n][1][0] = -sin(alpha);
				scan_rotations_[n][1][1] = cos(alpha);
				scan_rotations_[n][1][2] = 0.0f;
				scan_rotations_[n][2][0] = -cos(alpha)*sin(omega);
				scan_rotations_[n][2][1] = -sin(alpha)*sin(omega);
				scan_rotations_[n][2][2] = cos(omega);
				beam_properties_[n].azimuth = alpha;
				beam_properties_[n].zenith = omega;
				beam_properties_[n].blanked = false;
				n++;;
			}
		}
	}
	else {
		// assumes look_to = x, look_up = z. 
		for (int i = 0; i < num_h_steps; i++) {
			float alpha = hfov_low + i * hres;
			for (int j = 0; j < num_v_steps; j++) {
				float omega = vfov_low + j * vres;
				scan_rotations_[n][0][0] = cos(omega)*cos(alpha);
				scan_rotations_[n][0][1] = cos(omega)*sin(alpha);
				scan_rotations_[n][0][2] = sin(omega);
				scan_rotations_[n][1][0] = -sin(alpha);
				scan_rotations_[n][1][1] = cos(alpha);
				scan_rotations_[n][1][2] = 0.0f;
				scan_rotations_[n][2][0] = -cos(alpha)*sin(omega);
				scan_rotations_[n][2][1] = -sin(alpha)*sin(omega);
				scan_rotations_[n][2][2] = cos(omega);
				beam_properties_[n].azimuth = alpha;
				beam_properties_[n].zenith = omega;
				beam_properties_[n].blanked = false;
				n++;;
			}
		}
	}

	int npoints = num_h_steps * num_v_steps;
	num_points_per_scan_ = npoints;
	if (mode_ == 3) npoints = 2 * npoints;
	points_.resize(npoints);
	normals_.resize(npoints);
	point_colors_.resize(npoints);
	segment_points_.resize(npoints);
	point_labels_.resize(npoints);
	distances_.resize(npoints, 0.0f);
	intensities_.resize(npoints, 0.0f);
}

void Lidar::DisplayLidarCamera(environment::Environment *env) {
	int n = 0;
	int nx = 512;
	int ny = 128;
	if (render_disp_.width() <= 0) {
		//render_disp_.assign(nx, ny, name_.c_str());
		render_disp_.assign(nx, ny);
		render_disp_.set_title("%s", name_.c_str());
	}
	rendered_image_.assign(pc_width_, pc_height_, 1, 3, 0.0);
	glm::vec3 x(1.0f, 0.0f, 0.0f);
	for (int i = 0; i < pc_width_; i++) {
		for (int j = 0; j < pc_height_; j++) {
			glm::vec3 direction = ((orientation_*scan_rotations_[n])*x);
			direction = direction / glm::length(direction);
			raytracer::Intersection inter = env->GetClosestIntersection(position_, direction);
			if (inter.dist > 0.0f) {
				rendered_image_.draw_point(pc_width_-i-1, pc_height_ - j - 1, (float*)&inter.color);
			}
			n++;
		}
	}
	rendered_image_.resize(nx, ny, 1, 3, 5);
	rendered_image_.normalize(0, 255);
	render_disp_ = rendered_image_;
}

void Lidar::SetBeamSpotEllipse(float horiz_div, float vert_div) {
	divergence_ = std::max(horiz_div, vert_div);
	float rh = (float)(0.7*tan(0.5*horiz_div));
	float rh2 = 0.5f*rh;
	float rv = (float)(0.7*tan(0.5*vert_div));
	float rv2 = 0.5f*rv;
	glm::vec3 d1(1.0f, rh, 0.0f);
	d1 = glm::normalize(d1);
	glm::vec3 d2(1, rh2, rv2);
	d2 = glm::normalize(d2);
	glm::vec3 d3(1, 0, rv);
	d3 = glm::normalize(d3);
	glm::vec3 d4(1, -rh2, rv2);
	d4 = glm::normalize(d4);
	beam_spot_points_.push_back(d1);
	beam_spot_points_.push_back(d2);
	beam_spot_points_.push_back(d3);
	beam_spot_points_.push_back(d4);
	/*
	glm::vec3 d5(1, -rh, 0);
	d5 = glm::normalize(d5);
	glm::vec3 d6(1, -rh2, -rv2);
	d6 = glm::normalize(d6);
	glm::vec3 d7(1, 0, -rv);
	d7 = glm::normalize(d7);
	glm::vec3 d8(1, rh2, -rv2);
	d8 = glm::normalize(d8);
	beam_spot_points_.push_back(d5);
	beam_spot_points_.push_back(d6);
	beam_spot_points_.push_back(d7);
	beam_spot_points_.push_back(d8);
	*/
}

void Lidar::SetBeamSpotRectangular(float horiz_div, float vert_div) {
	divergence_ = std::max(horiz_div, vert_div);
	float rh = (float)(0.7*tan(0.5*horiz_div));
	float rv = (float)(0.7*tan(0.5*vert_div));
	glm::vec3 d1(1.0f, rh, 0.0f);
	d1 = glm::normalize(d1);
	glm::vec3 d2(1, rh, rv);
	d2 = glm::normalize(d2);
	glm::vec3 d3(1, 0, rv);
	d3 = glm::normalize(d3);
	glm::vec3 d4(1, -rh, rv);
	d4 = glm::normalize(d4);
	beam_spot_points_.push_back(d1);
	beam_spot_points_.push_back(d2);
	beam_spot_points_.push_back(d3);
	beam_spot_points_.push_back(d4);
	/*
	glm::vec3 d5(1, -rh, 0);
	d5 = glm::normalize(d5);
	glm::vec3 d6(1, -rh, -rv);
	d6 = glm::normalize(d6);
	glm::vec3 d7(1, 0, -rv);
	d7 = glm::normalize(d7);
	glm::vec3 d8(1, rh, -rv);
	d8 = glm::normalize(d8);
	beam_spot_points_.push_back(d5);
	beam_spot_points_.push_back(d6);
	beam_spot_points_.push_back(d7);
	beam_spot_points_.push_back(d8);
	*/
}

void Lidar::SetBeamSpotCircular(float div) {
	divergence_ = div;
	//do calculations at a distance of 1 meter
	float r = (float)(0.7*tan(0.5*div));
	float r2 = 0.5f*r;
	glm::vec3 d1(1, r, 0);
	d1 = glm::normalize(d1);
	glm::vec3 d2(1, r2, r2);
	d2 = glm::normalize(d2);
	glm::vec3 d3(1, 0, r);
	d3 = glm::normalize(d3);
	glm::vec3 d4(1, -r2, r2);
	beam_spot_points_.push_back(d1);
	beam_spot_points_.push_back(d2);
	beam_spot_points_.push_back(d3);
	beam_spot_points_.push_back(d4);
	/*
	d4 = glm::normalize(d4);
	glm::vec3 d5(1, -r, 0);
	d5 = glm::normalize(d5);
	glm::vec3 d6(1, -r2, -r2);
	d6 = glm::normalize(d6);
	glm::vec3 d7(1, 0, -r);
	d7 = glm::normalize(d7);
	glm::vec3 d8(1, r2, -r2);
	d8 = glm::normalize(d8);
	beam_spot_points_.push_back(d5);
	beam_spot_points_.push_back(d6);
	beam_spot_points_.push_back(d7);
	beam_spot_points_.push_back(d8);
	*/
}

void Lidar::ZeroData() {
	glm::vec3 zero(0.0f, 0.0f, 0.0f);
	std::fill(intensities_.begin(), intensities_.end(), 0.0f);
	std::fill(distances_.begin(), distances_.end(), 0.0f);
	std::fill(points_.begin(), points_.end(), zero);
	std::fill(normals_.begin(), normals_.end(), zero);
	std::fill(point_colors_.begin(), point_colors_.end(), zero);
	std::fill(segment_points_.begin(), segment_points_.end(), 0);
	std::fill(point_labels_.begin(), point_labels_.end(), "");
}

void Lidar::ProcessPulse(std::vector<float> &dist,
	std::vector<float> &intens,
	std::vector<glm::vec3> &norms,
	std::vector<int> &ids,
	std::vector<std::string> &labels,
	glm::vec3 &look_to,
	int i) {
	glm::vec3 x(1, 0, 0);
	look_to = scan_rotations_[i] * x;
	if (mode_ == 0) { //average
		float dsum = 0.0;
		float isum = 0.0;
		int n_closest = -1;
		float closest = max_range_;
		for (int n = 0; n < (int)dist.size(); n++) {
			if (dist[n] < closest) {
				closest = dist[n];
				n_closest = n;
			}
		}
		float nsum = 0.0;
		for (int n = 0; n < (int)dist.size(); n++) {
			if ((dist[n] - closest) < cutoff_len_) {
				dsum += dist[n];
				isum += intens[n];
				nsum += 1.0;
			}
		}
		distances_[i] = dsum / nsum;
		intensities_[i] = isum;
		points_[i] = look_to * distances_[i];
		normals_[i] = norms[n_closest];
		segment_points_[i] = ids[n_closest];
		point_labels_[i] = labels[n_closest];
	}
	if (mode_ == 1) { //strongest/closest
		float strongest = max_range_;
		int nkeep = -1;
		for (int n = 0; n < (int)dist.size(); n++) {
			if (dist[n] < strongest) {
				strongest = dist[n];
				nkeep = n;
			}
		}
		distances_[i] = dist[nkeep];
		intensities_[i] = intens[nkeep];
		segment_points_[i] = ids[nkeep];
		point_labels_[i] = labels[nkeep];
		points_[i] = look_to * distances_[i];
		normals_[i] = norms[nkeep];
	}
	else if (mode_ == 2) { //last
		float last = 0.0;
		int nkeep = -1;
		for (int n = 0; n < (int)dist.size(); n++) {
			if (dist[n] > last) {
				last = dist[n];
				nkeep = n;
			}
		}
		distances_[i] = dist[nkeep];
		intensities_[i] = intens[nkeep];
		segment_points_[i] = ids[nkeep];
		point_labels_[i] = labels[nkeep];
		points_[i] = look_to * distances_[i];
		normals_[i] = norms[nkeep];
	}
	else if (mode_ == 3) { //both
		float strongest = 0.0;
		float last = 0.0;
		int n_last = -1;
		int n_strong = -1;
		for (int n = 0; n < (int)dist.size(); n++) {
			if (intens[n] > strongest) {
				strongest = intens[n];
				n_strong = n;
			}
			if (dist[n] > last) {
				last = dist[n];
				n_last = n;
			}
		}

		if (fabs(dist[n_last] - dist[n_strong]) > cutoff_len_) { //keep one point
			distances_[i] = 0.5f*(dist[n_last] + dist[n_strong]);
			intensities_[i] = 0.5f*(intens[n_last] + intens[n_strong]);
			segment_points_[i] = ids[n_strong];
			point_labels_[i] = labels[n_strong];
			points_[i] = look_to * distances_[i];
			normals_[i] = norms[n_strong];
		}
		else {
			distances_[i] = dist[n_strong];
			intensities_[i] = intens[n_strong];
			points_[i] = look_to * dist[n_strong];
			segment_points_[i] = ids[n_strong];
			normals_[i] = norms[n_strong];
			point_labels_[i] = labels[n_strong];
			int j = i + num_points_per_scan_;
			distances_[j] = dist[n_last];
			intensities_[j] = intens[n_last];
			points_[j] = look_to * dist[n_last];
			normals_[j] = norms[n_last];
			segment_points_[j] = ids[n_last];
			point_labels_[j] = labels[n_last];
		}
	}
} // process pulse

void Lidar::SetHorizontalBlankingRangeDegrees(float minblank, float maxblank) {
	if (maxblank <= minblank) return;
	float thetamin = (float)mavs::kDegToRad*minblank;
	float thetamax = (float)mavs::kDegToRad*maxblank;
	for (int i = 0; i < (int)beam_properties_.size(); i++) {
		if (beam_properties_[i].azimuth >= thetamin && beam_properties_[i].azimuth <= thetamax) {
			beam_properties_[i].blanked = true;
		}
	}
}

void Lidar::Pulse(int i, environment::Environment *env) {
	if (beam_properties_[i].blanked)return;
	glm::mat3 rot_mat = orientation_ * scan_rotations_[i];
	std::vector<float> dist_accum;
	std::vector<float> intens_accum;
	std::vector<glm::vec3> norm_accum;
	std::vector<int> id_accum;
	std::vector<std::string> label_accum;
	glm::vec3 look_to;
	for (int n = 0; n < (int)beam_spot_points_.size(); n++) {
		glm::vec3 scan_dir = rot_mat * beam_spot_points_[n];
		if (n == 0)look_to = scan_dir;
		raytracer::Intersection inter =
			env->GetClosestIntersection(position_, scan_dir);
		if (inter.dist > min_range_ && inter.dist < max_range_ && inter.dist>blanking_dist_) {
			inter.normal = inter.normal / glm::length(inter.normal);
			float dp = fabs(glm::dot(scan_dir, inter.normal));
			if (dp < 0) {
				dp = -dp;
				inter.normal = -1.0f*inter.normal;
			}
			float intens = inter.color.x*dp;
			if (n==0)point_colors_[i] = 255.0f*inter.color;
			if (inter.spectrum_name.size() > 0) {
				float rho = env->GetScene()->GetReflectance(inter.spectrum_name, wavelength_);
				intens = dp*rho;
			}
			if ((intens / (inter.dist*inter.dist)) > noise_cutoff_) {
				intens_accum.push_back(intens);
				dist_accum.push_back(inter.dist);
				norm_accum.push_back(inter.normal);
				id_accum.push_back(inter.object_id);
				label_accum.push_back(inter.label);
			}
		}
		else if (n == 0) {
			// if the center ray doesn't hit anything, don't do the other pulses
			break;
		}
	}

	glm::vec3 this_dir = (scan_rotations_[i] * beam_spot_points_[0]);
	glm::vec3 global_dir = rot_mat*beam_spot_points_[0];
	if (!dist_accum.empty()) {
		//ProcessPulse(dist_accum, intens_accum, id_accum, label_accum, look_to, i);
		ProcessPulse(dist_accum, intens_accum, norm_accum, id_accum, label_accum, this_dir, i);
	}
	//glm::vec3 scan_dir = scan_rotations_[i] * beam_spot_points_[0];
	//glm::vec3 curr_dir = (scan_rotations_[i] * beam_spot_points_[0]);
	TraceParticleSystems(env, this_dir, global_dir, i);

	if (env->IsRaining()) {
		TraceRain(env, look_to, i);
	}

	if (env->IsSnowing()) {
		TraceSnow(env, look_to, i);
	}

	//add noise
	if (range_noise_meters_>0.0f){
		float delta = distribution_(generator_);
		distances_[i] += delta;
		points_[i]  = this_dir*distances_[i];
		psys_errors_ += fabs(delta);
	}

} // Pulse

int fa_accum = 0;

void Lidar::TraceSnow(environment::Environment *env, glm::vec3 direction, int i) {
	if (distances_[i] > cutoff_len_) {
		float L0 = distances_[i];
		float P = intensities_[i] * exp(-2.0f*alpha_snow_*L0) / (L0*L0);
		if (P < noise_cutoff_) {
			distances_[i] = 0.0f;
			intensities_[i] = 0.0f;
			points_[i] = glm::vec3(0.0f, 0.0f, 0.0f);
			normals_[i] = glm::vec3(0.0f, 0.0f, 1.0f);
			psys_errors_ += L0;
		}
		else {
			float old_dist = distances_[i];
			//Check for false alarms according to rate measured by rasshofer 2011
			//rasshofer doesn't list a relationship to snowfall rate, so taking a guess
			float prob_of_false = (float)(1.0-exp(-0.000001*env->GetSnowRate()/(L0*L0)));
			float testval = mavs::math::rand_in_range(0.0f, 1.0f);
			if (prob_of_false > testval) {
				fa_accum++;
				distances_[i] = (prob_of_false - testval)*10.0f;
				points_[i] = points_[i] * (distances_[i] / old_dist);
				//normals_[i] = -direction;
				psys_errors_ += fabs(distances_[i] - old_dist);
				intensities_[i] = 0.8f;
			}
			else {
				distances_[i] = distances_[i] + mavs::math::rand_in_range(-0.015f, 0.015f);
				points_[i] = points_[i] * (distances_[i] / old_dist);
				//normals_[i] = -direction;
				psys_errors_ += fabs(distances_[i] - old_dist);
				intensities_[i] = P;
			}
		}
	}
}

void Lidar::TraceRain(environment::Environment *env, glm::vec3 direction, int i) {
	if (distances_[i] > 0.0) {
		float L0 = distances_[i];
		float P = intensities_[i] * exp(-2.0f*alpha_rain_*L0) / (L0*L0);
		if (P < noise_cutoff_) {
			distances_[i] = 0.0f;
			intensities_[i] = 0.0f;
			points_[i] = glm::vec3(0.0f, 0.0f, 0.0f);
			//normals_[i] = -direction;
			psys_errors_ += L0;
		}
		else {
			float old_dist = distances_[i];
			distances_[i] = distances_[i] + mavs::math::rand_in_range(-0.015f, 0.015f);
			points_[i] = points_[i]*(distances_[i]/old_dist);
			//normals_[i] = -direction;
			psys_errors_ += fabs(distances_[i]-old_dist);
			intensities_[i] = P;
		}
	}
}

void Lidar::TraceParticleSystems(environment::Environment *env,
	glm::vec3 direction, glm::vec3 global_direction, int i) {
	//Monte-carlo calculation based on optical depth, see
	// "Probabilistic Model for Simulating the Effect of Airborne
	// Dust on Ground-Based LIDAR", Goodin et al.

	//check for intersections with particle system
	float transparency = 1.0;
	float d_close = std::numeric_limits<float>::max();
	environment::ParticleSystem *systems = env->GetParticleSystems();
	float rho = 0.0;
	for (int ps = 0; ps < (int)env->GetNumParticleSystems(); ps++) {
		//if (systems[ps].RayIntersectsSystem(position_, direction)) {
			environment::Particle *particles = systems[ps].GetParticles();
			for (int p = 0; p < (int)systems[ps].GetNumParticles(); p++) {
				glm::vec2 d = particles[p].GetIntersection(position_, global_direction);
				if (d.y > 0.0 && (d.y < distances_[i] || distances_[i] <= 0.0)) {
					if (d.x<1.0 && d.x>0.0) {
						float s = 1.0f - d.x;
						float t = 1.0f - s * (1.0f - particles[p].transparency_);
						transparency *= t;
						rho = d.x;
						if (d.y < d_close)d_close = d.y;
					}
				}
			}
		//}
	}
	float optical_depth = -log(transparency);
	bool returned_from_ps = false;
	if (optical_depth > optical_depth_thresh_) {
		returned_from_ps = true;
	}
	else {
		float thresh = optical_depth / optical_depth_thresh_;
		float test_point = mavs::math::rand_in_range(0.0f, 1.0f);
		if (thresh > test_point) returned_from_ps = true;
	}
	if (returned_from_ps) {
		psys_errors_ += fabs(distances_[i]-d_close);
		distances_[i] = d_close;
		points_[i] = direction * distances_[i];
		normals_[i] = -direction;
		intensities_[i] = (1.0f-transparency)*rho;
		point_labels_[i] = "dust";
	}
} // TraceParticleSystems

void Lidar::Update(environment::Environment *env, double dt) {
	local_time_step_ = dt;
	recharge_dt_ = (float)dt / (1.0f*pc_width_);
	if (local_sim_time_ == 0.0) {
		//noise_cutoff_ = 0.001/(kPi*max_range_*max_range_);
		noise_cutoff_ = (float)(0.9f / (kPi*max_range_*max_range_));
	}
	if (env->IsRaining()) {
		//alpha_rain_ = 0.003*env->GetRainRate();
		//alpha_rain_ = (float)(0.009f*pow(env->GetRainRate(),0.37))
		alpha_rain_ = (float)(0.01f*pow(env->GetRainRate(), 0.6));
	}
	if (env->IsSnowing()) {
		alpha_snow_ = env->GetSnow()->GetAlpha();//-2.0E-4f*env->GetSnowRate();
	}
	image_filled_ = false;
	registration_complete_ = false;
	ZeroData();
	psys_errors_ = 0.0f;
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < (int)scan_rotations_.size(); i++) {
		if (i%pc_height_ == 0) {
			position_ = position_ + recharge_dt_ * velocity_;
		}
#ifdef USE_MPI    
		if (i%comm_size_ == comm_rank_) {
#endif
			Pulse(i, env);
#ifdef USE_MPI      
		}
#endif
	}
#ifdef USE_MPI
	ReduceData();
#endif  
	if (log_data_ && comm_rank_ == 0) {
		std::ostringstream ss;
		ss << std::setw(5) << std::setfill('0') << nsteps_;
		std::string num_string(ss.str());
		WritePointsToText(log_file_name_ + num_string + ".txt");
		WritePointsToImage(log_file_name_ + num_string + ".bmp");
	}

	//register points to world coordinates
	/*
	if (registered_points_.size() != points_.size()) {
		registered_points_.resize(points_.size());
	}
	for (int i = 0; i < (int)points_.size(); i++) {
		registered_points_[i] = position_ + points_[i].x*look_to_ +
			points_[i].y*look_side_ + points_[i].z*look_up_;
	}
*/
	local_sim_time_ += local_time_step_;
	nsteps_++;
	updated_ = true;
}

void Lidar::RegisterPoints(){
	if (!registration_complete_){
		//register points to world coordinates
		if (registered_points_.size() != points_.size()) {
			registered_points_.resize(points_.size());
		}
		for (int i = 0; i < (int)points_.size(); i++) {
			registered_points_[i] = position_ + points_[i].x*look_to_ +
				points_[i].y*look_side_ + points_[i].z*look_up_;
		}
		registration_complete_ = true;
	}
}

#ifdef USE_MPI
void Lidar::ReduceData() {
	if (comm_size_ > 1) {
		int buff_size = (int)intensities_.size();
		MPI_Allreduce(MPI_IN_PLACE, &intensities_[0], buff_size, MPI_FLOAT, MPI_SUM,
			comm_);
		MPI_Allreduce(MPI_IN_PLACE, &distances_[0], buff_size, MPI_FLOAT, MPI_SUM,
			comm_);
		MPI_Allreduce(MPI_IN_PLACE, &points_[0], 3 * buff_size, MPI_FLOAT, MPI_SUM, comm_);
	}
}
#endif
#ifdef USE_MPI
void Lidar::PublishData(int root, MPI_Comm broadcast_to) {
	MPI_Bcast(&updated_, 1, MPI_LOGICAL, root, broadcast_to);
	int bsize = (int)intensities_.size();
	int psize = (int)registered_points_.size();
	MPI_Bcast(&bsize, 1, MPI_INT, root, broadcast_to);
	MPI_Bcast(&psize, 1, MPI_INT, root, broadcast_to);
	if (intensities_.size() != bsize) {
		intensities_.resize(bsize);
		distances_.resize(bsize);
		points_.resize(bsize);
	}
	if (registered_points_.size() != psize) {
		registered_points_.resize(psize);
	}
	if (updated_) {
		MPI_Bcast(&intensities_[0], bsize, MPI_FLOAT, root, broadcast_to);
		MPI_Bcast(&distances_[0], bsize, MPI_FLOAT, root, broadcast_to);
		MPI_Bcast(&points_[0], 3 * bsize, MPI_FLOAT, root, broadcast_to);
		MPI_Bcast(&registered_points_[0], 3 * psize, MPI_FLOAT, root, broadcast_to);
		MPI_Bcast(&is_planar_, 1, MPI_C_BOOL, root, broadcast_to);
		updated_ = false;
	}
}
#endif

void Lidar::WritePointsToText(std::string fname) {
	RegisterPoints();
	std::ofstream fout;
	fout.open(fname.c_str());
	fout << "x y z i" << std::endl;
	for (int i = 0; i < (int)points_.size(); i++) {
		fout << points_[i].x << " " << points_[i].y << " " << points_[i].z << " " << intensities_[i] << std::endl;
	}
	fout.close();
}

void Lidar::WriteRegisteredPointsToText(std::string fname) {
	RegisterPoints();
	std::ofstream fout;
	fout.open(fname.c_str());
	fout << "x y z i" << std::endl;
	for (int i = 0; i < (int)registered_points_.size(); i++) {
		if (intensities_[i] > 0.0f) {
			fout << registered_points_[i].x << " " << registered_points_[i].y << " " <<
				registered_points_[i].z << " " <<
				intensities_[i] << std::endl;
		}
	}
	fout.close();
}

void Lidar::WriteColorizedRegisteredPointsToText(std::string fname) {
	RegisterPoints();
	std::ofstream fout;
	fout.open(fname.c_str());
	fout << "x y z r g b" << std::endl;
	for (int i = 0; i < (int)registered_points_.size(); i++) {
		if (intensities_[i] > 0.0f) {
			fout << registered_points_[i].x << " " << registered_points_[i].y << " " <<
				registered_points_[i].z << " " <<
				point_colors_[i].x << " " << point_colors_[i].y << " " << point_colors_[i].z << std::endl;
		}
	}
	fout.close();
}

void Lidar::WriteLabeledRegisteredPointsToText(std::string fname) {
	RegisterPoints();
	std::ofstream fout;
	fout.open(fname.c_str());
	fout << "x y z i label" << std::endl;
	for (int i = 0; i < (int)registered_points_.size(); i++) {
		if (intensities_[i] > 0.0f) {
			fout << registered_points_[i].x << " " << registered_points_[i].y << " " <<
				registered_points_[i].z << " " <<
				intensities_[i] <<" "<< segment_points_[i]<<std::endl;
		}
	}
	fout.close();
}

void Lidar::WriteUnregisteredPointsToText(std::string fname) {
	std::ofstream fout;
	fout.open(fname.c_str());
	fout << "x y z i" << std::endl;
	for (int i = 0; i < (int)points_.size(); i++) {
		fout << points_[i].x << " " << points_[i].y << " " <<
			points_[i].z << " " << intensities_[i] << std::endl;
	}
	fout.close();
}

void Lidar::AnnotateFrame(environment::Environment *env, bool semantic) {
	RegisterPoints();
	for (int i = 0; i < (int)registered_points_.size(); i++) {
		if (intensities_[i] > 0.0) {
			int obj_num = segment_points_[i];
			std::string mesh_name = env->GetObjectName(obj_num);
			std::string label_name = point_labels_[i]; // env->GetLabel(mesh_name);
			int lab_num = env->GetLabelNum(label_name);
			if (semantic) {
				if (annotations_.count(label_name) == 0) { // new annotation
					annotations_[label_name] = Annotation(label_name, registered_points_[i], lab_num);
				}
				point_colors_[i] = 255.0f*env->GetLabelColor(label_name);
			}
			else {
				if (object_annotations_.count(obj_num) == 0) {
					object_annotations_[obj_num] = Annotation(mesh_name, registered_points_[i], obj_num);
					//annotations_[label_name] = Annotation(mesh_name, registered_points_[i], obj_num);
					glm::quat q_obj = env->GetObjectOrientation(obj_num);
					annotations_[label_name].SetOrientation(q_obj);
					raytracer::BoundingBox box = env->GetObjectBoundingBox(obj_num);
					annotations_[label_name].SetLLCorner(box.GetLowerLeft());
					annotations_[label_name].SetURCorner(box.GetUpperRight());
				}
				else { //existing annotation
					glm::vec3 p = registered_points_[i];
					glm::vec3 ur = object_annotations_[obj_num].GetURCorner();
					glm::vec3 ll = object_annotations_[obj_num].GetLLCorner();
					if (p.x < ll.x) { object_annotations_[obj_num].SetLLCorner(p.x, ll.y, ll.z); }
					if (p.x > ur.x) { object_annotations_[obj_num].SetURCorner(p.x, ur.y, ur.z); }
					if (p.y < ll.y) { object_annotations_[obj_num].SetLLCorner(ll.x, p.y, ll.z); }
					if (p.y > ur.y) { object_annotations_[obj_num].SetURCorner(ur.x, p.y, ur.z); }
					if (p.z < ll.z) { object_annotations_[obj_num].SetLLCorner(ll.x, ll.y, p.z); }
					if (p.z > ur.z) { object_annotations_[obj_num].SetURCorner(ur.x, ur.y, p.z); }
				}
				point_colors_[i] = anno_colors_.GetColor(obj_num);
			} // if semantic
			segment_points_[i] = lab_num;
		} // if intensities > 0
	} // loop over points
}

void Lidar::SaveAnnotation() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << nsteps_;
	std::string num_string(ss.str());
	std::string outname = prefix_ + type_ + name_ + num_string + "_annotated";
	SaveAnnotation(outname);
}

void Lidar::SaveAnnotation(std::string fname) {
	std::string imout = fname + ".txt";
	std::string csvout = fname + ".csv";
	WriteSegmentedPointsToText(imout);
	SaveSemanticAnnotationsCsv(csvout);
}

void Lidar::SaveSemanticAnnotationsCsv(std::string fname) {
	// Handle frame annotations
	std::ofstream annotationFile;
	annotationFile.open(fname.c_str());
	//annotationFile << "Name, Number, LL.x, LL.y, LL.z, UR.x, UR.y, UR.z" << std::endl;
	annotationFile << "Name, Number, length, width, height, tx, ty, tz, qw, qx, qy, qz" << std::endl;
	//std::map<int, Annotation>::iterator iter;
	std::map<std::string, Annotation>::iterator iter;
	iter = annotations_.begin();
	int i = 0;
	for (iter = annotations_.begin(); iter != annotations_.end(); ++iter) {
		glm::vec3 ll = iter->second.GetLLCorner();
		glm::vec3 ur = iter->second.GetURCorner();
		glm::vec3 dim = ur - ll;
		glm::vec3 pos = 0.5f*dim + ll;
		glm::quat q = iter->second.GetOrientation();
		annotationFile << iter->second.GetName() << ", " << iter->second.GetClassNum() << ", " <<
			dim.x << ", " << dim.y << ", " << dim.z << ", " <<
			pos.x << ", " << pos.y << ", " << pos.z << ", " <<
			q.w << ", " << q.x << ", " << q.y << ", " << q.z << std::endl;
		i++;
	}
	annotationFile.close();
}

void Lidar::WriteSegmentedPointsToText(std::string fname) {
	RegisterPoints();
	std::ofstream fout;
	fout.open(fname.c_str());
	fout << "x y z intensity object" << std::endl;
	for (int i = 0; i < (int)registered_points_.size(); i++) {
		if (intensities_[i] > 0.0f) {
			int intens = (int)(100.0f * intensities_[i]);
			fout << registered_points_[i].x << " " << registered_points_[i].y << " " <<
				registered_points_[i].z << " " << intens << " " <<
				segment_points_[i] << std::endl;
		}
	}
	fout.close();
}

void Lidar::WritePcdWithLabels(std::string fname) {
	int np = 0;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			np++;
		}
	}
	std::ofstream fout(fname.c_str());
	fout << "VERSION 0.7" << std::endl;
	fout << "FIELDS x y z intensity label" << std::endl;
	fout << "SIZE 4 4 4 4 4" << std::endl;
	fout << "TYPE F F F F F" << std::endl;
	fout << "COUNT 1 1 1 1 1" << std::endl;
	fout << "WIDTH " << np << std::endl;
	fout << "HEIGHT " << 1 << std::endl;
	glm::quat q = quat_cast(orientation_);
	fout << "VIEWPOINT " << position_.x << " " << position_.y << " " << position_.z << " " <<
		q.w << " " << q.x << " " << q.y << " " << q.z << std::endl;
	fout << "POINTS " << np << std::endl;
	fout << "DATA ascii" << std::endl;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			fout << points_[i].x << " " << points_[i].y << " " <<
				points_[i].z << " " << intensities_[i] << " "<< segment_points_[i]<<std::endl;
		}
	}
	fout.close();
}

void Lidar::WritePcdWithNormalsAndLabels(std::string fname) {
	int np = 0;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			np++;
		}
	}
	std::ofstream fout(fname.c_str());
	fout << "VERSION 0.7" << std::endl;
	fout << "FIELDS x y z intensity normal_x normal_y normal_z label curvature" << std::endl;
	fout << "SIZE 4 4 4 4 4 4 4 4 4" << std::endl;
	fout << "TYPE F F F F F F F F F" << std::endl;
	fout << "COUNT 1 1 1 1 1 1 1 1 1" << std::endl;
	fout << "WIDTH " << np << std::endl;
	fout << "HEIGHT " << 1 << std::endl;
	glm::quat q = quat_cast(orientation_);
	fout << "VIEWPOINT " << position_.x << " " << position_.y << " " << position_.z << " " <<
		q.w << " " << q.x << " " << q.y << " " << q.z <<std::endl;
	fout << "POINTS " << np << std::endl;
	fout << "DATA ascii" << std::endl;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			fout << points_[i].x << " " << points_[i].y << " " <<
				points_[i].z << " " << intensities_[i] << " " << normals_[i].x<<" "<<normals_[i].y<<" "<<normals_[i].z<<" "<<segment_points_[i] << " "<<0.0<<std::endl;
		}
	}
	fout.close();
}

void Lidar::WritePcd(std::string fname) {
	int np = 0;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			np++;
		}
	}
	std::ofstream fout(fname.c_str());
	fout << "VERSION 0.7" << std::endl;
	fout << "FIELDS x y z intensity" << std::endl;
	fout << "SIZE 4 4 4 4" << std::endl;
	fout << "TYPE F F F F" << std::endl;
	fout << "COUNT 1 1 1 1" << std::endl;
	fout << "WIDTH " << np << std::endl;
	fout << "HEIGHT " << 1 << std::endl;
	glm::quat q = quat_cast(orientation_);
	fout << "VIEWPOINT " << position_.x << " " << position_.y << " " << position_.z << " " <<
		q.w << " " << q.x << " " << q.y << " " << q.z << std::endl;
	fout << "POINTS " << np << std::endl;
	fout << "DATA ascii" << std::endl;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0) {
			fout << points_[i].x << " " << points_[i].y << " " <<
				points_[i].z << " " << intensities_[i] << std::endl;
		}
	}
	fout.close();
}

void Lidar::SaveRaw() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << nsteps_;
	std::string num_string(ss.str());

	std::string base_name = prefix_ + type_ + name_ + num_string;

	//std::string outname = base_name + ".txt";
	//WritePointsToText(outname);
	std::string pcdname = base_name + ".pcd";
	WritePcd(pcdname);
	//WritePointsToImage(base_name + ".bmp");
}

std::vector<glm::vec3> Lidar::GetRegisteredPoints() {
	RegisterPoints();
	return registered_points_;
}

std::vector<glm::vec4> Lidar::GetRegisteredPointsXYZI() {
	RegisterPoints();
	std::vector<glm::vec4> xyzi;
	xyzi.resize(registered_points_.size());
	for (int i = 0; i < (int)registered_points_.size(); i++) {
		xyzi[i] = glm::vec4(registered_points_[i], intensities_[i]);
	}
	return xyzi;
}

void Lidar::WriteHaloOutput(std::string basename, bool append, float sim_time) {
	std::string pts_fname = basename + ".pts";
	std::ofstream ptsfile;
	if (append) {
		if (mavs::utils::file_exists(pts_fname)) {
			if (!append_file_opened_) {
				std::ofstream fout;
				fout.open(pts_fname.c_str(), std::ios::binary | std::ios::out);
				fout.close();
			}
		}
		ptsfile.open(pts_fname.c_str(), std::ios::binary | std::ios::out | std::ios::app);
	}
	else {
		ptsfile.open(pts_fname.c_str(), std::ios::binary | std::ios::out);
	}

	//header
	int np = (int)intensities_.size();
	
	nvidia::dwLidarProperties props;
	for (int i = 0; i < (int)name_.size(); i++) {
		props.deviceString[i] = name_[i];
	}
	props.deviceString[name_.size()] = '\0';
	props.pointsPerSpin = np;
	props.pointsPerSecond = props.pointsPerSpin*(int)(rotation_rate_);
	props.packetsPerSecond = props.pointsPerSecond / pc_height_;
	props.packetsPerSpin = props.packetsPerSecond / (int)(rotation_rate_);
	props.pointsPerPacket = pc_height_;
	props.pointStride = 4; //xyzi
	if (!append_file_opened_) {
		ptsfile.write((char *)&props, sizeof(nvidia::dwLidarProperties));
		append_file_opened_ = true;
	}
	int num_packets = np / props.pointsPerPacket;
	int duration = (int)((1.0 / rotation_rate_)*(num_packets / np)/1.0E9);
	int time_stamp = (int)(sim_time / 1.0E-6);
	for (int n = 0; n < num_packets; n++) {
		//write the packet info
		nvidia::dwLidarDecodedPacket packet;
		packet.duration = duration;
		packet.hostTimestamp = time_stamp + n*duration;
		packet.maxPoints = props.pointsPerPacket;
		packet.nPoints = props.pointsPerPacket;
		packet.sensorTimestamp = time_stamp + n*duration;
		packet.stride = props.pointStride;
		ptsfile.write((char *)&packet, sizeof(nvidia::dwLidarDecodedPacket));
		//write the points in the packet
		nvidia::dwLidarPointXYZI *point_data = new nvidia::dwLidarPointXYZI[props.pointsPerPacket];
		for (int p = 0; p < (int)props.pointsPerPacket; p++) {
			int pn = n * props.pointsPerPacket + p;
			point_data[p].intensity = intensities_[pn];
			point_data[p].x = points_[pn].x;
			point_data[p].y = points_[pn].y;
			point_data[p].z = points_[pn].z;
		}
		//packet.points = point_data;
		ptsfile.write((char *)point_data, props.pointsPerPacket*sizeof(nvidia::dwLidarPointXYZI));
		delete[] point_data;
	}

	ptsfile.close();
}

void Lidar::WriteHaloOutputMultiple(std::string basename) {
	
	size_t n_dp = 4 * intensities_.size();
	
	float *point_data = new float[n_dp];
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0.0f) {
			point_data[i * 4] = points_[i].x;
			point_data[i * 4 + 1] = points_[i].y;
			point_data[i * 4 + 2] = points_[i].z;
			point_data[i * 4 + 3] = (intensities_[i] / 100.0f);
		}
		else {
			point_data[i * 4] = 0.0f;
			point_data[i * 4 + 1] = 0.0f;
			point_data[i * 4 + 2] = 0.0f;
			point_data[i * 4 + 3] = 0.0f;
		}
	}
	std::string pts_fname = basename + "_data.pts";
	std::ofstream ptsfile;
	ptsfile.open(pts_fname.c_str(), std::ios::binary | std::ios::out);
	ptsfile.write((char *)point_data, n_dp*sizeof(float));
	ptsfile.close();
	delete[] point_data;
	
	std::string props_name = basename + "_props.txt";
	std::ofstream propsfile(props_name.c_str());
	propsfile << "deviceString packetsPerSecond packetsPerSpin pointsPerPacket pointsPerSecond pointsPerSpin pointStride spinFrequency" << std::endl;
	int points_per_spin = (int)points_.size();
	int spins_per_second = (int)(rotation_rate_);
	int point_stride = 4;
	int points_per_second = points_per_spin * spins_per_second;
	int points_per_packet = pc_height_;
	int packets_per_second = points_per_second / points_per_packet;
	int packets_per_spin = packets_per_second / spins_per_second;
	propsfile << name_ << " " << packets_per_second << " " << packets_per_spin << " " << points_per_packet << " " << points_per_second << " " << points_per_spin << " " << point_stride << " " << spins_per_second << std::endl;
	propsfile.close();

	std::string info_name = basename + "_packet_info.txt";
	std::ofstream infofile(info_name.c_str());
	for (int i = 0; i < packets_per_spin; i++) {
		infofile << "0 0 " << points_per_packet << " " << points_per_packet << " 0 4" << std::endl;
	}
	infofile.close();
}

/// Get a vector of labeled xyzi points
std::vector<labeled_point> Lidar::GetColorizedPoints() {
	std::vector<labeled_point> points;
	for (int i = 0; i < (int)points_.size(); i++) {
		labeled_point p;
		p.x = points_[i].x;
		p.y = points_[i].y;
		p.z = points_[i].z;
		p.intensity = intensities_[i];
		p.label = -1;
		p.color = point_colors_[i];
		points.push_back(p);
	}
	return points;
}


/// Get a vector of labeled xyzi points
std::vector<labeled_point> Lidar::GetLabeledPoints() {
	std::vector<labeled_point> points;
	for (int i = 0; i < (int)points_.size(); i++) {
		//if (intensities_[i] > 0.0f) {
			labeled_point p;
			p.x = points_[i].x;
			p.y = points_[i].y;
			p.z = points_[i].z;
			p.intensity = intensities_[i];
			p.label = segment_points_[i];
			p.color = point_colors_[i];
			points.push_back(p);
		//}
	}
	return points;
}

/// Get a vector of labeled xyzi points registered to world coordinates
std::vector<labeled_point> Lidar::GetRegisteredLabeledPoints() {
	RegisterPoints();
	std::vector<labeled_point> points;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0.0f) {
			labeled_point p;
			p.x = registered_points_[i].x;
			p.y = registered_points_[i].y;
			p.z = registered_points_[i].z;
			p.intensity = intensities_[i];
			p.label = segment_points_[i];
			p.color = point_colors_[i];
			points.push_back(p);
		}
	}
	return points;
}

void Lidar::WritePointsToImage(std::string fname) {
	if (!image_filled_)FillImage();
	image_.normalize(0, 255);
	image_.save(fname.c_str());
}

glm::vec3 Lidar::GetPointDisplayColor(int i, float min_height, float max_height) {
	mavs::utils::Mplot plotter;
	glm::vec3 color(255.0f, 255.0f, 255.0f);
	if (display_color_type_ == "white") {
		color = glm::vec3(255.0f, 255.0f, 255.0f);
	}
	else if (display_color_type_ == "color") {
		color = point_colors_[i];
	}
	else if (display_color_type_ == "range") {
		mavs::utils::Color c = plotter.MorelandColormap(distances_[i], min_range_, max_range_);
		color = glm::vec3(c.r, c.g, c.b);
	}
	else if (display_color_type_ == "height") {
		if (registered_points_.size() != points_.size()) RegisterPoints();
		mavs::utils::Color c = plotter.MorelandColormap(registered_points_[i].z, min_height, max_height);
		color = glm::vec3(c.r, c.g, c.b);
	}
	else if (display_color_type_ == "intensity") {
		mavs::utils::Color c = plotter.MorelandColormap(intensities_[i], 0.0f, 1.0f);
		color = glm::vec3(c.r, c.g, c.b);
	}
	else if (display_color_type_ == "label") {
		color = point_colors_[i];
	}
	else {
		color = glm::vec3(255.0f, 255.0f, 255.0f);
	}
	return color;
}

void Lidar::WriteProjectedLidarImage(std::string fname) {
	int image_width = pc_width_; 
	int image_height = pc_height_;
	int half_width = (int)(0.5f*image_width);
	int half_height = (int)(0.5f*image_height);
	cimg_library::CImg<float> image;
	image.assign(image_width, image_height, 1, 3, 0.0f);
	float min_height = 1000000.0f;
	float max_height = -1000000.0f;
	if (display_color_type_ == "height") {
		for (int i = 0; i < (int)points_.size(); i++) {
			if (points_[i].z > max_height)max_height = points_[i].z;
			if (points_[i].z < min_height)min_height = points_[i].z;
		}
	}

	//Get geometry of Lidar
	float h_res_rad = (float)mavs::k2Pi / (float)pc_width_;
	float v_range = vertical_fov_max_ - vertical_fov_min_;
	float v_res_rad = v_range / (float)pc_height_;

	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0.0) {
			float d = sqrt(points_[i].x*points_[i].x + points_[i].y*points_[i].y + points_[i].z*points_[i].z);
			float r = sqrt(points_[i].x*points_[i].x + points_[i].y*points_[i].y);
			float x = atan2(points_[i].y, points_[i].x);
			float y = atan2(points_[i].z, d);
			int px = (int)((x + (float)mavs::kPi) / h_res_rad);
			int py = (int)((y - vertical_fov_min_) / v_res_rad);
			if (px >= 0 && px < image_width && py >= 0 && py < image_height) {
				glm::vec3 color = GetPointDisplayColor(i, min_height, max_height);
				image.draw_circle(image_width-px-1, image_height-py-1, 1, (float*)(&color));
			}
		}
	}
	image.save(fname.c_str());
}

void Lidar::FillImage() {
	float min_height = 1000000.0f;
	float max_height = -1000000.0f;
	float max_intens = -1000000.0f;
	float dw2 = 0.5f*display_width_;
	float dh2 = 0.5f*display_height_;
	float max_dist = max_range_; // 0.0f;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > max_intens)max_intens = intensities_[i];
		if (points_[i].z > max_height)max_height = points_[i].z;
		if (points_[i].z < min_height)min_height = points_[i].z;
		//float dist = sqrt(points_[i].x*points_[i].x + points_[i].y*points_[i].y);
		//if (dist > max_dist)max_dist = dist;
	}
	float hrange = max_height - min_height;
	min_height = min_height - hrange;
	hrange = max_height - min_height;
	if (first_display_)image_.assign(display_width_, display_height_, 1, 3, 0.0);
	image_ = 0.0;
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0.0f) {
			int px = (int)(dw2 + (dw2 - 1)*(points_[i].x / max_dist));
			int py = (int)(dh2 + (dh2 - 1)*(points_[i].y / max_dist));
			glm::vec3 color = GetPointDisplayColor(i, min_height, max_height);
			/*
			if (hrange == 0.0f) {
				color.x = 0.0;
				color.y = 1.0;
			}
			else {
				color.x = math::clamp((points_[i].z - min_height) / hrange, 0.0f, 1.0f);
				color.y = math::clamp(1.0f - color.x, 0.5f, 1.0f);
			}
			color.z = math::clamp(intensities_[i], 0.0f, 1.0f);
			*/
			image_.draw_point(px, py, (float*)(&color));
		}
	}
	image_.mirror('y');
	image_filled_ = true;
}

void Lidar::FillImagePerspective() {
	FillImagePerspective(768, 256);
}

void Lidar::FillImagePerspective(int image_width, int image_height) {
	//int image_width = 768; // 512;
	//int image_height = 256; // 384; // 256;
	int half_width = (int)(0.5f*image_width);
	int half_height = (int)(0.5f*image_height);
	float flen = 400.0; // 650.0; // 500.0f;
	if (first_display_)image_.assign(image_width, image_height, 1, 3, 0.0);
	image_ = 0.0f;

	float min_height = 1000000.0f;
	float max_height = -1000000.0f;
	if (display_color_type_ == "height") {
		for (int i = 0; i < (int)points_.size(); i++) {
			if (points_[i].z > max_height)max_height = points_[i].z;
			if (points_[i].z < min_height)min_height = points_[i].z;
		}
	}

	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0.0) {
			float z = points_[i].x; 
			if (z > 1.0E-5f) {
				float s = flen / z;
				float x = s * points_[i].y; 
				float y = s * points_[i].z; 
				int px = image_width - (int)(x + half_width) - 1;
				int py = image_height - (int)(y + half_height) - 1;
				if (px >= 0 && px < image_width && py >= 0 && py < image_height) {
					//image_.draw_point(px, py, (float*)(&point_colors_[i]));
					glm::vec3 color = GetPointDisplayColor(i, min_height, max_height);
					image_.draw_circle(px, py, 1, (float*)(&color));
				}
			}
		}
	}
	image_filled_ = true;
}

void Lidar::SetDisplayColorType(std::string type) {
	if (type == "color" || type == "range" || type == "height" || type == "intensity" || type == "white" || type=="label") {
		display_color_type_ = type;
	}
	else {
		std::cerr << "Lidar display color type " << type << " not recognized." << std::endl;
		display_color_type_ = "color";
	}
}

void Lidar::Display() {
	DisplayTopDown();
}

void Lidar::DisplayPerspective() {
	DisplayPerspective(768, 256);
}

void Lidar::DisplayPerspective(int im_width, int im_height) {
	//if (!image_filled_)FillImage();
	//FillImage();
	FillImagePerspective(im_width,im_height);
	if (comm_rank_ == 0) {
		if (first_display_) {
			//disp_.assign(display_width_, display_height_, name_.c_str());
			disp_.assign(image_);
			disp_.set_title(name_.c_str());
			first_display_ = false;
		}
		disp_ = image_;
	}
}

void Lidar::DisplayTopDown() {
	//if (!image_filled_)FillImage();
	FillImage();
	//FillImagePerspective();
	if (comm_rank_ == 0) {
		if (first_display_) {
			//disp_.assign(display_width_, display_height_, name_.c_str());
			disp_.assign(image_);
			disp_.set_title(name_.c_str());
			first_display_ = false;
		}
		disp_ = image_;
	}
}

PointCloud Lidar::GetRosPointCloud() {
	PointCloud cloud;
	ChannelFloat32 intens;
	intens.name = "intensity";
	for (int i = 0; i < (int)points_.size(); i++) {
		if (intensities_[i] > 0.0f) {
			cloud.points.push_back(points_[i]);
			intens.values.push_back(intensities_[i]);
		}
	}
	cloud.channels.push_back(intens);
	return cloud;
}

std::vector<glm::vec4> Lidar::GetPointsXYZI() {
	std::vector<glm::vec4> xyzi;
	xyzi.resize(points_.size());
	for (int i = 0; i < (int)points_.size(); i++) {
		xyzi[i] = glm::vec4(points_[i], intensities_[i]);
	//if (intensities_[i] > 0.0) {
		//glm::vec4 p;
		//p.x = points_[i].x;
		//p.y = points_[i].y;
		//p.z = points_[i].z;
		//p.w = intensities_[i];
		//xyzi.push_back(p);
	//}
	}
	return xyzi;
}

PointCloud2 Lidar::GetPointCloud2Registered() {
	RegisterPoints();
	PointCloud2 pc;
	pc.height = pc_height_;
	pc.width = pc_width_;
	pc.is_bigendian = false;
	pc.is_dense = false;
	pc.point_step = sizeof(glm::vec4) ;
	pc.row_step = pc.point_step*pc.width;
	PointField x_field,y_field,z_field,intens_field;
	x_field.name = "x";
	x_field.datatype = x_field.FLOAT32; 
	x_field.offset = 0;
	x_field.count = 1;
	y_field.name = "y";
	y_field.datatype = y_field.FLOAT32; 
	y_field.offset = 4;
	y_field.count = 1;
	z_field.name = "z";
	z_field.datatype = z_field.FLOAT32; 
	z_field.offset = 8;
	z_field.count = 1;
	intens_field.name = "intensity";
	intens_field.datatype = intens_field.FLOAT32; 
	intens_field.offset = 12;
	intens_field.count = 1;
	pc.fields.push_back(x_field);
	pc.fields.push_back(y_field);
	pc.fields.push_back(z_field);
	pc.fields.push_back(intens_field);
	pc.data.resize(pc.height*pc.width);
	int n = 0;
	for (unsigned i = 0; i < pc.width; i++) {
		for (unsigned j = 0; j < pc.height; j++) {
			pc.data[n].x = registered_points_[n].x;
			pc.data[n].y = registered_points_[n].y;
			pc.data[n].z = registered_points_[n].z;
			if (intensities_[n]<=0.0){
				pc.data[n].x = 0.0f; 
				pc.data[n].y = 0.0f; 
				pc.data[n].z = 0.0f;
			}
			pc.data[n].w = (100.0f*intensities_[n]);
			n++;
		}
	}
	return pc;
}

PointCloud2 Lidar::GetPointCloud2() {
	PointCloud2 pc;
	pc.height = pc_height_;
	pc.width = pc_width_;
	pc.is_bigendian = false;
	pc.is_dense = false;
	pc.point_step = sizeof(glm::vec4) ;
	pc.row_step = pc.point_step*pc.width;
	PointField x_field,y_field,z_field,intens_field;
	x_field.name = "x";
	x_field.datatype = x_field.FLOAT32; 
	x_field.offset = 0;
	x_field.count = 1;
	y_field.name = "y";
	y_field.datatype = y_field.FLOAT32; 
	y_field.offset = 4;
	y_field.count = 1;
	z_field.name = "z";
	z_field.datatype = z_field.FLOAT32; 
	z_field.offset = 8;
	z_field.count = 1;
	intens_field.name = "intensity";
	intens_field.datatype = intens_field.FLOAT32; 
	intens_field.offset = 12;
	intens_field.count = 1;
	pc.fields.push_back(x_field);
	pc.fields.push_back(y_field);
	pc.fields.push_back(z_field);
	pc.fields.push_back(intens_field);
	pc.data.resize(pc.height*pc.width);
	int n = 0;
	for (unsigned i = 0; i < pc.width; i++) {
		for (unsigned j = 0; j < pc.height; j++) {
			pc.data[n].x = points_[n].x;
			pc.data[n].y = points_[n].y;
			pc.data[n].z = points_[n].z;
			pc.data[n].w = (100.0f*intensities_[n]);
			n++;
		}
	}
	return pc;
}

void Lidar::GetLaserScan(LaserScan &scan) {
	scan.ranges = distances_;
	scan.intensities = intensities_;
	scan.angle_max = angle_max_;
	scan.angle_min = angle_min_;
	scan.angle_increment = angle_increment_;
	scan.range_min = min_range_;
	scan.range_max = max_range_;
}

std::vector<float> Lidar::GetReturnedDistances() {
	std::vector<float> dists;
	for (int i = 0; i < (int)distances_.size(); i++) {
		if (intensities_[i] > 0.0) {
			dists.push_back(distances_[i]);
		}
	}
	return dists;
}

} //namespace lidar
} //namespace sensor
} //namespace mav
