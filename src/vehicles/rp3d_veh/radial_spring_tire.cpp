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
#include <vehicles/rp3d_veh/radial_spring_tire.h>

namespace mavs {
namespace vehicle {
namespace radial_spring {

Tire::Tire() {
	orientation_[0][0] = 1.0f;
	orientation_[0][1] = 0.0f;
	orientation_[0][2] = 0.0f;
	orientation_[1][0] = 0.0f;
	orientation_[1][1] = 1.0f;
	orientation_[1][2] = 0.0f;
	orientation_[2][0] = 0.0f;
	orientation_[2][1] = 0.0f;
	orientation_[2][2] = 1.0f;
	tire_position_ = glm::vec3(0.0f, 0.0f, 1.0f);
	initialized_ = false;
	current_ground_normal_ = glm::vec3(0.0f, 0.0f, 1.0f);
}

void Tire::Initialize(float sw, float ud_r, float k, float dtheta_degrees, int num_slices) {
	section_width_ = sw; // 0.25f;
	undeflected_radius_ = ud_r; // 0.254f; // 10 inches converted to meters
	spring_constant_ = k; // 68947.6f; // 10 PSI converted to pascals

	nsprings_ = (int)ceil(360.0f / dtheta_degrees);
	dtheta_ = (float)(2.0f*mavs::kPi / (float)nsprings_);
	nslices_ = num_slices;
	float slice_size = section_width_ / (num_slices + 1.0f);
	slices_.resize(num_slices);
	for (int j = 0; j<num_slices; j++) {
		slices_[j].offset = (j + 1)*slice_size - 0.5f*section_width_;
		slices_[j].springs.resize(nsprings_);
		// set the angle for each spring
		for (int i = 0; i<slices_[j].springs.size(); i++) {
			slices_[j].springs[i].SetTheta(i*dtheta_);
		}
	}
	dslice_ = section_width_ / (1.0f*num_slices);
	initialized_ = true;
}

float Tire::GetNormalForce(environment::Environment *env) {
	// loop through and put the slices in the right position
	glm::vec3 look_side = orientation_[1];
	for (int j = 0; j<slices_.size(); j++) {
		slices_[j].position = tire_position_ + slices_[j].offset*look_side;
	}

	// in this loop, calculate the deflections at each sample point
	// this is different that Equation 1 of DAVIS because it assumes we'll have raytracing
	float ncontact = 0.0f;
	std::vector<float> normal_weights;
	std::vector <glm::vec3> weighted_normal;
#pragma omp parallel for reduction(+: ncontact) schedule(dynamic)
	for (int j = 0; j<nslices_; j++) {
		for (int i = 0; i<nsprings_; i++) {
			slices_[j].springs[i].SetDeflection(0.0f);
			glm::vec3 dir = orientation_ * slices_[j].springs[i].GetDirection();
			//if (dir.z<=0.0f) {
			glm::vec3 pos = slices_[j].position;
			raytracer::Intersection inter = env->GetClosestTerrainIntersection(pos, dir);
			float r = inter.dist;
			if (r > 0.0f && r < undeflected_radius_) {
				ncontact += 1.0f;
				float deflection = undeflected_radius_ - r;
				slices_[j].springs[i].SetDeflection(deflection);
				glm::vec3 gn = inter.normal;
				if (gn.z < 0.0f)gn = -1.0f * gn;
				weighted_normal.push_back(gn);
				normal_weights.push_back(deflection);
			}
			//}
			slices_[j].springs[i].SetPosition(slices_[j].position + slices_[j].springs[i].GetDirection()*(undeflected_radius_ - slices_[j].springs[i].GetDeflection()));
		}
	}

	// calculate the ground normal, weighted by tire defelction
	glm::vec3 ground_normal(0.0f, 0.0f, 0.0f);
	float denominator = 0.0f;
	for (int wn = 0; wn < (int)normal_weights.size(); wn++) {
		ground_normal += weighted_normal[wn] * normal_weights[wn];
		denominator += normal_weights[wn];
	}
	if (denominator > 0.0f) {
		ground_normal = ground_normal / denominator;
		float mag = sqrtf(ground_normal.x * ground_normal.x + ground_normal.y * ground_normal.y + ground_normal.z * ground_normal.z);
		ground_normal = ground_normal / mag;
		current_ground_normal_ = ground_normal;
	}

	// check if tire is not touching
	if (ncontact == 0)return 0.0f;
	// DAVIS, Equations 2, 4, and 5
	float theta_contact = ncontact * dtheta_ / nslices_;
	float d_max = undeflected_radius_ * (1.0f - cos(0.5f*theta_contact));
	float a_s = 0.5f*undeflected_radius_*undeflected_radius_*(theta_contact - sinf(theta_contact));
	float v_s = section_width_ * a_s;

	// in the this loop, calculate the deflected volume
	float v_t = 0.0f;
	//float a_t = 0.0f;
#pragma omp parallel for reduction(+: v_t) schedule(dynamic)
	for (int j = 0; j<slices_.size(); j++) {
		for (int i = 0; i<nsprings_; i++) {
			float di = slices_[j].springs[i].GetDeflection();
			// DAVIS Equation 3
			float displaced_area = (undeflected_radius_*di - 0.5f*di*di)*dtheta_;
			//a_t += displaced_area;
			v_t += displaced_area * dslice_;
		}
	}
	//a_t = a_t / nslices_;

	// DAVIS Equation 6, equivalent deflection
	float d_e = d_max * v_t / v_s;
	//float d_e = d_max * a_t / a_s;
	current_equivalent_deflection_ = d_e;
	// DAVIS Equation 17
	float fn = spring_constant_ * d_e; // *d_e;
	//std::cout << "FN: " << fn << std::endl;
	return fn;
}


void Tire::PrintSprings() {
	for (int j = 0; j<slices_.size(); j++) {
		for (int i = 0; i<nsprings_; i++) {
			glm::vec3 p = slices_[j].springs[i].GetPosition();
			std::cout << j << " " << i << " " <<
				p.x << " " << p.y << " " << p.z << " " <<
				slices_[j].springs[i].GetDeflection() << std::endl;
		}
	}
}

} // namespace radial_spring
}// namespace vehicle
}// namespace mavs
