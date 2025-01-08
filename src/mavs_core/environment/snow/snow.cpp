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
#include <mavs_core/environment/snow/snow.h>

#include <iostream>

#include <mavs_core/math/constants.h>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace environment {

Snow::Snow() {
	float sec_size = 1.0f;
	sector_step_ = (float)(sec_size*mavs::kDegToRad);
	num_sectors_ = (int)(360.0f / sec_size);
	sectors_.resize(num_sectors_);
}

void Snow::Initialize(float rate, float temperature) {
	snowflakes_.clear();
	temperature_ = temperature;
	snow_cylinder_radius_ = 3.0; // 20.0;
	float radius = snow_cylinder_radius_;
	snow_cylinder_height_ = 5.0; // 20.0f;
	//float add_factor = 0.25f;
	//int old_num_to_add = (int)(add_factor*rate * mavs::kPi*radius*radius*snow_cylinder_height_);
	float cyl_volume = (float)(mavs::kPi*radius*radius*snow_cylinder_height_);
	//from sekhon 1969, Fig 4, divided by 200 for empirical visualization
	int num_to_add = (int)(cyl_volume*1312.6f*rate/200.0f);
	for (int i = 0; i < num_to_add; i++) {
		Snowflake flake(temperature);
		glm::vec3 p(mavs::math::rand_in_range(-radius, radius), mavs::math::rand_in_range(-radius, radius), mavs::math::rand_in_range(0.0f, snow_cylinder_height_));
		flake.SetPosition(p);
		snowflakes_.push_back(flake);
	}
	Update(0.000001f, glm::vec3(0.0f, 0.0f, 0.0f));
	// #/m^3, rasshofer 2011 Eq 25 and 26
	if (temperature_ <= -1.0f) {
		// dry snow
		alpha_ = (15.0f*rate + 1.0f) / 1000.0f;
	}
	else {
		// wet snow
		alpha_ = (2.0f*rate - 0.1f) / 1000.0f;
	}
	// Is way too high and gives bad results, lower by factor of 10
	alpha_ = alpha_ / 10.0f;
}

void Snow::Update(float dt, glm::vec3 wind) {
	for (int i = 0; i < num_sectors_; i++) {
		sectors_[i].clear();
	}
	float radius = snow_cylinder_radius_;
	for (int i = 0; i < snowflakes_.size(); i++) {
		snowflakes_[i].UpdatePosition(wind, dt);
		glm::vec3 p = snowflakes_[i].GetPosition();
		//if snowflake has left cylinder or gone below ground, respawn it
		float r = (float)sqrt(p.x*p.x + p.y*p.y);
		if (r > radius || p.z < 0.0f) {
			Snowflake flake(temperature_);
			glm::vec3 pnew(mavs::math::rand_in_range(-radius, radius), 
				mavs::math::rand_in_range(-radius, radius),
				mavs::math::rand_in_range(0.0f, snow_cylinder_height_));
			flake.SetPosition(pnew);
			snowflakes_[i] = flake;
		}
		float theta = (float)(atan2(p.y, p.x) + mavs::kPi);
		int sector = (int)floor(theta / sector_step_);
		if (sector >= 0 && sector < num_sectors_) {
			sectors_[sector].push_back(i);
		}
	}
}

/**
* Get the distance from a line to a point
* x1 and x2 are points on the line
* x0 is the point you are getting the distance from, see
* http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
*/
float PointLineDistance(glm::vec3 x1, glm::vec3 x2, glm::vec3 x0) {
	float d = glm::length(glm::cross(x0 - x1, x0 - x2)) / glm::length(x2 - x1);
	return d;
}

float Snow::GetClosestIntersection(glm::vec3 position, glm::vec3 direction, float pixdim, float &snowdist) {
	//glm::vec3 color(0.0f, 0.0f, 0.0f);
	float closest = std::numeric_limits<float>::max();
	int closest_particle = -1;
	float flake_radius = 0.0f;
	//glm::vec3 x2 = position + direction;
	//glm::vec3 origin(0.0f, 0.0f, position.z);
	glm::vec3 origin(0.0f, 0.0f, 2.0);
	glm::vec3 x2 = origin + direction;
	int sector = (int)floor((atan2(direction.y, direction.x) + mavs::kPi) / sector_step_);
	int sec_lo = sector - 2;
	int sec_hi = sector + 2;
	std::vector<int> to_search;
	for (int sec = sec_lo; sec <= sec_hi; sec++) {
		int s = sec;
		if (s < 0) s = s + (int)sectors_.size();
		if (s >= (int)sectors_.size()) s = s - (int)sectors_.size();
		to_search.push_back(s);
	}
	for (int p = 0; p < to_search.size(); p++){
		int curr_sector = to_search[p];
		for (int s = 0; s < sectors_[curr_sector].size(); s++) {
			int i = sectors_[curr_sector][s];
			//float r = PointLineDistance(position, x2, snowflakes_[i].GetPosition());
			float r = PointLineDistance(origin, x2, snowflakes_[i].GetPosition());
			if (r < (snowflakes_[i].GetRadius() + pixdim)) {
				if (r < closest) {
					closest_particle = i;
					closest = r;
					flake_radius = snowflakes_[i].GetRadius();
					break;
				}
			}
		}
}
	float opacity = 0.0f;
	if (closest_particle > 0) {
		float x = closest / flake_radius;
		opacity = (float)pow(cos(x*mavs::kPi_2),0.5);
	}
	snowdist = closest;
	return opacity;
}

} // namespace environment
} // namespace mavs