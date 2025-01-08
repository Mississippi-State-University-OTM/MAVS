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
#include <mavs_core/terrain_generator/species.h>

#include <math.h>
#include <stdlib.h>

#include <iostream>

namespace mavs {
namespace terraingen {

Species::Species() {
	new_plants_per_year_per_meter_ = 10;
	relative_growth_rate_ = 0.15f; //exponential growth
	max_height_ = 5.0f; //meters
	diameter_height_ratio_ = 0.5f; //unitless = diameter = ratio*height
	mesh_default_height_ = 1.0f;
}

float Species::GetNextHeight(float current_height) {
	float age_current = log(1.0f - (current_height / max_height_)) / (-relative_growth_rate_);
	float new_age = age_current + 1.0f;
	float new_height = max_height_ * (1.0f - exp(-relative_growth_rate_ * new_age));
	return new_height;
}

int Species::GetNumNew(float area) {
	float newfac = area * new_plants_per_year_per_meter_;
	float new_over = newfac - floor(newfac);
	float test_val = ((float)rand() / (RAND_MAX));
	int num_new = (int)floor(newfac);
	if (new_over > test_val)num_new = num_new + 1;
	return num_new;
}

} //namespace environment
} //namespace mavs