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
#include <sensors/camera/lens_drop.h>
#include <iostream>
#include <time.h>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace camera{

static int lens_drop_counter_ = 1;
LensDrop::LensDrop(){
  radius_ = 2.5f/1000.0f;
  radius_pixels_ = 25;
  age_ = 0.0f;
  lifetime_ = 5.0f;
  color_ = glm::vec3(0.5f,0.5f, 0.5f);
  center_pixels_ = glm::ivec2(0, 0);
  int curr_time = (int)time(NULL);
  shape_noise_.SetSeed(curr_time+lens_drop_counter_);
  lens_drop_counter_++;
}

bool LensDrop::PixelInDrop(int pi, int pj) {
	shape_noise_.SetFrequency(0.5f / radius_pixels_);
	int x = pi - center_pixels_.x;
	int y = pj - center_pixels_.y;
	double z = shape_noise_.GetPerlin((double)(x), (double)(y));
	bool in_drop = false;
	if (z > -0.4)in_drop = true;
	return in_drop;
}

void LensDrop::Print() {
	std::cout << "Drop radius = " << radius_ << " "<<radius_pixels_<<std::endl;
	std::cout << "Drop center = (" << center_pixels_.x << ", " << center_pixels_.y << ") " << std::endl;
}

} //namespace camera
} //namespace sensor
} //namespace mavs
