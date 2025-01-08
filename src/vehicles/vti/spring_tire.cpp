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
#include "vti/spring_tire.h"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace mavs {
namespace vti {

SpringTire::SpringTire() {
	//SetLoadDeflection(1000.0, 0.05);
	spring_constant_ = 27500.0;//275000.0;
	damping_coefficient_ = 16000.0;//160000.0;
	current_velocity_ = 0.0;
	current_compression_ = 0.0;
}

void SpringTire::SetLoadDeflection(double load, double deflection) {
	spring_constant_ = (load / deflection);
	SetCriticalDamping(load*0.10197);
}

void SpringTire::SetSpringDamping(double k, double c) {
	spring_constant_ = k;
	damping_coefficient_ = c;
}

void SpringTire::SetCriticalDamping(double mass) {
	damping_coefficient_ = 2.0*sqrt(spring_constant_*mass);
}

double SpringTire::Update(double dt, double x) {
	x = std::min(std::max(0.0, x), section_height_);
	double dx = x - current_compression_;
	//double compression = std::max(0.0, radius_ - x);
	//std::cout << "forces " << x << " " << spring_constant_*x << std::endl;
	//double force = -damping_coefficient_ * (dx / dt) + spring_constant_ * x;
	double force =  spring_constant_ * x;
	force = std::max(0.0, force);
	current_compression_ = x;
	return force;
}

} //namespace vti
} //namespace mavs
