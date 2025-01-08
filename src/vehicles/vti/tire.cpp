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
#include "vti/tire.h"

namespace mavs {
namespace vti {

Tire::Tire() {

}

void Tire::Update(double dt, double z, double h,  double slip, double slip_angle) {
	double r = z - h;
	if (r > 0.0 && r < radius_) {
		double deflection = radius_ - r;
		double normal_force = spring_tire_.Update(dt, r);
		vti_.Update(100.0, normal_force, deflection, slip, slip_angle);
		forces_.normal = normal_force;
		forces_.longitudinal = vti_.GetLongitudinalTraction();
		forces_.lateral = vti_.GetLateralTraction();
	}
	else {
		double normal_force = spring_tire_.Update(dt, 0.0);
		forces_.normal = 0.0;
		forces_.longitudinal = 0.0;
		forces_.lateral = 0.0;
	}
}

} //namespace vti
} //namespace mavs

