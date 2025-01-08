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
#include "vti/coarse_grained.h"
#include <math.h>
#include <iostream>

namespace mavs {
namespace vti {

CoarseGrained::CoarseGrained() {
	SetWheelProperties(0.226, 0.704, 0.15);
	min_trac_ = -0.5;
	B_ = 7.0;
}

void CoarseGrained::SetWheelProperties(double width, double diameter, double section_height) {
	b_ = width;
	d_ = diameter;
	h_ = section_height;
	a1_ = pow(b_*d_, 1.5);
}

void CoarseGrained::CalculateNumerics(double rci, double load, double deflection, double slip) {
	if (deflection > h_)deflection = h_;
	// mason2016, eq 2
	double G = rci / 3.47;
	// detwiller2017, Eq 2
	N_s_ = G * a1_*deflection / (load*h_);
	// Williams2017, Eq 3
	N_s_p_ = rci * a1_ / ((1.0 - (deflection/h_))*load);
	// Williams2017, Eq 2
	N_s_u_ = N_s_p_ / (1.0 - (b_ / d_));
	// Mason2016, Eq 10
	double a0 = deflection + 0.026*d_;
	CP_ = load / (2.0 * b_*sqrt(d_*a0 - a0*a0)*pow(b_/h_,0.52));
	// mason2018, eq  6
	ratio_ = CP_ / rci;
	max_trac_ = 0.42 + 0.009 / ratio_;
	//mason2018, eq 7
	A_ = 0.76 + 0.014 / ratio_;
	// mason2018, eq 8
	C_ = 1.56 - 0.022 / ratio_;
}

void CoarseGrained::CalculateTraction(double slip) {
	//mason 2018, equation 5
	longi_traction_ = min_trac_ + (max_trac_ - min_trac_) / pow(1.0 + A_ * exp(-B_ * slip), 1.0 / C_);
}

void CoarseGrained::CalculatePoweredMotionResistance(double rci, double slip) {
	//williams2017, equation 10
	longi_motion_resistance_ = 0.4*ratio_ + 0.6*(slip / 100) - 0.03;
}

void CoarseGrained::CalculateUnpoweredMotionResistance(double rci, double slip) {
	//williams2017, equation 9
	longi_motion_resistance_ = 0.21*ratio_ - 0.75*(slip / 100.0) + 0.04;
}

void CoarseGrained::CalculatePoweredSinkage() {
	//mason2016, equation 4
	sinkage_ = d_ * 14.0 / N_s_p_;
}

void CoarseGrained::CalculateUnpoweredSinkage() {
	//mason2016, equation 16
	sinkage_ = d_ * 22 / pow(N_s_u_, 1.125);
}

void CoarseGrained::CalculateDrawbarPull(double slip) {
	//need single wheel equation here
}

void CoarseGrained::Update(double rci, double load, double deflection, double slip, double alpha) {
	CalculateNumerics(rci, load, deflection, slip);
	CalculateTraction(slip);
	if (powered_) {
		CalculatePoweredSinkage();
		CalculatePoweredMotionResistance(rci,slip);
	}
	else {
		CalculateUnpoweredSinkage();
		CalculateUnpoweredMotionResistance(rci,slip);
	}
	CalculateDrawbarPull(slip);
}

} //namespace vti
} //namespace mavs