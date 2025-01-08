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
#include "vti/brixius.h"
#include <math.h>
#include <iostream>
#include <algorithm>
#include "vti/nicolas_comstock.h"

namespace mavs {
namespace vti {

Brixius::Brixius() {
	SetWheelProperties(0.226, 0.704, 0.15);
	is_bias_ply_ = true;
}

void Brixius::SetWheelProperties(double width, double diameter, double section_height) {
	b_ = width;
	d_ = diameter;
	h_ = section_height;
	d2_ = d_ / 2.0;
}

void Brixius::CalculateNumerics(double rci, double load, double deflection, double slip) {
	if (deflection > h_)deflection = h_;
	// equation 16
	C_n_ = rci * b_*d_ / load;
	// equation 17
	B_n_ = C_n_ * (1.0 + 5.0*deflection / h_) / (1.0 + 3.0*b_ / d_);
	//rolling radius
	r_ = (2.5*d2_*(d2_ - deflection)) / (1.5*d2_ + (d2_ - deflection));
	//equation 24
	B_n_1_ = C_n_ * sqrt(d_ / h_) / (1.0 + 0.5*b_*d_);
}

void Brixius::CalculateResistance(double slip) {
	//equations 19-20
	if (is_bias_ply_) {
		C_rr_road_ = (1.0 / B_n_) + 0.5*fabs(slip) / sqrt(B_n_);
		C_rr_tire_ = 0.04;
	}
	else {
		C_rr_road_ = (0.9 / B_n_) + 0.5*fabs(slip) / sqrt(B_n_);
		C_rr_tire_ = 0.0325;
	}
	longi_motion_resistance_ = C_rr_road_ + C_rr_tire_;
}

void Brixius::CalculateTraction(double slip) {
	//equation 21
	if (fabs(slip) < 1.0E-5) {
		longi_traction_ = 0.0;
		return;
	}
	double sign = slip / fabs(slip);
	if (is_bias_ply_) {
		longi_traction_ = sign * 0.88*(1.0 - exp(-0.1*B_n_))*(1 - exp(-7.5*fabs(slip)));
	}
	else {
		longi_traction_ = sign * 0.88*(1.0 - exp(-0.1*B_n_))*(1 - exp(-10.5*fabs(slip)));
	}
}

void Brixius::CalculateLateralTraction(double steering) {
	//equations 23-24
	double mu_max, B;
	if (is_bias_ply_) {
		mu_max = 0.61 + (0.69 / B_n_1_);
		B = (2.34 + 0.088*B_n_1_) / mu_max;
	}
	else {
		mu_max = 0.8;
		B = (2.79 + 0.16*B_n_1_) / mu_max;
	}
	double steering_mag = fabs(steering);
	lat_traction_ = (steering/steering_mag)*mu_max * (1 - exp(-B * steering_mag));
}

void Brixius::Update(double rci, double load, double deflection, double slip, double alpha) {
	if (load < 1.0E-5) {
		longi_traction_ = 0.0;
		lat_traction_ = 0.0;
		return;
	}
	CalculateNumerics(rci, load, deflection, slip);
	CalculateTraction(slip);
	CalculateResistance(slip);
	lat_traction_ = 0.0;
	if (alpha != 0.0) {
		CalculateLateralTraction(alpha);
		NicolasComstock tire_model;
		TireForces forces = tire_model.GetForces(longi_traction_, lat_traction_, slip, alpha);
		longi_traction_ = forces.longitudinal;
		lat_traction_ = forces.lateral;
	}
}

} //namespace vti
} //namespace mavs