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
#include "vti/fine_grained.h"
#include <math.h>
#include <iostream>
#include <algorithm>

namespace mavs {
namespace vti {

FineGrained::FineGrained() {
	SetWheelProperties(0.226, 0.704, 0.15);
}

void FineGrained::SetWheelProperties(double width, double diameter, double section_height) {
	b_ = width;
	d_ = diameter;
	h_ = section_height;
	//see equations 1 & 3
	a1_ = 1.0f + b_ / d_;
	d1_ = pow(a1_, 0.75f);
}

void FineGrained::CalculateNumerics(double rci, double load, double deflection, double slip) {
	if (deflection > h_)deflection = h_;
	double a = 1.0f - (deflection / h_);
	//Equation 3
	N_cz_ = rci * b_*d_ / (load*pow(a, 1.5f));
	//Equation 1
	N_c_ = N_cz_ / d1_;
	//Equation 2
	S_sp_ = (21.0f / pow(N_c_, 2.5f)) + 0.005f;
}

void FineGrained::CalculateTraction(double slip) {
	if (slip <= 0.0) {
		longi_traction_ = 0.0;
		return;
	}
	// equation 29
	double term1 = 0.5*log10(slip / S_sp_)*pow(a1_, 0.25);
	double term2 = 12 / (N_c_*N_c_);
	double term3 = 0.007;
	double term4 = 0.032*pow(S_sp_, 0.547) / pow(slip, 0.777);
	//traction_ = term1 + term2 + term3 + term4;

	//equation 8
	longi_traction_ = term1 + term2 + term3;

	longi_traction_ = std::min(1.0, longi_traction_);
}

void FineGrained::CalculateMotionResistance() {
	// equation 27
	longi_motion_resistance_ = (5.07 / pow(N_c_, 1.71)) + 0.011;
}

void FineGrained::CalculatePoweredSinkage() {
	// equation 21
	double ssp = pow(S_sp_, 0.19);
	sinkage_ = 1.64 / pow(N_cz_ / ssp, 1.21);
}

void FineGrained::CalculateUnpoweredSinkage() {
	// equation 23
	sinkage_ = (2.66 / pow(N_cz_, 1.64)) - 0.0063;
}

void FineGrained::CalculateDrawbarPull(double slip) {
	double term1 = 0.5*log10(slip / S_sp_);
	double s = pow(slip, 4.36);
	double ssp = pow(S_sp_, 1.5);
	double term2 = exp(-2.68*ssp/s);
	drawbar_pull_ = term1 + term2;
}

void FineGrained::Update(double rci, double load, double deflection, double slip, double alpha) {
	CalculateNumerics(rci, load, deflection, slip);
	CalculateTraction(slip);
	CalculateMotionResistance();
	if (powered_) {
		CalculatePoweredSinkage();
	}
	else {
		CalculateUnpoweredSinkage();
	}
	CalculateDrawbarPull(slip);
}

} //namespace vti
} //namespace mavs