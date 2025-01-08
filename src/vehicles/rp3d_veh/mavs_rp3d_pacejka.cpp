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
#include <vehicles/rp3d_veh/mavs_rp3d_pacejka.h>
#include <math.h>
#include <iostream>
#include <mavs_core/math/constants.h>

namespace mavs {
namespace vehicle {
namespace mavs_rp3d {

MavsPacejka::MavsPacejka() {
	surface_type_ = "dry";
	CI_ = 250.0f*6894.76f;
	b_ = 0.3f;
	d_ = 0.5f;
	h_ = d_ / 3.0f;
	high_slip_angle_cutoff_radians_ = (float)(15.0*mavs::kDegToRad);
}

void MavsPacejka::SetSlipAngleCrossover(float angle) {
	high_slip_angle_cutoff_radians_ = (float)(angle*mavs::kDegToRad);
}

void MavsPacejka::SetTireParameters(float width, float diameter, float section_height) {
	b_ = width;
	d_ = diameter;
	h_ = section_height;
}

void MavsPacejka::SetSurfaceType(std::string type) {
	surface_type_ = type;
}

void MavsPacejka::SetSurfaceProperties(std::string type, float CI) {
	surface_type_ = type;
	CI_ = CI;
}

float MavsPacejka::GetCrolla(float alpha, float beta) {
	float Lc = 0.0f;
	if (alpha != 0.0f) {
		//From "Verification and validation of a winter driving simulator", Eq 13
		Lc = (alpha / fabs(alpha))* (1.0f - exp(-fabs(alpha) / beta));
	}
	return Lc;
}

void MavsPacejka::SetCoeffs(float W, float delta) {
	if (surface_type_ == "snow") {
		B_ = 5.0f;
		C_ = 2.0f;
		D_ = 0.3f;
		E_ = 1.0f;
		D_lat_ = D_;
	}
	else if (surface_type_ == "wet") {
		B_ = 12.0f;
		C_ = 2.3f;
		D_ = 0.82f;
		E_ = 1.0f;
		D_lat_ = D_;
	}
	else if (surface_type_ == "ice") {
		B_ = 4.0f;
		C_ = 2.0f;
		D_ = 0.1f;
		E_ = 1.0f;
		D_lat_ = D_;
	}
	else if (surface_type_ == "sand") {
		// Fit of  pacejka equation to the brixius equation
		float B_n = (CI_*b_*d_ / W)*(1.0f + 5.0f*delta / h_) / (1.0f + 3.0f*b_ / d_);
		if (B_n > 5.0f) {
			B_ = 1.6f*(float)(log(0.119f*(B_n - 4.033f))*exp(-0.113*(B_n - 4.033f))) + 4.906f;
			C_ = 1.213f*(1.0f - exp(-0.0765f*(B_n - 2.846f)));
			D_ = 1.0f;
			E_ = 1.0925f + 0.721f / pow(B_n - 4.693f, 0.981f);
			D_lat_ = 1.0f;
		}
		else {
			B_ = 10.0f;
			C_ = 1.9f;
			D_ = 0.0001f;
			E_ = 0.97f;
			D_lat_ = 1.0f;
		}
	}
	else if (surface_type_ == "clay") {
		float M = (CI_*b_*d_ / W)*sqrt(delta / h_) / (1.0f + 0.5f*b_ / d_);
		if (M > 7.0f) {
			B_ = 2.556E-5f*M*M + 3.61E-2f*M + 3.064f;
			C_ = 1.08f - 0.2878f*M - 1.794f / M;
			D_ = 1.0f;
			E_ = 1.018f - 59.278f*(float)(exp(-0.0192*M)*sin(6.283f*(M - 4.0f)));
			D_lat_ = 1.0f;
		}
		else { // paved
			B_ = 10.0f;
			C_ = 1.9f;
			D_ = 0.0001f;
			E_ = 0.97f;
			D_lat_ = 1.0f;
		}
	}
	else { //dry tarmac
		B_ = 10.0f;
		C_ = 1.9f;
		D_ = 1.0f;
		E_ = 0.97f;
		D_lat_ = 1.0f;
	}
}

rp3d::Vector2 MavsPacejka::GetCombinedTraction(float W, float delta, float slip, float alpha) {
	if (surface_type_ == "clay") {
		// Equations 1-8 from: 
		// Improving accuracy of vehicle-terrain interface algorithms for wheeled
		// vehicles on fine - grained soils through Bayesian calibration
		float Nc = (float)(CI_*b_*d_ / (W*pow(1.0-delta / h_,1.5) * pow(1.0f + b_ / d_,0.75)));
		float Ssp = (float)(21.0 / pow(Nc, 2.5));
		//float Ncz = Nc* pow(1.0f + b_ / d_, 0.75);
		float dbp = 0.5f*log10(slip / Ssp);
		float mr = 12.0f / (Nc*Nc) + 0.007f;
		float traction = std::min(1.0f, std::max(-1.0f,(float)(dbp * pow(1.0f + b_ / d_, 0.25f)) - mr));
		//float fy = erf(0.5f*alpha / high_slip_angle_cutoff_radians_);
		float fy = GetCrolla(alpha, high_slip_angle_cutoff_radians_);
		rp3d::Vector2 f(W*traction, -W*fy);
		return f;
	}/*
	else if (surface_type_ == "sand") {
		rp3d::Vector2 f(0.0f, 0.0f);
		if (W > 0.0f) {
			// Improved sinkage algorithms for powered and unpowered, wheeled vehicles operating on sand Eq 2
			float G = (CI_ / 3.47f)/6894.76f;
			//A unified equation for predicting traction for wheels on sand over a range of braked, towed, and powered operations, Eqs 9-10
			float dbp = 0.0f;
			if (slip != 0.0f) {
				float Ns = G * pow(b_*d_, 1.5)*delta / (W*h_);
				float X = 0.66f;
				float Y = 4.71f + 1.72f / slip;
				dbp = X * Y / (Ns + 10.0f + Y);
			}
			// New algorithms for predictinglongitudinal motion resistance of wheels on dry sand, Eq 1
			float X_m = 0.44 - 0.002287*W;
			float mr = X_m + sqrt(X_m*X_m + 0.0000457*W + 0.08f) + 0.05f*(delta / h_);
			float traction = std::min(1.0f, std::max(-1.0f, dbp - mr));
			//float fy = erf(0.5f*alpha / high_slip_angle_cutoff_radians_);
			float fy = GetCrolla(alpha, high_slip_angle_cutoff_radians_);
			f = rp3d::Vector2(W*traction, -W*fy);
		}
		return f;
	}*/
	else {
		
		//adjust coeffs
		SetCoeffs(W, delta);

		// longitudinal
		float Bs = B_ * slip;
		float fx = W * D_ * sin(C_*atan(Bs - E_ * (Bs - atan(Bs))));
		//float fx0 = W * D_;

		// lateral forces & combined forces
		//float fy = fy0 * sqrt(1.0f - pow(fx / fx0, 2));
		float fy = -W*GetCrolla(alpha, high_slip_angle_cutoff_radians_);
		rp3d::Vector2 f(fx, fy);
		return f;
	}
}


} // namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs