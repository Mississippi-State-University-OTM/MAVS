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
#include <algorithm>
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
	B_ = 10.0f;
	C_ = 1.9f;
	D_ = 1.0f;
	E_ = 0.97f;
	D_lat_ = 1.0f;

	// combined slip defaults; starting points, tune against data
	friction_limit_ = FrictionLimit::kEllipse;
	use_weighting_ = true;
	Bxa_ = 10.0f;
	Cxa_ = 1.0f;
	Byk_ = 8.0f;
	Cyk_ = 1.0f;

	// combined slip defaults for clay and sand; soft soils run at higher slip,
	// so the lateral reduction with longitudinal slip is gentler than on pavement
	Bxa_soft_ = 10.0f;
	Cxa_soft_ = 1.0f;
	Byk_soft_ = 2.0f;
	Cyk_soft_ = 1.0f;

	// soil-strength dependent lateral friction defaults (tunable, see Get*LateralCoeff)
	clay_mu_max_ = 0.9f;
	clay_N_half_ = 20.0f;
	sand_mu_max_ = 0.8f;
	sand_N_half_ = 15.0f;
	soft_mu_min_ = 0.05f;

	mr_v_reg_ = 0.2f;

	// MavsTire::CalcSlip returns (Vw - Vv)/max(|Vw|,|Vv|), clamped to [-1, 1]
	slip_convention_ = SlipConvention::kNormalizedByMax;
	sand_mr_kv_ = 0.0f;
	sand_mr_vref_ = 4.0f;
}

void MavsPacejka::SetSandMotionResistanceSpeedFactor(float k_v, float v_ref) {
	sand_mr_kv_ = std::max(0.0f, k_v);
	sand_mr_vref_ = std::max(0.1f, v_ref);
}

void MavsPacejka::SetMotionResistanceRegularization(float v_reg) {
	mr_v_reg_ = std::max(1.0e-3f, v_reg);
}

void MavsPacejka::SetSoftSoilCombinedSlipWeights(float Bxa, float Cxa, float Byk, float Cyk) {
	Bxa_soft_ = Bxa;
	Cxa_soft_ = Cxa;
	Byk_soft_ = Byk;
	Cyk_soft_ = Cyk;
	use_weighting_ = true;
}

void MavsPacejka::SetClayLateralParams(float mu_max, float N_half) {
	clay_mu_max_ = std::max(0.0f, mu_max);
	clay_N_half_ = std::max(1.0e-3f, N_half);
}

void MavsPacejka::SetSandLateralParams(float mu_max, float N_half) {
	sand_mu_max_ = std::max(0.0f, mu_max);
	sand_N_half_ = std::max(1.0e-3f, N_half);
}

void MavsPacejka::SetSoftSoilMinLateral(float mu_min) {
	soft_mu_min_ = std::max(0.0f, mu_min);
}

float MavsPacejka::GetSaturatingCoeff(float N, float mu_max, float N_half) const {
	// Hyperbolic saturation: rises with mobility number but approaches mu_max slowly,
	// so the coefficient keeps responding to soil strength over the usual CI range.
	float n = std::max(0.0f, N);
	return std::max(soft_mu_min_, mu_max * n / (n + N_half));
}

float MavsPacejka::GetClayLateralCoeff(float Nc) const {
	// Peak side force coefficient increases with mobility number on clay and sand:
	// Melzer, K.-J. (1976). Performance of Towed Wheels Operating in Turned Mode on Soft Soils - A Pilot Study.
	// Miscellaneous Paper M-76-17. U.S. Army Engineer Waterways Experiment Station,
	// Mobility and Environmental Systems Laboratory, Vicksburg, MS. August 1976. DTIC accession ADA028909.
	// The functional form and the constants clay_mu_max_ and clay_N_half_ are tuning choices,
	// not values taken from that report.
	return GetSaturatingCoeff(Nc, clay_mu_max_, clay_N_half_);
}

float MavsPacejka::GetSandNumeric(float W, float delta) const {
	// Turnage / VTI sand numeric, Mason et al. (2018) Eq 9:
	//   Ns = G*(b*d)^1.5 * delta / (W*h)
	// Dimensionless when G is in Pa/m, lengths in m, and W in N.
	// G ~= CI/3.47 (Turnage 1995) gives G in psi/in for CI in psi, i.e. the 3.47 is a
	// length in inches. In SI: G[Pa/m] = CI[Pa] / (3.47 in * 0.0254 m/in).
	float G = CI_ / (3.47f * 0.0254f);
	float Ns = (float)(G * pow(b_*d_, 1.5) * delta / (W * h_));
	return std::max(0.0f, Ns);
}

float MavsPacejka::GetSandCpCiRatio(float W, float delta) const {
	// Ratio of contact pressure to cone index, the primary variable of the DROVE-based
	// sand equations (Mason et al. 2018; Williams et al. 2019).
	// CP is the contact pressure on a rigid flat surface. It is estimated here from the
	// tire geometry: footprint length l = 2*sqrt(delta*(d - delta)) (chord of the deflected
	// tire) and width b, so CP ~= W / (b*l). This is an approximation; the real footprint
	// is somewhat smaller than b*l, which would give a slightly higher CP.
	// CP/CI is clamped to 0.05-0.9, roughly the range covered by the data.
	float defl = std::min(0.5f * d_, std::max(0.0f, delta));
	float l = 2.0f * sqrtf(defl * (d_ - defl));
	float area = b_ * l;
	float ratio = 0.9f;
	if (area > 1.0e-6f && CI_ > 0.0f) ratio = (W / area) / CI_;
	return std::min(0.9f, std::max(0.05f, ratio));
}

float MavsPacejka::GetSandMaxGrossTraction(float W, float delta) const {
	// Maximum gross traction coefficient on dry sand from the DROVE data,
	// Mason et al. (2018), Eq 6:  Max = 0.42 + 0.009 / (CP/CI)
	return 0.42f + 0.009f / GetSandCpCiRatio(W, delta);
}

float MavsPacejka::ToIstvsSlip(float slip) const {
	// Convert the caller's slip to ISTVS slip s = 1 - Vv/Vw. The two conventions agree
	// for slip >= 0. For a wheel turning slower than the vehicle, the normalized-by-max
	// convention gives (Vw - Vv)/Vv, which is the braked slip sb, and s = sb/(1 + sb).
	if (slip_convention_ == SlipConvention::kNormalizedByMax && slip < 0.0f) {
		float sb = std::max(-0.999f, slip);
		return sb / (1.0f + sb);
	}
	return slip;
}

float MavsPacejka::GetSandMotionResistance(float r, float s) const {
	// Williams, J.M., Vahedifard, F., Mason, G.L., Priddy, J.D. (2019). New algorithms for
	// predicting longitudinal motion resistance of wheels on dry sand. Journal of Defense
	// Modeling and Simulation 16(3), 283-295. Slip in the paper is in percent; here it is
	// a fraction, so s/100 in the paper is s here.
	//   Towed   (Eq 9):  MR = 0.21*CP/CI - 0.75*s  + 0.04   (s <= 0, ISTVS slip)
	//   Powered (Eq 10): MR = 0.40*CP/CI + 0.60*s  - 0.03   (s > 0)
	//   Braked  (Eq 11): MR = 0.10*CP/CI - 0.90*sb + 0.18   (sb = Vw/Vv - 1 = s/(1-s))
	// Towed and braked MR is the full net resisting force (minus the drawbar pull), so for
	// s <= 0 there is no separate gross traction term. Powered MR is gross traction minus
	// drawbar pull and is subtracted from the VTI gross traction.
	float towed0 = 0.21f * r + 0.04f;
	float mr = 0.0f;
	if (s > 0.0f) {
		// The powered data start at 5% slip. Blend linearly from the towed value at zero
		// slip to the powered value at 5% slip so the force is continuous through s = 0.
		const float s_blend = 0.05f;
		// Slip is also capped at 0.55, the largest slip in the powered data, rather than
		// extrapolating the linear slip term to a fully spinning wheel.
		float powered = 0.40f * r + 0.60f * std::min(0.55f, std::max(s, s_blend)) - 0.03f;
		if (s < s_blend) {
			float t = s / s_blend;
			mr = (1.0f - t) * towed0 + t * powered;
		}
		else {
			mr = powered;
		}
	}
	else {
		// Towed applies near zero slip, braked at large braking slip. Taking the smaller of
		// the two follows the towed line until it meets the braked curve, then the braked
		// curve, which levels off at a locked wheel (sb = -1) instead of growing without bound.
		float towed = towed0 - 0.75f * s;
		float sb = s / (1.0f - s);
		float braked = 0.10f * r - 0.90f * sb + 0.18f;
		mr = std::min(towed, braked);
	}
	return std::max(0.0f, mr);
}

// see: Turnage, G. W. (1972). Performance of Soils Under Tire Loads, Report 8: 
// Application of Test Results to Tire Selection for Off-Road Vehicles. 
// Technical Report No. 3-666. 
// U.S. Army Engineer Waterways Experiment Station, Vicksburg, MS. 
// September 1972. NTIS/DTIC AD-751 750.
// AND
// Durham, G.N., 1976. Powered Wheels in the Turned Mode Operating on Yielding Soils. 
// Technical Report M-76-9, U.S. Army Engineer Waterways Experiment Station, Vicksburg, Mississippi.
float MavsPacejka::GetSandLateralCoeff(float W, float delta) const {
	// The functional form and the constants sand_mu_max_ and sand_N_half_ are tuning
	// choices; only the sand numeric itself comes from the literature.
	return GetSaturatingCoeff(GetSandNumeric(W, delta), sand_mu_max_, sand_N_half_);
}

void MavsPacejka::SetSlipAngleCrossover(float angle) {
	// angle is in degrees
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

void MavsPacejka::SetCombinedSlipWeights(float Bxa, float Cxa, float Byk, float Cyk) {
	Bxa_ = Bxa;
	Cxa_ = Cxa;
	Byk_ = Byk;
	Cyk_ = Cyk;
	use_weighting_ = true;
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
	else { //dry tarmac
		B_ = 10.0f;
		C_ = 1.9f;
		D_ = 1.0f;
		E_ = 0.97f;
		D_lat_ = 1.0f;
	}
}

rp3d::Vector2 MavsPacejka::ApplyCombinedSlipWeighting(rp3d::Vector2 f, float slip, float alpha, bool soft_soil) const {
	// Pacejka-style combined slip weighting functions.
	// Both are even functions equal to 1 at zero cross-slip, so sign conventions don't matter.
	if (!use_weighting_) return f;
	float Bxa = soft_soil ? Bxa_soft_ : Bxa_;
	float Cxa = soft_soil ? Cxa_soft_ : Cxa_;
	float Byk = soft_soil ? Byk_soft_ : Byk_;
	float Cyk = soft_soil ? Cyk_soft_ : Cyk_;
	float Gxa = cosf(Cxa * atanf(Bxa * alpha));
	float Gyk = cosf(Cyk * atanf(Byk * slip));
	// keep the weights non-negative so a force never reverses direction
	Gxa = std::max(0.0f, Gxa);
	Gyk = std::max(0.0f, Gyk);
	return rp3d::Vector2(f.x * Gxa, f.y * Gyk);
}

rp3d::Vector2 MavsPacejka::ApplyFrictionLimit(rp3d::Vector2 f, float fx_max, float fy_max) const {
	if (friction_limit_ == FrictionLimit::kNone || fx_max <= 0.0f || fy_max <= 0.0f) return f;
	if (friction_limit_ == FrictionLimit::kEllipse) {
		float u = f.x / fx_max;
		float v = f.y / fy_max;
		float r2 = u * u + v * v;
		if (r2 > 1.0f) {
			float s = 1.0f / sqrtf(r2);
			f.x *= s;
			f.y *= s;
		}
	}
	else { // longitudinal priority
		f.x = std::max(-fx_max, std::min(fx_max, f.x));
		float u = f.x / fx_max;
		float fy_avail = fy_max * sqrtf(std::max(0.0f, 1.0f - u * u));
		f.y = std::max(-fy_avail, std::min(fy_avail, f.y));
	}
	return f;
}

rp3d::Vector2 MavsPacejka::GetCombinedTraction(float W, float delta, float slip, float alpha) {
	// Legacy interface: the direction of travel is unknown, so motion resistance is
	// assumed to oppose forward travel at full strength. Use the overload with
	// long_vel on clay and sand.
	return ComputeTraction(W, delta, slip, alpha, 1.0f, 0.0f);
}

rp3d::Vector2 MavsPacejka::GetCombinedTraction(float W, float delta, float slip, float alpha, float long_vel) {
	// Motion resistance opposes the direction of travel. Below mr_v_reg_ it is ramped
	// linearly to zero so it cannot push a stationary vehicle or cause chatter.
	float mr_dir = std::min(1.0f, std::max(-1.0f, long_vel / mr_v_reg_));
	return ComputeTraction(W, delta, slip, alpha, mr_dir, fabs(long_vel));
}

rp3d::Vector2 MavsPacejka::ComputeTraction(float W, float delta, float slip, float alpha, float mr_dir, float speed) {
	rp3d::Vector2 f(0.0f, 0.0f);
	float fx_max = 0.0f;
	float fy_max = 0.0f;
	// Motion resistance force on soft soil, kept separate from the tractive force so the
	// combined-slip weighting (which models loss of grip) does not reduce it.
	float mr_force = 0.0f;
	// Most negative net force (along the direction of travel) a driving sand wheel may
	// produce: that of the same wheel rolling freely (towed). Zero means no floor.
	float sand_drive_floor = 0.0f;

	if (W <= 0.0f) return f;

	if (surface_type_ == "clay") {
		// Equations 1-8 from: 
		// Improving accuracy of vehicle-terrain interface algorithms for wheeled
		// vehicles on fine - grained soils through Bayesian calibration
		// guard against deflection >= section height, which would make pow() return NaN
		float defl_ratio = std::min(0.99f, std::max(0.0f, delta / h_));
		float Nc = (float)(CI_*b_*d_ / (W*pow(1.0 - defl_ratio, 1.5) * pow(1.0f + b_ / d_, 0.75)));
		float Ssp = (float)(21.0 / pow(Nc, 2.5));
		float mr = 12.0f / (Nc*Nc) + 0.007f;
		// Gross traction: the log term of the traction equation, before motion resistance.
		// It is undefined for slip <= 0 and negative for slip < Ssp, so it is evaluated
		// with |slip|, floored at zero, and mirrored for braking.
		float abs_slip = fabs(slip);
		float gross = 0.0f;
		if (abs_slip > 1.0e-6f) {
			gross = (float)(0.5f*log10(abs_slip / Ssp) * pow(1.0f + b_ / d_, 0.25f));
			gross = std::min(1.0f, std::max(0.0f, gross));
			if (slip < 0.0f) gross = -gross;
		}
		// Net traction = gross - MR. MR always acts, including when coasting at zero slip
		// and when gross < MR, so a vehicle that cannot overcome MR decelerates and sticks.
		mr_force = -W * mr * mr_dir;
		float mu_lat = GetClayLateralCoeff(Nc);
		float fy = -W * mu_lat * GetCrolla(alpha, high_slip_angle_cutoff_radians_);
		f = rp3d::Vector2(W * gross, fy);
		fx_max = W;
		fy_max = W * mu_lat;
	}
	else if (surface_type_ == "sand") {
		float Ns = GetSandNumeric(W, delta);
		// VTI gross traction equation, as given in Mason et al. (2018),
		// "A unified equation for predicting traction for wheels on sand over a range of
		// braked, towed, and powered operations", Eqs 9-10:
		//   T/W = X - X*Y/(Ns + 10 + Y),  X = 0.66,  Y = 4.71 + 1.72/slip
		// Used for driving wheels only. For towed and braked wheels the Williams et al.
		// motion resistance below is the full net force, so the gross term is zero.
		// The sand relations depend on the operating mode relative to the direction of
		// travel. MavsTire::CalcSlip gives slip the sign of the force the wheel pushes with,
		// so in reverse, driving slip is negative and braking slip is positive. The wheel is
		// driving when slip and travel direction agree; at standstill (mr_dir == 0) any
		// nonzero slip means the wheel is spinning up, i.e. driving.
		float abs_slip = fabs(slip);
		bool driving = (fabs(mr_dir) < 1.0e-3f) ? (abs_slip > 1.0e-6f) : (slip * mr_dir > 0.0f);
		float gross = 0.0f;
		if (driving && abs_slip > 1.0e-6f) {
			float X = 0.66f;
			float Y = 4.71f + 1.72f / abs_slip;
			gross = X - X * Y / (Ns + 10.0f + Y);
			// The VTI equation keeps rising past the 50% slip covered by the data and can
			// approach 0.66, which over-predicts slope climbing. Cap it at the measured
			// maximum (Mason et al. 2018, Eq 6).
			gross = std::min(gross, GetSandMaxGrossTraction(W, delta));
			// gross traction acts in the direction the wheel pushes
			if (slip < 0.0f) gross = -gross;
		}
		// Motion resistance from Williams et al. (2019), evaluated with the slip in the
		// travel frame (positive = powered, negative = towed/braked), opposing the motion,
		// with an optional speed factor.
		float s_mode = driving ? abs_slip : -abs_slip;
		float mr = GetSandMotionResistance(GetSandCpCiRatio(W, delta), ToIstvsSlip(s_mode));
		if (sand_mr_kv_ > 0.0f) {
			float vr = speed / sand_mr_vref_;
			mr *= 1.0f + sand_mr_kv_ * vr * vr;
		}
		mr_force = -W * mr * mr_dir;
		if (driving && fabs(mr_dir) >= 1.0e-3f) {
			float towed0 = GetSandMotionResistance(GetSandCpCiRatio(W, delta), 0.0f);
			if (sand_mr_kv_ > 0.0f) {
				float vr = speed / sand_mr_vref_;
				towed0 *= 1.0f + sand_mr_kv_ * vr * vr;
			}
			sand_drive_floor = W * towed0 * fabs(mr_dir);
		}
		float mu_lat = GetSandLateralCoeff(W, delta);
		float fy = mu_lat * GetCrolla(alpha, high_slip_angle_cutoff_radians_);
		f = rp3d::Vector2(W*gross, -W*fy);
		fx_max = W;
		fy_max = W * mu_lat;
	}
	else { // dry, wet, ice, snow
		//adjust coeffs
		SetCoeffs(W, delta);

		// longitudinal
		float Bs = B_ * slip;
		float fx = W * D_ * sin(C_*atan(Bs - E_ * (Bs - atan(Bs))));

		// lateral
		float fy = -W * D_lat_ * GetCrolla(alpha, high_slip_angle_cutoff_radians_);
		f = rp3d::Vector2(fx, fy);
		fx_max = W * D_;
		fy_max = W * D_lat_;
	}

	// combined slip: soft reduction, then hard cap at the friction ellipse
	bool soft_soil = (surface_type_ == "clay" || surface_type_ == "sand");
	f = ApplyCombinedSlipWeighting(f, slip, alpha, soft_soil);
	if (soft_soil) {
		f.x = std::min(W, std::max(-W, f.x + mr_force));
		// A driven wheel must not resist motion more than the same wheel rolling freely.
		// Without this floor, a spinning wheel that is also cornering (where the combined-
		// slip weighting cuts its gross traction but not its slip-dependent motion
		// resistance) acts as a brake; on a steered wheel that braking force points
		// across the turn and can reverse the yaw. The DROVE powered data also have
		// positive drawbar pull throughout (Williams et al. 2019, Table 3).
		if (sand_drive_floor > 0.0f) {
			float dir = (mr_dir > 0.0f) ? 1.0f : -1.0f;
			if (f.x * dir < -sand_drive_floor) f.x = -sand_drive_floor * dir;
		}
	}
	f = ApplyFrictionLimit(f, fx_max, fy_max);
	return f;
}


} // namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs
