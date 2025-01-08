#include <math.h>
#include <random>
#include "mavs_core/math/curve_fitter.h"
#include "mavs_core/plotting/mavs_plotting.h"

static const double clog2 = log(2.0);

static double Fresnel(double x) {
	// (1-x)^5.
	double d = 1.0 - x;
	double d2 = d * d;
	return d2 * d2 * d;
}

static double FresnelApprox(double x, std::vector<double> p) {
	// An approximation of (1-x)^5. on the range [0,1]
	return pow(2, p[0] * x * x + p[1] * x);
}

static std::vector<double> FresnelApproxJacob(double x, std::vector<double> p){
	std::vector<double> j = p;
	j[0] = clog2 * x * x * pow(2, p[0] * x * x + p[1] * x);
	j[1] = clog2 * x * pow(2, p[0] * x * x + p[1] * x);
	return j;
}

int main(int argc, char *argv[]) {

	mavs::math::curve_fit::CurveFitter solver;

	solver.SetRmseThreshold(0.0025);
	// Case 1: Find  parameters of an approximating function
	std::vector<double> x_data, y_data;
	for (double x = 0.0; x <= 1.0; x += (1.0 / 1000.0)) {
		x_data.push_back(x);
		y_data.push_back(Fresnel(x));
	}
	std::vector<double> p_guess{ 0.0, 0.0 };  // 2 parameter model
		
	std::vector<double> params = solver.Solve(&FresnelApprox, &FresnelApproxJacob, x_data, y_data, p_guess);
	solver.PrintResults();

	solver.Reset();
	solver.SetMaxIterations(10000);
	solver.SetRmseThreshold(0.0025);
	// Case 3: Fit a noisy 6th-order polynomial
	x_data.clear();
	y_data.clear();
	std::vector<double> in_params { 3.0,2.0,-5.0, -0.1, 14.3, -27.8, 0.75 };
	for (double x = 0.0; x <= 6.3; x += (1.0 / 35.0)) {
		x_data.push_back(x);
		double yoff = 1.5 * (((double)rand() / (RAND_MAX)) - 0.5);
		double y = mavs::math::curve_fit::Polynomial(x, in_params) + yoff;
		y_data.push_back(y);
	}
	p_guess = std::vector<double>{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
	std::vector<double> params_out = solver.Solve(&mavs::math::curve_fit::Polynomial, &mavs::math::curve_fit::PolynomialJacobian, x_data, y_data, p_guess);
	solver.PrintResults();
	
	return 0;
}