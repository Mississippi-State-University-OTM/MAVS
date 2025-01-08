/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
/**
 * \class CurveFitter
 * 
 * A class that allows the fit of arbitrary curves using the Gauss-Newton methd.
 * See: https://gist.githubusercontent.com/Erkaman/0ecc19026b9bca59e425074c1497f5da/raw/f53ed0d7e34558c24d31cd7f2c99dffd461bb7ac/gauss_newton_algorithm.cpp
 * 
 * Uses function pointers. You must define a function of the form:
 * 
 * double FunctionToFit(double x, std::vector<double> p)
 * 
 * where x is the input variable and p is a vector of the function parameters to be fit. 
 * You must also define the jacobian of your function, which is of the form:
 * 
 * std::vector<double> FunctionToFitJacobian(double x, std::vector<double> p)
 * 
 * where x is the input variable, p is the vector of functon parameters, and the return value
 * is a set of partial derivatives with respect to each parameter, evaluated at x
 * See "Polynomial" and "PolynomialJacobian" below as an example.
 *
 * \author Chris Goodin
 *
 * \date 10/9/2024
 */
#ifndef MAVS_CURVE_FITTER_H_
#define MAVS_CURVE_FITTER_H_
// c includes
#include <math.h>
 // c++ includes
#include <vector>
#include <string>
// project includes
#include "mavs_core/math/matrix.h"

namespace mavs {
namespace math {
namespace curve_fit {

class CurveFitter {
public:
	/// Create a curve fitter object
	CurveFitter();

	/// Reset all the parameters of the curve fitter, should be done between "Solve" calls
	void Reset();

	/**
	* Fits the function "func", which has jacobian "jac" to parameters "p_init" using data "x" and "y"
	* \param func The input function to fit, of the form double FunctionToFit(double x, std::vector<double> p). Returns the function evaluated at x
	* \param jac Jacobian of the input function, of the form std::vector<double> FunctionToFitJacobian(double x, std::vector<double> p). Returns the partial derivatives of the function with respect to each parameter, evaluated at x
	* \param x Vector list of the x-values of the data to fit
	* \param y Vector list of the corresponding y-values of the data to fit
	* \param p_init Initial guess for the fit parameters. Must be the same length as the total number of fit parameters
	**/
	std::vector<double> Solve(double(*func)(double, std::vector<double>), std::vector<double>(*jac)(double, std::vector<double>), std::vector<double> x, std::vector<double> y, std::vector<double> p_init);

	/// Print the parameter values after a fit to standard out
	void PrintResults() const;

	/// Print the x,y, and y_fit data to standard out, in columns
	void PrintFit() const;

	/**
	* Set the convergence threshold, default is 0.001
	* \param rmse_thresh The desired convergence threshold
	**/
	void SetRmseThreshold(double rmse_thresh) { rmse_threshold_ = rmse_thresh; }

	/// Return the current value of the convergence threshold
	double GetRmseThreshold() const { return rmse_threshold_; }

	/**
	* Set the maximum number of iterations the fit will run. Default is 1000
	* \param max_iter The desired max number of iterations
	**/
	void SetMaxIterations(int max_iter) { max_iterations_ = max_iter; }

	/// Return the current value of the convergence threshold
	int GetMaxIterations() const { return max_iterations_; }

	/**
	* Turn on/off warning printing to screen
	* \param show_warnings Set to true to turn warnings on
	**/
	void SetShowWarnings(bool show_warnings) { show_warnings_ = show_warnings; }

	/// Return true if warnings are being shown
	bool GetShowWarnings() const { return show_warnings_; }

	/// Return the value of the RMSE for the most recent fit
	double GetRmse() const { return rmse_last_; }

	/// Return the number of iterations for the most recent fit
	int GetNumIterations() const { return num_iter_; }

	/// Return the value of the fitted parameters for the most recent fit
	std::vector<double> GetParams() const { return params_; }

	/// Return the y-values of the fit function corresponding to the input x data
	std::vector<double> GetYFit() { return y_fit_; }

private:

	int num_params_; // number of parameters in the model

	std::vector<double> params_; // current solution for the parameters

	int num_samples_;

	double rmse_threshold_;

	int max_iterations_;

	int num_iter_;

	double rmse_last_;

	bool show_warnings_;

	int warnings_;

	std::vector<double> x_in_, y_in_, y_fit_;

	void NonConvergeWarning();

	void MaxIterationWarning();

	void Init();

};

struct Sample {
	Sample() {
		x = 0.0;
		y = 0.0;
	}
	Sample(double _x, double _y) {
		x = _x;
		y = _y;
	}
	double x;
	double y;
};

inline double Polynomial(double x, std::vector<double> p) {
	double y = 0.0;
	int np = (int)p.size();
	for (int i = 0; i < np; i++) y += p[i] * pow(x, i);
	return y;
}

inline std::vector<double> PolynomialJacobian(double x, std::vector<double> p) {
	std::vector<double> j = p;
	int np = (int)p.size();
	for (int i = 0; i < np; i++)j[i] = pow(x, i);
	return j;
}

} // namespace curve_fit
} //namespace math
} // namespace mavs

#endif // include guard