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
#include "vti/nicolas_comstock.h"
#include <math.h>
#include <algorithm>

/** Based on the papers :
* Brach, R., & Brach, M. (2011). 
* The tire-force ellipse (friction ellipse) and tire characteristics 
* (No. 2011-01-0094). SAE Technical Paper.
*
* Brach, R. M., & Brach, R. M. (2009). 
* Tire models for vehicle dynamic simulation and accident reconstruction 
* (No. 2009-01-0102). SAE Technical Paper.
*/

namespace mavs {
namespace vti {

NicolasComstock::NicolasComstock() {
	C_a_ = 66723.0; //Newtons/radian
	C_s_ = 44482.0; //Newtons
	C_a_2_ = C_a_ * C_a_;
	C_s_2_ = C_s_ * C_s_;
}

TireForces NicolasComstock::GetForces(double pure_longi, double pure_lat, double s, double a) {
	//s = slip (-1 to 1)
	//a = steering angle (radians)
	//Equations 3 and 4
	TireForces forces;
	double s2 = s * s;
	double ta = tan(fabs(a));
	double sa = sin(fabs(a));
	double sa2 = sa * sa;
	double ca = cos(a);
	double ca2 = ca * ca;
	double fx = pure_longi;
	double fy = pure_lat;
	double fx2 = fx * fx;
	double fy2 = fy * fy;
	double ta2 = ta * ta;
	double denom = sqrt(s2*fy2 + fx2 * ta2);
	double fac = pow(1.0 - fabs(s), 2);
	forces.longitudinal = (fx*fabs(fy*s) / denom)*(sqrt(s2*C_a_2_ + fac * ca2*a*fx2) / (fabs(s)*C_a_));
	forces.lateral = (fy*fabs(fx*ta) / denom)*(sqrt(fac*ca2*fy2 + sa2 * C_s_2_) / (C_s_*fabs(sa)));
	return forces;
}

} //namespace vti
} //namespace mavs