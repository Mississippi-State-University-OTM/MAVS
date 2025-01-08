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

#include <raytracers/fresnel.h>

namespace mavs{
namespace raytracer{

// The indices of refraction should be complex

float GetFresnelTransmissionAngle(float n1, float n2, float theta_i) {
	float theta_t = (float)(asin((n1/n2)*sin(theta_i)));
	return theta_t;
}

float GetFresnelReflectance(float n1, float n2, float theta_i, float theta_t) {
	glm::vec2 r = GetPolarizedFresnelReflectance(n1, n2, theta_i, theta_t);
	float refl = 0.5f*(r.x + r.y);
	return refl;
}

glm::vec2 GetPolarizedFresnelReflectance(float n1, float n2, float theta_i, float theta_t) {
	float ci = cos(theta_i);
	float ct = cos(theta_t);
	float n1ci = n1 * ci;
	float n2ct = n2 * ct;
	float n1ct = n1 * ct;
	float n2ci = n2 * ci;
	float rs = (float)pow((n1ci - n2ct) / (n1ci + n2ct), 2.0f);
	float rp = (float)pow((n1ct - n2ci) / (n1ct + n2ci), 2.0f);
	glm::vec2 r(rs, rp);
	return r;
}

glm::vec4 GetFresnelCoeffs(float n1, float n2, float theta_i, float theta_t) {
	glm::vec2 c_r = GetPolarizedFresnelReflectance(n1, n2, theta_i, theta_t);
	glm::vec4 coeffs(c_r.x, c_r.y, 1.0f - c_r.x, 1.0f - c_r.y);
	return coeffs;
}


} //namespace raytracer
} //namespace mavs
