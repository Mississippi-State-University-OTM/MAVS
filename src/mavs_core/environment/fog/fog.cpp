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
#include <mavs_core/environment/fog/fog.h>
#include <time.h>
#include <iostream>

namespace mavs {
namespace environment {

Fog::Fog() {
	fog_spatial_lo_.SetFrequency(0.02); //Wavelength = 50.0
	fog_spatial_hi_.SetFrequency(0.2);
	int curr_time = (int)time(NULL);
	fog_spatial_lo_.SetSeed(curr_time);
	fog_spatial_hi_.SetSeed(curr_time+1);
	k_ = 0.0f;
}

float Fog::GetK(glm::vec3 &p0, glm::vec3 &p1) {
	float k = 0.0f;
	float t = 0.0f;
	float dt = 0.1f;
	while (t <= 1.0f) {
		float tp = 1.0f - t;
		glm::vec3 p = tp * p0 + t * p1;
		float f_lo = (float)fog_spatial_lo_.GetPerlin(p.x, p.y, p.z);
		if (f_lo > 0.0f) {
			k += ((float)(f_lo + 0.5f*(1.0f + fog_spatial_hi_.GetPerlin(p.x, p.y, p.z))))*exp(-0.1f*p.z);
		}
		t += dt;
	}
	k = k_*0.5f*k*dt;
	return k;
}

} // namespace environment
} // namespace mavs
