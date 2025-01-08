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
#include <sensors/imu/mems_sensor.h>

#include <stdlib.h>
#include <math.h>

#include <algorithm>

namespace mavs {
namespace sensor {
namespace imu {

void MemsSensor::SetAxisMisalignment(glm::vec3 misalign) {
	axis_misalignment_ = misalign;
	alignment_matrix_[0][0] = 1.0f;
	alignment_matrix_[0][1] = misalign.y / 100.0f;
	alignment_matrix_[0][2] = misalign.z / 100.0f;
	alignment_matrix_[1][0] = misalign.x / 100.0f;
	alignment_matrix_[1][1] = 1.0f;
	alignment_matrix_[1][2] = misalign.z / 100.0f;
	alignment_matrix_[2][0] = misalign.x / 100.0f;
	alignment_matrix_[2][1] = misalign.y / 100.0f;
	alignment_matrix_[2][2] = 1.0f;
}

float MemsSensor::Filter1(float z, float sample_rate) {
	float h = 1.0f / (1.0f + ((2.0f/sample_rate)-1.0f)*(1.0f / z));
	return h;
}

float MemsSensor::Filter2(float z) {
	float h = 1.0f / (1.0f - (1.0f / z));
	return h;
}



glm::vec3 MemsSensor::Update(glm::vec3 input, float temperature, float sample_rate) {

	glm::vec3 output(0.0f, 0.0f, 0.0f);

	glm::vec3 b = alignment_matrix_ * input + constant_bias_;

	float w1 = ((float)rand() / (RAND_MAX)) + 1; //0 to 1
	float w = 2.0f*(w1 - 0.5f); //-1 to 1

	//float h1 = 1.0f; // filter?
	glm::vec3 beta1 = w * bias_instability_;
	for (int i = 0; i < 3; i++) {
		beta1[i] = Filter1(beta1[i], sample_rate);
	}

	glm::vec3 beta2 = (w*(float)sqrt(0.5f*sample_rate))*noise_density_;

	//float h2 = 1.0f; //filter?
	glm::vec3 beta3 = (float)(w / sqrt(0.5f*sample_rate))*random_walk_;
	for (int i = 0; i < 3; i++) {
		beta3[i] = Filter2(beta3[i]);
	}
	glm::vec3 env_noise = (temperature - 25.0f)*temperature_bias_;

	glm::vec3 c = beta1 + beta2 + beta3 + env_noise + b;

	glm::vec3 one(1.0f, 1.0f, 1.0f);
	glm::vec3 scale_factor_error = one + ((temperature - 25.0f) / 100.0f)*temperature_scale_factor_;

	glm::vec3 d(c.x * scale_factor_error.x, c.y * scale_factor_error.y, c.z * scale_factor_error.z);
	glm::vec3 e;
	for (int i = 0; i < 3; i++) {
		e[i] = std::max(std::min(measurement_range_, d[i]), -measurement_range_);
		output[i] = resolution_ * round(e[i] / resolution_);
	}
	return output;
}

} //namespace imu
} //namespace sensor
} //namespace mavs