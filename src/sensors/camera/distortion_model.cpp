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
#include <sensors/camera/distortion_model.h>

#include <iostream>

namespace mavs {
namespace sensor {
namespace camera {

DistortionModel::DistortionModel() {
	kc_.resize(5, 0.0f);
	cc_ = glm::vec2(0.0f, 0.0f);
	fc_ = glm::vec2(0.0f, 0.0f);
	alpha_c_ = 0.0f;
	fnom_ = 1.0;
	hscale_ = 1.0;
	vscale_ = 1.0;
}

DistortionModel::~DistortionModel() {

}

DistortionModel::DistortionModel(const DistortionModel &dm) {
	kc_ = dm.kc_;
	cc_ = dm.cc_;
	fc_ = dm.fc_;
	alpha_c_ = dm.alpha_c_;
}

void DistortionModel::SetDistortionParameters(glm::vec2 cc, 
	glm::vec2 fc, float alpha_c, std::vector<float> kc) {
	kc_ = kc;
	alpha_c_ = alpha_c;
	fc_ = fc;
	cc_ = cc;
	hscale_ = fnom_ / fc_.x;
	vscale_ = fnom_ / fc_.y;
}

void DistortionModel::SetNominalFocalLength(float f) {
	fnom_ = f;	
	hscale_ = fnom_ / fc_.x;
	vscale_ = fnom_ / fc_.y;
}

glm::vec2 DistortionModel::PixelToMeters(glm::vec2 pixel) {
	glm::vec2 xm(hscale_*(pixel.x - cc_.x),vscale_*(cc_.y - pixel.y));
	return xm;
}

glm::vec2 DistortionModel::MetersToPixels(glm::vec2 pixel){
	glm::vec2 xp((pixel.x/hscale_) - cc_.x, cc_.y-(pixel.y/vscale_));
	return xp;
}

glm::vec2 DistortionModel::Distort(glm::vec2 x_n) {
	// see http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
	float x = x_n.x;
	float y = x_n.y;
	float xy = x * y;
	float x2 = x * x;
	float y2 = y * y;
	float r2 = x2 + y2;
	float r4 = r2 * r2;
	float r6 = r4 * r2;
	glm::vec2 dx(2 * kc_[2] * xy + kc_[3] * (r2 + 2 *x2),
		kc_[2] * (r2 + 2 * y2) + 2 * kc_[3] * xy);	
	float s = 1.0f + kc_[0]*r2 + kc_[1]*r4 + kc_[4]*r6;
	glm::vec2 xd = s * x_n + dx;
	glm::vec2 xp(fc_[0]*(xd.x + alpha_c_*xd.y) + cc_[0],
		fc_[1] * xd[1] + cc_[1]);
	return xp;
}

glm::vec2 DistortionModel::Undistort(glm::vec2 x_kk) {
	//see normalize.m and comp_distortion_oulu.m 
	// in the Camera Calibration toolbox
	// output is a "normalized" location in the image plane

	glm::vec2 xd((x_kk[0] - cc_[0]) / fc_[0],
		(x_kk[1] - cc_[1]) / fc_[1]);
	xd[0] = xd[0] - alpha_c_ * xd[1];

	glm::vec2 x = xd;
	
	if (kc_[0] != 0.0f || kc_[1] != 0.0f || kc_[2] != 0.0f
		|| kc_[3] != 0.0f || kc_[4] != 0.0f) {
		float k1 = kc_[0];
		float k2 = kc_[1];
		float k3 = kc_[4];
		float p1 = kc_[2];
		float p2 = kc_[3];
		int num_iter = 0;
		float dx = 1.0;
		//error threshold in pixels
		float thresh = 0.01f;
		while (num_iter<20 && dx>thresh) {
			float r_2 = x[0]*x[0] + x[1]*x[1];
			float r_4 = r_2 * r_2;
			float k_radial = 1.0f + k1 * r_2 + k2 * r_4 + k3 * r_2*r_4;
			float x0x1 = x[0] * x[1];
			glm::vec2 delta_x (2 * p1*x0x1 + p2 * (r_2 + 2 * x[0]*x[0] ),
			p1 * (r_2 + 2 * x[1]*x[1]) + 2 * p2*x0x1);
			x = (xd - delta_x) /(k_radial);
			dx = glm::length(x);
			num_iter++;
		}
	}

	return x;
}

} //namespace camera
} // namespace sensor
} //namespace mavs