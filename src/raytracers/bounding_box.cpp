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
#include <raytracers/bounding_box.h>

#include <limits>
#include <iostream>

namespace mavs {
namespace raytracer {

void BoundingBox::CalcPoints() {
	center_ = 0.5f*(upper_right_ + lower_left_);
	corners_.resize(8);
	corners_[0] = lower_left_;
	corners_[1] = glm::vec3(lower_left_.x, upper_right_.y, lower_left_.z);
	corners_[2] = glm::vec3(upper_right_.x, upper_right_.y, lower_left_.z);
	corners_[3] = glm::vec3(upper_right_.x, lower_left_.y, lower_left_.z);
	corners_[4] = glm::vec3(lower_left_.x, lower_left_.y, upper_right_.z);
	corners_[5] = glm::vec3(lower_left_.x, upper_right_.y, upper_right_.z);
	corners_[6] = upper_right_; 
	corners_[7] = glm::vec3(upper_right_.x, lower_left_.y, upper_right_.z);
}

BoundingBox::BoundingBox() {
	float max = std::numeric_limits<float>::max();
	float min = std::numeric_limits<float>::lowest();
	lower_left_ = glm::vec3(min, min, min);
	upper_right_ = glm::vec3(max, max, max);
	CalcPoints();
}

BoundingBox::BoundingBox(glm::vec3 ll, glm::vec3 ur) {
	lower_left_ = ll;
	upper_right_ = ur;
	CalcPoints();
}

BoundingBox::BoundingBox(float llx, float lly, float llz,
		float urx, float ury, float urz) {
	lower_left_ = glm::vec3(llx, lly, llz);
	upper_right_ = glm::vec3(urx, ury, urz);
	CalcPoints();
}

void BoundingBox::Transform(glm::mat3x4 aff_rot) {
	//Create translation vector and rotation/scale matrix
	glm::vec3 T(aff_rot[0][3], aff_rot[1][3], aff_rot[2][3]);
	glm::mat3x3 R;
	for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) { R[i][j] = aff_rot[i][j]; } }
	double xmag = sqrt(aff_rot[0][0] * aff_rot[0][0] + aff_rot[1][0] * aff_rot[1][0] + aff_rot[2][0] * aff_rot[2][0]);
	double ymag = sqrt(aff_rot[0][1] * aff_rot[0][1] + aff_rot[1][1] * aff_rot[1][1] + aff_rot[2][1] * aff_rot[2][1]);
	double zmag = sqrt(aff_rot[0][2] * aff_rot[0][2] + aff_rot[1][2] * aff_rot[1][2] + aff_rot[2][2] * aff_rot[2][2]);
	center_.x *= (float)xmag;
	center_.y *= (float)ymag;
	center_.z *= (float)zmag;
	//Rotate the corners
	for (int p = 0; p < (int)corners_.size(); p++) {
		glm::vec3 v = corners_[p] - center_;
		glm::vec3 vprime = R * v;
		corners_[p] = vprime + T + center_;
	}

	//find the new lower left and upper right
	float max = std::numeric_limits<float>::max();
	float min = std::numeric_limits<float>::lowest();
	float xll = max; float yll = max; float zll = max;
	float xur = min; float yur = min; float zur = min;
	for (int p = 0; p < (int)corners_.size(); p++) {
		if (corners_[p].x < xll) xll = corners_[p].x;
		if (corners_[p].y < yll) yll = corners_[p].y;
		if (corners_[p].z < zll) zll = corners_[p].z;
		if (corners_[p].x > xur) xur = corners_[p].x;
		if (corners_[p].y > yur) yur = corners_[p].y;
		if (corners_[p].z > zur) zur = corners_[p].z;
	}
	lower_left_ = glm::vec3(xll, yll, zll);
	upper_right_ = glm::vec3(xur, yur, zur);

	//Recalcuate the corners
	CalcPoints();

	/*glm::vec3 vll = lower_left_ - center_;
	glm::vec3 vllprime = R * vll;
	lower_left_ = vllprime + T + center_;
	//glm::vec4 vur(upper_right_.x, upper_right_.y, upper_right_.z, 1.0f);
	glm::vec3 vur = upper_right_ - center_;
	glm::vec3 vurprime = aff_rot * vur;
	upper_right_ = vurprime+T+center_;*/

}

void BoundingBox::Print() {
	std::cout << lower_left_.x << " " << lower_left_.y << " " << lower_left_.z << " " << upper_right_.x << " " << upper_right_.y << " " << upper_right_.z << std::endl;
}

} //namespace raytracer 
} //namespace mavs