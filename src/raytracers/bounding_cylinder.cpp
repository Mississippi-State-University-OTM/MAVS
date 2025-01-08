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
#include <raytracers/bounding_cylinder.h>
#include <limits>

namespace mavs {
namespace raytracer {

BoundingCylinder::BoundingCylinder() {
	float max = std::numeric_limits<float>::max();
	float min = std::numeric_limits<float>::min();
	base_center_ = glm::vec3(0.0f, 0.0f, min);
	radius_ = max;
	height_ = max;
	center_ = glm::vec3(0.0f,0.0f,0.0f);
}

BoundingCylinder::BoundingCylinder(glm::vec3 base_center, float radius, float height) {
	base_center_ = base_center;
	radius_ = radius;
	height_ = height;
	center_ = base_center_;
	center_.z += 0.5f*height;
}

BoundingCylinder::BoundingCylinder(float bsx, float bsy, float bsz,
	float radius, float height) {
	base_center_ = glm::vec3(bsx, bsy, bsz);
	radius_ = radius;
	height_ = height;
	center_ = base_center_;
	center_.z += 0.5f*height;
}

void BoundingCylinder::Transform(glm::mat3x4 aff_rot) {
	float old_height = height_;
	glm::vec4 vc(center_.x, center_.y, center_.z, 1.0f);
	glm::vec3 vcprime = aff_rot * vc;
	center_ = vcprime;

	glm::vec4 vbc(base_center_.x, base_center_.y, base_center_.z, 1.0f);
	glm::vec3 vbcprime = aff_rot * vbc;
	base_center_ = vbcprime;

	height_ = 2.0f*(center_.z - base_center_.z);
	float s = height_ / old_height;
	radius_ = s * radius_;
}

} //namespace raytracer 
} //namespace mavs