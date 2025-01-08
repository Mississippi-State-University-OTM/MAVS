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
#include <mavs_core/math/segment.h>

namespace mavs {
namespace math {

// see https://gamedev.stackexchange.com/questions/111100/intersection-of-a-line-and-a-rectangle
bool SegSegIntersect(glm::vec2 ps1, glm::vec2 pe1, glm::vec2 ps2, glm::vec2 pe2, glm::vec2 &inter_point){
	// Get A,B of first line - points : ps1 to pe1
	float A1 = pe1.y - ps1.y;
	float B1 = ps1.x - pe1.x;
	// Get A,B of second line - points : ps2 to pe2
	float A2 = pe2.y - ps2.y;
	float B2 = ps2.x - pe2.x;

	// Get delta and check if the lines are parallel
	float delta = A1 * B2 - A2 * B1;
	if (delta == 0) return false;

	// Get C of first and second lines
	float C2 = A2 * ps2.x + B2 * ps2.y;
	float C1 = A1 * ps1.x + B1 * ps1.y;
	//invert delta to make division cheaper
	float invdelta = 1.0f / delta;
	// now return the glm::vec2 intersection point
	inter_point = glm::vec2((B2*C1 - B1 * C2)*invdelta, (A1*C2 - A2 * C1)*invdelta);
	return true;
}

bool SegBoxIntersect(glm::vec2 p1, glm::vec2 p2, glm::vec2 r1, glm::vec2 r2, glm::vec2 r3, glm::vec2 r4, glm::vec2 &inter_point){
	bool intersection = SegSegIntersect(p1, p2, r1, r2, inter_point);
	if (!intersection) intersection = SegSegIntersect(p1, p2, r2, r3, inter_point);
	if (!intersection) intersection = SegSegIntersect(p1, p2, r3, r4, inter_point);
	if (!intersection) intersection = SegSegIntersect(p1, p2, r4, r1, inter_point);
	return intersection;
}

bool SegAABBIntersect(glm::vec2 p1, glm::vec2 p2, glm::vec2 ll, glm::vec2 ur) {
	// first check the simple cases
	if (p1.x > ur.x && p2.x > ur.x) return false;
	if (p1.y > ur.y && p2.y > ur.y) return false;
	if (p1.x < ll.x && p2.x < ll.x) return false;
	if (p1.y < ll.y && p2.y < ll.y) return false;
	//check if p1 is inside the box
	if (p1.x >= ll.x && p1.x <= ur.x && p1.y >= ll.y && p1.y <= ur.y) return true;
	//check if p2 is inside the box
	if (p2.x >= ll.x && p2.x <= ur.x && p2.y >= ll.y && p2.y <= ur.y) return true;
	glm::vec2 r2(ll.x, ur.y);
	glm::vec2 r4(ur.x, ll.y);
	glm::vec2 inter_point;
	return SegBoxIntersect(p1, p2, ll, r2, ur, r4, inter_point);
}

float RaySegmentIntersection(glm::vec2 origin, glm::vec2 direction, glm::vec2 p1, glm::vec2 p2) {
	glm::vec2 v = p2 - p1;
	if (direction.x == 0.0f) {
		if (v.x == 0.0f) {
			//both lines are vertical
			if (p1.x == origin.x) {
				//they overlap
				if (direction.y < 0.0f) {
					// this is not correct
					// need to check where ray origin is related to points
					return -1.0f;
				}
				else if (direction.y > 0.0f) {
					// this is not correct
					// need to check where ray origin is related to points
					return -1.0f;
				}
			}
			else {
				return -1.0f;
			}
		}
		else {
			float m2 = (v.y) / (v.x);
			float b2 = p1.y - m2 * p1.x;
			glm::vec2 inter(origin.x, m2*origin.x + b2);
			return glm::length(origin - inter);
		}
	}
	else if (v.x == 0.0) {
		float m1 = direction.y / direction.x;
		float b1 = origin.y - m1 * origin.x;
		glm::vec2 inter(p1.x, m1*p1.x + b1);
		return glm::length(origin - inter);
	}
	else {
		float m1 = direction.y / direction.x;
		float m2 = (v.y) / (v.x);
		if (m2 == m1) { //lines are parallel
			return -1.0f;
		}
		float b1 = origin.y - m1 * origin.x;
		float b2 = p1.y - m2 * p1.x;

		glm::vec2 inter;
		inter.x = (b2 - b1) / (m1 - m2);
		inter.y = m1 * inter.x + b1;
		float d = glm::length(inter - p1) / glm::length(v);
		if (d >= 0.0f && d <= 1.0f) {
			return glm::length(inter - origin);
		}
	}
	return -1.0f;
}

float PointLineDistance(glm::vec2 x1, glm::vec2 x2, glm::vec2 x0) {
	glm::vec3 x01(x0.x - x1.x, x0.y - x1.y, 0.0);
	glm::vec3 x02(x0.x - x2.x, x0.y - x2.y, 0.0);
	glm::vec2 x21 = x2 - x1;
	float d = glm::length(glm::cross(x01, x02)) / glm::length(x21);
	return d;
}

float PointToSegmentDistance(glm::vec2 ep1, glm::vec2 ep2, glm::vec2 p) {
	glm::vec2 v21 = ep2 - ep1;
	glm::vec2 pv1 = p - ep1;
	if (glm::dot(v21, pv1) <= 0.0) {
		float d = glm::length(pv1);
		return d;
	}
	glm::vec2 v12 = ep1 - ep2;
	glm::vec2 pv2 = p - ep2;
	if (glm::dot(v12, pv2) <= 0.0) {
		float d = glm::length(pv2);
		return d;
	}
	float d0 = PointLineDistance(ep1, ep2, p);
	return d0;
}

} //namespace math
} //namespace mavs
