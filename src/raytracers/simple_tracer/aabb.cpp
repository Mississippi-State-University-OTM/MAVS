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
#include <raytracers/simple_tracer/aabb.h>
#include <iostream>

namespace mavs {
namespace raytracer {

Aabb::Aabb() {

}

Aabb::Aabb(glm::vec3 position, glm::vec3 dimensions) {
	SetPosition(position.x, position.y, position.z);
	SetSize(dimensions.x, dimensions.y, dimensions.z);
}

Aabb::Aabb(float px, float py, float pz, float sx, float sy, float sz) {
	SetPosition(px, py, pz);
	SetSize(sx, sy, sz);
}

Aabb::~Aabb() {

}

///Set the size dimensions in (x,y,z), meters
void Aabb::SetSize(double xdim, double ydim, double zdim) {
	glm::vec3 ll(position_.x - 0.5*xdim, position_.y - 0.5*ydim, position_.z - 0.5*zdim);
	glm::vec3 ur(position_.x + 0.5*xdim, position_.y + 0.5*ydim, position_.z + 0.5*zdim);
	bounds_[0] = ll;
	bounds_[1] = ur;
	glm::vec3 v0(ll.x, ll.y, ll.z);
	glm::vec3 v1(ll.x, ll.y, ur.z);
	glm::vec3 v2(ll.x, ur.y, ll.z);
	glm::vec3 v3(ll.x, ur.y, ur.z);
	glm::vec3 v4(ur.x, ll.y, ll.z);
	glm::vec3 v5(ur.x, ll.y, ur.z);
	glm::vec3 v6(ur.x, ur.y, ll.z);
	glm::vec3 v7(ur.x, ur.y, ur.z);

	triangles_.resize(12);
	//left side
	triangles_[0].vertex0 = v0;
	triangles_[0].vertex1 = v2;
	triangles_[0].vertex2 = v1;
	triangles_[0].CalculateNormal();
	triangles_[1].vertex0 = v1;
	triangles_[1].vertex1 = v2;
	triangles_[1].vertex2 = v3;
	triangles_[1].CalculateNormal();
	//right side
	triangles_[2].vertex0 = v4;
	triangles_[2].vertex1 = v5;
	triangles_[2].vertex2 = v6;
	triangles_[2].CalculateNormal();
	triangles_[3].vertex0 = v5;
	triangles_[3].vertex1 = v7;
	triangles_[3].vertex2 = v6;
	triangles_[3].CalculateNormal();
	//bottom side
	triangles_[4].vertex0 = v0;
	triangles_[4].vertex1 = v1;
	triangles_[4].vertex2 = v4;
	triangles_[4].CalculateNormal();
	triangles_[5].vertex0 = v1;
	triangles_[5].vertex1 = v5;
	triangles_[5].vertex2 = v4;
	triangles_[5].CalculateNormal();
	//top side
	triangles_[6].vertex0 = v2;
	triangles_[6].vertex1 = v6;
	triangles_[6].vertex2 = v3;
	triangles_[6].CalculateNormal();
	triangles_[7].vertex0 = v3;
	triangles_[7].vertex1 = v6;
	triangles_[7].vertex2 = v7;
	triangles_[7].CalculateNormal();
	//front side
	triangles_[8].vertex0 = v0;
	triangles_[8].vertex1 = v4;
	triangles_[8].vertex2 = v2;
	triangles_[8].CalculateNormal();
	triangles_[9].vertex0 = v2;
	triangles_[9].vertex1 = v4;
	triangles_[9].vertex2 = v6;
	triangles_[9].CalculateNormal();
	//back side
	triangles_[10].vertex0 = v1;
	triangles_[10].vertex1 = v3;
	triangles_[10].vertex2 = v5;
	triangles_[10].CalculateNormal();
	triangles_[11].vertex0 = v3;
	triangles_[11].vertex1 = v7;
	triangles_[11].vertex2 = v5;
	triangles_[11].CalculateNormal();
}

//ray-triangle intersection routine, see
// https://en.wikipedia.org/wiki/Moller-Trumbore_intersection_algorithm
static bool RayTriangleIntersection(glm::vec3 rayOrigin,
	glm::vec3 rayVector,
	Triangle* inTriangle,
	glm::vec3& outIntersectionPoint) {
	const float EPSILON = 0.0000001f;
	glm::vec3 vertex0 = inTriangle->vertex0;
	glm::vec3 vertex1 = inTriangle->vertex1;
	glm::vec3 vertex2 = inTriangle->vertex2;
	glm::vec3 edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	h = glm::cross(rayVector, edge2);
	a = glm::dot(edge1, h);
	if (a > -EPSILON && a < EPSILON) {
		return false;
	}
	f = 1 / a;
	s = rayOrigin - vertex0;
	u = f * (glm::dot(s, h));
	if (u<0.0 || u> 1.0) {
		return false;
	}
	q = glm::cross(s, edge1);
	v = f * glm::dot(rayVector, q);
	if (v<0.0 || (u + v)> 1.0) {
		return false;
	}
	float t = f * glm::dot(edge2, q);
	if (t > EPSILON) {
		outIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else {
		return false;
	}
}

Intersection Aabb::GetIntersection(glm::vec3 orig, glm::vec3 dir) {
	Intersection inter;
	inter.dist = -1.0;
	inter.object_name = "box";
	float closest = 1.0E6;
	int tnum = -1;
	for (int i = 0; i < (int)triangles_.size(); i++) {
		glm::vec3 point;
		bool inter = RayTriangleIntersection(orig, dir, &triangles_[i], point);
		if (inter) {
			float dist = glm::length(orig - point);
			if (dist < closest) {
				closest = dist;
				tnum = i;
			}
		}
	}
	if (tnum >= 0) {
		inter.dist = closest;
		inter.color = color_;
		inter.material.kd = color_;
		inter.normal = triangles_[tnum].normal;
	}
	return inter;
}

} //namespace primitive
} //namespace raytracer
