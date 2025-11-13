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
#ifdef USE_MPI
#include <mpi.h>
#endif

#include <iostream>
#include <omp.h>
#include <mavs_core/math/utils.h>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <tinyfiledialogs.h>

namespace mavs{
namespace math {

glm::mat3x4 GetAffineIdentity() {
	glm::mat3x4 rot_scale;
	rot_scale[0][0] = 1.0f; rot_scale[0][1] = 0.0f; rot_scale[0][2] = 0.0f;
	rot_scale[1][0] = 0.0f; rot_scale[1][1] = 1.0f; rot_scale[1][2] = 0.0f;
	rot_scale[2][0] = 0.0f; rot_scale[2][1] = 0.0f; rot_scale[2][2] = 1.0f;
	rot_scale[0][3] = 0.0f; rot_scale[1][3] = 0.0f; rot_scale[2][3] = 0.0f;
	return rot_scale;
}

glm::mat3x4 ScaleAffine(glm::mat3x4 rot_scale, float x_scale, float y_scale, float z_scale) {
	rot_scale[0][0] *= x_scale; rot_scale[0][1] *= y_scale; rot_scale[0][2] *= z_scale;
	rot_scale[1][0] *= x_scale; rot_scale[1][1] *= y_scale; rot_scale[1][2] *= z_scale;
	rot_scale[2][0] *= x_scale; rot_scale[2][1] *= y_scale; rot_scale[2][2] *= z_scale;
	return rot_scale;
}

glm::mat3x4 SetAffineOffset(glm::mat3x4 rot_scale, float x_off, float y_off, float z_off) {
	rot_scale[0][3] = x_off; rot_scale[1][3] = y_off; rot_scale[2][3] = z_off;
	return rot_scale;
}

glm::mat3x4 SetAffineOffset(glm::mat3x4 rot_scale, glm::vec3 offset) {
	rot_scale[0][3] = offset.x; rot_scale[1][3] = offset.y; rot_scale[2][3] = offset.z;
	return rot_scale;
}

glm::mat3 GetIdentity() {
	glm::mat3 rot;
	rot[0][0] = 1.0f; rot[0][1] = 0.0f; rot[0][2] = 0.0f;
	rot[1][0] = 0.0f; rot[1][1] = 1.0f; rot[1][2] = 0.0f;
	rot[2][0] = 0.0f; rot[2][1] = 0.0f; rot[2][2] = 1.0f;
	return rot;
}

glm::mat3x4 GetRotFromEuler(glm::vec3 euler_angles) {
	glm::mat3x4 rot_scale;
	glm::mat3x3 om = orientate3(euler_angles);
	for (int ii = 0; ii < 3; ii++) {
		for (int jj = 0; jj < 3; jj++) {
			rot_scale[ii][jj] = om[ii][jj];
		}
	}
	rot_scale[0][3] = 0.0f; rot_scale[1][3] = 0.0f; rot_scale[2][3] = 0.0f;
	return rot_scale;
}

//ray-triangle intersection routine, see
// https://en.wikipedia.org/wiki/Moller-Trumbore_intersection_algorithm
bool RayTriangleIntersection(glm::vec3 rayOrigin,
	glm::vec3 rayVector,
	glm::vec3 vertex0,
	glm::vec3 vertex1,
	glm::vec3 vertex2,
	glm::vec3& outIntersectionPoint) {
	const float EPSILON = 0.0000001f;
	//glm::vec3 vertex0 = inTriangle->vertex0;
	//glm::vec3 vertex1 = inTriangle->vertex1;
	//glm::vec3 vertex2 = inTriangle->vertex2;
	glm::vec3 edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	h = glm::cross(rayVector, edge2);
	a = glm::dot(edge1, h);
	if (a>-EPSILON && a<EPSILON) {
		return false;
	}
	f = 1.0f / a;
	s = rayOrigin - vertex0;
	u = f * (glm::dot(s, h));
	if (u<0.0f || u> 1.0f) {
		return false;
	}
	q = glm::cross(s, edge1);
	v = f * glm::dot(rayVector, q);
	if (v<0.0 || (u + v)> 1.0) {
		return false;
	}
	float t = f * glm::dot(edge2, q);
	if (t>EPSILON) {
		outIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else {
		return false;
	}
}

void QuatToEulerAngle(const glm::dquat q, double &pitch, double &roll, double &yaw) {
	glm::dvec3 euler = glm::eulerAngles(q);
	roll = euler.x;
	pitch = euler.y;
	yaw = euler.z;
}

float ChamferDistance(std::vector<glm::vec3> pca, std::vector<glm::vec3> pcb) {
	std::vector<int> closest;
	closest.resize(pca.size(), 0);
#pragma omp parallel for
	for (int i = 0; i < (int)pca.size(); i++) {
		int close = 0;
		float d = std::numeric_limits<float>::max();
		for (unsigned int j = 0; j < pcb.size(); j++) {
			float dist = glm::distance(pca[i], pcb[j]);
			if (dist < d) {
				close = j;
				d = dist;
			}
		}
		closest[i] = close;
	}
	float dist_accum = 0.0f;
	for (unsigned int i = 0; i < pca.size(); i++) {
		dist_accum += glm::distance(pca[i], pcb[closest[i]]);
	}
	return dist_accum / (float)pca.size();
}

glm::mat3 IdentityMatrix3() {
	glm::mat3 Iden;
	for (int i = 0; i < 3; i++) { 
		for (int j = 0; j < 3; j++) { 
			if (i == j) { 
				Iden[i][j] = 1.0f; 
			} 
			else { 
				Iden[i][j] = 0.0f; 
			} 
		} 
	}
	return Iden;
}

glm::mat3 GetRotationMatrixFromVectors(glm::vec3 a, glm::vec3 b) {
	// See: https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d

	glm::vec3 v = glm::cross(a, b);
	float s = glm::length(v);
	float c = glm::dot(a, b);
	glm::mat3 vm;
	vm[0][0] = 0.0f; vm[0][1] = v[2]; vm[0][2] = -v[1];
	vm[1][0] = -v[2]; vm[1][1] = 0.0f; vm[1][2] = v[0];
	vm[2][0] = v[1]; vm[2][1] = -v[0]; vm[2][2] = 0.0f;
	glm::mat3 I = IdentityMatrix3();
	glm::mat3 R = I + vm + (1.0f/(1.0f + c))*(vm*vm);
	return R;
}

glm::dquat EulerToQuat(double pitch, double roll, double yaw) {
	glm::dquat q = glm::quat(glm::vec3(roll, pitch, yaw));
	return q;
}

glm::vec3 RodriguesRotation(glm::vec3 v, glm::vec3 k, float theta) {
	float c = (float)cos(theta);
	float s = (float)sin(theta);
	glm::vec3 v_rot = v * c + glm::cross(k, v)*s + k * glm::dot(k, v)*(1.0f - c);
	return v_rot;
}

} //namespace math

namespace utils{

std::string GetSaveFileName() {
	std::string fname = "output.bmp";
	char const * lTheSelectFileName;
	lTheSelectFileName = tinyfd_saveFileDialog("Select file to save", "output.bmp", 0, NULL, "Output file name");
	if (lTheSelectFileName) {
		fname = std::string(lTheSelectFileName);
	}
	return fname;
}

void SleepSeconds(double x){
#ifdef USE_MPI
  double t1 = MPI_Wtime();
  double s = (double) x;
  while (x>(MPI_Wtime()-t1)){}
#endif
  return;
}

std::string GetPathFromFile(const std::string& path) {
	size_t found = path.find_last_of("/\\");
	std::string folder = path.substr(0, found);
	return folder;
}

std::string GetFileExtension(const std::string& file) {
	size_t found = file.find_last_of(".");
	return file.substr(found + 1);
}

/*
* Erase First Occurrence of given  substring from main string.
*/
void EraseSubString(std::string & mainStr, const std::string & toErase){
	// Search for the substring in string
	size_t pos = mainStr.find(toErase);
	if (pos != std::string::npos){
		// If found then erase it from string
		mainStr.erase(pos, toErase.length());
	}
}

} // namespace utils
} // namespace mavs
