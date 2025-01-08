/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
/**
 * \file utils.h
 * 
 * Defines some useful math functions. 
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <vector>
#include <iostream>

#include <mavs_core/math/constants.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace mavs{
namespace math{

/** 
* Conversion, euler angles to quaternion 
* \param pitch Input rotation about y-axis, radians
* \param roll Input rotation about x-axis, radians
* \param yaw Input rotation about z-axis, radians
*/
glm::dquat EulerToQuat(double pitch, double roll, double yaw);

/**
* Conversion, quaternion to euler angles
* \param q Input quaternion to convert
* \param pitch Output rotation about y-axis, radians
* \param roll Output rotation about x-axis, radians
* \param yaw Output rotation about z-axis, radians
*/
void QuatToEulerAngle(const glm::dquat q, double& pitch, double& roll, double& yaw);

/**
* Find the rotation matrix that rotates unit vector a onto unit vector b
* \param a Unit vector that you want to rotate from
* \param b Unit vector that you want to rotate to
*/
glm::mat3 GetRotationMatrixFromVectors(glm::vec3 a, glm::vec3 b);

/// Return a 3x3 identity matrix
glm::mat3 IdentityMatrix3();

/**
* Rotate a vector using rodrigues formula. Returns the rotated vector
* \param v The vector to rotate
* \param k The normalized axis to rotate about
* \param theta The angle to rotate through, in radians
*/
glm::vec3 RodriguesRotation(glm::vec3 v, glm::vec3 k, float theta);

/**
 * \fn clamp
 * 
 * Clamps a number between a min and a max value.
 *
 * \param val Parameter to be clamped.
 * \param min Minimum value.
 * \param max Maximum value.
 */
template <class T>
inline T clamp(T val, T min, T max){
  T result = val;
  if (result>max) result=max;
  if (result<min) result=min;
  return result;
};

/**
* \fn ChamferDistance
* 
* Compute the chamfer distance between two point clouds
**/
float ChamferDistance(std::vector<glm::vec3> pca, std::vector<glm::vec3> pcb);

/**
 * \fn clamp_radians
 *
 * Clamps an angle between 0 and 2pi radians
 * 
 * \param x The angle to be clamped
 */
template <class T> 
inline T clamp_radians(T x){
  T tpi = (T)k2Pi;
  T b = x/tpi;
  T a = tpi*(b-(long)(b));
  if (a<0) a = tpi + a;
  return a;
};

/**
 * \fn rand_in_range
 *
 * Returns a random number between the range [lo,hi]
 *
 * \param lo The lowest allowed value
 * \param hi The highest allowed value
 */
template <class T>
inline T rand_in_range(T lo, T hi){
  T r = lo + static_cast<T> (rand())/(static_cast<T> (RAND_MAX/(hi-lo)));
  return r;
};

/**
* \fn heaviside
* Heaviside function
* \param x X-coordinate of the function
*/
template <class T>
inline T heaviside(T x){
  T y = (T) 0.0;
  if (x>=(T)0.0)y=(T)1.0;
  return y;
}

/**
* \fn get_sign
* Returns 1 or -1, depending on the sign of the input
* \param val Value to determine the sign of
*/
template <typename T> int get_sign(T val) {
	return (T(0) < val) - (val < T(0));
}

/**
* Numeric implementation of the inverse error function
* Take from: 
* https://stackoverflow.com/questions/27229371/inverse-error-function-in-c
* \param a The input to the inverse error function
*/
template <class T>
inline T erfinv(T a) {
	double p, r, t;
	t = fmaf(a, 0.0f - a, 1.0f);
	t = log(t);
	if (fabs(t) > 6.125) { // maximum ulp error = 2.35793
		p = 3.03697567e-10; //  0x1.4deb44p-32 
		p = fma(p, t, 2.93243101e-8); //  0x1.f7c9aep-26 
		p = fma(p, t, 1.22150334e-6); //  0x1.47e512p-20 
		p = fma(p, t, 2.84108955e-5); //  0x1.dca7dep-16 
		p = fma(p, t, 3.93552968e-4); //  0x1.9cab92p-12 
		p = fma(p, t, 3.02698812e-3); //  0x1.8cc0dep-9 
		p = fma(p, t, 4.83185798e-3); //  0x1.3ca920p-8 
		p = fma(p, t, -2.64646143e-1); // -0x1.0eff66p-2 
		p = fma(p, t, 8.40016484e-1); //  0x1.ae16a4p-1 
	}
	else { // maximum ulp error = 2.35456
		p = 5.43877832e-9;  //  0x1.75c000p-28 
		p = fma(p, t, 1.43286059e-7); //  0x1.33b458p-23 
		p = fma(p, t, 1.22775396e-6); //  0x1.49929cp-20 
		p = fma(p, t, 1.12962631e-7); //  0x1.e52bbap-24 
		p = fma(p, t, -5.61531961e-5); // -0x1.d70c12p-15 
		p = fma(p, t, -1.47697705e-4); // -0x1.35be9ap-13 
		p = fma(p, t, 2.31468701e-3); //  0x1.2f6402p-9 
		p = fma(p, t, 1.15392562e-2); //  0x1.7a1e4cp-7 
		p = fma(p, t, -2.32015476e-1); // -0x1.db2aeep-3 
		p = fma(p, t, 8.86226892e-1); //  0x1.c5bf88p-1 
	}
	r = a * p;
	return (T)r;
}

/**
* \fn RayTriangleIntersection
* Returns the intersection between a ray and triangle
* Returns true if there is an intersection, false if not
* \param orig Origin of the ray
* \param dir Direction of the ray
* \param vertex0 First vertex of the triangle
* \param vertex1 Second vertex of the triangle
* \param vertex2 Third vertex of the triangle
* \param out_intersection_point The intersection point
*/
bool RayTriangleIntersection(glm::vec3 orig,
	glm::vec3 dir,
	glm::vec3 vertex0,
	glm::vec3 vertex1,
	glm::vec3 vertex2,
	glm::vec3& out_intersection_point);
} //namespace math

namespace utils{
/// Print glm vec3 to stdout
inline void PrintVector(const glm::vec3& vec) {
	std::cout << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")" << std::endl;
}

/// Print glm mat3 to stdout
inline void PrintMatrix(const glm::mat3& mat) {
	std::cout << "|" << mat[0][0] << ", " << mat[0][1] << ", " << mat[0][2] << "|" << std::endl;
	std::cout << "|" << mat[1][0] << ", " << mat[1][1] << ", " << mat[1][2] << "|" << std::endl;
	std::cout << "|" << mat[2][0] << ", " << mat[2][1] << ", " << mat[2][2] << "|" << std::endl;
}

/**
* Sleep for the specified number of milliseconds
* \param mseconds Number of milliseconds to sleep
*/
inline void sleep_milliseconds(unsigned int mseconds) {
	clock_t goal = mseconds + clock();
	while (goal > clock());
}

/**
 * Determine if system is big endian or little endian.
 * 1 = little endian
 * 0 = big endian
 */
inline int endianness(){
  unsigned int x = 1;
  char *c = (char*) &x;
  return (int)*c;
};

/// Check if file exists
inline bool file_exists(std::string fname) {
	std::ifstream ifile(fname.c_str());
	return (bool)ifile;
}

/// Check if a file path is valid
inline bool path_is_valid(std::string file_path) {
	struct stat info;
	int retval = stat(file_path.c_str(), &info);
	if (retval == 0) {
		return true;
	}
	else {
		return false;
	}
}

/// Convert any type to a string
template <class T>
inline std::string ToString(T x){
  std::stringstream ss;
  ss<<x;
  std::string str = ss.str();
  return str;
};

/// Convert any type to a string
template <class T>
inline std::string ToString(T x, int zero_padding) {
	std::stringstream ss;
	ss << std::setfill('0')<<std::setw(zero_padding)<<x;
	std::string str = ss.str();
	return str;
};

/// Allocate a 2D vector
template <class T>
inline std::vector< std::vector<T> > Allocate2DVector(int nx, int ny, T initval) {
	std::vector< std::vector<T> > A;
	for (int i = 0; i<nx; i++) {
		std::vector<T> row;
		for (int j = 0; j<ny; j++) {
			row.push_back(initval);
		}
		A.push_back(row);
	}
	return A;
};

/// Allocate a 3D vector
template <class T>
inline std::vector< std::vector< std::vector<T> > > Allocate3DVector(int nx, int ny, int nz, T initval) {
	std::vector< std::vector< std::vector<T> > > A;
	for (int i = 0; i<nx; i++) {
		std::vector <std::vector<T> > B;
		for (int j = 0; j<ny; j++) {
			std::vector<T> row;
			for (int k = 0; k < nz; k++) {
				row.push_back(initval);
			}
			B.push_back(row);
		}
		A.push_back(B);
	}
	return A;
};

/// Convert string to int
inline int StringToInt(std::string s){
  std::stringstream temp(s);
  int i = 0;
  temp>>i;
  return i;
}

/// Convert string to double
inline double StringToDouble(std::string s){
  std::stringstream temp(s);
  double x = 0.0;
  temp>>x;
  return x;
}
 
/**
* \fn GetPathFromFile
* Returns the leading file path from a full path file name
* \param path Full path to file
*/
std::string GetPathFromFile(const std::string& path);

/**
* \fn GetFileExtension
* Return the extension of a file
* \param file The file for which you want the extension
*/
std::string GetFileExtension(const std::string& file);

/*
* Erase First Occurrence of given  substring from main string.
*/
void EraseSubString(std::string & mainStr, const std::string & toErase);

/// Open a prompt to get a save file name
std::string GetSaveFileName();

/** 
 * Convert a scalar into an RGB colormap
 * See https://www.particleincell.com/2014/colomap
*/
inline glm::vec3 ColorMap(float x){
  float a = (1.0f-x)*4.0f;
  float X = std::floor(a);
  float Y = std::floor(255*(a-X));
  glm::vec3 color(0.0f,0.0f,0.0f);
  if (Y==0){
    color.x = 255.0f;
    color.y = Y;
  }
  else if (Y==1){
    color.x = 255.0f-Y;
    color.y = 255.0f;
  }
  else if (Y==2){
    color.y = 255.0f;
    color.z = Y;
  }
  else if (Y==3){
    color.y = 255.0f-Y;
    color.z = 255.0f;
  }
  else if (Y==4){
    color.z = 255.0f;
  }
  return color;
}

/// sleep for x seconds
void SleepSeconds(double x);

}//namespace utils
} //namespace math

#endif
