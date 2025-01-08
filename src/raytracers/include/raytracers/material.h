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
#ifndef MATERIAL_H
#define MATERIAL_H

#include <string>
#include <iostream>
#include <glm/glm.hpp>

#include <mavs_core/math/constants.h>

namespace mavs{

/// Material structure based on wavefront .mtl file
struct Material{
Material() : ka(zero_vec3), kd(one_vec3), ks(zero_vec3), tr(zero_vec3),
    ke(zero_vec3), ns(2.0f), ni(-1.0f), dissolve(0.0f), illum(1.0f) {}
  /// Name of the material
  std::string name;
  /// Ambient RGB reflectance
  glm::vec3 ka;
  /// Diffuse RGB reflectance
  glm::vec3 kd;
  /// Specular RGB reflectance
  glm::vec3 ks;
  glm::vec3 tr;
  glm::vec3 ke;
  /// Specular reflecance exponent for the Phong model
  float ns;
	// index of refraction
  float ni;
  float dissolve;
  float illum;
  /// Diffuse reflectance texture map
  std::string map_kd;
  /// Ambient reflectance texture map
  std::string map_ka;
  /// Specular reflectance texture map
  std::string map_ks;
  /// Phong specular exponent texture map
  std::string map_ns;
  /// Normal texture map
  std::string map_bump;
  /// Alpha (transparency) texture map
  std::string map_d;
  /// Height texture map
  std::string disp;
  /// Name of spectrum file for the  material
  std::string refl;
};

inline std::ostream& operator<<(std::ostream& os, const Material& obj) {
	os << "name: " << obj.name << std::endl;
	os << "ka:  " << obj.ka.x << " " << obj.ka.y << " " << obj.ka.z << std::endl;
	os << "kd:  " << obj.kd.x << " " << obj.kd.y << " " << obj.kd.z << std::endl;
	os << "ks:  " << obj.ks.x << " " << obj.ks.y << " " << obj.ks.z << std::endl;
	os << "tr:  " << obj.tr.x << " " << obj.tr.y << " " << obj.tr.z << std::endl;
	os << "ke:  " << obj.ke.x << " " << obj.ke.y << " " << obj.ke.z << std::endl;
	os << "ns, ni: " << obj.ns << " " << obj.ni << std::endl;
	os << "dissolve, illum: " << obj.dissolve << " " << obj.illum << std::endl;
	std::cout << "maps: " << std::endl;
	if (obj.map_kd.length() > 0)std::cout << "\t " << obj.map_kd << " ";
	if (obj.map_ks.length() > 0)std::cout << "\t " << obj.map_ks << " ";
	if (obj.map_ns.length() > 0)std::cout << "\t " << obj.map_ns << " ";
	if (obj.map_bump.length() > 0)std::cout << "\t " << obj.map_bump << " ";
	if (obj.map_d.length() > 0)std::cout << "\t " << obj.map_d << " ";
	if (obj.disp.length() > 0)std::cout << "\t " << obj.disp << " ";
	if (obj.refl.length() > 0)std::cout << "\t " << obj.refl;
	return os;
}

} // namespace mavs

#endif
