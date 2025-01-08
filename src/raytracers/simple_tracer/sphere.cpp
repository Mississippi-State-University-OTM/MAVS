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
#include <iostream>

#include <raytracers/simple_tracer/sphere.h>

namespace mavs{
namespace raytracer{

Sphere::Sphere(){
  radius_ = 1.0f;
  r2_ = 1.0f;
}

Sphere::~Sphere(){

}

static bool SolveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1){
  float discr = b*b - 4.0f*a*c;
  if (discr<0) return false;
  else if (discr==0.0f) x0 = x1 = -0.5f*b/a;
  else {
    float q = (b>0.0f)?
      -0.5f*(float)(b+sqrt(discr)) :
      -0.5f*(float)(b-sqrt(discr));
    x0 = q/a;
    x1 = c/q;
  }
  if (x0>x1) std::swap(x0,x1);
  return true;
}

Intersection Sphere::GetIntersection(glm::vec3 orig, glm::vec3 dir){
  Intersection inter;
  inter.dist = -1.0;
  float t0,t1;
  glm::vec3 L = orig - position_;
  float a = glm::dot(dir,dir);
  float b = 2*glm::dot(dir,L);
  float c = glm::dot(L,L)-r2_;
  if(!SolveQuadratic(a,b,c,t0,t1)) return inter;
  if(t0>t1) std::swap(t0,t1);
  if (t0<0){
    t0 = t1;
    if (t0<0) return inter;
  }
  inter.dist = t0;
  inter.normal = (orig + t0*dir)-position_;
  inter.normal = inter.normal/length(inter.normal);
	inter.material.kd = color_;
	//inter.material.ns = ns_;
	inter.color = color_;
	inter.object_name = "sphere";
	inter.material = material_;
  return inter;
}

} //namespace raytracer
} //namespace mavs
