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
#include <raytracers/simple_tracer/simple_tracer.h>
#include <mavs_core/math/utils.h>
namespace mavs{
namespace raytracer{

SimpleTracer::SimpleTracer(){

}

SimpleTracer::~SimpleTracer(){

}

void SimpleTracer::ClearPrimitives() {
	primitives_.clear();
}

void SimpleTracer::GeneratePrimitiveList() {
	for (int i = 0; i < boxes_.size(); i++) {
		primitives_.push_back((Primitive *)&boxes_[i]);
	}
	for (int i = 0; i < balls_.size(); i++) {
		primitives_.push_back((Primitive *)&balls_[i]);
	}
}

float SimpleTracer::GetSurfaceHeight(float x, float y) {
	float z = -1000000.0f;
	glm::vec3 origin(x, y, z);
	glm::vec3 direction(0, 0, 1);
	Intersection inter = GetClosestIntersection(origin, direction);
	return (z + inter.dist);
}

void SimpleTracer::GetClosestIntersection(glm::vec3 orig, glm::vec3 dir, Intersection &inter_closest) {
	inter_closest.dist = -1.0;
	float closest = 1.0E6;
	for (int i = 0; i<(int)primitives_.size(); i++) {
		Intersection inter = primitives_[i]->GetIntersection(orig, dir);
		if (inter.dist>0 && inter.dist<closest) {
			inter_closest = inter;
			closest = inter.dist;
			inter_closest.dist = inter.dist;
			inter_closest.object_id = i;
			inter_closest.object_name = inter.object_name;
		}
	}
}

Intersection SimpleTracer::GetClosestIntersection(glm::vec3 orig, glm::vec3 dir){
  Intersection inter_closest;
  inter_closest.dist = -1.0;
  float closest = 1.0E6;;
  for (int i=0; i<(int)primitives_.size(); i++){
    Intersection inter = primitives_[i]->GetIntersection(orig,dir);
    if (inter.dist>0 && inter.dist<closest){
      inter_closest = inter;
      closest = inter.dist;
			inter_closest.dist = inter.dist;
			inter_closest.object_id = i;
			inter_closest.object_name = inter.object_name;
			inter_closest.material = primitives_[i]->GetMaterial();
    }
  }
  return inter_closest;
}

bool SimpleTracer::GetAnyIntersection(glm::vec3 orig, glm::vec3 dir){
  bool intersected = false;
  for (int i=0; i<(int)primitives_.size(); i++){
    Intersection inter = primitives_[i]->GetIntersection(orig,dir);
    if (inter.dist>0){
      intersected = true;
      break;
    }
  }
  return intersected;
}

void SimpleTracer::SetPrimitiveMaterial(int prim_id, Material material) {
	if (prim_id >= 0 && prim_id < primitives_.size()) {
		primitives_[prim_id]->SetMaterial(material);
	}
}

void SimpleTracer::CreateForest() {
	int nboxes = 100;
	float range = 100.0f;
	std::vector<mavs::raytracer::Aabb> box;
	box.resize(nboxes);
	std::vector<glm::vec2> box_locations;
	box_locations.resize(nboxes);
	for (int i = 0; i < nboxes; i++) {
		float x = mavs::math::rand_in_range(-range, range);
		float y = mavs::math::rand_in_range(-range, range);
		float sx = mavs::math::rand_in_range(0.5f, 1.0f);
		float sy = mavs::math::rand_in_range(0.5f, 1.0f);
		float sz = mavs::math::rand_in_range(2.0f, 10.0f);
		box_locations[i].x = x;
		box_locations[i].y = y;
		box[i].SetPosition(x, y, 2.0f);
		box[i].SetSize(sx, sy, sz);
		box[i].SetColor(0.25f, 0.75f, 0.1f);
		AddPrimitive(box[i]);
	}
	mavs::raytracer::Aabb ground;
	ground.SetSize(1.0E6, 1.0E6, 0.01);
	ground.SetColor(0.38f,0.247f,0.063f);
	ground.SetPosition(0.0f, 0.0f, 0.0f);
	AddPrimitive(ground);
}

void SimpleTracer::CreateTestScene() {
	// Add ground
	glm::vec3 green(0.25f, 1.0f, 0.25f);
	mavs::raytracer::Aabb box;
	box.SetSize(1.0E6f, 1.0E6f, 0.01f);
	box.SetColor(green.x, green.y, green.z);
	box.SetPosition(0.0f, 0.0f, 0.0f);
	AddPrimitive(box);

	// add yellow box
	glm::vec3 yellow(0.9f, 0.9f, 0.1f);
	mavs::raytracer::Aabb box2;
	box2.SetPosition(0.0f, 4.0f, 3.0f);
	box2.SetSize(10.0f, 10.0f, 2.0f);
	box2.SetColor(yellow.x, yellow.y, yellow.z);
	AddPrimitive(box2);

	//add shiny res sphere
	glm::vec3 red(0.7f, 0.15f, 0.15f);
	mavs::Material shiny_red;
	shiny_red.kd = red;
	shiny_red.ks = glm::vec3(0.3f, 0.3f, 0.3f);
	shiny_red.ns = 30.0f;
	mavs::raytracer::Sphere sred;
	sred.SetPosition(1.0f, -2.0f, 10.0f);
	sred.SetMaterial(shiny_red);
	sred.SetRadius(5.0f);
	sred.SetColor(red.x, red.y, red.z);
	AddPrimitive(sred);
}

} //namespace raytracer
} //namespace mavs
