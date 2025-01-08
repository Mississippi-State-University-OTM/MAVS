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
 * \class SimpleTracer
 *
 * A simple ray tracer for axis-aligned bounding boxes and spheres that can be
 * used to run debug tests when the embree ray-tracer is not available.
 *
 * \author Chris Goodin
 *
 * \date 12/18/2017
 */
#ifndef SIMPLE_TRACER_H
#define SIMPLE_TRACER_H

#include <vector>
#include <iostream>
#include <glm/gtc/quaternion.hpp>
#include <raytracers/raytracer.h>
#include <raytracers/simple_tracer/primitive.h>
#include <raytracers/simple_tracer/aabb.h>
#include <raytracers/simple_tracer/sphere.h>

namespace mavs{
namespace raytracer{
  
class SimpleTracer : public Raytracer {
 public:
  SimpleTracer();
  ~SimpleTracer();

  /**
   * Overwritten virtual method from the Raytracer base class. 
   */
  Intersection GetClosestIntersection(glm::vec3 origin, glm::vec3 direction);

	/**
	* Overwritten virtual method from the Raytracer base class.
	*/
	void GetClosestIntersection(glm::vec3 origin, glm::vec3 direction, Intersection &intersection);

  /**
   * Overwritten virtual method from the Raytracer class.
   */
  bool GetAnyIntersection(glm::vec3 origin, glm::vec3 direction);

	///Adds a primitive to the scene.
	int AddPrimitive(Aabb box) { 
		ClearPrimitives();
		boxes_.push_back(box);
		GeneratePrimitiveList();
		int id = (int)boxes_.size();
		return id;
	}

	///Adds a primitive to the scene.
	int AddPrimitive(Sphere ball) {
		ClearPrimitives();
		balls_.push_back(ball);
		GeneratePrimitiveList();
		int id = (int)(boxes_.size() + balls_.size());
		return id;
	}

	/** 
	* Move a primitive to a new spot
	* \param id ID number of the primitive, as it was added to the scene
	* \param new_pos New position of the primitive
	*/
	void MovePrimitive(int id, glm::vec3 new_pos) {
		primitives_[id]->SetPosition(new_pos.x, new_pos.y, new_pos.z);
	}

  /// Return the number of primitives in the scene
  int GetNumPrimitives(){return (int)primitives_.size();}

  /// Overwritten method to load input file, which doesn't do anything yet
  bool Load(std::string input_file){return true;};
  
	/// Overwritten method to add actor
	std::vector<int> AddActors(std::string actorfile){
		std::vector<int> v;
		return v;
	}

	/// Overwritten method to update actor
	void UpdateActor(glm::vec3 position, glm::quat orientation,
		glm::vec3 scale, int actor_id) {}

	/**
	* Inherited method that gets the height of the surface.
	* If a surface mesh is specified separately,
	* it will query this surface. If not, function will look for lowest point
	* in the scene.
	* \param x The x-coordinate of the surface in global ENU
	* \param y The y-coordinate of the surface in global ENU
	*/
	float GetSurfaceHeight(float x, float y);

	/**
	* Inherited from raytracer base class
	* Return the name of an object with the given ID
	* \param id ID number of the desired object
	*/
	std::string GetObjectName(int id) {
		std::string name = "";
		if (id >= 0 && id < (int)primitives_.size()) {
			name =  primitives_[id]->GetName();
		}
		return name;
	}

	/// Create simple test scene with a floor, a sphere, and a box
	void CreateTestScene();

	/// Create a test scene with a "forest" of cuboids
	void CreateForest();

	void SetPrimitiveMaterial(int prim_id, Material material);

 private:
  std::vector<Primitive *> primitives_;
	std::vector<Sphere> balls_;
	std::vector<Aabb> boxes_;

	void ClearPrimitives();

	void GeneratePrimitiveList();
};

} //namespace raytracer
} //namespace mavs

#endif
