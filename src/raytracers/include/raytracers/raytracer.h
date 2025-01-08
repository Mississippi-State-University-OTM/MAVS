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
/**
 * \file raytracer.h
 *
 * MAVS will be modular with respect to ray-tracing / rendering engines.
 * The ray-tracer will need to return some basic information about the 
 * intersection.
 *
 * \author Chris Goodin
 *
 * \date 12/18/2017
 */

#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <vector>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <raytracers/material.h>
#include <raytracers/reflectance_spectrum.h>
#include <raytracers/bounding_box.h>
#include <raytracers/animation.h>

namespace mavs{
namespace raytracer{

/**
 * The intersection returns the distance, color, material, normal, and instance ID of object. If there
 * is no intersection, it should return a negative distance.
 */
struct Intersection{
	Intersection() : color(zero_vec3), normal(one_vec3), dist(0.0f), velocity(zero_vec3),object_id(0) {}
	glm::vec3 color;
	Material material;
	glm::vec3 normal;
	float dist;
	glm::vec3 velocity;
	int object_id;
	std::string object_name;
	std::string label;
	std::string spectrum_name;
};

inline std::ostream& operator<<(std::ostream& os, const Intersection& obj) {
	os << "object_name, id: " << obj.object_name << " " << obj.object_id << std::endl;
	os << "color:  " << obj.color.x << " " << obj.color.y << " " << obj.color.z << std::endl;
	os << "normal: " << obj.normal.x << " " << obj.normal.y << " " << obj.normal.z << std::endl;
	os << "velocity " << obj.velocity.x << " " << obj.velocity.y << " " << obj.velocity.z << std::endl;
	os << "dist, id: " << obj.dist << " " << obj.object_id << std::endl;
	os << "label: " << obj.label << std::endl;
	os << "spectrum: " << obj.spectrum_name << std::endl;
	os << "material: " << obj.material;
	return os;
}

/**
 * Base class for raytracers in the MAVS simulation. The environment structure 
 * contaings a pointer to the ray-tracer being used. Need to implement two
 * virtual methods.
 *
 */
class Raytracer {
 public:

	Raytracer(){
		perform_labeling_ = false;
		use_spectral_ = false;
		use_surface_textures_ = true;
		lower_left_corner_ = glm::vec3(0.0f, 0.0f, 0.0f);
		upper_right_corner_ = glm::vec3(0.0f, 0.0f, 0.0f);
	}

  /**
   * Virtual method that gets the closest intersection in a scene. 
   * \param origin Origin of the ray to trace.
   * \param direction Normalized direction of the ray.
   */
  virtual Intersection GetClosestIntersection(glm::vec3 origin, 
					      glm::vec3 direction){
    Intersection inter;
    inter.dist = -1;
    return inter;
  }

	/**
	* Virtual method that gets the closest terrain intersection in a scene.
	* \param origin Origin of the ray to trace.
	* \param direction Normalized direction of the ray.
	*/
	virtual Intersection GetClosestTerrainIntersection(glm::vec3 origin,
		glm::vec3 direction) {
		Intersection inter;
		inter.dist = -1;
		return inter;
	}

    /**
   * Virtual method that gets the closest intersection in a scene. 
   * Pass by reference for the return value for optimization reasons
   * \param origin Origin of the ray to trace.
   * \param direction Normalized direction of the ray.
   * \param
   */
  virtual void GetClosestIntersection(glm::vec3 origin, 
					      glm::vec3 direction, Intersection &inter){
    inter.dist = -1;
  }

  /**
   * Virtual method to find any intersection in the scene. Used to check for
   * shadows or blocked GPS satellites, among other things.
   * \param origin Origin of the ray to trace.
   * \param direction Normalized direction of the ray.
   */
  virtual bool GetAnyIntersection(glm::vec3 origin, glm::vec3 direction){
    return false;
  }

	/**
	* Gets the height of the surface. If a surface mesh is specified separately,
	* it will query this surface. If not, function will look for lowest point 
	* in the scene.
	* \param x The x-coordinate of the surface in global ENU
	* \param y The y-coordinate of the surface in global ENU
	*/
	virtual float GetSurfaceHeight(float x, float y) { return 0.0f;  }

	/**
	* Gets the height of the surface and normal. 
	* If a surface mesh is specified separately,
	* it will query this surface. If not, function will look for lowest point
	* in the scene.
	* \param x The x-coordinate of the surface in global ENU
	* \param y The y-coordinate of the surface in global ENU
	*/
	virtual glm::vec4 GetSurfaceHeightAndNormal(float x, float y) { 
		return glm::vec4(0.0f, 0.0f, 1.0f, 0.0f); 
	}

	/**
	* Gets the height of the surface and normal from a specified elevation.
	* If a surface mesh is specified separately,
	* it will query this surface. If not, function will look for lowest point
	* in the scene.
	* \param x The x-coordinate of the surface in global ENU
	* \param y The y-coordinate of the surface in global ENU
	* \param z The z-coordinate to sample from, in global ENU
	*/
	virtual glm::vec4 GetSurfaceHeightAndNormal(float x, float y, float z) {
		return glm::vec4(0.0f, 0.0f, 1.0f, 0.0f);
	}

  ///Tells the number of facets loaded in the scene geometry.
  virtual unsigned long int GetNumberTrianglesLoaded(){return 0;}
    
  ///Returns the upper right corner of the scene in ENU coordinates.
  glm::vec3 GetUpperRightCorner(){return upper_right_corner_;}

  ///Returns the lower left corner of the scene in ENU coordinates.
  glm::vec3 GetLowerLeftCorner(){return lower_left_corner_;}

  /// Virtual method to load scene input file
  virtual bool Load(std::string input_file){return true;}

	/// Add dynamic mesh to the scene
	virtual std::vector<int> AddActors(std::string actorfile) {
		std::vector<int> v;
		return v;
	}

	/**
	* Add a single actor to the scene based on certain parameters
	* \param meshfile The mesh file associated with the actor
	* \param y_to_z Rotate the mesh?
	* \param x_to_y Rotate the mesh?
	* \param y_to_x Rotate the mesh?
	* \param offset Offset of the mesh w.r.t. actor position
	* \param scale Scale factor fo the actor mesh
	*/
	virtual std::vector<int> AddActor(std::string meshfile, bool y_to_z, bool x_to_y, bool y_to_x, glm::vec3 offset, glm::vec3 scale) {
		std::vector<int> v;
		return v;
	}

	/// Update the position/orientation of actor 
	virtual void UpdateActor(glm::vec3 position, glm::quat orientation,
		glm::vec3 scale, int actor_id, bool commit_scene) {}

	/// Update the position/orientation of actor 
	virtual void UpdateActor(glm::vec3 position, glm::quat orientation,
		glm::vec3 scale, int actor_id) {}

	/**
	* Update all animations in the scene
	*/
	virtual void UpdateAnimations(float dt) {}

	/**
	* Tell the position of an animation number anim_num
	* If anim_num is not a valid ID, then it will
	* return (0,0,0).
	* \param anim_num The number of the animation for which to get position
	*/
	virtual glm::vec3 GetAnimationPosition(int anim_num) { return glm::vec3(0.0f, 0.0f, 0.0f); }

	/**
	 * Return a pointer to a given animation. Returns a null pointer if the animation id is not valid.
	 * \param anim_num The number of the animation to get.
	 */
	virtual Animation* GetAnimation(int anim_num){ return NULL; }

	/**
	* Return the name of an object with the given ID
	* \param id ID number of the desired object
	*/
	virtual std::string GetObjectName(int id) { return ""; }

	/**
	* Return the orientation of an object with the given ID
	* \param id ID number of the desired object
	*/
	virtual glm::quat GetObjectOrientation(int id) { return glm::quat(1.0f,0.0f,0.0f,0.0f); }

	/**
	* Return the bounding box of an object with the given ID
	* \param id ID number of the desired object
	*/
	raytracer::BoundingBox GetObjectBoundingBox(int id) {
		BoundingBox bb; 
		if (id >= 0 && id < (int)bounding_boxes_.size()) {
			bb = bounding_boxes_[id];
		}
		return bb;
	}

	/**
	* Get the number of a given label
	* \param label_name The name of the label
	*/
	virtual int GetLabelNum(std::string label_name) {
		return 0;
	}

	/// Get the number of objects in the scene
	virtual int GetNumberObjects() { return 0; }

	/**
	* Set the position and heading of the animation.
	* \param anim_id The ID number of the animation to set.
	* \param x The x-coordinate in local ENU.
	* \param y The y-coordinate in local ENU.
	* \param heading Heading, in radians relative to +X.
	*/
	virtual void SetAnimationPosition(int anim_id, float x, float y, float heading) {}

	/**
	* Get the reflectance from a spectrum at a given wavelength.
	* \param spec_name The spectrum ID/name.
	* \param wavelength The wavelength to query, in microns.
	*/
	virtual float GetReflectance(std::string spec_name, float wavelength) {
		return 0.0f;
	}

	/**
	* Get user defined color associated with semantic label
	\param label_name The name of the label
	*/
	virtual glm::vec3 GetLabelColor(std::string label_name) {
		glm::vec3 color(1.0f, 1.0f, 1.0f);
		return color;
	}

	/**
	* Get the semantic label of a given mesh
	* \param mesh_name The name of the mesh
	*/
	virtual	std::string GetLabel(std::string mesh_name) {
		return "";
	}

	/**
	* Return the type of material by code, which can be
	* "dry", "wet", "snow", "ice", "clay", or "sand"
	*/
	virtual std::string GetSurfaceMaterialType() { return "dry"; }

	/// Return the cone index of the surface in PSI
	virtual float GetSuraceConeIndex() { return 250.0f; }

	void TurnOffLabeling(){ perform_labeling_ = false; }
	void TurnOnLabeling(){ perform_labeling_ = true; }
	void TurnOnSpectral(){use_spectral_ = true; }
	void TurnOffSpectral(){use_spectral_ = false; }
	void TurnOffSurfaceTextures(){ use_surface_textures_ = false; }
	void TurnOnSurfaceTextures(){ use_surface_textures_ = true; }

	virtual void SetActorVelocity(int actor_id, glm::vec3 velocity) { return; }

 protected:
  glm::vec3 lower_left_corner_;
  glm::vec3 upper_right_corner_;
  bool perform_labeling_;
  bool use_spectral_;
  bool use_surface_textures_;
  std::vector<BoundingBox> bounding_boxes_;
  std::vector<glm::quat> orientations_;

};

} //namespace raytracer
} //namespace mavs

#endif
