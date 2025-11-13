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
 * \file embree_tracer.h
 *
 * Structs and classes for a ray tracer based on the embree ray-tracing kernel.
 * 
 * \author Chris Goodin
 *
 * \date 12/19/2017
 */
#ifdef USE_EMBREE
#ifndef EMBREE_TRACER_H
#define EMBREE_TRACER_H
#ifdef USE_MPI
#include <mpi.h>
#endif
// The rapidjson includes must be first because of 
// a weird Bool typedef
#ifdef Bool
#undef Bool
#endif
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
#ifdef Bool
#undef Bool
#endif

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#ifdef _WIN32

#else
#define cimg_use_jpeg
#endif

#include <CImg.h>
#include <glm/gtc/quaternion.hpp>

//#if defined(WIN32) || defined(_WIN32)
//#define EMBREE_STATIC_LIB
//#endif
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>

#include <raytracers/reflectance_spectrum.h>
#include <raytracers/mesh.h>
#include <raytracers/raytracer.h>
#include <raytracers/texture_layers/layered_surface.h>
#include <raytracers/animation.h>
#ifdef Bool
#undef Bool
#endif

namespace mavs{
namespace raytracer{
namespace embree{

/**
 * Derived raytracer class based on the embree raytracing library.
 */
class EmbreeTracer : public Raytracer {
 public:
  EmbreeTracer();
  ~EmbreeTracer();
  EmbreeTracer(const EmbreeTracer &scene);

  /** 
   * Load a json input file that lists the meshes with their position,
   * orientation, and scale.
   * \param input_file Full path to the json input file.
   */
  bool Load(std::string input_file);

	/**
	* Load a json input file that lists the meshes with their position,
	* orientation, and scale. Specify a random seed for objects that are placed randomly
	* \param input_file Full path to the json input file.
	* \param seed The seed for the c++ random function
	*/
	bool Load(std::string input_file, unsigned int seed);

	/// Commit any changes made to the scene through the API
	void CommitScene();

	/// Commit any changes made to the animation scene through the API
	void CommitAnimationScene();

	/// Commit any changes made to the scene through the API
	void CommitSceneWithoutTextureReload();

  /**
   * Inherited method from the Raytracer base class.
   */
  Intersection GetClosestIntersection(glm::vec3 origin, glm::vec3 direction);

	/**
	* Inherited method from the Raytracer base class.
	*/
	Intersection GetClosestTerrainIntersection(glm::vec3 origin, glm::vec3 direction);

  /**
   * Inherited method from the Raytracer base class.
   */
  void GetClosestIntersection(glm::vec3 origin, glm::vec3 direction, Intersection &inter);
  
  /**
   * Inherited method from the Raytracer base class.
   */
  bool GetAnyIntersection(glm::vec3 origin, glm::vec3 direction);

  ///Tells the number of triangles in the scene
  unsigned long int GetNumberTrianglesLoaded(){return num_facets_in_scene_;}
  
  /**
  * Tells the number of meshes in the scene
  */
  int GetNumberMeshes() { return num_meshes_; }

	/**
	* Inherited from the sensor class, returns the number of objects in the scene
	*/
	int GetNumberObjects();
  
	/**
	* Add dynamic actors to the scene that can move.
	* Returns a vector with a list of ID numbers for 
	* identifying the actors back to the ray-tracer.
	* \param actorfile The input file defining the actor
	* meshes and paths in the evironment.
	*/
	std::vector<int> AddActors(std::string actorfile);

	/**
	* Add a single actor to the scene based on certain parameters
	* \param meshfile The mesh file associated with the actor
	* \param y_to_z Rotate the mesh?
	* \param x_to_y Rotate the mesh?
	* \param y_to_x Rotate the mesh?
	* \param offset Offset of the mesh w.r.t. actor position
	* \param scale Scale factor fo the actor mesh
	*/
	std::vector<int> AddActor(std::string meshfile, bool y_to_z, bool x_to_y, bool y_to_x, glm::vec3 offset, glm::vec3 scale);

	/**
	* Update the position and orientation of an actor.
	* \param position New position
	* \param orientation New orientation
	* \param scale The (x,y,z) scale factor for the actor
	* \param actor_id Integer identifying the actor to the ray-tracer.
	* Was returned from the call to AddActors().
	*/
	void UpdateActor(glm::vec3 position, glm::quat orientation,
		glm::vec3 scale, int actor_id);

	/**
	* Update the position and orientation of an actor.
	* \param position New position
	* \param orientation New orientation
	* \param scale The (x,y,z) scale factor for the actor
	* \param actor_id Integer identifying the actor to the ray-tracer.
	* \param commit_scene bool, true if you want to recommit the scene
	*/
	void UpdateActor(glm::vec3 position, glm::quat orientation,
		glm::vec3 scale, int actor_id, bool commit_scene);

	/**
	* Tell the position of an animation number anim_num
	* If anim_num is not a valid ID, then it will
	* return (0,0,0). Inherited from raytracer base class
	* \param anim_num The number of the animation for which to get position
	*/
	glm::vec3 GetAnimationPosition(int anim_num);

	/**
	 * Return a pointer to a given animation. Returns a null pointer if the animation id is not valid.
	 * \param anim_num The number of the animation to get.
	 */
	Animation* GetAnimation(int anim_num){ 
		if (anim_num >= 0 && anim_num < animations_.size()) {
			return &animations_[anim_num];
		}
		else{
			return NULL;
		}
	}

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
	* Gets the height of the surface and normal.
	* If a surface mesh is specified separately,
	* it will query this surface. If not, function will look for lowest point
	* in the scene.
	* \param x The x-coordinate of the surface in global ENU
	* \param y The y-coordinate of the surface in global ENU
	*/
	glm::vec4 GetSurfaceHeightAndNormal(float x, float y);

	/**
	* Gets the height of the surface and normal from a specified elevation.
	* If a surface mesh is specified separately,
	* it will query this surface. If not, function will look for lowest point
	* in the scene.
	* \param x The x-coordinate of the surface in global ENU
	* \param y The y-coordinate of the surface in global ENU
	* \param z The z-coordinate to sample from, in global ENU
	*/
	glm::vec4 GetSurfaceHeightAndNormal(float x, float y, float z);

	/**
	* Add a mesh to the embree raytracer scene. 
	* \param mesh The MAVS mesh object to add
	* \param aff_rot The affine transformation specifying the position,
	* orientation, and scale of the mesh to add.
	* \param instnum The instance number of the mesh
	* \param state 0 for static, 1 for dynamic
	*/
	unsigned int AddMesh(Mesh &mesh, glm::mat3x4 aff_rot, int instnum, int state);

	/**
	* Add a mesh to the embree raytracer scene. This overlaod included only for backwards compatibility with old API
	* \param mesh The MAVS mesh object to add
	* \param aff_rot The affine transformation specifying the position,
	* orientation, and scale of the mesh to add.
	* \param instnum The instance number of the mesh
	* \param state 0 for static, 1 for dynamic
	* \param mesh_id unused
	*/
	unsigned int AddMesh(Mesh &mesh, glm::mat3x4 aff_rot, int instnum, int state, int &mesh_id) { 
		return AddMesh(mesh, aff_rot, instnum, state); 
	}

	/**
	* Get a mesh from the embree raytracer scene.
	* \param meshnum The number of the mesh object to get
	*/
	Mesh * GetMesh(int meshnum); 

	/**
	* Get a mesh by instance ID from the embree raytracer scene.
	* \param instID The instance ID of the mesh object to get
	*/
	Mesh * GetMeshByInstID(int instID);

	/**
	* Inherited from raytracer base class
	* Return the name of an object with the given ID
	* \param id ID number of the desired object
	*/
	std::string GetObjectName(int id);

	/**
	* Inherited from the raytracer base class
	* Return the orientation of an object with the given ID
	* \param id ID number of the desired object
	*/
	glm::quat GetObjectOrientation(int id);

	/**
	* Set the surface mesh for the scene.
	* \param surface The surface mesh
	* \param aff_rot The affine matrix describing the 
	* position, orientation, and scale of the mesh.
	*/
	unsigned int SetSurfaceMesh(Mesh &surface, glm::mat3x4 aff_rot);

	/**
	* Set the layered surface mesh for the scene.
	* \param surface The surface mesh
	* \param aff_rot The affine matrix describing the
	* position, orientation, and scale of the mesh.
	*/
	unsigned int SetLayeredSurfaceMesh(Mesh &surface, glm::mat3x4 aff_rot);

	/// Call this to load all the scene textures
	void LoadTextures();

	/**
	* Get the slope of a surface in a given direction
	* over a given length.
	* \param point The point at which to get the slope
	* \param direction The direction at which to take 
	the slope
	* \param h The length scale for the slope calculation
	*/
	float GetSurfaceSlope(glm::vec2 point,
		glm::vec2 direction, float h);

	/// Writes the scene stats to a file, scene_stats.txt
	void WriteSceneStats();

	/// Writes the scene stats to a file in the specified directoryh, scene_stats.txt
	void WriteSceneStats(std::string output_directory);

	/**
	* Inherited from raytracer base class
	* Get user defined color associated with semantic label
	\param label_name The name of the label
	*/
	glm::vec3 GetLabelColor(std::string label_name) {
		glm::vec3 color(0.0f, 0.0f, 0.0f);
		if (label_name == "sky") {
			color = glm::vec3(0.529,0.808,0.922);
		}
		else if (label_colors_.find(label_name)!=label_colors_.end()) {
			color = label_colors_[label_name];
		}
		return color;
	}

	/**
	* Get the number of a given label
	* \param label_name The name of the label
	*/
	int GetLabelNum(std::string label_name) {
		if (labels_loaded_) {
			return label_nums_[label_name];
		}
		else {
			return 0;
		}
	}

	/**
	* Get the semantic label of a given mesh
	* \param mesh_name The name of the mesh
	*/
	std::string GetLabel(std::string mesh_name) {
		if (mesh_name == "textured_surface") {
			return "ground";
		}
		if (mesh_name == "sky") {
			return "sky";
		}
		if (labels_loaded_) {
			return semantic_labels_[mesh_name];
		}
		else {
			return "";
		}
	}

	/**
	* Set if the scene has been loaded
	* \param loaded Set to true of the scene has been loaded
	*/
	void SetLoaded(bool loaded) { scene_loaded_ = loaded; }

	/**
	* Get the reflectance of a spectrum at a given wavelength
	* \param spec_name The name of the spectrum
	* \param wavelength The wavelength in microns
	*/
	float GetReflectance(std::string spec_name, float wavelength);

	/**
	* Update all animations, inherited from raytracer base class
	* \param dt The time step in seconds
	*/
	void UpdateAnimations(float dt);

	/**
	* Add an animation to the raytracing
	* \param anim The animation to add
	*/
	int AddAnimation(Animation &anim);

	/**
	* Set the position and heading of the animation.
	* \param anim_id The ID number of the animation to set.
	* \param x The x-coordinate in local ENU.
	* \param y The y-coordinate in local ENU.
	* \param heading Heading, in radians relative to +X.
	*/
	void SetAnimationPosition(int anim_id, float x, float y, float heading);

	/**
	* Inherited ray tracer virtual function 
	* Return the type of material by code, which can be
	* "dry", "wet", "snow", "ice", "clay", or "sand"
	*/
	std::string GetSurfaceMaterialType() { return surface_material_; }

	/// Inherited ray tracer virtual function , return the cone index of the surface in PSI
	virtual float GetSuraceConeIndex() { return surface_cone_index_; }

	/// Inherited ray tracer virtual function
	void SetActorVelocity(int actor_id, glm::vec3 velocity);

	/// Set the default MAVS file path
	void SetFilePath(std::string file_path) { file_path_ = file_path; }

	/// Add a surface layer
	void AddLayeredSurface(LayeredSurface layer) { layered_surfaces_.push_back(layer); }

	/// Load the label file
	void LoadSemanticLabels(std::string label_file);

	/// Set if labels have been loaded
	void SetLabelsLoaded(bool labels_loaded) { labels_loaded_ = labels_loaded; }

 private:
  glm::mat3 kidentity_matrix_;
  glm::mat3 MatrixFromNormal(float nx, float ny, float nz);

	unsigned int AddMeshToScene(Mesh &mesh, RTCScene &scene, RTCBuildQuality flags);
	bool surface_loaded_;
	
	bool labels_loaded_;
	bool scene_loaded_;

	glm::vec2 surface_ll_, surface_ur_;
	std::string surface_material_;
	float surface_cone_index_;

	void LoadVegDistribution(const rapidjson::Value& d);
	void LoadObjects(const rapidjson::Value& d);
	void LoadSurfaceMesh(const rapidjson::Value& d);
	void LoadLayeredSurface(const rapidjson::Value& d);
	void LoadKeyframeAnimation(std::string anim_file);

  std::vector<std::string> texture_names_;
	std::vector<std::string> spectrum_names_;
  std::vector<Mesh> meshes_;
	std::vector<Mesh> surface_meshes_;
	std::vector<Mesh> layered_surface_meshes_;
	std::vector<LayeredSurface> layered_surfaces_;
  std::unordered_map<std::string,cimg_library::CImg<float> > textures_;
	std::unordered_map<std::string, ReflectanceSpectrum > spectra_;
	std::vector<int> inst_to_meshnum_;

	//label information
	std::unordered_map<std::string, std::string> semantic_labels_;
	std::unordered_map<std::string, int> label_nums_;
	std::unordered_map<std::string, glm::vec3> label_colors_;

	//animation info
	std::vector<Animation> animations_;
	std::vector<int> animation_ids_;
	std::unordered_map<int, int> anim_map_;
	void MoveAnimationVerts(int anim_id);

	std::string file_path_;
	bool use_full_paths_;

	std::vector<glm::vec4> potholes_;
	bool IsInPothole(float x, float y);

  RTCDevice rtc_device_;
  RTCScene rtc_scene_;
	RTCScene surface_;
	RTCScene layered_surface_;
	RTCScene animation_scene_;
  std::vector<RTCScene> instanced_scenes_;
	unsigned int inst_id_;
	unsigned int num_dynamic_;
	std::vector<int> actor_nums_;

  unsigned long int num_facets_in_scene_;
	int num_meshes_;
	double load_time_;

  void GetInterpolatedTextures(int instID, int primiD,
			       int texwidth, int texheight,
			       float u_in, float v_in,
			       int &u_out, int &v_out);

	glm::vec3 GetInterpolatedNormal(int instID, int primID, float u, float v, glm::vec3 vd);

	glm::vec3 GetInterpolatedNormalFromLayeredSurface(int instID, int primID, float u, float v, glm::vec3 vd);
};

} //namespace embree
} //namespace raytracer 
} //namespace mavs
#endif

#endif //USE_EMBREE