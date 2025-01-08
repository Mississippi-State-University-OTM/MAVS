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
* \class LayeredSurface
*
* Class for creating a surface with layered textures
* Automatically creates texture and normal maps with 
* user specified texture layers.
*
* \author Chris Goodin
*
* \date 9/7/2018
*/

#ifndef LAYERED_SURFACE_H
#define LAYERED_SURFACE_H

#include <vector>
#include <string>

#include <glm/glm.hpp>
#include <mavs_core/pose_readers/trail.h>
#include <raytracers/texture_layers/texture_layer.h>
#include <raytracers/material.h>

namespace mavs {
namespace raytracer {

class LayeredSurface {
public:
	/// Create a layered surface
	LayeredSurface();

	/**
	* Load the surface texture description file
	* \param path_to_meshes Full path to the location of the meshes in the mavs data tree
	* \param texture_input_file json file that defines the texture layers
	*/
	void LoadSurfaceTextures(std::string path_to_meshes, std::string texture_input_file);

	/**
	* Return the rgb color and surface normal at an x-y point on the surface
	* \param p The x-y point on the suraface
	* \param color The return value for the color at p
	* \param normal The return value for the surface normal at p
	*/
	Material* GetColorAndNormalAtPoint(glm::vec2 p, glm::vec3 &color, glm::vec3 &normal);

	/**
	* Set the trail location of the surface.
	* Use this when there is only a single trail on the scene
	*/
	void SetTrail(Trail &trail);

	/**
	* Add a trail to the surface.
	* Use this when there are multiple trails in the scene
	*/
	void AddTrail(Trail &trail);

	/// Return true if an (x,y) location is an a trail
	bool IsPointOnTrail(float x, float y);

	/**
	* Return a pointer to the material of the surface layer.
	* \param ln The layer num, can be 0-2
	*/
	Material * GetMaterial(int ln) {
		Material *mat;
		if (ln>=0 && ln<3) mat = layers_[ln].GetMaterial();
		return mat;
	}

private:
	std::vector<TextureLayer> layers_;
	void GetLayerColorAndNormalAtPoint(glm::vec2 p, int ln, glm::vec3 &color, glm::vec3 &point);
	std::vector<Trail> trails_;
	float trail_width_;

	/*void InitTrail();
	std::vector<std::vector<bool> > on_trail_;
	float mapres_; 
	glm::vec2 trail_ll_;
	glm::vec2 trail_ur_;
	int nx_;
	int ny_;
	*/
};

} //namespace raytracer 
} //namespace mavs
#endif