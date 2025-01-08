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
* \class RandomSurface
*
* Create a surface of given dimensions and resolution with 
* two decades of perlin noise. Can erode the surface using the
* erosion simulator for a more realistic appearance.
*
* \author Chris Goodin
*
* \date 8/10/2018
*/

#ifndef MAVS_SURFACE_H
#define MAVS_SURFACE_H
#include <vector>
#include <iostream>
#include <glm/glm.hpp>
#include <mavs_core/terrain_generator/heightmap.h>
#include <mavs_core/pose_readers/trail.h>
#include <raytracers/texture_layers/layered_surface.h>
#include <CImg.h>

namespace mavs {
namespace terraingen {

class Surface {
public:
	/// Creates a empty surface, does not initialize
	Surface();

	/// Random surface destructor
	~Surface();

	void SetDimensions(float llx, float lly, float urx, float ury, float res);

	/**
	* Create a texture for the surface based on the layer inputs
	*/
	//void TextureSurface();

	/**
	* Save the current surface to an obj file named "surface.obj"
	*/
	void WriteObj();

	/**
	* Save the current surface to an obj file of the specified name.
	* \param surfname The base name (without extension) of the obj file
	*/
	void WriteObj(std::string surfname);

	/**
	* Set the color of the surface that will be written to the .mtl file
	* \param r The reflectance (0-1) in the red band.
	* \param g The reflectance (0-1) in the green band.
	* \param b The reflectance (0-1) in the blue band.
	*/
	void SetColor(float r, float g, float b) {
		color_ = glm::vec3(r, g, b);
	}

	/**
	* Add a path through the terrain, will be used to create texture
	* \param trail The MAVS trail to use
	*/
	void SetTrail(Trail &trail);

	/**
	* Create a trail through the terrain
	* \param path_type The type of path, can be either "Ridges", "Valleys", "Square", or "Loop"
	*/
	Trail GetTrail(std::string path_type);

	/**
	* Get the height of the surface at a specific x-y point
	* \param p The point to get the height
	*/
	float GetHeightAtPoint(glm::vec2 p) {
		return heightmap_.GetHeightAtPoint(p);
	}

	/// Display the current heightmap
	void Display() {
		heightmap_.Display(false);
	}

	/**
	* Display the current heightmap
	* \param trim If true, then the mouse can be used to trim the heightmap
	*/
	void Display(bool trim) {
		heightmap_.Display(trim);
	}

	/// Display the slope of the heightmap
	void DisplaySlopes() {
		heightmap_.DisplaySlopes();
	}

	/**
	* Define the layers of the surface
	* \param surf The MAVS layered surface data structure
	*/
	void SetSurfaceLayers(raytracer::LayeredSurface &surf) {
		layered_surface_ = surf;
	}

	/// Get the heightmap object
	HeightMap GetHeightMap() {
		return heightmap_;
	}

	/// Get the heightmap object
	HeightMap * GetHeightMapPointer() {
		return &heightmap_;
	}

	/// Recenter the terrain at (0,0,0)
	void Recenter() {
		heightmap_.Recenter();
	}

	/**
	* Resize the terrain to nx by ny cells
	* \param nx The number of cells in the East-West direction
	* \param ny The number of cells in the North-South direction
	*/
	void Resize(int nx, int ny) {
		heightmap_.Resize(nx, ny);
	}

protected:
	glm::vec3 color_;

	HeightMap heightmap_;
	//int nx_;
	//int ny_;

	//std::vector< std::vector<float> > heightmap_;

	//bool MapsSameSize(std::vector< std::vector<float> > &hm);

	Trail trail_;
	bool trail_set_;

	cimg_library::CImg<float> surface_texture_;
	raytracer::LayeredSurface layered_surface_;
	bool textured_;

	float texture_pixres_;
	int max_image_dim_;

	void GetPoseAtPosition(glm::vec2 pos, glm::vec2 lt, glm::vec3 &position, glm::quat &orientation);

};

} //namespace terraingen
} //namespace mavs
#endif
