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
* \file texture_layer.h
*
* Class for a texture layer in a multiple layered surface
* The texture layer consists of a tile-able texture image
* (which must be a square) and associated spatial dimensions.
*
* \author Chris Goodin
*
* \date 9/10/2018
*/

#ifndef TEXTURE_LAYER_H
#define TEXTURE_LAYER_H

#include <string>
#include <raytracers/material.h>
#include <glm/glm.hpp>
#include <CImg.h>

namespace mavs {
namespace raytracer {

class TextureLayer {
public:
	TextureLayer();

	/**
	 * Load the image file for the texture
	 * \param imagefile Full path to the image file
	 */
	void LoadImage(std::string imagefile);

	
	/**
	 * Set the size in meters of the layer 
	 * \param size Size in meters of the layer
	 */
	void SetSize(float size) { size_ = size; }

	/// Return the size in meters of the layer
	float GetSize() { return size_; }

	/**
	 * Set the spatial dimension of the pixels, in meters
	 * \param pd The dimension of the pixel, in meters
	 */
	void SetPixdim(float pd) { pixdim_ = pd; }

	/**
	 * Get the spatial dimension of the pixels, in meters
	 */
	float GetPixdim() { return pixdim_; }

	/// Get the width of the image, in number of pixels
	int GetWidth() {
		return image_.width();
	}

	/// Get the height of the image, in number of pixels
	int GetHeight() {
		return image_.height();
	}

	/**
	 * Get the color of the pixel (i,j), in 0-255 RGB
	 * \param i The horizontal coordinate
	 * \param j The vertical coordinate
	 * \param color The returned color
	 */
	void GetColor(int i, int j, glm::vec3 &color) {
		color = glm::vec3 (image_(i, j, 0, 0),
			image_(i, j, 0, 1),
			image_(i, j, 0, 2));
	}

	/**
	 * Get the normal of the pixel (i,j), in x-y-z global coordinaes
	 * \param i The horizontal coordinate
	 * \param j The vertical coordinate
	 * \param normal The returned normal
	 */
	void GetNormal(int i, int j, glm::vec3 &normal) {
		normal = glm::vec3 (normals_(i, j, 0, 0),
			normals_(i, j, 0, 1),
			normals_(i, j, 0, 2));
	}

	/**
	* Assign a material to the layer
	* \param material The material to assign.
	*/
	void SetMaterial(Material &material) {
		material_ = material;
	}

	/// Return a pointer to the layer material
	Material* GetMaterial() { return &material_; }

private:
	void CreateNormalMap();
	Material material_;
	cimg_library::CImg<float> image_;
	cimg_library::CImg<float> normals_;
	float size_;
	float pixdim_;
};

} //namespace raytracer 
} //namespace mavs
#endif