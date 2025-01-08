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

#ifndef RANDOM_SURFACE_H
#define RANDOM_SURFACE_H
#include <vector>
#include <iostream>
#include <glm/glm.hpp>
#include <raytracers/texture_layers/layered_surface.h>
#include <CImg.h>
#include <mavs_core/terrain_generator/surface.h>

namespace mavs {
namespace terraingen {

class RandomSurface : public Surface {
public:
	/// Creates a empty random surface, does not initialize
	RandomSurface();

	/// Random surface destructor
	~RandomSurface();

	/**
	* Save the HeightMap as an image file
	* \param image_name Full path to the save file with extension
	*/
	void SaveImage(std::string image_name) {
		heightmap_.WriteImage(image_name);
	}

	/**
	* Create a randomized heightmap based on the 
	* Perlin noise parameters and scene limits.
	* \param llx Lower left corner of the surface, x
	* \param lly Lower left corner of the surface, y
	* \param urx Upper right corner of the surface, x
	* \param ury Upper right corner of the surface, y
	* \param res The resolution of the terrain map, (meters)
	*/
	void GenerateHeightMap(float llx, float lly, float urx, float ury, float res);

	/**
	* Create a randomized heightmap based on the
	* Perlin noise parameters and scene limits.
	* The roughness varies over the scene.
	* \param llx Lower left corner of the surface, x
	* \param lly Lower left corner of the surface, y
	* \param urx Upper right corner of the surface, x
	* \param ury Upper right corner of the surface, y
	* \param res The resolution of the terrain map, (meters)
	*/
	void GenerateVariableRoughness(float llx, float lly, float urx, float ury, float res);

	/**
	* Create a randomized heightmap based on the
	* Gaussian noise parameters and scene limits.
	* \param llx Lower left corner of the surface, x
	* \param lly Lower left corner of the surface, y
	* \param urx Upper right corner of the surface, x
	* \param ury Upper right corner of the surface, y
	* \param res The resolution of the terrain map, (meters)
	*/
	void GenerateGaussianHeightMap(float llx, float lly, float urx, float ury, float res);

	void GenerateHeightmapWithGap(float llx, float lly, float urx, float ury, float res, float gh, float gw, float g_theta);

	/**
	* Set the roughness parameters used to create the surface.
	* Surface is created with two decades of perlin noise. 
	* Low frequency noise creates rolling hills.
	* High frequency noise creates surface roughness.
	* \param lofreq_len The wavelength of low frequency noise (m)
	* \param lomag The magnitude of low frequency noise (m)
	* \param hifreq_len The wavelength of high frequency noise (m)
	* \param himag The magnitude of high frequency noise (m)
	*/
	void SetRoughnessParams(float lofreq_len, float lomag, float hifreq_len, float himag) {
		hifreq_feature_len_ = hifreq_len;
		lofreq_feature_len_ = lofreq_len;
		hifreq_feature_mag_ = himag;
		lofreq_feature_mag_ = lomag;
	}

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
	* Define the layers of the surface
	* \param surf The MAVS layered surface data structure
	*/
	void SetSurfaceLayers(raytracer::LayeredSurface &surf) {
		layered_surface_ = surf;
	}

	/**
	* Add potholes to the road
	* \param pothole_depth The desired pothole depth in meters
	* \param pothole_diameter The desired pothole diameter in meters
	* \param number_potholes The number of potholes to add
	*/
	void AddPotholes(float pothole_depth, float pothole_diameter, int number_potholes, std::vector<glm::vec2> pothole_locations);

	/**
	* Add a pothole at a particular location
	* \param x The X coordinate of the hole in ENU
	* \param y The Y coordinate of the hole in ENU
	* \param pothole_rad The radius of the hole in meters
	* \param pothole_depth The depth of the hole in meters
	*/
	void AddPotholeAtLocation(float x, float y, float pothole_rad, float pothole_depth);

private:

	float hifreq_feature_len_;
	float hifreq_feature_mag_;
	float lofreq_feature_len_;
	float lofreq_feature_mag_;
	glm::vec3 color_;

	cimg_library::CImg<float> surface_texture_;
	raytracer::LayeredSurface layered_surface_;
	bool textured_; 

	float texture_pixres_;
	int max_image_dim_;
};

} //namespace terraingen
} //namespace mavs
#endif
