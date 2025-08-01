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
 * \file terrain_elevation_functions.h
 *
 * A set of functions and classes defining different terrain shapes that can be generated programatically without loading a file from disk.
 *
 * \author Chris Goodin
 *
 * \date 6/25/2025
 */
#ifndef TERRAIN_ELEVATION_FUNCTIONS_H
#define TERRAIN_ELEVATION_FUNCTIONS_H
#include <random>
#include <memory>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif

namespace mavs {
namespace terraingen {

/// Class for dynamically created terrains
class TerrainElevationFunction {
public:
	TerrainElevationFunction();

	~TerrainElevationFunction();

	/// TerrainElevationFunction copy constructor
	TerrainElevationFunction(const TerrainElevationFunction& s);

	/** . 
	* Args are the x-y coordinates in ENU meters. 
	* Returns the elevation (z) in ENU metrs
	* \param x X (easting) coordinate in local ENU meters
	* \param y Y (northing) coordinate in local ENU meters
	*/
	float GetElevation(float x, float y);

	/**
	* Initialize a sloped terrain. Slope is along the x-direction
	* There is no variation in the y direction. Elevation = 0 at X = 0
	* Input is the fractional slope, i.e. fractional_slope = 45 degrees
	* \param fractional_slope The slope as you would define for a line with the equation y = m*x
	*/
	void SlopedTerrain(float fractional_slope);

	/**
	* Initialize the hole terrain
	* Specify the (x,y) location of the hole, it's depth, it's diameter, and the side steepness parameter
	* \param x X (easting) coordinate of the center of the hole in local ENU meters
	* \param y Y (northing) coordinate of the center of the hole in local ENU meters
	* \param d Depth of the hole in meters. Negative depth is an obstacle
	* \param s Steepness of the hole sides
	*/
	void HoleTerrain(float x, float y, float depth, float diameter, float steepness);

	/**
	* Initialize a rough terrain. Uses a simple RMS model.
	* Input is the desired RMS roughness in meters
	* \param rms The RMS roughness in meters
	*/
	void RoughTerrain(float rms);

	/**
	* Initialize a parabolic terrain. Change in slope is along the x-direction
	* There is no variation in the y direction. Elevation = 0 at X = 0
	* Input is the quadratic coefficient
	* \param square_coeff The coefficient as you would define for a parabola with the equation y = a*x*x
	*/
	void ParabolicTerrain(float square_coeff);

	/**
	* Initialize a terrain with a trapezoidal obstacle.
	* The obstacle can be a ditch (positive depth) or a hill (negative depth)
	* Runs along the y-direction, traveling along X will cross the obstacle
	* \param bottom_width Width in meters at the bottom of th ditch / apex of the obstacle
	* \param top_width Width in meters at the ground level. Top width must be greater than bottom width
	* \param depth Depth of the ditch in meters (positive number) or height of the obstacle (negative number)
	* \param x0 X (easting) position of the center of the trapezoid
	*/
	void TrapezoidalObstacle(float bottom_width, float top_width, float depth, float x0);

private:
	std::string terrain_type_;
	float terrain_params_[6];
	std::default_random_engine generator_;
	std::normal_distribution<float> distribution_;
};

/// Class to dynamically create a terrain from a series of elevation functions
class TerrainCreator {
public:
	/// TerrainCreator constructor
	TerrainCreator();

	/// TerrainCreator destructor 
	~TerrainCreator();

	/// TerrainCreator copy constructor
	TerrainCreator(const TerrainCreator& s);

	/**
	* Creates a terrain surface with a user-defined size and shape.
	* User provides the boundaries for the lower-left and upper-right corners,
	* And a pointer to a terrain geometry class that inherits from the TerrainElevationFuction base class
	* \param llx X (easting) coordinate of the lower-left (southwest) coordinate of the terrain in local ENU meters
	* \param lly Y (northing) coordinate of the lower-left (southwest) coordinate of the terrain in local ENU meters
	* \param urx X (easting) coordinate of the upper-right (northeast) coordinate of the terrain in local ENU meters
	* \param ury Y (northing) coordinate of the upper-right (northeast) coordinate of the terrain in local ENU meters
	*/
	void CreateTerrain(float llx, float lly, float urx, float ury, float res);

	/// Return the created scene. Must be called after the "CreateScene" function
	//mavs::raytracer::embree::EmbreeTracer GetScene() { return scene_; }

	/// Return a pointer to the created scene. Must be called after the "CreateScene" function
	mavs::raytracer::embree::EmbreeTracer* GetScenePointer() { return &scene_; }

	/**
	* Add a trapezoidal obstacle to the terrain.
	* The obstacle can be a ditch (positive depth) or a hill (negative depth)
	* Runs along the y-direction, traveling along X will cross the obstacle
	* \param bottom_width Width in meters at the bottom of th ditch / apex of the obstacle
	* \param top_width Width in meters at the ground level. Top width must be greater than bottom width
	* \param depth Depth of the ditch in meters (positive number) or height of the obstacle (negative number)
	* \param x0 X (easting) position of the center of the trapezoid
	*/
	void AddTrapezoid(float bottom_width, float top_width, float depth, float x0);

	/**
	* Add roughness to the terrain. Uses a simple RMS model.
	* Input is the desired RMS roughness in meters
	* \param rms The RMS roughness in meters
	*/
	void AddRoughness(float rms);

	/**
	* Add a hole feature to the terrain
	* Specify the (x,y) location of the hole, it's depth, it's diameter, and the side steepness parameter
	* \param x X (easting) coordinate of the center of the hole in local ENU meters
	* \param y Y (northing) coordinate of the center of the hole in local ENU meters
	* \param d Depth of the hole in meters. Negative depth is an obstacle
	* \param s Steepness of the hole sides
	*/
	void AddHole(float x, float y, float depth, float diameter, float steepness);

	/**
	* Add parabolic (constantly increasing slope) to the terrain. Change in slope is along the x-direction
	* There is no variation in the y direction. Elevation = 0 at X = 0
	* Input is the quadratic coefficient
	* \param square_coeff The coefficient as you would define for a parabola with the equation y = a*x*x
	*/
	void AddParabolic(float square_coeff);

	/**
	* Add a slope feature to the terrain. Slope is along the x-direction
	* There is no variation in the y direction. Elevation = 0 at X = 0
	* Input is the fractional slope, i.e. fractional_slope = 45 degrees
	* \param fractional_slope The slope as you would define for a line with the equation y = m*x
	*/
	void AddSlope(float fractional_slope);

private:
	mavs::raytracer::embree::EmbreeTracer scene_;
	std::vector<TerrainElevationFunction> terrain_features_;
}; // TerrainCreator class

} // namespace terraingen
} // namespace mavs

#endif