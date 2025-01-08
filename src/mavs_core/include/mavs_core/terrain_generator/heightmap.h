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
* \class Heightmap
*
* Class for storing and manipulating heightmap data
* stored in a regularly spaced x-y grid.
*
* \author Chris Goodin
*
* \date 9/26/2018
*/
#ifndef MAVS_HEIGHTMAP_H
#define MAVS_HEIGHTMAP_H

#include <vector>
#include <glm/glm.hpp>
#include <raytracers/mesh.h>
#include <CImg.h>

namespace mavs {
namespace terraingen {

class HeightMap {
public:
	/// Create an empty heightmap
	HeightMap();

	/// Heightmap copy constructor
	HeightMap(const HeightMap &hm) {
		ll_corner_ = hm.ll_corner_;
		ur_corner_ = hm.ur_corner_;
		scene_area_ = hm.scene_area_;
		resolution_ = hm.resolution_;
		heights_ = hm.heights_;
		max_height_ = hm.max_height_;
		min_height_ = hm.min_height_;
		nx_ = hm.nx_;
		ny_ = hm.ny_;
	}

	/**
	* Set the size of the scene by setting the corners
	* \param lo_x Lower left corner x-coordinate, in local ENU (m)
	* \param lo_y Lower left corner y-coordinate, in local ENU (m)
	* \param hi_x Upper right corner x-coordinate, in local ENU (m)
	* \param hi_y Upper right corner y-coordinate, in local ENU (m)
	*/
	void SetCorners(float lo_x, float lo_y, float hi_x, float hi_y) {
		ll_corner_ = glm::vec2(lo_x, lo_y);
		ur_corner_ = glm::vec2(hi_x, hi_y);
		scene_area_ = (ur_corner_.x - ll_corner_.x)*(ur_corner_.y - ll_corner_.y);
	}

	/**
	* Set the resolution of the heightmap
	* \param res The desired resolution in meters
	*/
	void SetResolution(float res) {
		resolution_ = res;
	}

	/**
	* Set the height of cell (i,j)
	* \param i The horizontal cell index to set
	* \param j The vertical cell index to set
	* \param z The height value to set
	*/
	void SetHeight(int i, int j, float z);

	/**
	* Set the height of cell (i,j)
	* \param x The horizontal coordinate of the point
	* \param y The vertical coordinate of the point
	* \param z The height value to set
	*/
	void AddPoint(float x, float y, float z);

	/**
	* Get the surface slope at a given point
	* \param p The test point in world coordinates
	*/
	glm::vec2 GetSlopeAtPoint(glm::vec2 p);

	/**
	* Return the height of a given point
	*/
	float GetHeightAtPoint(glm::vec2 p);

	/// Get the resolution of the grid.
	float GetResolution() {
		return resolution_;
	}

	/// Return the lower left corner in ENU (m)
	glm::vec2 GetLLCorner() { return ll_corner_; }

	/// Return the upper right corner in ENU (m)
	glm::vec2 GetURCorner() { return ur_corner_; }

	/// Return the area of the surface in m^2
	float GetArea() {
		return scene_area_;
	}

	/// Get the horizontal (E-W) dimension of the heightmap
	size_t GetHorizontalDim() {
		return heights_.size();
	}

	/// Get the vertical (N-S) dimension of the heightmap
	size_t GetVerticalDim() {
		if (heights_.size() > 0) {
			return heights_[0].size();
		}
		else {
			return 0;
		}
	}

	/**
	* Returns the number of cells in the vertical/horizontal direction.
	*/
	glm::ivec2 GetMapDimensions() {
		glm::ivec2 dim(0, 0);
		if (heights_.size() > 0) {
			dim.x = (int)heights_.size();
			dim.y = (int)heights_[0].size();
		}
		return dim;
	}

	/// Returns the X by Y size of the heightmap
	glm::vec2 GetSize() {
		return (ur_corner_ - ll_corner_);
	}

	/**
	* Return the height of a given cell, in meters
	* \param i The horizontal cell index
	* \param j The vertical cell index
	*/
	float GetCellHeight(int i, int j) {
		if (i >= 0 && j >= 0 && i < (int)GetHorizontalDim() && j < (int)GetVerticalDim()) {
			return heights_[i][j];
		}
		else {
			return 0.0f;
		}
	}

	/**
	* Return the height at a given coordinate
	* \param x The horizontal coordinate
	* \param y The vertical coordinate
	*/
	float GetHeightAtPoint(float x, float y);

	/**
	* Returns true if two maps have the same dimensions
	* \param hm The heightmap to compare
	*/
	bool MapsSameSize(HeightMap &hm) {
		bool same = false;
		if (GetHorizontalDim()==hm.GetHorizontalDim() && GetVerticalDim() == hm.GetVerticalDim()) {
			same = true;
		}
		return same;
	}

	/**
	* Change the dimensions of the heightmap. Resets all heights to zero
	* \param nx The horizontal (E-W) dimension
	* \param ny The vertical (N-S) dimension
	*/
	void Resize(int nx, int ny);

	/**
	* Change the dimensions of the heightmap. Resets all heights to NODATA value
	* \param nx The horizontal (E-W) dimension
	* \param ny The vertical (N-S) dimension
	* \param NODATA The value to reset the heights too
	*/
	void Resize(int nx, int ny, float NODATA);

	/// Center the heightmap at (0,0,0)
	void Recenter();

	/// Return a 2D array of the slope of the terrain
	std::vector< std::vector<float> > GetSlopeMap();

	/**
	* Smooth cells that give slopes over a certain threshold
	* \param thresh The slop threshold
	*/
	void CleanHeights(float thresh);

	/**
	* Replace cells that have "nodata" flag with local average
	* \param nodata_value The value signifying missing data in the heightmap
	*/
	void ReplaceCellsByValue(float nodata_value);

	/**
	* Get the slope at a given coordinate
	* \param i The horizontal coordinate
	* \param j The vertical coordinate
	*/
	glm::vec2 GetSlopeAtCoordinate(int i, int j);

	/**
	* Display the current heightmap
	* \param trim If true, then the mouse can be used to trim the heightmap
	*/
	void Display(bool trim);

	/// Display the slope of the heightmap
	void DisplaySlopes();

	/// Return the maximum height of the heightmap
	float GetMaxHeight() {
		return max_height_;
	}

	/// Return the minimum height of the heightmap
	float GetMinHeight() {
		return min_height_;
	}

	/**
	* Create a heightmap from a surface mesh
	* \parameter surface_mesh The mesh to create a heightmap from
	*/
	void CreateFromMesh(std::string surface_file);

	/**
	* Write the current heightmap to an obj file
	* \param surfname The output obj file
	* \param textured Set to true if the output file will be textured
	* \param floor Set to true to add a separate floor to the mesh
	*/
	void WriteObj(std::string surfname, bool textured, bool floor);

	/**
	* Write the current heightmap to an obj file with floor
	* \param surfname The output obj file
	* \param textured Set to true if the output file will be textured
	*/
	void WriteObj(std::string surfname, bool textured) {
		WriteObj(surfname, textured, true);
	}

	/**
	* Save the HeightMap as an image file
	* \param image_name Full path to the save file with extension
	*/
	void WriteImage(std::string image_name);

	/**
	* Get the pose of a vehicle on the terrain at a given point and facing a given direction
	* \param pos The input position
	* \param lt The input "look-to" direction in the planar x-y coordinate system 
	* \param position The output 3D position
	* \param orientation The output 3D orientation
	*/
	void GetPoseAtPosition(glm::vec2 pos, glm::vec2 lt, glm::vec3 &position, glm::quat &orientation);

	/**
	* Create a heightmap from an input grayscale image.
	* \param heightmap The input grayscale image
	* \param resolution The spatial resolution of the map in the horizontal dimension (meters)
	* \param zstep The z-resolution in the vertical dimension of the 8-bit scale.
	*/
	void CreateFromImage(cimg_library::CImg<float> heightmap, float resolution, float zstep);

	/**
	* Downsample the heightmap by a factor of ds_factor
	* Reduces the resolution of the heightmap and the number of cells
	* \param ds_factor The downsample factor
	*/
	void Downsample(int ds_factor, float nodata_val);

	/**
	* Get the x-y coordinage of a map index in ENU
	* \param i The horizontal map index
	* \param j The vertical map index
	*/
	glm::vec2  IndexToCoordinate(int i, int j);

private:
	void CropSelf(int xsize, int ysize, int LLx, int LLy);

	float GetLocalAverage(int i, int j, float nodata_value,float window_size);

	float GetLocalAverage(int i, int j, float nodata_value);

	glm::ivec2 CoordinateToIndex(float x, float y);

	glm::ivec2 CoordinateToIndex(glm::vec2 v);

	std::vector< std::vector<float> > heights_;

	glm::vec2 ll_corner_;
	glm::vec2 ur_corner_;
	float max_height_;
	float min_height_;
	float scene_area_;
	float resolution_;
	int nx_;
	int ny_;
};

} //nampespace terraingen
} //namespace heightmap

#endif
