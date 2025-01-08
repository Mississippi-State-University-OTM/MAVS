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
* \class GridSurface
*
* Class for loading, manipulating, and writing grid files in ascii format
* Assumption is that the input grid file (.asc format) is units of meters, with UTM meter x-y coordinates and meter elevations.
* All calculations are performed in SI units (meters).
*
* \author Chris Goodin
*
* \date 9/26/2018
*/
#ifndef GRID_SURFACE_H
#define GRID_SURFACE_H
#include <string>
#include <mavs_core/terrain_generator/surface.h>
#include <mavs_core/terrain_generator/heightmap.h>

namespace mavs {
namespace terraingen {

class GridSurface : public Surface {	
public:
	/// Constructor
	GridSurface();
	
	/// Construct from HeightMap
	GridSurface(HeightMap &hm);

	/// Destructor
	~GridSurface();
	
	/**
	* Loads a GRD ascii file
	* \param grdName The name of the .asc file to load
	*/
	void LoadAscFile(std::string grdName); 

	/**
	* Add and x,y,z point to the height grid
	* \param x Easting ENU coordinate to add
	* \param y Northing ENU coordinate to add
	* \param z Up ENU coordinate to add
	*/
	void AddPoint(float x, float y, float z);

	/**
	* Set the height at a given cell in the heightmap
	* \param i The horizontal cell index to set
	* \param j The vertical cell index to set
	* \param z The height value to set
	*/
	void SetHeightAtPoint(int i, int j, float z) {
		heightmap_.SetHeight(i, j, z);
	}
 
	/**
	* Writes the .asc file to a 5 column text file
	* horiz_index vert_index UTM_x UTM_Y elev(m) 
	*/
	void WriteToPointsFile(std::string filename); 

	/// Writes the elevations of the current terrain to a grd file
	void WriteElevToGrd(std::string outputfile);
	
	/**
	* Returns a subdomain of the existing grd. Will return an error if the requested subgrid exceeds the size of the grid
	* \param xsize Integer number of grid cells of the requested subgrid in the horizontal dimension
	* \param ysize Integer number of grid cells of the requested subgrid in the vertical dimension
	* \param LLx Integer x-coordinate of the lower left corner of the requested subgrid
	* \param LLy Integer y-coordinate of the lower left corner of the requested subgrid
	*/
	GridSurface GetSubCell(int xsize, int ysize, int LLx, int LLy);

	/**
	* Trims the grid by a certain number of pixels on each side
	* \param trim Number of points to trim from each side
	*/
	GridSurface TrimCell(int trim);

	/**
	* Set what the "NODATA" value for this file will be
	* \param val The "NODATA" value, should be a large negative number
	*/
	void SetNoDataValue(float val) {
		nodata_value_ = val;
	}

	/// Return the value used to flag "nodata"
	float GetNoDataValue() { return nodata_value_; }

	/**
	* Loop through the cells and remove all "No Data" 
	* values by doing a nearest neighbor averaging.
	*/
	void RemoveNoDataCells();

	/**
	* Remove cells above a certain slope and replace with local average
	* \param thresh The slope threshold
	*/
	void Smooth(float thresh);

private:
	float nodata_value_; /**< flag value for no data  */
};
} //namespace terraingen
} //namespace mavs
#endif
