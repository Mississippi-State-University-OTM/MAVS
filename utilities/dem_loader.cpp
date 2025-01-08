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
* \file dem_loader.cpp
*
* Load a DEM in .asc format and display it
*
* Usage: >./dem_loader asc_file.asc
*
* where the ASC file is an ascii DEM file.
*
* An example can be found in mavs/data/hmf_files
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <iostream>
#include <string>

#include <mavs_core/terrain_generator/grd_surface.h>

int main(int argc, char *argv[]) {

	if (argc < 2) {
		std::cerr << "ERROR: no .asc file supplied " << std::endl;
		return 1;
	}
	std::string infile(argv[1]);

	mavs::terraingen::GridSurface surface;

	surface.LoadAscFile(infile);

	//surface.RemoveNoDataCells();

	//surface.Recenter();

	int nx = (int)surface.GetHeightMapPointer()->GetHorizontalDim();
	int ny = (int)surface.GetHeightMapPointer()->GetVerticalDim();

	std::cout << "Loaded heightmap of dimension " << nx << "X" << ny << std::endl;

	//mavs::terraingen::GridSurface new_surface = surface.GetSubCell(1200, 1200, 1000, 1000);

	//surface.GetHeightMapPointer()->Downsample(2, surface.GetNoDataValue());

	//int nx_new = (int)surface.GetHeightMapPointer()->GetHorizontalDim();
	//int ny_new = (int)surface.GetHeightMapPointer()->GetVerticalDim();

	//std::cout << "Resized to " << nx_new << "X" << ny_new << std::endl;

	//surface.DisplaySlopes();

	//surface.Display();

	surface.WriteObj("from_dem");

	return 0;
}