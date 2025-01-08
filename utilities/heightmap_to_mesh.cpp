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
* \file heightmap_to_mesh.cpp
*
* Create a mesh from a heightmap file
*
* Usage: >./heightmap_to_mesh heightmap.bmp resolution z_scale (lower_left_corner_x lower_left_corner_y)
*
* heightmap.bmp is the input heightmap file
*
* resolution is the pixel resolution of the heigthmap in meters
* z_scale is the range of z-values over the heightmap corresponding from 0-255 pixel values
* lower_left_corner_x (optional) is the x-coordinate of the lower left corner of the height map in local ENU meters
* lower_left_corner_y (optional) is the y-coordinate of the lower left corner of the height map in local ENU meters
*
* \author Chris Goodin
*
* \date 1/23/2023
*/
// C++ includes
#include <iostream>
// MAVS includes
#include <mavs_core/terrain_generator/heightmap.h>

int main(int argc, char *argv[]) {
	if (argc<4) {
		std::cerr << "Usage: ./heightmap_to_mesh heightmap.bmp resolution z_scale (lower_left_corner_x) (lower_left_corner_y)" << std::endl;
		return 1;
	}

	std::string scene_file(argv[1]);
	float res = (float)atof(argv[2]);
	float z_scale = (float)atof(argv[3]);

	cimg_library::CImg<float> img;
	img.load(scene_file.c_str());

	mavs::terraingen::HeightMap hm;
	hm.CreateFromImage(img, res, z_scale);

	if (argc > 5) {
		float llx = (float)atof(argv[4]);
		float lly = (float)atof(argv[5]);
		int nx = (int)hm.GetHorizontalDim();
		int ny = (int)hm.GetVerticalDim();
		hm.SetCorners(llx, lly, llx + nx * res, lly + ny * res);
	}

	hm.WriteObj("converted_surface", true, false);

	return 0;
}


