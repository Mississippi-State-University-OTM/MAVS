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
 * \file embree_datatypes.h
 *
 * Defines some datatypes that are usefule for embree
 * since MAVS doesn't use ISPC by default
 * 
 * \author Chris Goodin
 *
 * \date 7/29/2019
 */
#ifndef EMBREE_DATATYPES_H
#define EMBREE_DATATYPES_H

#include <string>
#include <vector>
#include <raytracers/mesh.h>

namespace mavs{
namespace raytracer{
namespace embree{

/// Mesh vertex structure
struct Vertex {
	float x;
	float y;
	float z;
	float a;
};

/**
* Mesh triangle structure. 
* Each int is an index to a vertex
*/
struct Triangle {
	unsigned int v0;
	unsigned int v1;
	unsigned int v2;
};

/**
* Data for a vegetation species
*/
struct VegSpecies {
	std::string meshfile;
	Mesh mesh;
	std::string name;
	bool rotated;
	float mesh_height;
	int num_added;
};

/**
* A datapoint for a species contains
* a density, min and max trunk diameters
*/
struct VegSpecDescriptor {
	float density;
	float max_diameter;
	float min_diameter;
};

/**
* Each vegetation measurement point has a
* spatial location in local x-y plane and a list
* of spcies descriptors
*/
struct VegDataPoint {
	float x;
	float y;
	std::vector<VegSpecDescriptor> veg;
};

/**
* Holds measured vegetation data
*/
struct VegCheck {
	std::string name;
	float x;
	float y;
};

} //namespace embree
} //namespace raytracer 
} //namespace mavs

#endif