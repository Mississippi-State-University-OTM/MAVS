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
* \class RandomScene
*
* The RandomScene class creates a randomized environment
* and can also create a trail through that environment, 
* along with corresponding ANVEL replay file.
*
* \author Chris Goodin
*
* \date 9/13/2018
*/
#ifndef RANDOM_SCENE_H
#define RANDOM_SCENE_H

#include <vector>
#include <string>
#include <algorithm>
#include <glm/glm.hpp>
#include "mavs_core/math/constants.h"

namespace mavs {
namespace terraingen {

/**
* Struct to hold info about a pothole
* 
*/
struct Pothole {
	float depth;
	float diameter;
	glm::vec2 location;
};

/**
* Struct to hold inputs defining the random scene
* Json file with inputs created with the MAVS GUI
*/
struct RandomSceneInputs {
	RandomSceneInputs() {
		terrain_width = 20.0f;
		terrain_length = 20.0f;
		lo_mag = 1.0f;
		hi_mag = 0.01f; 
		mesh_resolution = 0.25f;
		trail_width = 0.0f;
		track_width = 0.0f;
		wheelbase = 0.0f;
		eco_file = "";
		output_directory = "./";
		basename = "scene";
		dem_file = "";
		vprp_file = "";
		plant_density = 0.0f;
		surface_rough_type = "perlin"; // can be perlin, gaussian, or variable
	}
	float terrain_width;
	float terrain_length;
	float lo_mag;
	float hi_mag;
	float mesh_resolution;
	float trail_width;
	float track_width;
	float wheelbase;
	std::string path_type;
	std::string eco_file;
	std::string output_directory;
	std::string basename;
	std::string dem_file;
	std::string vprp_file;
	std::string surface_rough_type;
	float plant_density;
	std::vector<Pothole> potholes;
	//bool potholes;
	//float pothole_depth;
	//float pothole_diameter;
	//int number_potholes;
	//std::vector<glm::vec2> pothole_locations;
};

class RandomScene{
public:
	/// Create an empty random scene
	RandomScene();

	/**
	* Load a random scene input file
	* The inputs can be created with the MAVS GUI
	* \param infile The full path to the json input file
	*/
	void Load(std::string infile);

	/// Create the scene with the specified parameters
	void Create();

	/**
	* Set the inputs to the random scene creator directly
	* \param inputs The desired inputs of the random scene
	*/
	void SetInputs(RandomSceneInputs inputs);

	/// Print current inputs to stdout
	void PrintInputs();

	/// Returns the working output directory
	std::string GetOutputDirectory() { return inputs_.output_directory; }

	void SetGapProperties(float gap_width, float gap_depth, float gap_angle_radians) {
		gap_width_ = gap_width;
		gap_depth_ = gap_depth;
		gap_angle_radians = std::min((float)mavs::kPi_2, std::max(0.0f, gap_angle_radians));
		gap_angle_radians_ = (float)mavs::kPi_2 - gap_angle_radians;
	}

private:

	void WritePotholeFile(std::vector<Pothole> potholes, std::string basename);

	void CheckOutputDirectoryIsValid();

	RandomSceneInputs inputs_;
	bool loaded_;
	bool use_dem_;
	bool use_vprp_;

	float gap_width_, gap_depth_, gap_angle_radians_;
};

} //namespace terraingen
} //namespace mavs
#endif