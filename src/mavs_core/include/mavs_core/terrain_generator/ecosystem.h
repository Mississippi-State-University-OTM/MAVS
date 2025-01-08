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
* \class Ecosystem
*
* The ecosystem class can be used to create
* realistic natural scenes by "growing" competing
* species for any number of years. 
*
* \author Chris Goodin
*
* \date 8/2/2018
*/
#ifndef MAVS_ECOSYSTEM_H
#define MAVS_ECOSYSTEM_H

#include <vector>
#include <map>
#include <string>

#include <mavs_core/terrain_generator/plant.h>
#include <mavs_core/terrain_generator/heightmap.h>
#include <mavs_core/pose_readers/trail.h>
#include <mavs_core/pose_readers/waypoints.h>
#include <raytracers/mesh.h>

namespace mavs {
namespace terraingen {

typedef std::vector<int> plant_list;

class Ecosystem {
public:
	/// Create an ecosystem
	Ecosystem();

	/**
	* Load a terrain file that will be the ecosystem surface
	* \param terrain_file Full path to surface obj file
	*/
	void LoadTerrain(std::string terrain_file);

	/**
	* Initialize the trail properties for the ecosystem
	*/
	void InitializeTrail();

	/**
	* Progress the ecosystem simulation by 1 year
	*/
	void Step();

	/**
	* Save the plant position, species number, 
	* height, and diameter to a flat, space 
	* delimited column file.
	* \param outfile Full path to the text file to save.
	*/
	void SaveEcosystemToFile(std::string outfile);

	/**
	* Save the ecosystem to a MAVS scene file
	* \param outfile Full path to the json file to save.
	*/
	void SaveToMavsScene(std::string outfile);

	/**
	* Add a plant species to the ecosystem. 
	* \param species Species to add
	*/
	void AddSpecies(Species species) {
		species_.push_back(species);
		int num = (int)species_.size();
		species_num_[species_.back().GetName()] = num;
	}

	/**
	* Remove all plants less than the minimum 
	* species height from the ecosystem. It 
	* should be called before saving a scene file.
	*/
	void PruneRunts();

	/// Tell the total number of plants in the ecosystem
	int GetNumberOfPlants() { return (int)plants_.size(); }

	/**
	* Set the directory where the surface and veg meshes
	* are located.
	* \param path_to_meshes Full file path to mesh directory
	*/
	void SetPathToMeshes(std::string path_to_meshes) {
		path_to_meshes_ = path_to_meshes;
	}

	/**
	* Set the number of years to run the simulation
	* \param years Number of simulated years to run the sim
	*/
	void SetSimLength(int years) { sim_length_years_ = years; }

	/**
	* Set the ground clearance of the vehicle that
	* drives on the trail. Vegetation on the trail
	* will not grow above this height.
	* \param clearance The ground clearance of the trail vehicle, in meters
	*/
	void SetGroundClearance(float clearance) { ground_clearance_ = clearance; }

	/**
	* Run the ecosystem simulation using the previously
	* loaded parameters
	*/
	void Simulate();

	/**
	* Load an ecosystem from a json file
	* \param jsonfile The input file to load
	*/
	void Load(std::string jsonfile);

	/**
	* Set the trail through the ecosystem
	* Use when there's only one trail
	* \param trail The desired trail
	*/
	void SetTrail(Trail &trail);

	/**
	* Add a trail to the the ecosystem
	* Use when there are multiple trails
	* \param trail The desired trail
	*/
	void AddTrail(Trail &trail);

	/**
	* Set the MAVS HeightMap to use for calculating heights
	* \param hm The Mavs HeightMap
	*/
	void SetHeightMap(HeightMap &hm) {
		surf_ = hm;
	}

	/**
	* Set scaling factor for ecosystem growth rates
	* \param fac The scaling factor. 1.0=normal growth
	*/
	void SetGrowthFactor(float fac) {
		growth_factor_ = fac;
	}

	/**
	* Set the name of the pothole file associated with this ecosystem
	*/
	void SetPotholeFile(std::string pothole_file) { pothole_file_ = pothole_file; }

private:

	int sim_length_years_;

	float growth_factor_;

	/// Set the bounds of the ecosystem
	void SetCorners(glm::vec2 lower_left,
		glm::vec2 upper_right) {
		llc_ = lower_left;
		urc_ = upper_right;
	}

	//plant info
	void AddPlants();
	std::vector<Species> species_;
	std::map<std::string, int> species_num_;
	std::vector<Plant> plants_;
	std::string path_to_meshes_;
	std::string pothole_file_;
	/// Tell of two plants overlap / compete
	bool PlantsOverlap(Plant *plant1, Plant *plant2);

	//surface info
	float GetTerrainHeightAtPoint(glm::vec2 point);
	raytracer::Mesh surface_mesh_;
	bool terrain_loaded_;
	HeightMap surf_;

	//path info
	glm::vec2 llc_;
	glm::vec2 urc_;
	float ecosys_area_;

	//variables and methods for creating tracks
	// in the road
	float ground_clearance_;
	//Trail trail_;
	std::vector<Trail> trails_;
	std::string texture_file_;
	std::string surface_mesh_file_;
	bool use_textures_;

	// info for acceleration structure
	// for plant overlap
	float grid_cell_size_;
	int nx_cells_;
	int ny_cells_;
	std::vector< std::vector<plant_list> > cells_;
};

} //namespace terraingen
} //namespace mavs
#endif