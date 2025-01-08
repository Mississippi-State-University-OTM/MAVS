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
* \class Species
*
* The species class is for generating ecosystem environments
* Class contains information and methods related to competitive
* growth of species, as well as info about the visualization 
* assets associated with the species
*
* \author Chris Goodin
*
* \date 8/2/2018
*/
#ifndef MAVS_PLANT_SPECIES_H
#define MAVS_PLANT_SPECIES_H

#include <string>

namespace mavs {
namespace terraingen {

class Species {
public:
	/// Create a new species
	Species();

	/**
	* For a plant of this species, determine what the
	* height will be after a year of growth. Uses the
	* exponential growth rate curve to estimate the age
	* and calculate new growth.
	* \param current_height The current_height of the plant (m)
	*/
	float GetNextHeight(float current_height);

	/**
	* Returns the estimated diameter of the plant given
	* a particular height.
	* \param current_height The current height of the plant (m)
	*/
	float GetDiameter(float current_height) {
		return (diameter_height_ratio_*current_height);
	}

	/**
	* Calculate the number of new plants in a given area
	* \param area Growth area in square meters
	*/
	int GetNumNew(float area);

	/// Tell the name/obj file of the species
	std::string GetName() { return name_; }

	/**
	* Set the name / obj file for the species
	* The species name doubles as the associated obj file
	* \param name The name of the obj file, without the path
	*/
	void SetName(std::string name) { name_ = name; }

	/// Return the average lifetime of this species, in years
	float GetLifetime() { return lifetime_; }

	/**
	* Set a minimum height for this species. After the growth simulation,
	* plants of this species still under this height will be removed.
	* \param minsize The minimum height of the species in meters
	*/
	void SetMinHeight(float minsize) { min_height_ = minsize; }

	/// Return the minimum height for this species
	float GetMinHeight() { return min_height_; }

	/// Return the max height for this species
	float GetMaxHeight() { return max_height_; }

	/**
	* Set the number of new plants that spring up each year, 
	* per square meter
	* \param num_new The numbe rof new plants per square meter
	*/
	void SetNumNewPerArea(float num_new) { new_plants_per_year_per_meter_ = num_new; }

	/**
	* Set the growth rate for the species. The growth rate formula is 
	* height = max_height* (1 - exp(-growth_rate *age))
	* max_height[m], age[years], growth_rate[1/years]
	* \param rate The relative growth rate
	*/
	void SetGrowthRate(float rate) { relative_growth_rate_ = rate; }

	/**
	* Set the maximum height in meters that this species can reach
	* \param height The maximum height in meters
	*/
	void SetMaxHeight(float height) { max_height_ = height; }

	/**
	* Set the ratio of diameter to height for this species. This is not 
	* the physical diameter of the plant, but the "competitive" diameter
	* for which it will compete with plants around it.
	* \param ratio diameter/height ratio
	*/
	void SetDiamToHeightRatio(float ratio) { diameter_height_ratio_ = ratio; }

	/**
	* Set the maximum age of the plant, in years.
	* For typical species, 50-100 years works OK
	* \param max_age The maximum age of the speices in years
	*/
	void SetMaxAge(float max_age) { lifetime_ = max_age; }

	/** 
	* Set the unscaled height of the input mesh. 
	* This can be determined from the MAVS mesh_manipulator utility program
	* \param height The default mesh height for the species
	*/
	void SetDefaultHeight(float height) { mesh_default_height_ = height; }

	/**
	* Specify if the mesh associated with the species needs to
	* be rotate from Y-up to Z-up.
	* \param rotate Set to true if the mesh needs to be rotated
	*/
	void SetRotateMesh(bool rotate) { rotate_mesh_ = rotate; }

	/// Return the unscaled height of the species mesh (m)
	float GetDefaultHeight() { return mesh_default_height_; }

	/// Tell if the species mesh needs to be rotated
	bool GetRotate() { return rotate_mesh_; }

private:
	float new_plants_per_year_per_meter_;
	float relative_growth_rate_; //exponential growth
	float max_height_; //meters
	float min_height_; //meters
	float mesh_default_height_;
	bool rotate_mesh_;
	float diameter_height_ratio_; //unitless = diameter = ratio*height
	float lifetime_;
	//std::string mesh_file_; // mesh file associated with species
	std::string name_; // name of the species
};

} //namespace environment
} //namespace mavs

#endif