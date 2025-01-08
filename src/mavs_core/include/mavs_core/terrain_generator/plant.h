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
* \class Plant
*
* The plant class specifies instances of 
* species with certain location and height.
*
* \author Chris Goodin
*
* \date 8/2/2018
*/
#ifndef MAVS_PLANT_H
#define MAVS_PLANT_H

#include <mavs_core/terrain_generator/species.h>
#include <glm/glm.hpp>

namespace mavs {
namespace terraingen {

class Plant {
public:
	///  Create a plant
	Plant();

	/**
	* Grow the plant based on the growth rate of the species.
	* Once plant exceeds max age, it will die.
	*/
	void Grow();

	/**
	* Set the 2D position of the plant
	* \param p The position of the plant in local ENU coordinates
	*/
	void SetPosition(glm::vec2 p) { position_ = p; }

	/**
	* Set the height of the plant in meters
	* \param h Height of the plant in meters
	*/
	void SetHeight(float h) { height_ = h; }
	
	/**
	* Return the x-y position of the plant in local ENU
	*/
	glm::vec2 GetPosition() { return position_; }

	/**
	* Get the effective competitive radius of the plant
	*/
	float GetRadius() { return radius_; }

	/// Get the current height of plant in meters
	float GetHeight() { return height_; }

	/// Get the maximum height of plant in meters
	float GetMaxHeight() { return species_.GetMaxHeight(); }

	/**
	* Set the maximum height of the plant in meters. 
	* This overrides the max-height of the species
	* \param height Max height of the plant (m)
	*/
	void SetMaxHeight(float height) { species_.SetMaxHeight(height); }

	/// Kill the plant
	void Kill() { alive_ = false; }

	/// Tell if the plant is still alive
	bool IsAlive() { return alive_; }

	/**
	* Set the species of the plant
	* \param species Desired species of the plant
	*/
	void SetSpecies(Species species) { species_ = species; }

	/**
	* Return a pointer to the plant speices
	*/
	Species *GetSpecies() { return &species_; }

	/// Get the name of the plant species
	std::string GetSpeciesName() { return species_.GetName(); }

	/**
	* Set the coordinates of the spatial grid that 
	* the plant belongs to.
	* \param i Coordinate in the x/east direction
	* \param j Coordinate in the y/north direction
	*/
	void SetCell(int i, int j) {
		cell_ = glm::ivec2(i, j);
	}

	/// Tell which spatial grid cell the plant is in.
	glm::ivec2 GetCell() { return cell_; }

private:
	glm::vec2 position_;
	glm::ivec2 cell_;
	float height_;
	Species species_;
	float radius_;
	float age_;
	bool alive_;
};

} //namespace terraingen
} //namespace mavs


#endif

