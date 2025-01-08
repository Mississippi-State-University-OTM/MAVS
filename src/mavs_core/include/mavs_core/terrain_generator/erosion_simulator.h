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
* \class ErosionSimulator
*
* Simulates the erosion of a surface over time due to
* rainfall, presence of rivers, and surface properties.
* 
* Adapted from: 
* "Fast Hydraulic Erosion Simulation and Visualization on GPU" 
* by Mei, Decaudin, and Hu
*
* \author Chris Goodin
*
* \date 8/10/2018
*/
#ifndef EROSION_SIMULATOR_H
#define EROSION_SIMULATOR_H
#include <vector>
#include <glm/glm.hpp>
#include <mavs_core/terrain_generator/random_surface.h>

namespace mavs {
namespace terraingen {

/**
* Struct containing the information 
* to be updated in each cell of the 
* heightmap for the erosion simulation.
*/
struct ErosionCell {
	/// Elevation of the cell in meters
	float terrain_height;
	/// Water height (over terrain height) in meters
	float water_height;
	/// Amount of suspended sediment in the cell
	float susp_sediment;
	/// Water flux going north (+y) through the cell
	float flux_top;
	/// Water flux going south (-y) through the cell
	float flux_bottom;
	/// Water flux going east (+x) through the cell
	float flux_right;
	/// Water flux going west (-x) through the cell
	float flux_left;
	/// Water velocity (m/s) in the cell
	glm::vec2 water_velocity;
	/// Slope of the cell
	float slope;
	/// If the cell is a source, amount added per time step
	float water_source;
};

class ErosionSimulator {
public:
	/// Create empty erosion simulator
	ErosionSimulator();

	/**
	* Create and initialize erosion simulator
	* \param surface The surface to be eroded
	*/
	ErosionSimulator(RandomSurface &surface);

	/// Destory erosion simulator
	~ErosionSimulator();

	/**
	* Initialize empty erosion simulator
	* \param surface The surface to be eroded
	*/
	void Initialize(RandomSurface &surface);

	/**
	* Set rainfall rate of the simulator
	* \param rf The rainfall rate (meters/cell/time step)
	*/
	void SetRainfall(float rf){
		rainfall_rate_ = rf;
	}

	/**
	* Erode the terrain for a given number of steps
	* \param num_steps The number of steps to erode
	*/
	void Erode(int num_steps);

	/// Return the current eroded heightmap
	std::vector<std::vector< float> > GetHeightMap();

	/// Call this if you want to write debug info at each step
	void SaveDebug() {
		save_debug_ = true;
	}

	void SetSedimentCapacityConstant(float k_c) {
		sed_capacity_const_ = k_c;
	}

	void SetSedimentDepositionConstant(float k_d) {
		sed_deposition_const_ = k_d;
	}

	void SetSedimentDissolveConstant(float k_s) {
		sed_dissolve_const_ = k_s;
	}

	void SetWaterEvaporationConstant(float k_e) {
		water_evap_constant_ = k_e;
	}

private:
	//data structure for terrain
	std::vector< std::vector<ErosionCell> > terrain_;

	//environmental constants
	float sed_capacity_const_; //k_c
	float sed_deposition_const_; //k_d
	float sed_dissolve_const_; //k_s
	float water_evap_constant_; //k_e
	float rainfall_rate_;

	// set at initialization
	float pipe_length_; 
	int nx_;
	int ny_;
	bool save_debug_;
	int nsteps_;
	float dt_;
	float elapsed_time_;

	//methods
	void CalculateSlopes();
	void Step();
	void WriteCellData();
};

} //namespace terraingen
} //namespace mavs
#endif
