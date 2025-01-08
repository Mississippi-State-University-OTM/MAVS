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
* \class ChronoWheeledJson
*
* MAVS Vehicle wrapper for Chrono Vehicle model defined by JSON inputs
* Note that Chrono only currently supports flat rigid terrain
* Support for mesh terrains will need to be implemented separately
*
* \author Chris Goodin
*
* \date 5/11/2018
*/
#ifndef CHRONO_WHEELED_JSON_H
#define CHRONO_WHEELED_JSON_H

#ifdef USE_CHRONO

// MAVS includes
#include <vehicles/vehicle.h>
// Chrono includes 
#include <chrono_vehicle/ChVehicleModelData.h>
#include <chrono_vehicle/ChConfigVehicle.h>
#ifdef Success
#undef Success
#endif
#ifdef Bool
#undef Bool
#endif
#ifdef Status
#undef Status
#endif
#include <chrono_vehicle/utils/ChUtilsJSON.h>
#include <chrono_vehicle/powertrain/SimplePowertrain.h>
#include <chrono_vehicle/terrain/RigidTerrain.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>
#include <chrono_vehicle/wheeled_vehicle/tire/RigidTire.h>

namespace mavs{
namespace vehicle{

struct TireKinematicState {
	double slip;
	double slip_angle;
	double camber_angle;
};

class ChronoWheeledJson : public Vehicle {
 public:
  /// Create the vehicle
  ChronoWheeledJson();

	/// Destructor
	~ChronoWheeledJson();

  /**
   * Inherited update method from vehicle base class
   * \param env Pointer to the environment, which includes the terrain surface
   * \param throttle Throttle setting from 0 to 1
   * \param steer Steering setting from -1 to 1
   * \param dt Time step in seconds
   */
  void Update(environment::Environment *env, float throttle, float steer, float brake, float dt);

  /**
	* Inherited method to load input file.
	* Should be a json file listing the vehicle,
	* terrain, powertrain, and tire inputs.
	* \param input_file Input file in json format
	*/
  void Load(std::string input_file);

	/** 
	* Get the longitudinal speed in m/s, as you might consider it on a speedometer
	* Inherited from base vehicle class
	*/
	float GetSpeed() {
		return (float)vehicle_->GetVehicleSpeed();
	}

	/**
	* Get position of the ith tire
	* Inherited from vehicle base class
	*/
	glm::vec3 GetTirePosition(int i);

	/**
	* Get orientation of the ith tire
	* Inherited from vehicle base class
	*/
	glm::quat GetTireOrientation(int i);

	/**
	* Get the terrain forces on a tire
	*/
	glm::vec3 GetTireTerrainForces(int i);

 private:

	 void GetInputFiles(std::string input_file);

	 double time_;

	 double max_dt_;

	 std::string vehicle_file_;

	 std::string terrain_file_;

	 std::string powertrain_file_;

	 std::string tire_file_;

	 chrono::vehicle::WheeledVehicle *vehicle_;

	 chrono::vehicle::RigidTerrain *terrain_;

	 std::shared_ptr<chrono::vehicle::ChPowertrain> powertrain_;

	 chrono::vehicle::ChDriver::Inputs driver_inputs_;
};

}// namespace vehicle
}// namespace mavs

#endif
#endif
