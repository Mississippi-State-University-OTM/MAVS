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
 * \class SensorSimulation
 *
 * The sensor simulation class reads a pose file and moves a sensor along
 * the path, writing out the data. The intent is to provide a framework
 * for doing sensor analysis
 *
 * \author Chris Goodin
 * \author Christopher Hudson
 *
 * \date 10/31/2018
 */

#ifndef SENSOR_SIMULATION_H
#define SENSOR_SIMULATION_H
#include <mavs_core/messages.h>
#include <sensors/sensor.h>
#include <mavs_core/environment/environment.h>
#include <raytracers/raytracer.h>

namespace mavs{

class SensorSimulation{
 public:
  /// Create the simulation.
  SensorSimulation();

  /// Simulation destructor
  ~SensorSimulation();

  /**
   * Load sensor definition file from JSON input
   * \param input_file Full path to the json input file
   */
  void Load(std::string input_file);

  /**
   * Run the sensor simulation
   */
  void Run();

  /**
   * Check if the simulation is valid and pointers are valid
   */
  bool IsValid();
  
 private:
  environment::Environment *environment_;
  raytracer::Raytracer *scene_;

	std::vector<sensor::Sensor*> sensors_;

  bool was_loaded_,display_sensors_,save_sensors_;

  std::vector<mavs::Pose> poses_;

  void LoadTextPoses(std::string pose_file);

};

}//namespace mavs

#endif
