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
 * \class Simulation
 *
 * The simulation class distributes tasks using MPI and manages simulation
 * execution.
 *
 * \author Chris Goodin
 *
 * \date 12/7/2017
 */

#ifndef SIMULATION_H
#define SIMULATION_H

#include <fstream>
#include <vector>
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <mavs_core/messages.h>
#include <vehicles/vehicle.h>
#include <drivers/driver.h>
#include <sensors/sensor.h>
#include <mavs_core/environment/environment.h>
#include <mavs_core/communicator.h>
#include <raytracers/raytracer.h>

namespace mavs{

class Simulation{
 public:
  /// Create the simulation.
  Simulation();

  /// Simulation destructor
  ~Simulation();

  /**
   * Load simulation control file from JSON input
   * \param input_file Full path to the json input file
   */
  void Load(std::string input_file);

  /**
   * Add a vehicle to the simulation. Must be added before "CreateGroups" is 
   * called and the simulation starts. There can only be one.
   * \param vehicle Reference to the vehicle to be added.
   * \param nprocs The number of processors the vehicle simulation will use.
   */
  void AddVehicle(vehicle::Vehicle &vehicle, const int nprocs);

  /**
   * Add a sensor to the simulation. Must be added before "CreateGroups" is 
   * called and the simulation starts. There can be many sensors.
   * \param sensor Reference to the sensor to be added.
   * \param nprocs The number of processors the sensor simulation will use.
   */
  void AddSensor(sensor::Sensor &sensor, const int nprocs);

  /**
   * Add a driver to the simulation. Must be added before "CreateGroups" is 
   * called and the simulation starts. There can only be one.
   * \param driver Reference to the dirver to be added.
   * \param nprocs The number of processors the driver simulation will use.
   */
  void AddDriver(driver::Driver &driver, const int nprocs);

  /**
   * Set the environment for the simulation. Environment will not be updated
   * by the simulation, must be updated externally.
   */
  void SetEnvironment(environment::Environment &env);

#ifdef USE_MPI
  /**
   * Separate all processors into groups of MPI communicators. Must be called 
   * after the driver, vehicle, and sensors have been added, but before the 
   * simulation starts
   */
  void CreateGroups(MPI_Comm parent_comm);
#endif
  
  /// Set the maximum duration of simulated time to max_sim_time seconds.
  void SetMaxSimTime(double max_sim_time){max_sim_time_ = max_sim_time;}

  /// Run the simulation in "self-contained" mode.
  void Run();

	/// Run the simulation in "interactive" mode.
	void RunInteractive();

  /// Run the simulation in time-stepped mode.
  void Update(double dt);

  /// Run the simulation in time-stepped mode, serially.
  void UpdateSerial(double dt);

  /// Get the current real vehicle state.
  VehicleState GetVehicleState();

  /// Determine if the simulation is complete
  bool Complete();

  /// Get elapsed simulation time
  double GetSimTime(){return sim_time_;}
  
  /**
   * Call this to set display_sensors_ = true. 
   * Will display sensor data in X11 window
   */
  void DisplaySensors(){display_sensors_ = true;}

  /**
   * Checks if the simulation is valid and has been allocated correctly
   */
  bool IsValid();
  
 private:
#ifdef USE_MPI 
  void SetCommunicators();
  // info to track the communicators
  MPI_Comm group_comm_;
  MPI_Comm broadcast_to_;
#endif
  
  std::vector<int> process_root_;
  std::vector<CommDef> processes_;
  int group_number_;
  int num_processes_;
  int proc_id_;
  int driver_id_num_;
  int vehicle_id_num_;
  std::vector<int> sensor_id_num_;
  //methods to set the comm info for communicators
  void SetSensor(int nprocs);
  void SetDriver(int nprocs);
  void SetVehicle(int nprocs);

  //pointers to the physical subsystems
  vehicle::Vehicle *vehicle_;
  environment::Environment *environment_;
  driver::Driver *driver_;
  std::vector<sensor::Sensor*> sensors_;

  //scene
  raytracer::Raytracer *scene_;

  //if simulation was loaded from json, will need to free pointers
  bool was_loaded_;
  
  //simulation timing information
  double sim_time_;
  double max_sim_time_;
  double sim_time_step_;

  //display &/or save sensor data?
  bool display_sensors_;
  bool save_data_;
  
  //timing information
  std::ofstream log_;
  double total_wall_;
  double total_sensor_;
  double total_reduce_;
  double total_vehicle_;
  double total_driver_;

  std::string scene_file_;
  
};

}//namespace mavs

#endif
