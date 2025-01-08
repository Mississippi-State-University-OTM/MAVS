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
 * \class Communicator
 *
 * Base class for all simulation participants that sorts them into
 * separate MPI communication blocks.
 *
 * \author Chris Goodin
 *
 * \date 12/7/2017
 */

#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <string>

#ifdef USE_MPI
#include <mpi.h>
#endif

namespace mavs{

/**
 * Structure that holds basic info on a communicator, without actually
 * being a communicator.
 */
struct CommDef{
  ///Number of processors in the communicator
  int num_procs;
  
  ///Unique identifier for the process in the communicator
  int id_num;
  
  ///Type of the communicator. Could be "sensor", "vehicle", or "driver".
  std::string type;
};

class Communicator{
 public:
  Communicator(){
#ifdef USE_MPI
    comm_ = MPI_COMM_WORLD;
#endif
    comm_rank_ = 0;
    comm_size_ = 1;
    local_sim_time_ = 0.0;
    local_time_step_ = 0.01;
    log_data_ = false;
  }

#ifdef USE_MPI  
  /// Set the MPI communicator
  void SetComm(MPI_Comm incomm){
    MPI_Comm_dup(incomm, &comm_);
    MPI_Comm_rank(comm_, &comm_rank_);
    MPI_Comm_size(comm_, &comm_size_);
  }
#endif  

#ifdef USE_MPI
  /**
   * Virtual method that inheriting classes should overwrite in order to 
   * communicate data back to other communciators back in the simulation.
   * \param root The root process in the communicator that data is being to. 
   * It could be MPI_COMM_WORLD or another sub-communicator.
   */
  virtual void PublishData(int root, MPI_Comm comm_to){}
#endif 
  /**
   * Returns the simulation time of the communicator. The simulation manager
   * will only call update functions when the simulation time is >= the local 
   * sim time for the communicator.
   */
  double GetLocalSimTime(){return local_sim_time_;}

  /**
   * Set the local time step to a desired value
   */
  void SetTimeStep(double dt){local_time_step_ = dt;}
  
  ///Tell if the sensor has been updated since the last publish
  bool Updated(){return updated_;}

  ///Call this to turn on data logging for the communicator
  void LogData(std::string fname){log_data_=true; log_file_name_ = fname;}

  /// Method to load data from input file. Derived classes should implement
  virtual void Load(std::string input_file){}
  
  /**
   * Virtual method to display the current sensor data in an X11 window
   */
  virtual void Display(){}

	/**
	* Set the requested number of processors for the process
	* \param np The requested number of processors
	*/
	void SetNumProcs(int np) {
		num_procs_ = np;
	}

	/**
	* Get the requested number of processors for the process
	*/
	int GetNumProcs() {
		return num_procs_;
	}

 protected:
#ifdef USE_MPI
  MPI_Comm comm_;
#endif
  int comm_rank_;
  int comm_size_;

  double local_sim_time_;
  double local_time_step_;

  ///Only true if the communicator has been updated since the last publish
  bool updated_;

  ///true if you want to log data at each update
  bool log_data_;
  std::string log_file_name_;

	int num_procs_;
};

} //namespace mavs

#endif
