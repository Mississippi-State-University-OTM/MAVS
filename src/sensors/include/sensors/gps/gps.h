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
 * \class Gps
 * 
 * Simulates a GPS sensor, including the motion of the satellites in the sky, 
 * ionosphere and troposphere induced errors, and trilateration errors. 
 * 
 * Based on Durst and Goodin, "High fidelity modelling and simulation of inertial sensors commonly used by
 * autonomous mobile robots", World Journal of Modelling and Simulation
 * Vol. 8 (2012) No. 3, pp. 172-184
 * and references therein.
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */

#ifndef GPS_H
#define GPS_H

#include <vector>
#include <string>
#include <fstream>

#include <glm/glm.hpp>

#include <sensors/sensor.h>
#include <sensors/gps/satellite.h>
#include <mavs_core/coordinate_systems/coord_conversions.h>

namespace mavs{
namespace sensor{
namespace gps{

class Gps : public Sensor {
 public:
  /// Create the simulated GPS sensor
  Gps();

  /// GPS destructor
  ~Gps();

    /// copy contstructor
  Gps(const Gps &s){
    enu_ = s.enu_;
    reciever_position_ = s.reciever_position_;
    sat_status_ = s.sat_status_;
    sat_fix_ = s.sat_fix_;
    gps_mode_ = s.gps_mode_;
    vel_enu_ = s.vel_enu_;
  }

  void SaveRaw();

  /** 
   * Load the gps nav file that is used to initialize the positions and 
   * velocities of the satellites.
  */
  void LoadNavFile(std::string inFile);
  
  /** 
   * Update the GPS sensor.
   * \param env The current environment.
   * \param dt The amount of time to simulate (seconds).
   */
  void Update(environment::Environment *env, double dt);

  /**
   * Set the type of GPS. 
   * 0 = normal
   * 1 = dual band (satellite augmented)
   * 2 = differential (ground augmented)
   * 3 = "perfect" sensor with no error
	 * \param gps_type The integer gps type
   */
  void SetType(int gps_type);

	/**
	* Set the type of GPS.
	* \param gps_type Options are "normal", "dual band", "differential", or "perfect"
	*/
	void SetType(std::string gps_type);

  /// Set the true position of the reciever in local ENU coordinates.
  void SetRecieverTruePositionENU(double x, double y, double z);

  /// Get the simulated position of the receiver in local ENU coordinates.
  glm::dvec3 GetRecieverPositionENU(){return enu_;}

	/// Return the reciever position in LLA
	glm::dvec3 GetRecieverPositionLLA();

	/// Return the reciever position in UTM
	glm::dvec3 GetRecieverPositionUTM();

  /// Get the simulated velocity of the receiver in local ENU coordinates.
  glm::dvec3 GetRecieverVelocityENU(){ return vel_enu_;}
  
  /// Get the current number of satellite signals
  size_t GetNumSignals(){return broadcast_distances_.size();}
  
  /// Return the NavSatStatus message
  NavSatStatus GetRosNavSatStatus();

  ///return the NavSatFix message
  NavSatFix GetRosNavSatFix();

	/// The the dilution of precision for the position calculation
	glm::dvec3 GetPositionDop() {
		return dilution_of_precision_;
	}

#ifdef USE_MPI  
  /** 
   * Publish the current simulated reciever position to the simulation 
   * communicator.
   */
  void PublishData(int root, MPI_Comm broadcast_to);
#endif
  
  /// make a deep copy
  virtual Gps* clone() const{
    return new Gps(*this);
  }

protected:
	int nsteps_;

 private:
  ///Initialize satellites with default data
  void SetDefaultSatellites();
  void Trilaterate();
  double TraceTo(glm::dvec3 rec_pos, glm::dvec3 sat_posm,
		 environment::Environment *env);
  void GetSignals(environment::Environment *env);
  void CalculateRecieverPositionENU();
  glm::dvec3 GetPosENU(int satNum);
  coordinate_system::CoordinateConverter coord_convert_;
  std::vector<Satellite> satellite_;
  glm::dvec3 reciever_position_;
  glm::dvec3 last_reciever_position_;
  glm::dvec3 reciever_true_position_;
  glm::dvec3 vel_enu_;
  glm::dvec3 enu_;
  coordinate_system::LLA local_origin_lla_;
  std::vector<glm::dvec3> broadcast_positions_;
  std::vector<double> broadcast_distances_;
  double local_latitude_;
  double local_altitude_;

	glm::dvec3 dilution_of_precision_;

  //atmospheric variables for error calculations
  double z200_;
  double pressure_kpa_;
  double ErrorEphem(double r, double theta);
  double ErrorIono(double epsilon);
  double ErrorTropoWet(double epsilon, double theta);
  double ErrorTropoDry(double theta);
  double GetAtmosphericErrors(double r, double epsilon, double theta);

  // return variables for the GPS messages
  NavSatStatus sat_status_;
  NavSatFix sat_fix_;
  int gps_mode_;
	std::map<std::string, int> gps_types_;

  // variables for multipath caclulation
  double multipath_cutoff_dist_;
  std::vector<glm::dvec3> multipath_check_directions_;
  void ComputeMultipathDirections();

  /// Calculates the reciever position in ECEF for a "perfect" sensor
  void CalculatePerfect();

  // log file stream
  void WriteState();
  //std::ofstream fout_;
};

} //namespace gps
} //namespace sensor
} //namespace mavs

#endif
