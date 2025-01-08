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
 * \class vehicle
 *
 * The base class for a vehicle simulation, which may or may not include 
 * a powertrain, VTI simulation, and vehicle dynamics module
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#undef None
#include <fstream>

#include <mavs_core/communicator.h>
#include <mavs_core/messages.h>
#include <mavs_core/environment/environment.h>
#include <glm/gtx/quaternion.hpp>

namespace mavs{
namespace vehicle{

class Vehicle : public Communicator {
 public: 
  Vehicle(){}

  ~Vehicle(){if (fout_.is_open())fout_.close();}

  /**
   * A vehicle must update with a steering and throttle command from the 
   * driver. The environment is also passed in to communicate the terrain
   * conditions.
   * \param env Reference to the current environment.
   * \param throttle Commanded throttle from 0 to 1.
   * \param steer Commanded steering angle (radians).
	 * \param dt The time step for the update
   */
  virtual void Update(environment::Environment *env, float throttle, float steer, float brake, float dt){};

  /**
   * Manually set the state of the vehicle in cartesian ENU coordinates.
   */
  void SetState(VehicleState veh_state) {
	  current_state_ = veh_state;
  }

  /**
   * Manually set the position of the vehicle in cartesian ENU coordinates.
   */
  void SetPosition(double x, double y, double z){
    current_state_.pose.position.x = x;
    current_state_.pose.position.y = y;
    current_state_.pose.position.z = z;
  }

  /**
   * Manually seet the orientation of the vehicle in ENU coordinates.
   */
  void SetOrientation(double w, double x, double y, double z){
    current_state_.pose.quaternion.w = w;
    current_state_.pose.quaternion.x = x;
    current_state_.pose.quaternion.y = y;
    current_state_.pose.quaternion.z = z;
  }

  ///Returns the current state of the vehicle.
  VehicleState GetState(){return current_state_;}

	/// Returns the current state as an odometry message
	Odometry GetOdometry(){
		Odometry odom;
		odom.pose.pose.position = current_state_.pose.position;
		odom.pose.pose.quaternion = current_state_.pose.quaternion;
		odom.twist.twist.angular = current_state_.twist.angular;
		odom.twist.twist.linear = current_state_.twist.linear;
		return odom;
	}

	/// Return the current vehicle position
	glm::vec3 GetPosition() {
		return current_state_.pose.position;
	}

	/// Return the current vehicle linear acceleration in world coordinates
	glm::vec3 GetLinearAcceleration() {
		return current_state_.accel.linear;
	}

	/// Return the current vehicle linear angular in world coordinates
	glm::vec3 GetAngularAcceleration() {
		return current_state_.accel.angular;
	}

	/// Return the current lateral acceleration in vehicle coordinates
	float GetLateralAcceleration() {
		glm::vec3 linear = current_state_.accel.linear;
		float lat_acc = glm::dot(linear, GetLookSide());
		return lat_acc;
	}

	/// Return the current longitudinal acceleration in vehicle coordinates
	float GetLongitudinalAcceleration() {
		glm::vec3 linear = current_state_.accel.linear;
		float lon_acc = glm::dot(linear, GetLookTo());
		return lon_acc;
	}

	/// Return the current orientation
	glm::quat GetOrientation() {
		return current_state_.pose.quaternion;
	}

	/// Get the "Look To" vector of the vehilce
	glm::vec3 GetLookTo() {
		glm::mat3 R = glm::toMat3(current_state_.pose.quaternion);
		glm::vec3 lt = R[0];
		return lt;
	}

	/// Get the "Look Side" vector of the vehilce
	glm::vec3 GetLookSide() {
		glm::mat3 R = glm::toMat3(current_state_.pose.quaternion);
		glm::vec3 ls = R[1];
		return ls;
	}

	/// Get the "Look Up" vector of the vehilce
	glm::vec3 GetLookUp() {
		glm::mat3 R = glm::toMat3(current_state_.pose.quaternion);
		glm::vec3 lu = R[2];
		return lu;
	}
  
	/// Get the longitudinal speed in m/s, as you might consider it on a speedometer
	virtual float GetSpeed() {
		return (float)(sqrt(current_state_.twist.linear.x*current_state_.twist.linear.x + current_state_.twist.linear.y*current_state_.twist.linear.y));
	}

	/// Get position of the ith tire
	virtual glm::vec3 GetTirePosition(int i) { return glm::vec3(0.0f, 0.0f, 0.0f); }

	/// Get orientation of the ith tire
	virtual glm::quat GetTireOrientation(int i) { return glm::quat(1.0f, 0.0f, 0.0f, 0.0f); }

#ifdef USE_MPI  
  /**
   * Publish the current vehicle state to MPI communicator.
   * \param root Number of the root processor to publish from.
   * \param broadcast_to MPI communicator to send the state to.
   */
  void  PublishData(int root, MPI_Comm broadcast_to){
    MPI_Bcast(&current_state_.pose.position,3,MPI_DOUBLE,root,broadcast_to);
    MPI_Bcast(&current_state_.pose.quaternion,4,MPI_DOUBLE,root,broadcast_to);
    MPI_Bcast(&current_state_.twist.linear,3,MPI_DOUBLE,root,broadcast_to);
  }
#endif
  
  ///Writes the current vehicle state to a log file
  void WriteState(){
    if (!fout_.is_open()){
      fout_.open(log_file_name_.c_str());
      fout_<<"Time(sec) Pos.x Pos.y Pos.z Throttle Steering"<<std::endl;
    }
    fout_<<local_sim_time_<<" "<<
      current_state_.pose.position.x<<" "<<
      current_state_.pose.position.y<<" "<<
      current_state_.pose.position.z<<" "<<
      current_state_.twist.linear.x<<" "<<
      current_state_.twist.linear.y<<" "<<
      current_state_.twist.linear.z<<
      std::endl;
  }
  
  /** 
   * Set the CG height of the vehicle.
   * \param cg_height CG height in meters.
   */
  void SetCgHeight(double cg_height){cg_height_ = cg_height;}

	/**
	* Return the current actual steering angle in radians.
	* current_steering_ must be set by derived classes.
	*/
	float GetSteeringAngle() { return current_steering_radians_; }

 protected:
  VehicleState current_state_;
	float current_steering_radians_;
  std::ofstream fout_;
  double cg_height_;
};

} //namespace vehicle
} // namespace mavs
#endif
