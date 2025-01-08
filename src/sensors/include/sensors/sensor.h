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
 * \class Sensor
 *
 * The base class for all exteroceptive sensors like camera, lidar, and gps.
 *
 * \author Chris Goodin
 *
 * \date 12/7/2017
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
#include <map>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include <mavs_core/communicator.h>
#include <mavs_core/messages.h>
#include <mavs_core/environment/environment.h>
#include <raytracers/raytracer.h>
#include <sensors/annotation_colors.h>
#include <sensors/annotation.h>

namespace mavs {
namespace sensor {

class Sensor : public Communicator {
 public:
  Sensor(){
    position_.x = 0.0f;
    position_.y = 0.0f;
    position_.z = 0.0f;
    velocity_.x = 0.0f;
    velocity_.y = 0.0f;
    velocity_.z = 0.0f;
    offset_.x = 0.0f;
    offset_.y = 0.0f;
    offset_.z = 0.0f;
    glm::mat3 ident(1.0f);
    orientation_ = ident;
    relative_orientation_ = ident;
    name_ = "sensor";
		draw_annotations_ = false;
		printed_rate_error_ = false;
  }

  virtual ~Sensor(){ }

  Sensor(const Sensor &s){
    position_ = s.position_;
    velocity_ = s.velocity_;
    offset_ = s.offset_;
    orientation_ = s.orientation_;
    relative_orientation_ = s.relative_orientation_;
    look_to_ = s.look_to_;
    look_side_ = s.look_side_;
    look_up_ = s.look_up_;
  }

  /**
   * A sensor is tied to a vehicle and must know the vehicle state as well as
   * the state of the enviornment.
   * \param env The current environment.
   * \param dt The time step (seconds) to increment the sensor.
   */
  virtual void Update(environment::Environment *env, double dt){};

  /**
   * Save the raw data out in appropriate format
   */
  virtual void  SaveRaw(){}
  
  /**
   * Sets the pose of the sensor based on the current vehicle state.
   */
  virtual void SetPose(VehicleState &state){
    glm::quat state_quat
      ((float)state.pose.quaternion.w, (float)state.pose.quaternion.x,
       (float)state.pose.quaternion.y, (float)state.pose.quaternion.z);
    glm::vec3 state_pos
      ((float)state.pose.position.x,
       (float)state.pose.position.y,(float)state.pose.position.z);
		SetPose(state_pos, state_quat);
    velocity_ = state.twist.linear;
		angular_velocity_ = state.twist.angular;
		acceleration_ = state.accel.linear;
  }

	/**
	* Set the velocity of the sensor in world coordinates in m/s
	*/
	void SetVelocity(float vx, float vy, float vz) {
		velocity_ = glm::vec3(vx, vy, vz);
	}

  /**
   * Set the pose relative to the vehicle coordinate frame.
   * \param off The (x,y,z) offset of the sensor from the vehicle CG (meters).
   * \param relor Relative orientation to the vehicle coordinate system.
   */
  void SetRelativePose(glm::vec3 off, glm::quat relor){
    relative_orientation_ = glm::toMat3(relor);
    offset_ = off;
  }
  
	/**
	* Set the pose of the sensor, takes offset into account
	* \param position Position of the sensor in world coordinates
	* \param orientation Quaternion orientation of the sensor in world coordiantes
	*/
  virtual void SetPose(glm::vec3 position, glm::quat orientation){
		glm::mat3 R = glm::toMat3(orientation);
		orientation_ = R*relative_orientation_;
    look_to_ = orientation_[0];
    look_side_ = orientation_[1];
    look_up_ = orientation_[2];
    position_ = position + R[0]*offset_.x + R[1]*offset_.y + R[2]*offset_.z;
  }

	/**
	* Returns the pose of the sensor in world coordinates
	* Includes offsets.
	*/
	mavs::Pose GetPose() {
		mavs::Pose pose;
		pose.position = position_;
		pose.quaternion = glm::quat(orientation_);
		return pose;
	}

	/// Return the position of the sensor in local ENU coordinates
	glm::vec3 GetPosition() {
		return position_;
	}

	/// Return the orientation matrix of the sensor
	glm::mat3 GetOrientationMatrix() {
		return orientation_;
	}

	/// Return the relative orientation matrix of the sensor
	glm::mat3 GetRelativeOrientationMatrix() {
		return relative_orientation_;
	}

	/// Return the offset from the vehicle cg
	glm::vec3 GetOffset() {
		return offset_;
	}

	/**
	* Returns the look_to vector of the sensor in world coordinates
	* Includes offsets.
	*/
	glm::vec3 GetLookTo() {
		return look_to_;
	}

	/**
	* Returns the look_up vector of the sensor in world coordinates
	* Includes offsets.
	*/
	glm::vec3 GetLookUp() {
		return look_up_;
	}

	/**
	* Save annotation of the existing sensor data to default named file
	*/
	virtual void SaveAnnotation() {
		// do nothing by default
	}

	/**
	* Save annotation of the existing sensor data to a file
	* \param ofname The name of the saved file
	*/
	virtual void SaveAnnotation(std::string ofname) {
		// do nothing by default
	}

	/**
	* Annotate the current camera frame with ground truth
	* \param env Pointer to the mavs environment object
	* \param semantic True if using semantic labeling
	*/
	virtual void AnnotateFrame(environment::Environment *env, bool semantic) {
		//do nothing by defualt
	}

	/// Call this to draw annotations on annotated outputs
	void SetDrawAnnotations() { draw_annotations_ = true; }

	/// Returns true if annotations are set to on
	bool GetDrawAnnotations() { return draw_annotations_; }

  ///Create a deep copy of the sensor
  virtual Sensor* clone() const {
    return new Sensor(*this);
  }

  ///Get the sensor type
  virtual std::string GetType(){return type_;}

  /**
   * Set the sensor type. Valid types are
   * "lidar"
   * "gps"
   * "camera"
   */
  virtual void SetType(std::string type){
    if (! ((type=="lidar") || (type=="camera") || (type=="gps") ||
	(type=="compass") || (type=="fisheye") || (type=="radar") || (type == "imu")) ){
      std::cerr<<"ERROR: "<<type<<" is not a valid sensor type."<<std::endl;
    }
    else {
      type_ = type;
    }
  }

  /**
   * Set the name of the sensor for indentifying it in display window.
   * \param name Sensor name
   */
  void SetName(std::string name){
    name_ = name;
  }

	/**
	* Check that the frequency does not exceed 100 Hz for camera sensors.
	*/
	void CheckFreq(double &dt) {
		/*double freq = 1.0 / dt;
		if (freq > 100.0 && (type_=="camera" || type_=="fisheye")) {
			if (!printed_rate_error_) {
				std::cerr << "WARNING: CAMERA ASKED FOR UPDATE TIME STEP OF " << dt << ". MAX ALLOWED UPDATE RATE IS 100 HZ. RESETTING DT TO 0.01" << std::endl;
				printed_rate_error_ = true;
			}
			dt = 1.0f / 100.0;
		}*/
	}

  /**
  * Virtual function to display the data
  * needs to be overwritten in derived classes
  */
  virtual void Display() {}

	/**
	* Set the directory where the sensor output will be written
	* \param direc Save directory, with a slash at the end
	*/
	void SetSaveDirectory(std::string direc) {
		prefix_ = direc;
	}

	/// Return the annotations data structure
	std::map<int, Annotation> GetObjectAnnotations() {
		return object_annotations_;
	}

	/// Return the pixel annotations
	std::map<std::string, Annotation> GetAnnotations() {
		return annotations_;
	}

 protected:
  std::string type_;
  std::string name_;
	std::string prefix_;

	bool printed_rate_error_;
  
  //these define the offset and relative orientation
  // with respect to the vehicle coordinate system
  glm::vec3 offset_;
  glm::mat3 relative_orientation_;
  
  glm::vec3 position_;
  glm::vec3 velocity_;
  glm::vec3 look_to_;
  glm::vec3 look_side_;
  glm::vec3 look_up_;
  glm::mat3 orientation_;

	glm::vec3 acceleration_;
	glm::vec3 angular_velocity_;

	//sensor annotation information
	AnnotationColors anno_colors_;
	std::map<int, Annotation> object_annotations_;
	std::map<std::string, Annotation> annotations_;
	bool draw_annotations_;
};

}//namespace sensor
}//namespace mavs

#endif
