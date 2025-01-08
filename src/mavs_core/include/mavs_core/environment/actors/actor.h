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
* \class Actor
*
* The base class for a MAVS actor. An actor is anything that 
* moves in the simulation. 
*
* \author Chris Goodin
*
* \date 5/25/2018
*/

#ifndef ACTOR_H
#define ACTOR_H
#include <string>

#include <mavs_core/pose_readers/waypoints.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace mavs {
namespace actor{
class Actor {
 public:
	 /// Construct an actor
	 Actor();

	 /// Destruct an actor
	 ~Actor();

	 /**
	 * Update the state of the actor
	 * \param dt The time step of the update
	 */
	 void Update(double dt);

	 /**
	 * Load the animation associated with the actor
	 * \param infile Full path to the input animation file
	 */
	 void LoadAnimation(std::string infile);

	 /**
	 * Define a path in local world coordinates for the animation to follow
	 * \param path Path in world coordinates (x-y-z points)
	 */
	 void SetPath(Waypoints path) { path_ = path; }

	 /**
	 * Load waypoints from JSON file
	 * \param infile The full path to the waypoints file
	 */
	 void LoadPath(std::string infile) { path_.Load(infile); }

	 /// Return the current position of the actor
	 glm::vec3 GetPosition() { return position_; }

	 /// Return the current orientation of the actor
	 glm::quat GetOrientation() { return orientation_; }

	 /**
	 * Set the position of the actor
	 * \param pos The position in global ENU coordinates.
	 */
	 void SetPosition(glm::vec3 pos) { position_ = pos; }

	 /**
	 * Set the current height/altitude of the actor
	 * in global coordinates.
	 * \param z The desired height.
	 */
	 void SetHeight(float z) { position_.z = z; }

	 /**
		* Get the Z-offsest of the actor, used
		* to place the actor at the right height on
		* the ground.
	 */
	 float GetZOffset() { return offset_.z; }

	 /**
	 * Set the offset of the actor for placing
	 * in the global coordinate system.
	 * \param x X offset in ENU meters
	 * \param y Y offset in ENU meters
	 * \param z Z offset in ENU meters
	 */
	 void SetOffset(float x, float y, float z) {
		 offset_ = glm::vec3(x, y, z);
	 }

	 /// Return the x-y-z offset of the actor mesh
	 glm::vec3 GetOffset() { return offset_; }

	 /**
	 * Set the relative scale of the actor for 
	 * \param sx X scale in ENU meters
	 * \param sy Y scale in ENU meters
	 * \param sz Z scale in ENU meters
	 */
	 void SetScale(float sx, float sy, float sz) {
		 scale_ = glm::vec3(sx, sy, sz);
	 }

	 /// Return the spatial scale of the actor
	 glm::vec3 GetScale() { return scale_; }

	 /// Get the ID number of the actor
	 int GetId() { return id_; }

	 /**
	 * Sets the ID number for the actor
	 * \param id Desired ID number for the actor
	 */
	 void SetId(int id) { id_ = id; }

	 /**
	 * Set the desired speed of the actor in m/s
	 * \param speed The desired speed in m/s.
	 */
	 void SetSpeed(double speed) { speed_ = speed; }

	 /**
	 * Set the position and orientation of the actor
	 * \param position Desired position in local ENU coordinates
	 * \param orientation Desired orientation in local ENU coordinates
	 */
	 void SetPose(glm::vec3 position, glm::quat orientation) {
		 position_ = position;
		 orientation_ = orientation;
	 }

	 /**
	 * Call this to lock the actor to the ground
	 */
	 void LockToGround() {
		 lock_to_ground_ = true;
	 }

	 /// Tell if the actor is locked to the ground
	 bool IsLockedToGround() {
		 return lock_to_ground_;
	 }

	 /**
	 * Set the actor velocity, in m/s
	 * \param velcoity Velocity in m/s
	 */
	 void SetVelocity(glm::vec3 velocity) {
		 velocity_ = velocity;
	 }

	 /// Get the current actor velocity
	 glm::vec3 GetVelocity() {
		 return velocity_;
	 }

	 /**
	 * Define if the actor will be updated automatically
	 * If set to false, the actor must be updated explicitly
	 * \param auto_update True if the actor will be updated automatically
	 */
	 void SetAutoUpdate(bool auto_update) {
		 auto_update_ = auto_update;
	 }

	 /// Return true of the actor is updated automatically
	 bool GetAutoUpdate() { return auto_update_; }

 private:
	 // identifiers
	 std::string name_;
	 int id_;
	 bool lock_to_ground_;
	 //state variables
	 glm::vec3 position_;
	 glm::quat orientation_;
	 glm::vec3 offset_;
	 glm::vec3 scale_;
	 glm::vec3 velocity_;
	 double speed_;
	 bool auto_update_;

	 //path following variables
	 bool complete_;
	 int current_waypoint_;
	 Waypoints path_;
	 double tolerance_;
};

} //namespace actor
} //namespace mavs


#endif