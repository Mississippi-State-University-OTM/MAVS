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
* \file animation.h
*
* Class for keyframe animations in Embree
*
* \author Chris Goodin
*
* \date 7/29/2019
*/
#ifndef EMBREE_ANIMATION_H
#define EMBREE_ANIMATION_H
#include <string>
#include <glm/glm.hpp>
#include <mavs_core/pose_readers/waypoints.h>
#include <raytracers/mesh.h>

namespace mavs {
namespace raytracer {

class Animation {
public:
	/// Create an animation
	Animation();

	/**
	* Update the state of the animation.
	* Will move the animation along the path 
	* and cycle to the next frame
	* \param dt The update time step in seconds
	*/
	void Update(float dt);

	/**
	* Set path to the folder where the mesh files are located
	* \param path_to Path to the folder 
	*/
	void SetPathToMeshes(std::string path_to) { path_to_frames_ = path_to; }

	/**
	* Load the text file containing the list of key-frame meshes
	* \param framefile The file containing the list of keyframe meshes
	*/
	void LoadFrameList(std::string framefile);

	/**
	* Set the frame rate of the sequence of keyframes
	\param frate The frame rate in Hz
	*/
	void SetFrameRate(float frate) { 
		frame_rate_ = frate;
		frame_time_ = (int)(time_unit_ / frame_rate_);
		sequence_time_ = (int)(keyframe_list_.size()*time_unit_ / frame_rate_);
	}

	/// Get the number of vertices in the keyframes
	int GetNumVerts() { return (int)vertices_.size(); }

	/**
	* Load a file that contains a path for the animation to follow
	* \param pathfile The file to load, including the path and extension
	*/
	void LoadPathFile(std::string pathfile);

	/**
	* Return the mesh associated with a given frame
	* \param frame_num The frame number to return
	*/
	Mesh GetMesh(int frame_num);

	/**
	* Get the i^th interpolated vertex at the current animation state
	* \param i The vertex number to get
	*/
	glm::vec3 GetVertex(int i);

	/**
	* Set the walking/driving speed of the animationon
	* \param speed Speed in m/s
	*/
	void SetSpeed(float speed) { speed_ = speed; }

	/**
	* Set a scale factor for the mesh
	* \param scale The scale factor
	*/
	void SetMeshScale(float scale) { mesh_scale_ = scale; }

	/// Return the mesh scale factor
	float GetMeshScale() { return mesh_scale_; }

	/**
	* Call this if the mesh needs to be rotated about the z axis
	* \param ryx Set to true to rotate the mesh
	*/
	void SetRotateYToX(bool ryx);
	
	/**
	* Call this if the mesh needs to be rotated about the x axis
	* \param ryz Set to true to rotate the mesh
	*/
	void SetRotateYToZ(bool ryz);

	/**
	* Call this to have the animation follw a certain behavior, 
	* rather than follow path. Options are "wander", "straight", "circle"
	*/
	void SetBehavior(std::string behave);

	/**
	* Set the height of th mesh to a certain value
	* \param z The height in local ENU
	*/
	void SetHeight(float z) { z_ = z; }

	/**
	* Return the current position of the animation
	*/
	glm::vec3 GetPosition() { return position_; }

	/**
	* Set the position
	* \param p The initial position in ENU
	*/
	void SetPosition(glm::vec3 p) { position_ = p; }

	/**
	* Set the position
	* \param x The initial x-position in ENU
	* \param y The initial y-position in ENU
	*/
	void SetPosition(float x, float y) { position_ = glm::vec3(x, y, z_); }

	/**
	* Set the heading of the animation
	* \param heading Heading, in radians relative to +X
	*/
	void SetHeading(float heading);

	/// Return the assigned behavior of the animation
	std::string GetBehavior() { return behavior_; }

	/**
	* Move an animation in the direction of a waypoint for dt seconds
	* \param dt The time step to move in seconds
	* \param wp The waypoint in local ENU coordinates
	*/
	void MoveToWaypoint(float dt, glm::vec3 wp);

	/**
	* Interpolate vertices along a fractional time between two frames
	* \param x The fraction of the frame time dt between first and second frame
	*/
	void InterpolateVerts(float x);

	/**
	* Update the current frame count of the animation 
	* \param dt The time step in seconds
	*/
	float UpdateFrameCount(float dt);

	/**
	* Move an animation on a random trajectory
	* \param dt The time step of the update in seconds
	*/
	void MoveWander(float dt);

private:
	Waypoints waypoints_;
	int current_waypoint_;
	bool waypoints_loaded_;
	glm::vec3 position_;
	glm::mat3 orientation_;
	glm::mat3 base_rotation_;
	float speed_;
	float mesh_scale_;
	bool rotate_y_to_x_;
	bool rotate_y_to_z_;
	std::string behavior_;
	bool behavior_set_;
	float z_;

	std::vector<std::string> keyframe_list_;

	int elapsed_time_;
	int sequence_time_;
	int frame_time_;
	float  frame_rate_;
	float total_time_;
	float time_unit_;

	int current_frame_;
	int next_frame_;
	Mesh current_mesh_;
	Mesh next_mesh_;
	std::string path_to_frames_;

	std::vector<glm::vec3> vertices_;

	void MoveVertsWaypoints(float dt);
	void MoveVertsBehavior(float dt);
	void MoveStraight(float dt);
	void MoveCircle(float dt);
	
	glm::vec3 random_goal_;
	float random_goal_age_;
};

void MoveCrowd(float dt, std::vector<Animation> &crowd);

} //namespace raytracer 
} //namespace mavs
#endif