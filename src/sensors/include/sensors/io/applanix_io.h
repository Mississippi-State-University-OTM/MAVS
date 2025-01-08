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
* \file applanix_io.h
*
* Definition for functions that read / write
* applanix heightmap (.hmf) files and convert them 
* to other MAVS formats for manipulation and editing
*
* \author Muhammad Islam
* \author Chris Goodin
*
* \date 10/2/2018
*/
#ifndef APPLANIX_IO
#define APPLANIX_IO

#include <string>
#include <vector>

#include <mavs_core/terrain_generator/grd_surface.h>
#include <mavs_core/terrain_generator/heightmap.h>
#include <mavs_core/messages.h>

#include <interfaces/applanix.h>
#include <glm/gtx/quaternion.hpp>

namespace mavs {
namespace io {
/**
* \class ApplanixHeightMap
*/
class ApplanixHeightMap {
public:
	/// Constructor
	ApplanixHeightMap();

	/** 
	* Construct a height map from a MAVS point cloud and pose
	* \param point_cloud A MAVS point cloud, identical to a ROS point cloud message
	*/
	ApplanixHeightMap(mavs::PointCloud &point_cloud);

	/**
	* Load a binary .hmf file
	* \param infile The full path to the file to load
	*/
	void LoadHmf(std::string infile);

	/**
	* Write the current object to a binary .hmf file
	* \param outfile The full path to the file to write
	*/
	void WriteHmf(std::string outfile);

	/**
	* Print the number of cells and x-y pose of the hmf
	*/
	void PrintStats();

	/**
	* Convert the hmf to a MAVS GridSurface
	*/
	mavs::terraingen::GridSurface ConvertToSurface();

	/**
	* Convert the hmf to a MAVS HeightMap
	*/
	mavs::terraingen::HeightMap ConvertToMavsHeightMap();

	/**
	* Set the Latitude, longitude, altitude of the pose
	* \param lat Latitude to set, decimal degrees
	* \param lon Longitude to set, decimal degrees
	* \param alt Altitude to set, meters above sea level
	*/
	void SetLatLongAlt(float lat, float lon, float alt) {
		pose_.latitude = lat;
		pose_.longitude = lon;
		pose_.altitude = alt;
	}

	/**
	* Set the velocity of the vehicle pose
	* \param vel The vehicle velocity (m/x) in ENU coordinates
	*/
	void SetVelocity(glm::vec3 vel) {
		pose_.v_east = vel.x;
		pose_.v_north = vel.y;
		pose_.v_up = vel.z;
		pose_.speed = glm::length(vel);
		pose_.track = atan2(vel.y, vel.x);
	}

	/**
	* Set the velocity smoothed position offset
	* from the initial position. This would be
	* the position calculated by dead-reckoning
	* with an IMU. 
	* \param pos The position vector
	*/
	void SetPosition(glm::vec3 pos) {
		pose_.smooth_x = pos.x;
		pose_.smooth_y = pos.y;
		pose_.smooth_z = pos.z;
	}

	/**
	* Set the orientation associated with the pose
	* \param roll The roll in radians
	* \param pitch The pitch in radians
	* \param yaw The yaw in radians
	*/
	void SetOrientation(float roll, float pitch, float yaw) {
		pose_.roll = roll;
		pose_.pitch = pitch;
		pose_.yaw = yaw;
	}

	/**
	* Set the orientation associated with the pose
	* \param q_orient Quaternion describing the orientation
	*/
	void SetOrientation(glm::quat q_orient) {
		glm::vec3 pyr = glm::eulerAngles(q_orient);
		pose_.roll = pyr.z;
		pose_.pitch = pyr.x;
		pose_.yaw = pyr.y;
	}

	/**
	* Set the angular velocity of the pose in radians/s
	* about the pitch/yaw/roll direction
	* \param angvel The angular velocity, ordered (pitch,yaw,roll)
	*/
	void SetAngularVelocity(glm::vec3 angvel) {
		pose_.ar_roll = angvel.z;
		pose_.ar_pitch = angvel.x;
		pose_.ar_yaw = angvel.z;
	}

	/**
	* Set the angular velocity of the pose in radians/s
	* about the pitch/yaw/roll direction
	* \param pitchvel The angular velocity about the pitch direction
	* \param yawvel The angular velocity about the pitch direction
	* \param rollvel The angular velocity about the pitch direction
	*/
	void SetAngularVelocity(float pitchvel, float yawvel, float rollvel) {
		pose_.ar_roll = rollvel;
		pose_.ar_pitch = pitchvel;
		pose_.ar_yaw = yawvel;
	}

	/**
	* Set the angular acceleration about the x-y-z directions in the body frame
	* Units are m/s^2. Would be measured by a body-fixed IMU
	* \param ang_accel The angular accelration in the body frame
	*/
	void SetAngularAccleration(glm::vec3 ang_accel) {
		pose_.a_x = ang_accel.x;
		pose_.a_y = ang_accel.y;
		pose_.a_z = ang_accel.z;
	}

private:
	std::vector<applanix::vlr::PerceptionRayTracingCell> cells_;
	applanix::dgc::ApplanixPose pose_;
};

} //namespace mavs
} //namespace io
#endif