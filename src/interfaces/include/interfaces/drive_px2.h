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
* \file drive_px2.h
*
* File that contains headers for sending and recieving
* lidar and vehicle data for the drive px2 NVIDIA interfaces
*
* \author Chris Goodin
*
* \date 11/15/2018
*/
#ifndef DRIVE_PX2_H
#define DRIVE_PX2_H

#include <stdint.h>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
__pragma(pack(1))
#else
#endif

#ifdef __GNUC__
#define NVIDIA_PACK_END __attribute__((packed))
#else
#define NVIDIA_PACK_END
#endif

namespace mavs {
namespace nvidia {

typedef float float32_t;

struct NVIDIA_PACK_END VehicleFeedback {
	uint32_t nVeh; // Actual vehicle speed [0.1 mph], [0-204.8]
	uint8_t PRND; // Park, reverse, drive, or neutral, 6=Park 5=Neutral 4=Reverse 3=Drive
	int64_t SteeringAngle; //Actual steering angle [0.1 deg], [-3276.8-3276.8]
	int64_t nVehError; // Vehicle speed vs requested speed error, [0.01 mph]  [-327.68-327.68]
	uint8_t AutoMode; // Binary flag for auto vs manual mode; vehicle ready for autonomous commands, 0=manual 1=auto
	uint64_t timestamp;
};

struct NVIDIA_PACK_END VehicleReq {
	VehicleReq() : nVehReq(0), SteeringAngleReq(0), AutoError(0), PRNDReq(0), LTurnReq(0), RTurnReq(0), HazardReq(0), HornReq(0), LockReq(0), UnlockReq(0), HeadlightsReq(0), DayLightsReq(0), VehStateReq(0), BrakeSwReq(0), BrakedStop(0) {}
	uint32_t nVehReq;	//[0.1 mph]	Vehicle Speed Requested	 0 - 204.8
	int64_t SteeringAngleReq;	// [0.1 deg]	Steering Angle Requested - 3276.8 - 3276.8
	uint8_t AutoError; // [flag]	Autonomous system error flag
	uint8_t PRNDReq; //	PRND Request	6 = Park 5 = Neutral 4 = Reverse 3 = Drive
	uint8_t LTurnReq; // Left turn signal
	uint8_t RTurnReq; // Right turn signal
	uint8_t HazardReq; // Hazard light request
	uint8_t HornReq; // 
	uint8_t LockReq; // Door Lock
	uint8_t UnlockReq; // Door Unlock
	uint8_t HeadlightsReq; // Headlights on request
	uint8_t DayLightsReq; // Daytime running lights request
	uint8_t VehStateReq; // Vehicle state request	0 = off, 1 = acc, 2 = run, 3 = start
	uint8_t BrakeSwReq; // Brake switch enable request
	uint8_t BrakedStop;	// [flag] Emergency / held stop flag, constant high brake request tbd on vehicle
};

struct NVIDIA_PACK_END dwLidarDecodedPacket {
	uint64_t	duration;	//Time difference between the first measurement and the last.
	uint64_t	hostTimestamp;	//Timestamp measured on the host.
	uint32_t	maxPoints;	//Maximum number of points in the packet.
	uint32_t	nPoints;	//Current number of valid points in the packet.
	const void *	points;	//Pointer to the array of points.
	//To be casted to one of the point structures dwLidarPointXYZIor dwLidarPointRTheta, depending on the selected decoding format.
	uint64_t	sensorTimestamp;	//Timestamp of the first point in the point cloud packet.
	uint32_t	stride;	//Number of float32 elements per point.Typically 4.
};

struct NVIDIA_PACK_END dwLidarPointRTheta {
	float32_t	intensity;	//Reflection intensity in the 0 to 1 range.
	float32_t	phi;	//Lidar right - handed polar coord system, vertical, in rads.
	float32_t	radius;	//Lidar right - handed polar coord system, distance, in m.
	float32_t	theta;	//Lidar right - handed polar coord system, planar, in rads.
};

struct NVIDIA_PACK_END dwLidarPointXYZI {
	float32_t	intensity;	//Reflection intensity in the 0 to 1 range.
	float32_t	x;	//Lidar right - handed coord system, planar, in meters.
	float32_t	y;	//Lidar right - handed coord system, planar, in meters.
	float32_t	z;	//Lidar right - handed coord system, vertical, in meters.
};

struct NVIDIA_PACK_END dwLidarProperties {
	char	deviceString[256];	//ASCII string identifying the device.
	uint32_t	packetsPerSecond;	//Number of packets per second the sensor produces.
	uint32_t	packetsPerSpin;	//Number of packets per sensor full spin.
	uint32_t	pointsPerPacket;	//Maximum number of points in a packet.
	uint32_t	pointsPerSecond;	//Number of points per second the sensor provides.
	uint32_t	pointsPerSpin;	//Maximum number of points on a full sensor spin.
	uint32_t	pointStride;	//Number of float32 elements per point.
	float32_t	spinFrequency;	//Current spin frequency.
};

} //namespace nvidia
} //namespace mavs

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
__pragma(pack())
#else
#endif

#endif
