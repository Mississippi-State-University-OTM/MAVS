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
* \file swiftnav_msg.h
*
* Implements structures for swiftnav piksi messages
*
* See "Swift Navigation Binary Protocol Specification v2.3.15"
* Alsoe https://github.com/swift-nav/libsbp/tree/master/c for c implementation
* \author Chris Goodin
*
* \date 11/12/2018
*/
#ifndef SWIFTNAV_MSG_H
#define SWIFTNAV_MSG_H

#include <stdint.h>
#include <ostream>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
__pragma(pack(1))
#else
#endif

#ifdef __GNUC__
#define SBP_PACK_END __attribute__((packed))
#else
#define SBP_PACK_END
#endif

namespace mavs {
namespace swiftnav {
/**
* This position solution message reports the absolute geodetic coordinates and the status (single point
* vs pseudo-absolute RTK) of the position solution. If the rover receiver knows the surveyed position of
* the base station and has an RTK solution, this reports a pseudo-absolute position solution using the
* base station position and the rover's RTK baseline vector. The full GPS time is given by the preceding
* MSG GPS TIME with the matching time-of-week (tow).
* 
* Fix mode values:
* 0 Invalid
* 1 Single Point Position (SPP)
* 2 Differential GNSS (DGNSS)
* 3 Float RTK
* 4 Fixed RTK
* 5 Dead Reckoning
* 6 SBAS Position
* 
* Inertial Navigation Mode values
* 0 None
* 1 INS used
*
* Serial size should be 34
*/
struct SBP_PACK_END msg_pos_llh_t {
	uint32_t tow; // [ms] GPS Time of Week
	double lat; // [deg] Latitude
	double lon; // [deg] Longitude
	double height; // [m] Height above WGS84 ellipsoid
	uint16_t h_accuracy; //  [mm] Horizontal position estimated std dev
	uint16_t v_accuracy; //  [mm] Vertical position estimated std dev
	uint8_t n_sats; // number of satellites used in solution
	uint8_t flags; // status flags
}; 

/**
* This message reports the velocity in local North East Down (NED) coordinates. The NED coordinate
* system is dened as the local WGS84 tangent plane centered at the current position. The full GPS time
* is given by the preceding MSG GPS TIME with the matching time-of-week (tow).
* 
* Velocity Mode values:
* 0 Invalid
* 1 Measured Doppler derived
* 2 Computed Doppler derived
* 3 Dead Reckoning
* 
* INS Navigation mode values
* 0 None
* 1 INS Used
*/
struct SBP_PACK_END msg_vel_ned_t {
	uint32_t tow; // [ms] GPS Time of Week
	int32_t n; // [mm/s] Velocity North Coordinate
	int32_t e; // [mm/s] Velocity East Coordinate
	int32_t d; // [mm/s] Velocity Down Coordinate
	uint16_t h_accuracy; //  [mm/s] Horizontal velocity estimated std dev
	uint16_t v_accuracy; //  [mm/s] Vertical velocity estimated std dev
	uint8_t n_sats; // number of satellites used in solution
	uint8_t flags; // status flags
};

/**
* This dilution of precision (DOP) message describes the eect of navigation satellite geometry on po-
* sitional measurement precision. The* ags eld indicated whether the DOP reported corresponds to
* differential or SPP solution.
* Fix mode values:
* 0 Invalid
* 1 Single Point Position (SPP)
* 2 Dierential GNSS (DGNSS)
* 3 Float RTK
* 4 Fixed RTK
* 5 Undefined
* 6 SBAS Position
*/
struct SBP_PACK_END msg_dops_t {
	uint32_t tow; // [ms] GPS Time of Week
	uint16_t gdop; // [0.01] Geometric dilution of precision
	uint16_t pdop; // [0.01] Position dilution of precision
	uint16_t tdop; // [0.01] Time dilution of precision
	uint16_t hdop; // [0.01] Horizontal dilution of precision
	uint16_t vdop; // [0.01] Vertical dilution of precision
	uint8_t flags; // Indicates the position solution with which the DOPS message corresponds
}; 

/**
* This message reports the GPS time, representing the time since the GPS epoch began on midnight
* January 6, 1980 UTC. GPS time counts the weeks and seconds of the week. The weeks begin at the
* Saturday/Sunday transition. GPS week 0 began at the beginning of the GPS time scale.
* Within each week number, the GPS time of the week is between between 0 and 604800 seconds
* (=60*60*24*7). Note that GPS time does not accumulate leap seconds, and as of now, has a small oset
* from UTC. In a message stream, this message precedes a set of other navigation messages referenced
* to the same time (but lacking the ns field) and indicates a more precise time of these messages.
* Time source values:
* 0 None (invalid)
* 1 GNSS Solution
*/
struct SBP_PACK_END msg_gps_time_t {
	uint16_t wn; // [weeks] GPS Week number
	uint32_t tow; // [ms] GPS time of week rounded to the nearest millisecond
	int32_t ns_residual; // [ns] Nanosecond residual of millisecond-rounded TOW(ranges from - 500000 to 500000)
	uint8_t flags; //status flags;
}; 

/**
* This message reports the Universal Coordinated Time (UTC). Note the* flags which indicate the source
* of the UTC oset value and source of the time fix.
*
* Time Source Values:
* 0 None (invalid)
* 1 GNSS Solution
*
* UTC offset source  values
* 0 Factory Default
* 1 Non Volatile Memory
* 2 Decoded this Session
*/
struct SBP_PACK_END msg_utc_time_t {
	uint32_t tow; // [ms] GPS time of week rounded to the nearest millisecond
	uint8_t flags; // Indicates source and time validity
	uint16_t year; // year
	uint8_t month; // month [1-12]
	uint8_t day; // day [1-31]
	uint8_t hours; // hour [0-23]
	uint8_t minutes; // minutes [0-59]
	uint8_t seconds; // seconds [0-59]
	uint32_t ns; // nanoseconds [0-999999999]
}; 

/**
* This message reports the baseline heading pointing from the base station to the rover relative to True
* North. The full GPS time is given by the preceding MSG GPS TIME with the matching time-of-week
* (tow). It is intended that time-matched RTK mode is used when the base station is moving.
* Fixed mode values:
* 0 Invalid
* 1 Reserved
* 2 Differential GNSS (DGNSS)
* 3 Float RTK
* 4 Fixed RTK
*/
struct SBP_PACK_END msg_baseline_heading_t{
	uint32_t tow; // [ms] GPS Time of Week
	uint32_t heading; // [milli-degrees] Heading
	uint8_t n_sats; // number of satellites used in solution
	uint8_t flags; //status flags
};

/**
* This message reports the yaw, pitch, and roll angles of the vehicle body frame. The rotations should
* applied intrinsically in the order yaw, pitch, and roll in order to rotate the from a frame aligned with the
* local-level NED frame to the vehicle body frame.
* INS navigation mode values:
* 0 Invalid
* 1 Valid
*/
struct SBP_PACK_END msg_orient_euler_t {
	uint32_t tow; // [ms] GPS Time of Week
	int32_t roll; // [microdegrees] rotation abou the forward axis of the vehicle
	int32_t pitch; // [microdegrees] rotation abou the rightward axis of the vehicle
	int32_t yaw; // [microdegrees] rotation abou the downward axis of the vehicle
	float roll_accuracy; // [degrees] estimated std dev of roll
	float pitch_accuracy; // [degrees] estimated std dev of pitch
	float yaw_accuracy; // [degrees] estimated std dev of yaw
	uint8_t flags; // status flags
};

/**
* This message reports the quaternion vector describing the vehicle body frame's orientation with respect
* to a local-level NED frame. The components of the vector should sum to a unit vector assuming that
* the LSB of each component has a value of 2^-31.
* INS navigation mode values:
* 0 Invalid
* 1 Valid
*/
struct SBP_PACK_END msg_orient_quat_t {
	uint32_t tow; // [ms] GPS Time of Week
	int32_t w; // Real component
	int32_t x; // 1st imaginary component
	int32_t y; // 2nd imaginary component
	int32_t z; // 3rd imaginary component
	float w_accuracy; // estimated std dev of w
	float x_accuracy; // estimated std dev of x
	float y_accuracy; // estimated std dev of y
	float z_accuracy; // estimated std dev of z
	uint8_t flags; //status flags
}; 

/**
* This message reports the orientation rates in the vehicle body frame. The values represent the measure-
* ments a strapped down gyroscope would make and are not equivalent to the time derivative of the Euler
* angles. The orientation and origin of the user frame is specied via device settings. By convention, the
* vehicle x-axis is expected to be aligned with the forward direction, while the vehicle y-axis is expected to
* be aligned with the right direction, and the vehicle z-axis should be aligned with the down direction.
* INS navigation mode values:
* 0 Invalid
* 1 Valid
*/
struct SBP_PACK_END msg_angular_rate_t {
	uint32_t tow; // [ms] GPS Time of Week
	int32_t x; // [microdegrees] angular rate about the x-axis
	int32_t y; // [microdegrees] angular rate about the y-axis
	int32_t z; // [microdegrees] angular rate about the z-axis
	uint8_t flags; //status flags
}; 

/**
* Raw data from the Inertial Measurement Unit, containing accelerometer and gyroscope readings. The
* sense of the measurements are to be aligned with the indications on the device itself.
*/
struct SBP_PACK_END msg_imu_raw_t {
	uint32_t tow; // [ms] Milliseconds since start of GPS week. If the high bit is set, the time is unknown or invalid.
	uint8_t tow_f; // [ms/256] Milliseconds since start of GPS week, fractional part
	int16_t acc_x; // Acceleration in the IMU frame X axis
	int16_t acc_y; // Acceleration in the IMU frame Y axis
	int16_t acc_z; // Acceleration in the IMU frame Z axis
	int16_t gyr_x; // Angular rate around the IMU frame X axis
	int16_t gyr_y; // Angular rate around the IMU frame Y axis
	int16_t gyr_z; // Angular rate around the IMU frame Z axis
};

/**
* Raw data from the magnetometer.
*/
struct SBP_PACK_END msg_mag_raw_t {
	uint32_t tow; // [ms] Milliseconds since start of GPS week. If the high bit is set, the time is unknown or invalid.
	uint8_t tow_f; // [ms/256] Milliseconds since start of GPS week, fractional part
	int16_t mag_x; // Magnetic field in the body frame X axis
	int16_t mag_y; // Magnetic field in the body frame Y axis
	int16_t mag_z; // Magnetic field in the body frame Z axis
}; 

//nvidia typedef used to stamp the messag
typedef uint64_t dwTime_t;

/// Custom flag structure
struct SBP_PACK_END swiftnavFLAGS {
	uint8_t pos_llh;
	uint8_t vel_ned;
	uint8_t dops;
	uint8_t utc_time;
	uint8_t orient_euler;
	uint8_t orient_quat;
	uint8_t angular_rate;
	uint8_t imu_raw;
	uint8_t mag_raw;
}; 

/// Custom struct with specific sbp meassages and driveworks timestamp
struct SBP_PACK_END sbpMessagesWithTimeStamp {
	dwTime_t time;
	/* SBP structs that messages from Piksi will feed. */
	msg_pos_llh_t pos_llh;
	msg_vel_ned_t vel_ned;
	msg_dops_t dops;
	//msg_gps_time_t gps_time;
	msg_utc_time_t utc_time;
	//msg_baseline_heading_t baseline_heading;
	msg_orient_euler_t orient_euler;
	msg_orient_quat_t orient_quat;
	msg_angular_rate_t angular_rate;
	msg_imu_raw_t imu_raw;
	msg_mag_raw_t mag_raw;
	swiftnavFLAGS flags;
	void Write(std::string ofname);
	void AppendToFile(std::string ofname);
	void Read(std::string ifname);
}; 

std::ostream& operator<< (std::ostream& out, const msg_pos_llh_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_vel_ned_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_dops_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_gps_time_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_utc_time_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_baseline_heading_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_orient_euler_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_orient_quat_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_angular_rate_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_imu_raw_t& msg);
std::ostream& operator<< (std::ostream& out, const msg_mag_raw_t& msg);
std::ostream& operator<< (std::ostream& out, const swiftnavFLAGS& msg);
std::ostream& operator<< (std::ostream& out, const sbpMessagesWithTimeStamp& msg);

} //namespace swiftnav
} //namespace mavs

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
__pragma(pack())
#else
#endif

#endif