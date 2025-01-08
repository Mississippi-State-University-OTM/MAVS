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
#ifndef APPLANIX_HEADERS_H
#define APPLANIX_HEADERS_H

namespace applanix {
namespace dgc {
/**
* Primary applanix pose message, used to read/write the binary files
*/
struct ApplanixPose {
	double smooth_x;     /** Velocity-integrated, smooth position (East).  0=where applanix was started */
	double smooth_y;     /** Velocity-integrated, smooth position (North). 0=where applanix was started */
	double smooth_z;     /** Velocity-integrated, smooth position (Up).    0=where applanix was started */
	double latitude;     /**< Latitude, in degrees */
	double longitude;    /**< Longitude, in degrees */
	double altitude;     /**< Altitude, in meters */
	float v_north;       /**< north velocity, in m/s */
	float v_east;        /**< east velocity, in m/s */
	float v_up;          /**< up velocity, in m/s */
	float speed;         /**< Magitude of velocity vector, in m/s */
	float track;         /**< planar heading of velocity vector, in radians */
	double roll;         /**< roll, in radians */
	double pitch;        /**< pitch, in radians */
	double yaw;          /**< yaw, in radians */
	double ar_roll;      /**< roll rate, in rad/sec */
	double ar_pitch;     /**< pitch rate, in rad/sec */
	double ar_yaw;       /**< yaw rate, in rad/sec */
	double a_x;          /**< acceleration in x, in m/s/s (body frame) */
	double a_y;          /**< acceleration in y, in m/s/s (body frame) */
	double a_z;          /**< acceleration in z, in m/s/s (body frame) */
	double wander;       /**< wander angle, in radians */
	unsigned int ID;     /**< unique ID for internal tracking */
	int postprocess_code;  /**< 0 = Real Time.  1 = Post Processed.  2 = Post Processed with Base Station. */
	double hardware_timestamp;    /**< Timestamp from the Applanix hardware, in UTC seconds of the week. */
	int hardware_time_mode;        /**< Mode of timestamp from the Applanix hardware.  0 = None.  1 = Acquire.  2 = Locked. */
	double timestamp;    /**< DGC timestamp */
	char host[10];       /**< hostname associated with timestamp */
};
} //namespace dgc

namespace vlr {
/// Used to read/write applanix .hmf binary files
struct PerceptionRayTracingCell {
	float x;
	float y;
	float z_min;
	float height;
	char type; //obstacle type: 0 = not an obstacle, 1 = obstacle, 
};

/// Used to read/write applanix .hmf binary files
struct PerceptionRayTracedMap {
	int num_cells;
	vlr::PerceptionRayTracingCell *cells;
};

} //namespace vlr
} //namespace applanix

#endif