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
 * \file messages.h
 *
 * Structs defining the messages passes between simulation components.
 * Not coincidentally, they look a lot like ROS message types.
 * For reference, see wiki.ros.org/common_msgs
 *
 * \author Chris Goodin
 *
 * \date 12/7/2017
 */

#ifndef MAVS_MESSAGES_H
#define MAVS_MESSAGES_H

#include <stdint.h>

#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

//note that:
//   ROS      C++
// uint32    uint32_t
// float64   double
// uint8     uint8_t


namespace mavs{

struct Header{
  uint32_t seq;
  uint32_t time;
  std::string frame_id;
};

/*---------------- Geometry messages ----------------------------------------*/
typedef glm::dvec3 Point; 

typedef glm::vec3 Point32;

typedef glm::dquat Quaternion; 

typedef glm::dvec3 Vector3;

/// Position and orientation in free space
struct Pose{
  /// Position in local East, North, Up (ENU) coordinates.
  Point position;

  /// Rotation in free space.
  Quaternion quaternion;
};

/// Pose with reference coordinate frame and time stamp
struct PoseStamped{
  /// Header containing coordinate frame ID and time stamp
  Header header;

  /// Pose
  Pose pose;
};

/// Pose in fee space with uncertainty
struct PoseWithCovariance{
  /// Estimated pose in free space.
  Pose pose;

  /**
   * Row-major representation of the 6x6 covariance matrix
   */
  double covariance[36];
};

/**
 * Transformation between two coordinate systems. The global coordinate system
 * is East, North, Up (ENU) with 
 * X = East
 * Y = North
 * Z = Up
 */
struct Transform{
  /// Translation in East,North,Up coordinates (meters)
  Vector3 translation;

  /// Rotation
  Quaternion rotation;
};

/// Force in free space
struct Wrench{
  /// Linear force (Newtons)
  Vector3 force;

  /// Torque (Newton * meters)
  Vector3 torque;
};

/// Velocity in free space
struct Twist{
  /// Linear velocity (m/s)
  Vector3 linear;

  /// Angular velocity (rad/s)
  Vector3 angular;
};

/// Twist in free space with uncertainty
struct TwistWithCovariance{
  /// Estimated twist in free space
  Twist twist;

  /**
   * Row-major representation of the 6x6 covariance matrix
   */
  double covariance[36];
};

/// Acceleration in free space
struct Accel{
  /// Linear acceleration (m/s^2)
  Vector3 linear;
  
  /// Angular acceleration (rad/s^2)
  Vector3 angular;
};

//A specification of a polygon where the first and last points are assumed to be connected
struct Polygon {
	std::vector<Point32> points;
};
/*--------------------- End Geometry Messages -------------------------------*/

/*--------------------- Nav Messages ----------------------------------------*/
/// Holds an array of poses (in local ENU coordinate) for the robot to follow.
struct Path{
  /// Frame ID and time stamp
  Header header;

  /// Pose in ENU coordinates
  std::vector<PoseStamped> poses;
};

/// Grid cells for float data such as elevation
struct GridCells{
  /// Frame ID and time stamp
  Header header;

  /// Cell width (meters) in x/east direction
  float cell_width;

  /// Cell height (meters) in y/north direction
  float cell_height;

  /// array containing cell data, could be elevations or other
  std::vector<Point> cells;
};

struct MapMetaData{
  /// time and which map was loaded
  time_t map_load_time;

  /// map resolution (m/cell)
  float resolution;

  /// map width (cells) in the east/x direction
  uint32_t width;

  /// map height (cells) in the north/y direction
  uint32_t height;

  /// origin of the map (meters, meters, rad) 
  Pose origin;
};

/// 2D grid map with each cell being a probability of occupancy
struct OccupancyGrid{
  /// Time stamp and frame id
  Header header;

  /// MetaData for the map
  MapMetaData info;

  /**
   * Map data, in row-major order, starting with (0,0). 
   * Occupancy probabilities are in the range [0,100]. Unknown = -1
   */
  std::vector<int> data;
};

/// Estimate of position and velocity in free space
struct Odometry{
  /// Frame ID and time stamp
  Header header;

  /// Frame ID for frame of twist
  std::string child_frame_id;

  ///Estimated pose
  PoseWithCovariance pose;

  ///Estimated twist (velocity)
  TwistWithCovariance twist;
};
/*---------------------- End Nav Messages -----------------------------------*/

/*---------------------- Sensor Messages ------------------------------------*/
/**
 * Holds optional data for point cloud channels such as R,G,B
 */
struct ChannelFloat32{
  /**
   * Common channel names include 
   * "u", "v" - row and column in stereo image.
   * "rgb" - For point clouds produced by color stereo cameras.
   * "intensity" - Laser or pixel intensity.
   * "distance"
   */
  std::string name;

  /// Values for the channel.
  std::vector<float> values;
};

/// Single scan from a planar laser range-finder.
struct LaserScan{
  Header header;

  /// start angle of the scan (rad)
  float angle_min;
  
  /// end angle of the scan (rad)
  float angle_max;
  
  /// angular distance between measurements (rad)
  float angle_increment;
  
  /// time between measurement (seconds)
  float time_increment;
  
  /// time between scans (seconds)
  float scan_time;
  
  /// minimum range value (meters)
  float range_min;

  /// maximum range value (meters)
  float range_max;

  /// range data (meters)
  std::vector<float> ranges;

  /// intensity data in device specific units
  std::vector<float> intensities;
};

/// Holds the description of one point entry in the point cloud message.
struct PointField{
  const uint8_t INT8;
  const uint8_t UINT8;
  const uint8_t INT16;
  const uint8_t UINT16;
  const uint8_t INT32;
  const uint8_t UINT32;
  const uint8_t FLOAT32;
  const uint8_t FLOAT64;
  PointField() : INT8(1), UINT8(2), INT16(3), UINT16(4), INT32(5), UINT32(6), 
                 FLOAT32(7), FLOAT64(8) {}

  /// name of the point field
  std::string name;

  /// offset from start of point struct
  uint32_t offset;

  /**
   * Datatype enumeration
   1 = int8_t
   2 = uint8_t
   3 = int16_t
   4 = uint16_t
   5 = int32_t
   6 = uint32_t
   7 = float
   8 = double
   */
  uint8_t datatype;

  /// number of elements in the field
  uint32_t count;
};

struct PointCloud{
  /// Time of data acquisition, coordinate frame ID
  Header header;

  /// Array of 3D points in the frame specified in the header
  std::vector<Point32> points;

  /// Should have the same number of elements as ponts array
  std::vector<ChannelFloat32> channels;
};

struct PointCloud2{
  /// Time of data acquisition, coordinate frame ID
  Header header;

  /**
   * 2D structure of the point cloud. If the cloud is unordered, height is 1 
   * and width is the length of the point cloud.
   */
  uint32_t height,width;
  
  /// Describes the channels and their layout in the point blob
  std::vector<PointField> fields;

  /// Is the data bigendian?
  bool is_bigendian;
  
  /// Length of a point in bytes.
  uint32_t point_step;

  /// Length of a row in bytes.
  uint32_t row_step;

  /// Point data, size is row_step*height
  //std::vector<uint8_t> data;
	std::vector<glm::vec4> data;

  /// True if there are no invalid points
  bool is_dense;
};

///Stucture for an uncompressed image
struct Image{
  /// Time stampe and frame ID
  Header header;

  /// Number of rows in the image data
  uint32_t height;

  /// Number of columns in the image data
  uint32_t width;

  /// Encoding of pixels
  std::string encoding;

  /// Is data bigendian?
  uint8_t is_bigendian;

  /// Full row length in bytes
  uint32_t step;

  ///Image matrix data, size is (step*rows)
  std::vector<uint8_t> data;
};

///Specify a region of interest within an image frame
struct RegionOfInterest{
  /// leftmost pixel of the ROI
  uint32_t x_offset;

  /// topmost pixel of the ROI
  uint32_t y_offset;
  
  /// height of ROI
  uint32_t height;

  /// width of ROI
  uint32_t width;

  /**
   * True if a distinct rectified ROI is calcluated from the "raw" image.
   * Typically should be false if ROI is not used.
   */
  bool do_rectify;
};

/// Meta information on a camera
struct CameraInfo{
  /// Time stamp and frame id
  Header header;

  /// Camera vertical resolution in pixels
  uint32_t height;

  /// Camera horizontal resolution in pixels
  uint32_t width;

  /// camera distortion parameters, size depends on distortion model
  std::vector<double> D;

  /// Intrinsic camera matrix for the raw (distorted) images, 3x3 row major.
  double K[9];

  /// Rectification matrix for stereo cameras, 3x3 row major.
  double R[9];

  /// Projection / camera matrix
  double P[12];

  /// Pixel binning factor in the horizontal direction
  uint32_t binning_x;

  /// Pixel binning factor in the vertical direction
  uint32_t binning_y;

  /// Region of interest subwindow. Defalut is full resolution.
  RegionOfInterest roi;
};

/// Message for a single point IR or acoustic ranger. Not to be used for LIDAR.
struct Range{
  /// Time stamp and frame id
  Header header;

  const uint8_t ULTRASOUND;
  const uint8_t INFRARED;
  Range() : ULTRASOUND(0), INFRARED(1) {}

  /**
   * Type of the sensor
   * 0 = ultrasound / acoustic
   * 1 = IR
   */
  uint8_t radiation_type;

  /// Size of the arc for which the range reading is valid
  double field_of_view;

  /// Minimum range value (meters)
  double min_range;

  /// Maximum range value (meters)
  double max_range;

  /// Detected range of the sensor (meters)
  double range;
};

/// Status of GPS return
struct NavSatStatus{
  int8_t STATUS_NO_FIX;
  int8_t STATUS_FIX;
  int8_t STATUS_SBAS_FIX;
  int8_t STATUS_GBAS_FIX;
  uint16_t SERVICE_GPS;
  uint16_t SERVICE_GLONASS; 
  uint16_t SERVICE_COMPASS;
  uint16_t SERVICE_GALILEO;
  NavSatStatus() : STATUS_NO_FIX(-1), STATUS_FIX(0), STATUS_SBAS_FIX(1), 
                   STATUS_GBAS_FIX(2), SERVICE_GPS(1), SERVICE_GLONASS(2), 
                   SERVICE_COMPASS(4), SERVICE_GALILEO(8) {}
  /**
   * Status of the return.
   * -1 = no fix
   *  0 = unaugmented fix
   *  1 = fix with satellite-based augmentation (dual band)
   *  2 = fix with ground-based augmentation (differential GPS)
   */
  int8_t status;

  /**
   * Type of positioning service.
   * 1 = GPS
   * 2 = GLONASS
   * 4 = COMPASS
   * 8 = GALILEO
   */
  uint16_t service;
};

struct NavSatFix{
  /// Contains frame id, usually location of antenna relative to the vehicle.
  Header header;
  
  /// Satellite fix status information.
  NavSatStatus status;

  /// Latitude (degrees). Positive = north
  double latitude;

  /// Longitude (degrees). Positive = east
  double longitude;

  /// Altitude (meters). Positive is above WGS84 ellipsoid.
  double altitude;

  ///Position covariance matrix in East, North, Up (ENU) coordinates.
  double position_covariance[9];

  uint8_t COVARIANCE_TYPE_UNKNOWN;
  uint8_t COVARIANCE_TYPE_APPROXIMATED;
  uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN;
  uint8_t COVARIANCE_TYPE_KNOWN;
  NavSatFix() : COVARIANCE_TYPE_UNKNOWN(0), COVARIANCE_TYPE_APPROXIMATED(1), 
                COVARIANCE_TYPE_DIAGONAL_KNOWN(2), COVARIANCE_TYPE_KNOWN(3) {}
  uint8_t position_covariance_type;
};
/*------------------- End Sensor Messages -----------------------------------*/

/// A cylindrical obstacle message
struct Obstacle {
	float x;
	float y;
	float height;
	float radius;
};

/** Radar Messagess
* Although there are not radar messages in ROS, these are similar to the ones on
*  https://github.com/astuff/astuff_sensor_msgs/tree/master/radar_msgs/msg 
*/
struct RadarDetection {
	uint16_t detection_id;
	//x and y components only
	Point position;
	//range_rate transformation to x-y components
	Point velocity;
	//detection amplitude in db
	double amplitude;
};

struct RadarDetectionArray {
	Header header;
	std::vector<RadarDetection> detections;
};

struct RadarTarget {
	uint16_t id;
	uint16_t status;
	//raw longitudinal range (m)
	float range;
	//raw longitudinal range velocity (m/s)
	float range_rate;
	//raw longitudinal range acceleration (m/s^2)
	float range_accleration;
	//heading angle (rad)
	float angle; 
	//width (m)
	float width;
	// lateral velocity (m/s)
	float lateral_rate;
	// forward distance in vehicle reference frame (m)
	float position_x;
	//left distance in vehicle reference frame (m)
	float position_y;
	//The rgb reflectance of the object
	Vector3 color;
};

//Lists all the targets the radar is tracking
struct RadarObjects {
	Header header;
	uint16_t number_of_targets;
	std::vector<RadarTarget> objects;
};

struct RadarTrack {
	//sequential identifier of each track
	uint16_t track_id;
	//approximaters the target shape
	Polygon track_shape;
	//x-y components only
	Vector3 linear_velocity;
	//x component only
	Vector3 linear_acceleration;
};

struct RadarTrackArray {
	Header header;
	std::vector<RadarTrack> tracks;
};

// The status of a radar system
struct RadarStatus {
	int16_t curvature;
	float yaw_rate;
	float vehicle_speed;
	uint8_t max_track_targets;
	bool raw_data_mode;
	int8_t temperature;
	bool partial_blockage;
	bool side_lobe_blockage;
};
/*------------------- Done with Radar messages ------------------------------*/

/*------------------- MAVS specific messages (no ROS equivalent) ------------*/
/// Vehicle kinetic state
struct VehicleState{
  /// Position and orientation in local East, North, Up (ENU) coordinates.
  Pose pose;

  /// Linear (m/s) and angular (rad/s) velocity of the vehicle in ENU coords.
  Twist twist;

  /// Linear (m/s^2) and angular (rad/s^2) acceleration in ENU coords.
  Accel accel;
};

/// Driving command message
struct DrivingCommand{
  /// Desired throttle in range [0,1].
  double throttle;

  /// Desired steering angle in radians.
  double steering;
};

/*-------------------End MAVS specific messages -----------------------------*/
} //namespace mavs
 
#endif


/*! \mainpage The MSU Autonomous Vehicle Simulator (MAVS)
 * 
 * This is the C++ API documentation for MAVS, a simulator for autonomous ground vehicles.
 *
 * For access to the source code, see https://github.com/CGoodin/msu-autonomous-vehicle-simulator.
 * 
 * For installation and user guide, see the documentation at https://mavs-documentation.readthedocs.io/en/latest/.
 */
