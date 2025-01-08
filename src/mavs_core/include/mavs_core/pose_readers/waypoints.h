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
#ifndef MAVS_WAYPOINTS_H
#define MAVS_WAYPOINTS_H
/**
* \class Waypoints
*
* Class for defining and manipulating waypoints
* in x-y global space. Can be used by autonomous
* drivers or simulation actors.
*
* \author Chris Goodin
*
* \date 5/31/2018
*/

#include <string>
#include <vector>

#include <mavs_core/messages.h>
#include <glm/glm.hpp>

namespace mavs {

class Waypoints {
 public:
	///Create a waypoints object
	Waypoints();
	/// Waypoints destructor
	~Waypoints();
	
	/**
	* Load waypoints from a json file
	* \param infile The full path to the json input
	* file for the waypoints.
	*/
	void Load(std::string infile);

	/**
	* Convert a mavs path message (similar 
	* structure of a ROS path message) to 
	* a waypoint object.
	* \param path The MAVS path message.
	*/
	void CreateFromPathMsg(mavs::Path path);

	/**
	* Create a waypoint structure from an ANVEL .vprp file
	* in text format.
	* \param anvel_file Full path to the ANVEL .vprp file
	*/
	void CreateFromAnvelVprp(std::string anvel_file);

	/**
	* Save the current waypoints to a .vprp file.
	* \param anvel_file The output .vprp file to save.
	*/
	void SaveAsVprp(std::string anvel_file);

	/**
	* Save the current waypoints to a json file.
	* \param json_file The output .json file to save.
	*/
	void SaveAsJson(std::string json_file);

	/**
	* Return a MAVS path message from a 
	* Waypoints object
	*/
	mavs::Path GetPathMsg();

	/// Return the number of waypoints
	size_t NumWaypoints() { return path_.size(); }

	/**
	 *  Return waypoint number wp
	 * \param wp The number of the waypoint to get
	 */ 
	glm::vec2 GetWaypoint(int wp);

	/// Return a list of all the waypoints
	std::vector<glm::vec2> GetWaypoints(){ return path_; }

	/**
	* Add a point to the WP path.
	* \param point The point to add, in global ENU coordinates
	*/
	void AddPoint(glm::vec2 point);

	/// Clear all waypoints
	void Clear() { path_.clear(); }

	/// Print the path to stdout
	void Print();

	/// Get the lower left corner of the path
	glm::vec2 GetLowerLeft() { return llc_; }

	/// Get the upper right corner of the path
	glm::vec2 GetUpperRight() { return urc_; }

	/**
	* Tells if a point is on the path, within a certain distance
	* \param point The test point
	* \param dist Distance parameter for test point
	* \param dp Output distance the point is from the trail
	*/
	bool IsPointOnPath(glm::vec2 point, float dist, float &dp);

	/**
	* Get the shortest distance to the path from a given point
	* \param point The test point
	*/
	float DistanceToPath(glm::vec2 point);

	/**
	* Set the resolution of the map resolution
	* Must be >= the desired trail width
	* \param res The desired resolution in meters
	*/
	void SetMapRes(float res) {
		mapres_ = res;
		CreateSegmentMap();
	}

 private:
	 float PointSegmentDistance(glm::vec2 s1, glm::vec2 s2, glm::vec2 p);
	 void CreateSegmentMap();
	 std::vector<glm::vec2> path_;
	 glm::vec2 llc_;
	 glm::vec2 urc_;
	 float mapres_;
	 int nx_;
	 int ny_;
	 std::vector<std::vector<std::vector<int> > > segments_in_cell_;
};

} //namespace mavs
#endif