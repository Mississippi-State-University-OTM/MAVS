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
#ifndef TRAIL_H
#define TRAIL_H

#include <string>
#include <mavs_core/pose_readers/waypoints.h>

namespace mavs {

class Trail {
public:
	/// Create an empty trail
	Trail();

	/**
	* Load the path of the trail from an ANVEL replay file
	* \param infile The ANVEL .vprp in text format
	*/
	void LoadPath(std::string infile);

	/**
	* Set the path of the trail from a set of MAVS wayponits
	* \param path The path used to create the trail
	*/
	void SetPath(Waypoints &path) {
		path_ = path;
	}

	/**
	* Set the width of the ruts / tracks in the trail (m)
	* \param width Rut width in meters
	*/
	void SetTrackWidth(float width) { track_width_ = width; }

	/**
	* Set the distance between the center of the ruts
	* in the trail, ie the wheelbase of the vehicle
	* that created the ruts.
	* \param wb The wheelbase in meters
	*/
	void SetWheelbase(float wb) { wheelbase_ = wb; }

	/**
	* Set the overall trail width in meters.
	* No large vegetation will grow on the trail.
	* \param width Trail width in meters
	*/
	void SetTrailWidth(float width) { 
		trail_width_ = width; 
		path_.SetMapRes(trail_width_);
	}

	/// Tell the width of the trail in meters
	float GetTrailWidth() { return trail_width_; }


	/// Tell the width of the tire tracks in meters
	float GetTrackWidth() { return track_width_; }

	/// Tell the length of the track wheelbase in meters
	float GetWheelbase() { return wheelbase_; }

	/**
	* Returns true if a point is in the wheel-tracks of the trail
	* \param point The coordinate of the test point in local ENU
	*/
	bool IsPointInTracks(glm::vec2 point);

	/// Get the lower left corner of the trail
	glm::vec2 GetLowerLeft() { return path_.GetLowerLeft(); }

	/// Get the upper right corner of the trail
	glm::vec2 GetUpperRight() { return path_.GetUpperRight(); }

	/**
	* Returns true if a point is anywhere on the trail, within a 
	* specified distance of the centerline
	* \param point The coordinate of the test point in local ENU
	* \param dist The max distance from the point to the trail centerline, equivalent to half the trail width
	* \param dp Output distance the point is from the trail
	*/
	bool IsPointOnTrail(glm::vec2 point,float dist, float &dp) {
		return path_.IsPointOnPath(point, dist, dp);
	}

	/// Return a pointer to the series of points defining the path
	Waypoints* GetPath() {
		return &path_;
	}

	/// Return the path file used to create the trail
	std::string GetPathFile() {
		return path_file_;
	}

	/**
	* Set the path file (ANVEL .vprp) that will be associated with the trail 
	* \param filename The name of the file
	*/
	void SetPathFile(std::string filename) {
		 path_file_ = filename;
	}

	/**
	* Save the trail waypoints to an ANVEL .vprp file
	* \param fname The name of the .vprp file to save
	*/
	void SaveToAnvelVprp(std::string fname);

	/**
	* Add a "pose" to the trail that includes an orientation
	* \param position The position of the point
	* \param orientation The orientation of the point
	*/
	void AddPose(glm::vec3 position, glm::quat orientation);

private:

	Waypoints path_;

	std::vector<mavs::Pose> poses_;

	float track_width_;
	float wheelbase_;
	float half_trail_width_;
	float tw_lo_;
	float tw_hi_;
	float trail_width_;
	std::string path_file_;
};

} // namespace mavs

#endif