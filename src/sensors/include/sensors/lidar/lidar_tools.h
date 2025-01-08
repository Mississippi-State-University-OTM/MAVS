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
* \class LidarTools
*
* Class for manipulating point cloud data, 
* combining and registering point clouds, and
* projecting clouds to spherical coordiantes.
*
* \author Chris Goodin
*
* \date 8/29/2018
*/
#ifndef LIDAR_TOOLS_H
#define LIDAR_TOOLS_H

#include <vector>
#include <string>

#include <sensors/lidar/lidar.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <CImg.h>

namespace mavs {
namespace sensor {
namespace lidar {

class LidarTools {
public:
	///Constructor, which does nothing
	LidarTools();

	/**
	* Function to write a point cloud to a Pcd file.
	* The format of the cloud points is (x,y,z) = position, w = intensity.
	* \param cloud The point cloud to write
	* \param orientation The orientation of the lidar sensor that acquired the cloud
	* \param position The position of the lidar sensor that acquired the cloud
	* \param fname The full path to the pcd file to save the cloud to
	*/
	void WriteCloudToPcd(std::vector<glm::vec4> &cloud, glm::vec3 position, glm::quat orientation, std::string fname);

	/**
	* Transforms a point cloud to the given coordinate system
	* \param cloud The cloud to transform
	* \param position The translation vector
	* \param orientation Quaternion rotation
	*/
	std::vector<labeled_point> TransformCloud(std::vector<labeled_point> &cloud, glm::vec3 position, glm::quat orientation);

	/**
	* Function to write a point cloud to a Pcd file.
	* The format of the cloud points is (x,y,z) = position, w = intensity.
	* Position of the sensor is assumed to be 0,0,0
	* \param cloud The point cloud to write
	* \param fname The full path to the pcd file to save the cloud to
	*/
	void WriteCloudToPcd(std::vector<glm::vec4> &cloud, std::string fname);

	/**
	* Function to write a point cloud to a Pcd file.
	* The format of the cloud points is (x,y,z) = position, w = intensity.
	* Position of the sensor is assumed to be 0,0,0
	* \param cloud The point cloud to write
	* \param fname The full path to the pcd file to save the cloud to
	*/
	void WriteCloudToPcd(std::vector<labeled_point> &cloud, std::string fname);

	/**
	* Function to write a point cloud to a space delimited text file
	* File has four columns - x,y,z,intensity
	* \param cloud The point cloud to write
	* \param fname The full path to the text file to save the cloud to
	*/
	void WriteCloudToText(std::vector<glm::vec4> &cloud, std::string fname);

	/**
	* Function to project a point cloud to an image using spherical coordinates.
	* The format of the cloud points is (x,y,z) = position, w = intensity.
	* Position of the sensor is assumed to be 0,0,0
	* \param cloud The point cloud to write
	* \param fname The full path to the image file to save the project cloud to
	* \param vert_channels The number of vertical channels in the output .npy file
	*/
	void ProjectAndSave(std::vector<labeled_point> &cloud, std::string fname, Lidar *lidar);

	/**
	* Method to write a labeled cloud to a space delimited comman file
	* \param cloud The point cloud to write
	* \param fname The full path to the image file to save the project cloud to
	*/
	void WriteLabeledCloudToText(std::vector<labeled_point> &cloud, std::string fname);

	/**
	* Merges two point clouds into a single cloud
	* \param cloud1 The first cloud
	* \param cloud2 The second cloud
	*/
	std::vector<labeled_point> AppendClouds(std::vector<labeled_point> &cloud1, std::vector<labeled_point> &cloud2);

	/**
	* Set the vertical field-of-view for the spherical projection
	* \param lo The low angle of the FOV in degrees
	* \param hi The hi angle of the FOV in degrees
	*/
	void SetVerticalFov(float lo, float hi) {
		v_fov_ = glm::vec2(lo, hi);
	}

	/**
	* Set the horizontal field-of-view for the spherical projection
	* \param lo The low angle of the FOV in degrees
	* \param hi The hi angle of the FOV in degrees
	*/
	void SetHorizontalFov(float lo, float hi) {
		h_fov_ = glm::vec2(lo, hi);
	}

	/**
	* Select how to color the points in the spherical projection image
	* Options are "reflectance", "height", "label", or "depth"
	* Default is "reflectance"
	* \param colorby The type of coloring to use in the projection
	*/
	void SetColorBy(std::string colorby) {
		val_ = colorby;
	}

	/**
	* Set the vertical resolution for the spherical projection
	* \param res The angular resolution in degrees
	*/
	void SetVerticalResolution(float res) {
		v_res_ = res;
	}

	/**
	* Set the horizontal resolution for the spherical projection
	* \param res The angular resolution in degrees
	*/
	void SetHorizontalResolution(float res) {
		h_res_ = res;
	}

	/**
	* Write labeled point cloud to labeled text, .npy
	* \param points The labeled point cloud
	* \param name The base file name to write
	* \param num The number to append to the base file name
	* \param vert_channels The number of vertical channels in the output .npy file
	*/
	void AnalyzeCloud(std::vector<mavs::sensor::lidar::labeled_point> &points, std::string name, int num, Lidar *lidar);

	/**
	* Call this to display the labeled lidar image
	*/
	void DisplayLidarImage() {
		display_ = true;
	}

	/**
	* Call this to display (or not) the labeled lidar image
	* \param disp Set to true to display, false to close display
	*/
	void DisplayLidarImage(bool disp) {
		display_ = disp;
	}

private:
	float v_res_;
	float h_res_; 
	glm::vec2 v_fov_; 
	glm::vec2 h_fov_; 
	std::string val_; 
	bool display_;
	cimg_library::CImgDisplay lidar_tools_display_;

};
} //namespace lidar
} //namespace sensor
} //namespace mavs

#endif

