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
 * \class Zed2iCamera
 *
 * Depth camera with three sensors
 *
 * \author Chris Goodin, Parastou Ahadpour Bakhtiari
 *
 * \date 4/16/2025
 */

#ifndef ZED2I_CAMERA_H
#define ZED2I_CAMERA_H

#include <sensors/camera/rgb_camera.h>

namespace mavs {
namespace sensor {
namespace camera {

class Zed2iCamera : public Sensor {
public:
	/// Constructor
	Zed2iCamera();

	/// Destructor
	~Zed2iCamera();

	/// Copy constructor
	Zed2iCamera(const Zed2iCamera& cam);

	///Inherited sensor update method from the sensor base class
	void Update(environment::Environment* env, double dt);

	/// Inherited method to set the sensor pose
	void SetPose(glm::vec3 pos, glm::quat ori);

	/// Inherited method to set the relative sensor pose
	void SetRelativePose(glm::vec3 pos, glm::quat ori);

	/// Display the image and depth windows
	void Display();

	/// Set the maximum depth the camera can measure, in meters
	void SetMaxDepth(float md) { max_depth_cm_ = 100.0f*md; }

	/// Set the block size for the stereo matching. Default is 5
	void SetBlockSize(int bs);

	///  Free the camera to be moved with the keyboard
	void FreePose() { left_cam_.FreePose(); }

	/// Return true if the camera display is open
	bool DisplayOpen() { return left_cam_.DisplayOpen(); }

	/// get the depth of a given pixel in centimeters
	float GetCmDepthAtPixel(int u, int v);

	/// Get the depth error at a given pixel in cm
	float GetCmDepthErrorAtPixel(int u, int v);

	/// Get the current pose of the camera, including the offset
	Pose GetPose() { return left_cam_.GetPose(); }

	int GetWidth() { return left_cam_.GetWidth(); }
	int GetHeight() { return left_cam_.GetHeight(); }
	int GetDepthImageWidth() { return left_cam_.GetWidth(); }
	int GetDepthImageHeight() { return left_cam_.GetHeight(); }

	float* GetImageBuffer() { return left_cam_.GetImageBuffer(); }

	int GetBufferSize() { return left_cam_.GetBufferSize();}

	int GetRangeBufferSize() { return (int)depth_cm_.size(); }

	float* GetRangeBuffer() { return depth_cm_.data(); }

	float GetMaxRangeCm() { return max_depth_cm_; }

	RgbCamera* GetCamera() { return &left_cam_; }

	void SetDisplayType(std::string display_type) { display_type_ = display_type; }

private:
	RgbCamera left_cam_;
	RgbCamera right_cam_;
	//RgbCamera center_cam_;
	cimg_library::CImgDisplay depth_disp_;
	cimg_library::CImg<float> depth_cm_;
	float max_depth_cm_;
	float min_depth_cm_;
	float baseline_cm_;
	int block_size_;
	std::vector<std::vector<float> > depth_error_;
	std::string display_type_;
	void GetDepth();
	glm::vec3 GetColorFromDepth(float depth_cm);
};

} //namespace camera
} //namespace sensor
} //namespace mavs

#endif
