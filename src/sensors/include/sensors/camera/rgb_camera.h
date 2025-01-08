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
 * \class RgbCamera
 *
 * A basic color camera that uses the default RGB materials and textures
 * defined in the .mtl files and traces only primary rays and one secondary
 * ray for shadowing. In contrast to most of the other sensors, all
 * calculations are done in floats for compatibility with embree and glm.
 *
 * \author Chris Goodin
 *
 * \date 5/23/2018
 */

#ifndef RGB_CAMERA_H
#define RGB_CAMERA_H

#include <sensors/camera/camera.h>

namespace mavs {
namespace sensor {
namespace camera {

/**
* Structure for return info from the RenderPixel method
*/
struct RgbPixel {
	RgbPixel() :object_id(-1), color(glm::vec3(0.0f, 0.0f, 0.0f)), normal(glm::vec3(0.0f, 0.0f, 1.0f)), distance(0.0f), label("sky") {}

	/// Color of the pixel
	glm::vec3 color;

	/// normal of the pixel
	glm::vec3 normal;

	/// Distance of the pixel
	float distance;

	/// ID of the object in the pixel, -1 if no object
	int object_id;

	/// The label of the pixel, "sky" if no object
	std::string label;
};

class RgbCamera : public Camera {
public:
	/// Constructor
	RgbCamera();

	/// Destructor
	~RgbCamera();

	/// Copy constructor
	RgbCamera(const RgbCamera& cam) {
		//image_buffer_ =  cam.image_buffer_;
		image_ = cam.image_;
		num_horizontal_pix_ = cam.num_horizontal_pix_;
		num_vertical_pix_ = cam.num_vertical_pix_;
		focal_length_ = cam.focal_length_;
		focal_array_width_ = cam.focal_array_width_;
		focal_array_height_ = cam.focal_array_height_;
		horizontal_pixdim_ = cam.horizontal_pixdim_;
		vertical_pixdim_ = cam.vertical_pixdim_;
		half_horizontal_dim_ = cam.half_horizontal_dim_;
		half_vertical_dim_ = cam.half_vertical_dim_;
		gamma_ = cam.gamma_;
		gain_ = cam.gain_;
		do_fast_lights_ = cam.do_fast_lights_;
		env_set_ = cam.env_set_;
		frame_count_ = cam.frame_count_;
		groundcolor_ = cam.groundcolor_;
		skycolor_ = cam.skycolor_;
		sun_direction_ = cam.sun_direction_;
		suncolor_ = cam.suncolor_;
		up_ = cam.up_;
		blur_on_ = cam.blur_on_;
	}

	///Inherited sensor update method from the sensor base class
	void Update(environment::Environment* env, double dt);

	/// Converts the image from RGB to RCCB filter
	void ConvertImageToRccb();

	/**
		* Sets some basic electronics of the system. For the RGB camera, a white
		* surface in sunlight will have RGB of (255,255,255). Gamma and gain are
		* relative to this value.
		* \param gamma Range compression exponent, unitless. Typically [0.5,1.0]
		* \param gain Scale factor for the raw pixel values.
		*/
	void SetElectronics(float gamma, float gain);

	/**
		* Sets certain lighting conditions, like the sun position and sky color,
		* based on current environmental properties.
		* \param env Pointer to the environment object.
		*/
	void SetEnvironmentProperties(environment::Environment* env);

	/// Set to true to do a fast approximation of artificial lights in the environment
	void SetFastLights(bool fast_lights) { do_fast_lights_ = fast_lights; }

protected:

	void RenderFrame(environment::Environment* env, double dt);

	RgbPixel RenderPixel(environment::Environment* env,
		glm::vec3 direction);

	//void RenderParticleSystems(environment::Environment *env);

	void ApplyElectronics(double dt);

	void CheckLights(environment::Environment* env);

	bool env_set_;
	bool do_fast_lights_;
	glm::vec3 sun_direction_;
	glm::vec3 suncolor_;
	glm::vec3 up_;
	glm::vec3 skycolor_;
	glm::vec3 groundcolor_;
	int frame_count_;

	// Different ways to sample / perform anti-aliasing
	void RenderLoopCorners(environment::Environment* env);
	void RenderLoopOversampled(environment::Environment* env);
	void RenderLoopAdaptive(environment::Environment* env);
	void RenderLoopSimple(environment::Environment* env);

	/**
	* Inherited from Camera base class.
	* Convert a point in world coordinates to pixel coordinates
	* returns true if point is in frame, false if it is not
	* \param point_world Point in world coordinates
	* \param pixel Calculated point in pixel coordinates
	*/
	bool WorldToPixel(glm::vec3 point_world, glm::ivec2& pixel);

	std::vector <float> exposure_;

};

} //namespace camera
} //namespace sensor
} //namespace mavs

#endif
