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
 * \class PathTracerCamera
 *
 * Uses path-tracing to do a high-fidelity rendering of the scene.
 *
 * \author Chris Goodin
 *
 * \date 10/31/2019
 */

#ifndef PATH_TRACER_CAMERA_H
#define PATH_TRACER_CAMERA_H

#include <sensors/camera/camera.h>

namespace mavs {
namespace sensor {
namespace camera {

class PathTracerCamera : public Camera {
public:
	/// constructor
	PathTracerCamera();

	/// destructor
	~PathTracerCamera();

	/// copy constructor
	PathTracerCamera(const PathTracerCamera &cam) {
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
	}

	/**
		* Creates a flat rendering of the environment with no lighting effects.
		* For debug purposes only.
		* \param env Pointer to the environment data structure.
		* \param dt Time step of render
	*/
	void Update(environment::Environment *env, double dt);

	/**
	* Set the number of samples per pixel
	* \param int num Number of samples per pixel
	*/
	void SetNumIterations(int num) { num_iter_ = num; }

	/**
	* Set the maximum ray depth (number of bounces)
	* \ param md The max ray depth
	*/
	void SetMaxDepth(int md) { max_depth_ = md; }

	/**
	* Set the russian-roulette (monte-carlo) depth cutoff value.
	* After the first bounce, this is the test value for the ray 
	* to keep reflecting or be terminated. Must be between 0 and 1.
	* If 0, rays will not be terminated until they reach max depth.
	* If 1, rays will always be terminated after the first bounce.
	* \param rr_val The cutoff value
	*/
	void SetRRVal(float rr_val) { rr_val_ = rr_val; }

	/** 
	* Turn on an averaging function to remove noisy pixels.
	* On by default, usually not necessary for skylit scenes.
	*/
	void TurnOnPixelSmoothing() {
		fix_pixels_ = true;
	}

	/**
	* Turn off an averaging function to remove noisy pixels.
	* On by default, usually not necessary for skylit scenes.
	*/
	void TurnOffPixelSmoothing() {
		fix_pixels_ = false;
	}

	/**
	* Options are "max" or "average"
	* Default is "average"
	* \param norm_type The normalization type
	*/
	void SetNormalizationType(std::string norm_type) { norm_type_ = norm_type; }

private:
	int num_iter_;
	int max_depth_;
	float rr_val_;
	float pix_theta_;

	std::string norm_type_;

	glm::vec3 TraceRay(glm::vec3 origin, glm::vec3 direction, int depth, environment::Environment *env);
	glm::vec3 TraceLights(glm::vec3 hit_point, glm::vec3 direction, environment::Environment *env);
	glm::vec3 OrientNormal(glm::vec3 normal, glm::vec3 direction);
	glm::vec4 sample_f(float exp, glm::vec3 normal, glm::vec3 outgoing);
	glm::vec3 SampleHemisphere(float u1, float u2, float exp);
	glm::vec3 f_func(glm::vec3 ks, float exp, glm::vec3 incoming, glm::vec3 outgoing, glm::vec3 normal);

	bool fix_pixels_;
	void FixBadPixels();
	void Normalize();
	glm::vec3 GetLocalAverage(int i, int j, int window_size, bool use_center, bool use_weighted);
	std::vector<float> GetLocalDistribution(int i, int j, int window_size, bool use_center, bool use_weighted);

	glm::vec3 sun_color_;
	glm::vec3 CalculateRainColor(glm::vec3 incolor, float distance, glm::vec3 direction, environment::Environment *env);
	void PTMask(mavs::environment::Environment *env, float dt);
	
	std::vector<float> rand_array_;
	unsigned long int randcount_;
	unsigned long int rand_array_size_;
};

} //namespace camera
} //namespace sensor
} //namespace mavs

#endif
