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
 * \class Camera
 *
 * Base class for different camera simulations with varying levels of fidelity.
 * Derived class still needs to implement the sensor Update method
 *
 * \author Chris Goodin
 *
 * \date 5/23/2018
 */

#ifndef CAMERA_H
#define CAMERA_H
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <CImg.h>

#include <mavs_core/environment/environment.h>
#include <sensors/sensor.h>
#include <sensors/camera/distortion_model.h>
#include <sensors/camera/lens_drop.h>
#include <sensors/annotation.h>
#include <FastNoise.h>

namespace mavs {
namespace sensor {
namespace camera {

class Camera : public Sensor {
public:
	/// Camera constructor
	Camera();

	/// Camera destructor
	~Camera();

	/// Camera copy constructor
	Camera(const Camera &cam) {
		//image_buffer_ = cam.image_buffer_;
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
		pixel_solid_angle_ = cam.pixel_solid_angle_;
		blur_on_ = cam.blur_on_;
		disp_is_free_ = cam.disp_is_free_;
		exposure_time_ = cam.exposure_time_;
		first_display_ = cam.first_display_;
		first_seg_display_ = cam.first_seg_display_;
		frame_grabbed_ = cam.frame_grabbed_;
		horizontal_mag_scale_ = cam.horizontal_mag_scale_;
		img_saturation_ = cam.img_saturation_;
		img_temperature_ = cam.img_temperature_;
		is_fisheye_ = cam.is_fisheye_;
		key_rot_rate_ = cam.key_rot_rate_;
		key_step_size_ = cam.key_step_size_;
		ncalled_ = cam.ncalled_;
		pixel_sample_factor_ = cam.pixel_sample_factor_;
		raindrops_on_lens_ = cam.raindrops_on_lens_;
		render_shadows_ = cam.render_shadows_;
		target_brightness_ = cam.target_brightness_;
		use_distorted_ = cam.use_distorted_;
		vertical_mag_scale_ = cam.vertical_mag_scale_;
	}

	/// Return the camera focal length
	float GetFocalLength() const { return focal_length_; }

	/**
		* Initialize the camera.
		* \param num_horizontal_pix The number of pixels in the horizontal direction.
		* \param num_vertical_pix The number of pixels in the vertical direction.
		* \param focal_array_width Horizontal size (meters) of the focal plane array.
		* \param focal_array_height Vertical size (meters) of the focal plane array.
		* \param focal_length Focal length (meters) of the lens.
		*/
	void Initialize(int num_horizontal_pix, int num_vertical_pix,
		float focal_array_width, float focal_array_height,
		float focal_length);

	/**
		* Displays the image in an XWindow. Inherted from sensor base class.
		*/
	void Display();

	/// Get a pointer to the display window
	cimg_library::CImgDisplay* GetDisplay() {
		return &disp_;
	}

	/// Tells if the display window is open
	bool DisplayOpen() {
		if (!disp_.is_closed()) {
			return true;
		}
		else {
			return false;
		}
	}

	///Close the current display, if it's open
	void CloseDisplay() {
		if (!disp_.is_closed()) {
			disp_.close();
		}
	}

	/**
		* Saves the image to the specified file name.
		* \param file_name The name of the file to be saved
		*/
	void SaveImage(std::string file_name);

	/**
	* Save the range buffer to an image
	* \param file_name The name of the range image file to be saved
	*/
	void SaveRangeImage(std::string file_name);

	/**
	* Saves an image color-coded by object
	* \param file_name The name of the file to be saved
	*/
	void SaveSegmentedImage(std::string file_name);

	/**
	* Inherited method from the sensor base class
	* to save annotated sensor data
	*/
	void SaveAnnotation();

	/**
	* Inherited method from the sensor base class
	* to save annotated sensor data to a named file
	* \param ofname The base name of the file to save, with no extension
	*/
	void SaveAnnotation(std::string ofname);

	/**
	* Annotate the current camera frame with ground truth
	* \param env Pointer to the mavs environment object
	* \param semantic Use semantic labeling if true
	*/
	void AnnotateFrame(environment::Environment *env, bool semantic);

	/**
	* Annotate the current frame with ground truth
	* using the full object dimensions, not just the
	* visible portion.
	* \param env Pointer to the mavs environment object
	*/
	void AnnotateFull(environment::Environment *env);

	/**
	* Save annotations to csv file
	* \param fname The file name to save
	*/
	void SaveAnnotationsCsv(std::string fname);

	/**
	* Save box/object annotations to csv file
	* \param fname The file name to save
	*/
	void SaveBoxAnnotationsCsv(std::string fname);

	/**
	* Save semantic annotations to csv file
	* \param fname The file name to save
	*/
	void SaveSemanticAnnotationsCsv(std::string fname);

	/**
	* Save annotations to an xml file in the VOC format.
	* http://host.robots.ox.ac.uk/pascal/VOC/
	* \param fname The file name to save
	*/
	void SaveAnnotationsXmlVoc(std::string fname);

	/// Saves the image as a 5-column text file.
	void WriteImageToText(std::string file_name);

	/// Inherited method from sensor base class
	void SaveRaw();

	/**
		* Loads the basic camera properties (pixel plane size, focal length, and
		* number of pixels, as well as gamma and gain, from an input json file.
		* \param input_file Full path the the input json file.
		*/
	void Load(std::string input_file);

	/**
		* Returns the ROS Image data structure.
		*/
	Image GetRosImage();

	/**
		* Return the disparity image that would be
		* generated by stereo pair of a given baseline
		* \param baseline The baseline in meters
		*/
	Image GetDisparityImage(environment::Environment *env, float baseline);

#ifdef USE_MPI  
	/// Inherited class that publishes the image buffer
	void PublishData(int root, MPI_Comm broadcast_to);
#endif
	/// make a deep copy
	virtual Camera* clone() const {
		return new Camera(*this);
	}

	/// Get the vertical dimension of the image
	int GetHeight() const { return num_vertical_pix_; }

	/// Get the horizontal dimension of the image
	int GetWidth() const { return num_horizontal_pix_; }

	/// Return a pointer to the image buffer
	float *GetImageBuffer() {
		//glm::vec3 *GetImageBuffer() {
			//return &image_buffer_[0];
		return image_.data();
	}

	int GetBufferSize() {
		return (int)image_.size();
	}

	/**
	* Set the gamma factor on the camera
	* \param gamma Gamma factor, should range from 0.5-1.0
	*/
	void SetGamma(float gamma) { gamma_ = gamma; }

	/**
	* Get the current compression factor
	*/
	float GetGamma() { return gamma_; }

	/**
	* Set the gain factor on the camera
	* \param gain Gain factor, default is 1.0
	*/
	void SetGain(float gain) { gain_ = gain; }

	/**
	* Get the current gain factor
	*/
	float GetGain() { return gain_; }

	/**
	* Set the number of rays to sample per pixel.
	* The default is 1. Setting to higher than 1
	* will improve aliasing but slow the sim.
	*/
	void SetPixelSampleFactor(int sample_factor) {
		pixel_sample_factor_ = sample_factor;
	}

	/// Display the current segmented image
	void DisplaySegmentedImage();

	/**
	* Set the type of anti-aliasing to use in
	* the camera simulation. Values are 'adaptive',
	* 'corners', or 'oversampled'.
	* \param anti_aliasing_type The type of anti-aliasing to use
	*/
	void SetAntiAliasing(std::string anti_aliasing_type) {
		if (anti_aliasing_type == "adaptive" ||
			anti_aliasing_type == "corners" ||
			anti_aliasing_type == "oversampled") {
			anti_aliasing_type_ = anti_aliasing_type;
		}
		else {
			anti_aliasing_type = "none";
		}
	}

	/**
	* Convert a point in world coordinates to pixel coordinates
	* returns true if point is in front of the camera, false if behind
	* Derived classes that use non-ideal or distorted projections must
	* overwrite this method with appropriate transfroms.
	* \param point_world Point in world coordinates
	* \param pixel Calculated point in pixel coordinates
	*/
	virtual bool WorldToPixel(glm::vec3 point_world, glm::ivec2 &pixel);

	/**
	* Set the camera exposure time in seconds
	* \param exp_time Exposure time in seconds
	*/
	void SetExposureTime(float exp_time) {
		exposure_time_ = exp_time;
	}

	/// Get the camera exposure time in seconds;
	float GetExposureTime() {
		return exposure_time_;
	}

	/// Call this to free the camera to move with the keyboard
	void FreePose() {
		disp_is_free_ = true;
	}

	/**
	* Return a vector of length 4 that tells if certain keys have been pressed.
	* If the key has been pressed, the elements will be true, else the element will be false
	* element 0 = w = forward
	* element 1 = s = backward
	* element 2 = a = left
	* element 3 = d = right
	*/
	std::vector<bool> GetKeyCommands();

	/**
	* Call this to free the camera and specify the
	* max step size and rotation rate
	*/
	void FreePose(float step_size, float rot_rate) {
		disp_is_free_ = true;
		key_step_size_ = step_size;
		key_rot_rate_ = rot_rate;
	}

	void SetImage(cimg_library::CImg<float> img_in) {
		if (IsImageCompatible(img_in)) {
			image_ = img_in;
		}
	}

	//Return the current CImg Image
	cimg_library::CImg<float> GetCurrentImage() {
		return image_;
	}

	cimg_library::CImg<float> *GetCImg() {
		return &image_;
	}

	/**
	* Turn shadow rendering on or off.
	* Shadow rendering is on by default.
	* \param render If true, shadows will be rendered, if false then they will not.
	*/
	void SetRenderShadows(bool render) {
		render_shadows_ = render;
	}

	/// If the display has requested frame grab
	bool GrabFrame() { return frame_grabbed_; }

	/// Turn frame grab request off
	void UnsetFrameGrab() { frame_grabbed_ = false; }

	/// Get an Unregistered PointCloud2 based on range image
	PointCloud2 GetPointCloudFromImage();

	/**
	* Get 2D array of registered points corresponding to 
	* the pixels of an image.
	*/
	std::vector<std::vector<glm::vec3> > GetPointsFromImage();

	/**
	* Render raindrops on the lens
	* \param on_lens Set to true to add raindrops to lens
	*/
	void SetRaindropsOnLens(bool on_lens) { raindrops_on_lens_ = on_lens; }

	/// Save the normals to a space delimited text file, "normals.txt"
	void SaveNormalsToText();
	
	/// Set the desired value for target_brightness, should be 0-255
	void SetTargetBrightness(float target_brightness) { target_brightness_ = target_brightness; }

	/**
	* Get the color of pixel (i,j)
	* \param i The horizontal pixel coordinate
	* \param j The vertical pixel coordinate
	*/
	glm::vec3 GetPixel(int i, int j) {
		glm::vec3 color(0.0f, 0.0f, 0.0f);
		if (i >= 0 && i < image_.width() && j >= 0 && j < image_.height()) {
			color.x = image_(i, j, 0);
			color.y = image_(i, j, 1);
			color.z = image_(i, j, 2);
		}
		return color;
	}

	/**
	* Return the camera type, which can be
	* nir, rgb, pathtraced, fisheye, spherical, simple, ortho, rccb, or rededge
	*/
	std::string GetCameraType() { return camera_type_; }

	/**
	* Apply an Rccb filter to the current image
	*/
	void ApplyRccbFilter();

	
	/**
	* Set the image saturation and temperature.
	* Defaults are saturation = 1.0, temp = 6500. Temp = [4200,11200]
	* \param saturation Desired image saturation
	* \param temperature Desired color temperature
	**/
	void SetSaturationAndTemperature(float saturation, float temperature) {
		img_saturation_ = saturation;
		img_temperature_ = temperature;
	}

	void AddLidarPointsToImage(std::vector<glm::vec4> registered_xyzi);

	/// turn image blur calculation on / off
	void UseBlur(bool blur_on) { blur_on_ = blur_on; }

	/// Get the total area of the imaging plane
	float GetLensArea() const { return horizontal_pixdim_ * vertical_pixdim_; }

	float GetHorizontalPixdim() const { return horizontal_pixdim_; }

	float GetVerticalPixdim() const { return vertical_pixdim_; }

protected:

	bool IsImageCompatible(const cimg_library::CImg<float>& img) {
		return (img.width() == image_.width() &&
			img.height() == image_.height() &&
			img.depth() == image_.depth() &&
			img.spectrum() == image_.spectrum());
	}

	void AdjustSaturationAndTemperature();

	glm::vec3 RgbToHsl(float r, float g, float b);
	glm::vec3 HslToRgb(float h, float s, float l);

	int GetFlattenedIndex(int i, int j) {
		return  j + i * num_vertical_pix_;
	}

	/// MPI reduce the image buffer across the local communicator
	void ReduceImageBuffer();
	void CopyBufferToImage();

	void CreateRainMask(mavs::environment::Environment *env, double dt);
	bool raindrops_on_lens_;

	void RenderSnow(mavs::environment::Environment *env, double dt);
	void RenderFog(mavs::environment::Environment *env);
	void RenderParticleSystems(environment::Environment *env);

	/// Set the image buffer to zeros
	void ZeroBuffer();

	/// Draw a rectangle on the segmented_image
	void DrawRectangle(glm::ivec2 ll, glm::ivec2 ur);

	//std::vector<glm::vec3> image_buffer_;
	std::vector<float> range_buffer_;
	std::vector<glm::vec3> normal_buffer_;
	std::vector<int> segment_buffer_;
	std::vector<std::string> label_buffer_;
	cimg_library::CImg<float> image_;
	cimg_library::CImg<float> segmented_image_;
	cimg_library::CImgDisplay disp_;
	cimg_library::CImgDisplay seg_display_;
	bool first_seg_display_;

	bool render_shadows_;
	bool blur_on_;

	int num_horizontal_pix_;
	int num_vertical_pix_;
	float focal_length_;
	float focal_array_width_;
	float focal_array_height_;
	float horizontal_pixdim_;
	float vertical_pixdim_;
	float half_horizontal_dim_;
	float half_vertical_dim_;
	float vertical_mag_scale_;
	float horizontal_mag_scale_;
	float pixel_solid_angle_;
	float gamma_;
	float gain_;
	float exposure_time_;
	float target_brightness_;

	std::string anti_aliasing_type_;
	int pixel_sample_factor_;

	std::string fisheye_projection_;
	bool is_fisheye_;
	std::string camera_type_;

	DistortionModel dm_;
	bool use_distorted_;

	//parameters for free camera
	void UpdatePoseKeyboard();
	bool disp_is_free_;
	float key_step_size_;
	float key_rot_rate_;
	std::vector<bool> key_commands_;

	std::vector<LensDrop> droplets_;
	bool first_display_;

	float img_saturation_;
	float img_temperature_;

private:
	
	int ncalled_;
	bool frame_grabbed_;
	FastNoise snow_noise_;
};



} //namespace camera
} //namespace sensor
} //namespace mavs

#endif
