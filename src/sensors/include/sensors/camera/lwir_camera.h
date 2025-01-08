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
 * \class LwirCamera
 *
 * A long-wave IR (9-14 microns) camera class
 * 
 * \author Chris Goodin
 *
 * \date 4/25/2022
 */

#ifndef LWIR_CAMERA_H
#define LWIR_CAMERA_H

#include <sensors/camera/camera.h>

namespace mavs{
namespace sensor{
namespace camera{

struct ThermalObject {
	ThermalObject() {
		temperature = 288.0f; //Kelvin
		emittance = 0.85f; 
	}
	float temperature;
	float emittance;
};

class CameraEfficiencyCurve {
public:
	CameraEfficiencyCurve() {
		wavelength.push_back(6.0f);
		efficiency.push_back(1.0f);
		wavelength.push_back(11.0f);
		efficiency.push_back(1.0f);
		wavelength.push_back(16.0f);
		efficiency.push_back(1.0f);
	}
	float GetEfficiencyAtWavelength(float wl_microns);
	std::vector<float> wavelength;
	std::vector<float> efficiency;
};

class LwirCamera : public Camera {
 public:
	 LwirCamera();

	 LwirCamera(int nx, int ny, float dx, float dy, float focal_length);

  ~LwirCamera();

	LwirCamera(const LwirCamera &cam){
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
		background_color_ = cam.background_color_;
		air_temperature_ = cam.air_temperature_;
		sky_emittance_ = cam.sky_emittance_;
		thermal_objects_ = cam.thermal_objects_;
		camera_efficiency_ = cam.camera_efficiency_;
		pixel_sample_factor_ = cam.pixel_sample_factor_;
  }

  /**
   * Update the sensor
   * \param env Pointer to the environment data structure.
   * \param dt Time step of render
   */
  void Update(environment::Environment *env, double dt);

	/**
	* Set the backgrournd/sky intensity, values range from 0-255
	* \param nir NIR background intensity 
	*/
	void SetBackgroundIntensity(float nir) {
		background_color_ = nir;
	}

	/**
	* Load the temperature and emissivity data necessary for the thermal simulation
	* \param input_file Full path to the thermal input file
	*/
	void LoadThermalData(std::string input_file);

private:
	float background_color_;
	float air_temperature_;
	float sky_emittance_;

	float IntegrateLuminance(float emittance, float temperature);

	std::map<std::string, ThermalObject> thermal_objects_;
	CameraEfficiencyCurve camera_efficiency_;
};

} //namespace camera
} //namespace sensor
} //namespace mavs

#endif
