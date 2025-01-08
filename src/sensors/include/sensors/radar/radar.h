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
 * \class Radar
 *
 * Base class for radar. See: 
 * Stanislas, L., & Peynot, T. (2015). 
 * Characterisation of the Delphi Electronically Scanning Radar for robotics applications. 
 * In Proceedings of the Australasian conference on robotics and automation 2015 (pp. 1-10). 
 * Australian Robotics and Automation Association.
 * 
 * \author Chris Goodin
 *
 * \date 6/26/2018
 */

#ifndef RADAR_H
#define RADAR_H
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <CImg.h>
#include <mavs_core/environment/environment.h>
#include <mavs_core/messages.h>
#include <sensors/sensor.h>

namespace mavs{
namespace sensor{
namespace radar{

struct RadarCoordinate {
	float x; 
	float y;
	float vx;
	float vy;
	float range;
	float theta;
	bool returned;
	int id;
	bool operator >(const RadarCoordinate &r) {
		return (theta > r.theta);
	}

	bool operator <(const RadarCoordinate &r) {
		return (theta < r.theta);
	}
};
/*

*/
class Radar : public Sensor {
 public:
  Radar();

  ~Radar();

	Radar(const Radar &radar);

	///Inherited sensor update method from the sensor base class
	void Update(environment::Environment *env, double dt);

  /**
   * Displays data in real-time. Inherted from sensor base class.
   */
  void Display();
  
	/**
	* Save the radar display to an image
	* \param fname The name of the file to be saved
	*/
	void SaveImage(std::string fname);

	/**
	* Initialize the scan pattern of the display
	*/
	void Initialize(float hfov_degrees, float vfov_degrees, float angular_resolution_degrees);

  /// Inherited method from sensor base class
  void SaveRaw();
  
	/**
	* Save the currently detected objects to a text file
	* \param outfile The name of the file to write.
	*/
	void WriteObjectsToText(std::string outfile);

	/**
	* Save the discrete points comprosing the radar lobe
	* \param outfile The name of the file to write.
	*/
	void WriteLobeToText(std::string outfile);

  /**
   * Loads the radar properties
   * \param input_file Full path the the input json file.
   */
  void Load(std::string input_file);

  /// make a deep copy
  virtual Radar* clone() const{
    return new Radar(*this);
  }

	/**
	* Get a list of all the object the radar detected
	*/
	std::vector<RadarTarget> GetDetectedTargets() {
		return objects_;
	}

#ifdef USE_MPI  
	/// Inherited class that publishes the image buffer
	void PublishData(int root, MPI_Comm broadcast_to);
#endif

	/**
	* Set the resolution, in degrees, at which to sample the lobe of the
	* radar. The default is 0.25 degrees
	* \param res_degrees The desired resolution in degrees
	*/
	void SetSampleResolution(float res_degrees) { sample_resolution_ = res_degrees; }

	/**
	* Set the maximum range, in meters, of the radar.
	* \param range The desired range in meters
	*/
	void SetMaxRange(float range) { max_range_ = range; return_thresh_ = (0.9f / (max_range_ * max_range_* max_range_ * max_range_)); }


	/**
	* Inherited method from the sensor base class
	* Annotate the current radar returns with ground truth
	* \param env Pointer to the mavs environment object
	* \param semantic Use semantic labeling if true
	*/
	void AnnotateFrame(environment::Environment *env, bool semantic);

	/**
	* Inherited method from the sensor base class
	* to save annotated radar data
	*/
	void SaveAnnotation();

 protected:
	 RadarStatus status_;
	 std::vector<RadarTrack> tracks_;
	 std::vector<RadarTarget> objects_;

	 float max_range_; //meters
	 float fov_; //degrees
	 float range_rate_neg_; //max relative velocity measured toward you
	 float range_rate_pos_; //max relative velocity measured away from you
	 float beam_width_; // vertical divergence of the beam, degrees
	 float sample_resolution_;
	 float return_thresh_;
	 float range_disc_;
	 std::vector<glm::vec3> beam_spot_points_;

 private:
	 void ResetBuffers();
	 void GetTargets();
	 void Group();
	 std::vector<RadarCoordinate> raw_returns_;

	 //methods and data for displaying image
	 void FillImage();
	 glm::vec2 SensorToImageCoords(glm::vec2 sensor_coords);
	 bool image_filled_;
	 bool first_display_;
	 cimg_library::CImgDisplay disp_;
	 cimg_library::CImg<float> image_;
	 int nsteps_;
	 int display_width_;
	 int display_height_;
	 bool initialized_;

	 void WriteLabeledTargetsToText(std::string fname);
	 void SaveSemanticAnnotationsCsv(std::string fname);
	 std::vector<int> object_label_nums_;
	 std::vector<glm::vec3> object_label_colors_;
};

} //namespace radar
} //namespace sensor
} //namespace mavs

#endif
