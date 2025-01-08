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
 * \class Lidar
 *
 * Lidar class for line scanning and 3D scanning pulsed, TOF Lidar like
 * the Sick LMS or Velodyne series.
 *
 * \author Chris Goodin
 *
 * \date 1/10/2018
 */
#ifndef LIDAR_H
#define LIDAR_H
 //for some reason, mpi has to be included before CImg or it won't build
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <random>
#include <CImg.h>

#include <sensors/sensor.h>
#include <sensors/annotation.h>

namespace mavs {
namespace sensor {
namespace lidar {

/**
* A labeled point has extra information like color
* used for training the classifier
*/
struct labeled_point {
	float x;
	float y;
	float z;
	float intensity;
	glm::vec3 color;
	int label;
};

/// Tells the angles, relative to the sensor origin, of the beam, and if it is blanked or not.
struct BeamDef {
	float zenith;
	float azimuth;
	float blanked;
};

class Lidar : public Sensor {
public:
	///Constructor, which does nothing
	Lidar();

	/**
		* Contstructor which sets the mode of the LIDAR.
		* \param mode Mode of the lidar.
		* 0 = average over all signal
		* 1 = strongest return
		* 2 = last return
		* 3 = both strongest and last
		*/
	Lidar(int mode);

	/// Lidar copy contstructor
	Lidar(const Lidar &s) {
		position_ = s.position_;
		velocity_ = s.velocity_;
		offset_ = s.offset_;
		orientation_ = s.orientation_;
		relative_orientation_ = s.relative_orientation_;
		look_to_ = s.look_to_;
		look_side_ = s.look_side_;
		look_up_ = s.look_up_;
		distances_ = s.distances_;
		intensities_ = s.intensities_;
		points_ = s.points_;
		registered_points_ = s.registered_points_;
		max_range_ = s.max_range_;
		min_range_ = s.min_range_;
		rotation_rate_ = s.rotation_rate_;
		recharge_dt_ = s.recharge_dt_;
		mode_ = s.mode_;
		cutoff_len_ = s.cutoff_len_;
		is_planar_ = s.is_planar_;
		blanking_dist_ = s.blanking_dist_;
		pc_width_ = s.pc_width_;
		pc_height_ = s.pc_height_;
	}

	/// Lidar destructor
	virtual ~Lidar() {
		CloseDisplay();
	}

	/**
		* Loads lidar parameter from an input json file
		* \param input_file Fullt path to the input json file
		*/
	void Load(std::string input_file);

	///Update method, inherited from sensor base class, performs a scan.
	void Update(environment::Environment *env, double dt);

	///Returns the current scan in a ROS point cloud message format.
	PointCloud GetRosPointCloud();

	/**
		* Set the scan pattern of a 2D scanning Lidar.
		* \param horiz_fov_low The lower bound of the horizontal FOV, degrees.
		* \param horiz_fov_high The upper bound of the horizontal FOV, degrees.
		* \param horiz_resolution Lidar horizontal resolution, degrees.
		*/
	void SetScanPattern(float horiz_fov_low, float horiz_fov_high,
		float horiz_resolution);

	/**
		* Set the scan pattern of a 3D scanning Lidar.
		* \param horiz_fov_low The lower bound of the horizontal FOV, degrees.
		* \param horiz_fov_high The upper bound of the horizontal FOV, degrees.
		* \param horiz_resolution Lidar horizontal resolution, degrees.
		* \param vert_fov_low The lower bound of the vertical FOV, degrees.
		* \param vert_fov_high The upper bound of the vertical FOV, degrees.
		* \param vert_resolution Lidar vertical resolution, degrees.
		*/
	void SetScanPattern(float horiz_fov_low, float horiz_fov_high,
		float horiz_resolution,
		float vert_fov_low, float vert_fov_high,
		float vert_resolution);

	/**
		* Set the scan pattern size explicitly
		*/
	void SetScanSize(float horiz_fov_low, float horiz_fov_high, int num_hor,
		float vert_fov_low, float vert_fov_high, int num_ver);

	/**
		* Write the points to a space delimited column file. Writes the
		* unregistered (x,y,z) coordinate in the sensor coordinate frame.
		* \param fname Output file name
		*/
	void WritePointsToText(std::string fname);

	/**
	* Write the points to a space delimited column file. Writes the
	* registered (x,y,z) coordinate in the global coordinate frame.
	* \param fname Output file name
	*/
	void WriteRegisteredPointsToText(std::string fname);
		
	/**
	* Write the points to a space delimited column file. Writes the
	* unregistered (x,y,z) coordinate in the local sensor frame.
	* \param fname Output file name
	*/
	void WriteUnregisteredPointsToText(std::string fname);

	/**
	* Write the points to a space delimited column file. Writes the
	*  (x,y,z,i,label) coordinate in the global frame.
	* \param fname Output file name
	*/
	void WriteLabeledRegisteredPointsToText(std::string fname);

	/**
	* Write the points to a space delimited column file. Writes the
	*  (x,y,z,r,g,b) coordinate in the global frame.
	* \param fname Output file name
	*/
	void WriteColorizedRegisteredPointsToText(std::string fname);

	/**
	* Write the segmented points to a space delimited column file. Writes the
	* registered (x,y,z) coordinate in the global coordinate frame.
	* 4th column is the object id / annotation
	* \param fname Output file name
	*/
	void WriteSegmentedPointsToText(std::string fname);

	/**
	* Write points to color coded point cloud image
	* \param fname Image file name
	*/
	void WritePointsToImage(std::string fname);

	/**
	* Write points to color coded point cloud image
	* \param fname Image file name
	*/
	void WriteProjectedLidarImage(std::string fname);

	/**
	* Save the current point cloud to the Point Cloud Library (PCL)
	* ascii format (.pcd).
	* \param fname The Pcd file name, with .pcd extension
	*/
	void WritePcd(std::string fname);

	/**
	* Save the current point cloud to the Point Cloud Library (PCL)
	* ascii format (.pcd) with a column for intensity and labels
	* \param fname The Pcd file name, with .pcd extension
	*/
	void WritePcdWithLabels(std::string fname);

	/**
	* Save the current point cloud to the Point Cloud Library (PCL)
	* ascii format (.pcd) with columns for intensity, normals, and labels
	* \param fname The Pcd file name, with .pcd extension
	*/
	void WritePcdWithNormalsAndLabels(std::string fname);

	///Inherited method from sensor class
	void SaveRaw();

	/// Set the total time step between vertical scans
	void SetRechargeDt(float dt) {
		recharge_dt_ = dt;
	}

	/**
		* Sets an elliptical beam spot.
		* \param horiz_div The divergence of the beam (radians) in the horizontal
		* direction, relative to the sensor reference frame.
		* \param vert_div The divergence of the beam (radians) in the vertical
		* direction, relative to the sensor reference frame.
		*/
	void SetBeamSpotEllipse(float horiz_div, float vert_div);

	/**
		* Sets a rectangular beam spot.
		* \param horiz_div The divergence of the beam (radians) in the horizontal
		* direction, relative to the sensor reference frame.
		* \param vert_div The divergence of the beam (radians) in the vertical
		* direction, relative to the sensor reference frame.
		*/
	void SetBeamSpotRectangular(float horiz_div, float vert_div);

	/**
		* Sets a circular beam spot.
		* \param div The divergence of the beam in radians.
		*/
	void SetBeamSpotCircular(float div);

	/**
		* Set the mode of the lidar. Default = 1.
		* 0 = first
		* 1 = strongest return
		* 2 = last return
		* 3 = both strongest and last
		* \param mode The desired mode of the lidar.
		*/
	void SetMode(int mode) { mode_ = mode; }
#ifdef USE_MPI
	/// Inherited method that publishes the point cloud to
	void PublishData(int root, MPI_Comm broadcast_to);
#endif
	/// make a deep copy
	virtual Lidar* clone() const {
		return new Lidar(*this);
	}

	/// Get the points vector directly
	std::vector<glm::vec3> GetPoints() { return points_; }

	/// Get the xyzi registered points
	std::vector<glm::vec4> GetRegisteredPointsXYZI();

	/// Get a vector of labeled xyzi points
	std::vector<labeled_point> GetLabeledPoints();
	
	/// Return a vector of labeled xyzi-rgb points with no labels
	std::vector<labeled_point> GetColorizedPoints();

	/// Get a vector of labeled xyzi points registered to world coordinates
	std::vector<labeled_point> GetRegisteredLabeledPoints();

	/// Return glm vec4 of x-y-z-intensity
	std::vector<glm::vec4> GetPointsXYZI();

	/// Returns data in ROS LaserScan format
	void GetLaserScan(LaserScan &scan);

	/**
	* Returns data in PointCloud2 format, sensor frame
	* \param pc The output point cloud
	*/
	PointCloud2 GetPointCloud2();

	/**
	* Returns registered data in PointCloud2 format, global frame
	* \param pc The output point cloud
	*/
	PointCloud2 GetPointCloud2Registered();

	///Returns the minimum horizontal scan angle (radians)
	float GetAngleMin() { return angle_min_; }

	///Returns the maximum horizontal scan angle (radians)
	float GetAngleMax() { return angle_max_; }

	///Returns the horizontal scan angle increment (radians)
	float GetAngleIncrement() { return angle_increment_; }

	///Returns the minimum scan range (meters)
	float GetRangeMin() { return min_range_; }

	///Returns the maximum scan range (meters)
	float GetRangeMax() { return max_range_; }

	/// Return the number of points in the point cloud.
	size_t GetNumPoints() { return points_.size(); }

	/// Return the number of vertical channels in the lidar
	int GetNumVerticalChannels() { return pc_height_; }

	/// Return the minumum of the vertical field of view
	float GetVerticalFovMin() { return vertical_fov_min_; }

	/// Return the maximum of the vertical field of view
	float GetVerticalFovMax() { return vertical_fov_max_; }

	/// Return the vertical resolution of the sensor in radians
	float GetVerticalRes() { return vertical_res_; }

	/// Return the horizontal resolution of the sensor in radians
	float GetHorizontalRes() { return horizontal_res_; }

	/// Return point number "i"
	glm::vec3 GetPoint(int i) { return points_[i]; }

	/// Print point number "i"
	void PrintPoint(int i) { std::cout<< points_[i].x<<" "<<points_[i].y<<" "<<points_[i].z<<std::endl; }

	/// Return point number "i" in world coordinates
	glm::vec3 GetRegisteredPoint(int i) { 
		if (!registration_complete_) {
			RegisterPoints();
			registration_complete_ = true;
		}
		return registered_points_[i]; 
	}

	/// Return intensity number "i"
	float GetIntensity(int i) { return intensities_[i]; }

	/// Return distance number "i"
	float GetDistance(int i) { return distances_[i]; }

	/**
		* Returns a vector of distances, including only points for which
		* the LIDAR returned
		*/
	std::vector<float> GetReturnedDistances();

	/// Returns all distances, including no-returns, with distance==0;
	std::vector<float> GetDistances() { return distances_; }

	/**
		* Returns "registered" points - points that are in world
		* coordinates rather than sensor coordinates. This is not something
		* you can get from a real sensor but is useful for debugging purposes.
		*/
	std::vector<glm::vec3> GetRegisteredPoints();

	/**
		* Sets the "cutoff_len_" paramter of the signal processing.
		* In the "first return" setting of the processing, sub-returns
		* farther than cutoff_len_ from the initial sub-return will be ignored.
		*/
	void SetProcessCutoffLength(float l) { cutoff_len_ = l; }

	/**
	* Set the maximum range of the lidar sensor
	* \param max_range The maximum range in meters
	*/
	void SetMaxRange(float max_range) {
		max_range_ = max_range;
	}

	/**
	* Set the wavelength of the lidar in micrometers
	* \param wavelen The wavelength of the lidar
	*/
	void SetWavelength(float wavelen) { wavelength_ = wavelen; }

	/// Return the divergence of the beam
	float GetDivergence() { return divergence_; }

	/**
	* Display a top-down view of the point cloud. Inherited from sensor
	* base class.
	*/
	void Display();

	/**
	* Display a top-down view of the point cloud. 
	*/
	void DisplayTopDown();

	/**
	* Display a perspective view of the point cloud. 
	*/
	void DisplayPerspective();

	/**
	* Display a perspective view of the point cloud.
	*/
	void DisplayPerspective(int im_width, int im_height);

	/**
	* Set the display color type. Options are:
	* "color" Display RGB point cloud.
	* "range" Color by range
	* "height" Color  by height
	* "intensity" Color by intensity
	* "label" Color by label
	* "white" Just make all the points white
	* \param type The color type
	*/
	void SetDisplayColorType(std::string type);

	/**
	* True if the LIDAR is a planar scanner like the LMS 291, false otherwise.
	*/
	bool IsPlanar() {
		return is_planar_;
	}

	/**
	* Set the order of th lidar points to read out in vertical strips
	\param vert_read Set to true for points to read out in vertical blocks, false for horizontal strips
	*/
	void SetVerticalReadout(bool vert_read) { vertical_readout_ = vert_read; }
	
	/// Tell simulation that LIDAR is planar
	void SetPlanar(bool planar) {
		is_planar_ = planar;
	}

	/// Set the noise threshold parameter from 0 to 1
	void SetNoiseCutoff(float thresh) {
		noise_cutoff_ = thresh;
	}

	/**
	* Set the maximum optical dewpth that the lidar can penetrate.
	* This is relevant for dust, and smoke interactions.
	* \param thresh The optical depth threshold, must be > 0, typically < 1.0;
	*/
	void SetOpticalDepthThreshold(float thresh) { optical_depth_thresh_ = thresh; }

	/**
	* Inherited method from the sensor base class
	* Annotate the current lidar point cloud with ground truth
	* \param env Pointer to the mavs environment object
	* \param semantic Use semantic labeling if true
	*/
	void AnnotateFrame(environment::Environment *env, bool semantic);

	/**
	* Inherited method from the sensor base class
	* to save annotated lidar data
	*/
	void SaveAnnotation();

	/**
	* Inherited method from the sensor base class
	* to save annotated lidar data to a given file name
	* \param fname the base file name without extension
	*/
	void SaveAnnotation(std::string fname);

	/**
	* Save Lidar annotations to a CSV file
	* \param fname The output file name
	*/
	void SaveSemanticAnnotationsCsv(std::string fname);

	/**
	* Return the object that the ith point belongs to
	* \param i The number of the point to query
	*/
	int GetSegmentPoint(int i) {
		int n = -1;
		if (i >= 0 && i < (int)segment_points_.size()) {
			n = segment_points_[i];
		}
		return n;
	}

	/**
	* Return the color of a given point
	* \param n The number of the point to return
	*/
	glm::vec3 GetPointColor(int n) {
		glm::vec3 color(0.0f, 0.0f, 0.0f);
		if (n >= 0 && n < (int)point_colors_.size()) {
			color = point_colors_[n];
		}
		return color;
	}

	///Close the current display, if it's open
	void CloseDisplay() {
		if (!disp_.is_closed()) {
			disp_.close();
		}
	}

	/**
	* Write output in the halo format, which will result in 1 binary
	* points file and 2 text files
	*/
	void WriteHaloOutputMultiple(std::string base_name);

	/**
	* Write points in single binary file, with header and packet info embedded
	*/
	void WriteHaloOutput(std::string base_name, bool append, float sim_time);

	/**
	* Set the blanking distance, in meters
	* Returns closer than blanking distance will be ignored
	* \param bd The blanking distance in meters
	*/
	void SetBlankingDist(float bd) {
		blanking_dist_ = bd;
	}

	/**
	* Show a rendering of the return of the rays
	* \param env The current environment to render
	*/
	void DisplayLidarCamera(environment::Environment *env);

	/// Create a registered point cloud
	void RegisterPoints();

	/**
	 * Set the RMS noise in the range returns
	 * \param noise The RMS noise in meters
	 */
	void SetRangeNoise(float noise);

	/**
	 * Return errors caused by rain, particles, and sensor noise
	 */
	float GetRangeErrors(){return (psys_errors_/(1.0f*distances_.size())); }

	/**
	* Define a range of angles to be blanked.
	\param minblank The minimum angle in the blank range, in degrees.
	\param maxblank The maximum angle of the blank range, in degrees.
	*/
	void SetHorizontalBlankingRangeDegrees(float minblank, float maxblank);

	/**
	* Set the rotation rate of the lidar in Hz
	*/
	void SetRotationRate(float rot_rate) { rotation_rate_ = rot_rate; }

protected:

	//Data collected by the lidar
	std::vector<float> distances_;
	std::vector<float> intensities_;
	std::vector<glm::vec3> normals_;
	std::vector<glm::vec3> points_;
	std::vector<int> segment_points_;
	std::vector<std::string> point_labels_;
	std::vector<glm::vec3> point_colors_;
	std::vector<glm::vec3> registered_points_;

	//properties of the lidar
	float max_range_;
	float min_range_;
	float rotation_rate_;
	float recharge_dt_;
	float blanking_dist_;
	float range_noise_meters_;
	bool vertical_readout_;

	//noise functions
	std::default_random_engine generator_;
	std::normal_distribution<float> distribution_;

	//Planar lidar only
	float angle_max_;
	float angle_min_;
	float angle_increment_;
	//signal processing params
	int mode_;
	float cutoff_len_;
	bool is_planar_;
	float noise_cutoff_;
	int nsteps_;
	float wavelength_;

	std::vector<glm::mat3> scan_rotations_;
	std::vector<BeamDef> beam_properties_;

	void Pulse(int i, environment::Environment *env);

	int GetPcWidth(){return pc_width_;}

private:
	//ray trace directions
	
	std::vector<glm::vec3> beam_spot_points_;

	//lidar properties
	int num_points_per_scan_;
	float divergence_;
	//int nsaved_;

	//private methods
	void Init(int mode);
	void ZeroData();
	void ReduceData();
	bool registration_complete_; 
	
	void ProcessPulse(std::vector<float> &distances,
		std::vector<float> &intensities,
		std::vector<glm::vec3> &norms,
		std::vector<int> &ids,
		std::vector<std::string> &labels,
		glm::vec3 &look_to,
		int i);
	void TraceParticleSystems(environment::Environment *env,
		glm::vec3 direction, glm::vec3 global_direction, int i);
	void TraceRain(environment::Environment *env,
		glm::vec3 direction, int i);
	void TraceSnow(environment::Environment *env,
		glm::vec3 direction, int i);

	void FillImage();
	void FillImagePerspective(int disp_width, int disp_height);
	void FillImagePerspective();
	bool image_filled_;
	bool first_display_;
	cimg_library::CImgDisplay disp_;
	cimg_library::CImg<float> image_;
	cimg_library::CImg<float> rendered_image_;
	cimg_library::CImgDisplay render_disp_;
	int display_width_;
	int display_height_;
	std::string display_color_type_;
	glm::vec3 GetPointDisplayColor(int i, float min_height, float max_height);

	//width and height of the point cloud
	int pc_width_;
	int pc_height_;
	float vertical_fov_min_;
	float vertical_fov_max_;
	float vertical_res_;
	float horizontal_res_;

	//the most opaque cloud the laser can penetrate
	float optical_depth_thresh_;

	//absorption coefficient of rain
	float alpha_rain_;
	float alpha_snow_;
	float psys_errors_;
	bool append_file_opened_;
};

} //namespace lidar
} //namespace sensor
} //namespace mavs

#endif
