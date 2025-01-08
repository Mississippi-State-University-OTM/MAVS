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
* \class HaloCar
*
* Class that simulates the halo car with all sensors
* User can drive the car or load a .vprp file
* Interactively adjust the environment, etc
*
* \author Chris Goodin
*
* \date 11/6/2018
*/
#ifdef USE_EMBREE
#ifndef HALO_CAR_H
#define HALO_CAR_H
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <CImg.h>

#include <vector>
#include <string>

//mavs sensors
#include <sensors/mavs_sensors.h>
#include <sensors/lidar/lidar_tools.h>

//mavs vehicles
#ifdef USE_CHRONO
#ifdef Bool
#undef Bool
#endif
#include <vehicles/chrono/chrono_wheeled_json.h>
#endif
//#include "vehicles/car/full_car.h"
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>

#include <vehicles/autonomous_vehicle.h>
#include <raytracers/embree_tracer/embree_tracer.h>
//nvidia vehicle request 
#include <interfaces/drive_px2.h>

namespace mavs {
namespace halo{

class HaloCar {
public:
	///Halo car constructor
	HaloCar();

	/**
	* Load the specified scene for the simulation
	* \param scenefile The full path to the scene file
	*/
	void LoadScene(std::string scenefile);

	/// Load a scene interactively with a file dialog
	void LoadScene();

	/// Run the simulation interactively
	void RunInteractive();

	/// Run the simulation closed-loop
	void Run();

	/// Step the simulation one time step (0.01 seconds)
	void Step(float throttle, float steering);

	/**
	* Run the simulation closed-loop until a certain number of frames is reached
	* \param nframes The desired number of frames
	*/
	void Run(int nframes);

	/**
	* Turn on a sensor through the API
	* 0 = SwiftNav INS
	* 1 = Front-left camera
	* 2 = Front-right camera
	* 3 = Rear-left camera
	* 4 = Rear-right camera
	* 5 = Left fisheye camera
	* 6 = Right fisheye camera
	* 7 = Top lidar
	* 8 = Left lidar
	* 9 = Right lidar
	* 10 = Front 2D Planar lidar (hidden)
	* 11 = Object Detector (hidden)
	* 12 = Occupancy Grid Detector (hidden)
	* \param sens_num The sensor number to turn on
	*/
	void TurnOnSensor(int sens_num);

	/**
	* Get the pose of the sensor relative to the vehicle base
	* 0 = SwiftNav INS
	* 1 = Front-left camera
	* 2 = Front-right camera
	* 3 = Rear-left camera
	* 4 = Rear-right camera
	* 5 = Left fisheye camera
	* 6 = Right fisheye camera
	* 7 = Top lidar
	* 8 = Left lidar
	* 9 = Right lidar
	* 10 = Front 2D Planar lidar (hidden)
	* 11 = Object Detector (hidden)
	* 12 = Occupancy Grid Detector (hidden)
	* \param sens_num The sensor number to turn on
	*/
	mavs::Pose GetSensorRelativePose(int sens_num);

	/**
	* Set the initial pose of the vehicle for the simulation
	* \param position Initial position in local ENU
	* \param orientation Intial orientation in local ENU
	*/
	void SetInitialVehiclePose(glm::vec3 position, glm::quat orientation) {
		veh_init_pos_ = position;
		veh_init_ori_ = orientation;
	}

	/**
	* Set the goal point for the vehicle in a non-interactive simulation
	* \param gx The x-coordinate of the goal position in local ENU coordinates
	* \param gy The y-coordinate of the goal position in local ENU coordinates
	*/
	void SetVehicleGoal(float gx, float gy) {
		veh_goal_ = glm::vec3(gx, gy, 0.0f);
	}

	/**
	* Calling this tells the sim to combine all active lidar into a 
	* single point cloud and save the output to a pcd file
	* \param basename The base file name for the output
	* \param blanking_dist Exclude returns closer than blanking distance (meters)
	*/
	void SetWriteCombinedPointCloud(std::string basename, float blanking_dist);

	/**
	* Load poses and run closed loop
	*/
	void LoadPoses(std::string);

	/**
	* Tell the halo car to use poses from .vprp or
	* take interactive input.
	* \param use True if using vprp poses
	*/
	void SetUsePoses(bool use) {
		using_poses_ = use;
	}

	/**
	* Turn the vehicle display on/off
	* \param display True to display sim window
	*/
	void SetDisplay(bool display) {
		display_ = display;
	}

	/**
	* Turn on file logging for active sensors
	* \param out_directory The directory in which to save the data
	*/
	void TurnOnLogging(std::string out_directory);

	/// Tell the number of images generated by the simulation
	int GetNumImageFrames() {
		return num_image_frames_generated_;
	}

	/**
	* Set the frame rate at which to save images in Hz.
	* Applies to all cameras
	* \param rate The desired frame rate in Hz
	*/
	void SetImageFrameRate(float rate);

	/**
	* Set the frame rate at which to save lidar scans in Hz.
	* Applies to all lidar
	* \param rate The desired frame rate in Hz
	*/
	void SetLidarFrameRate(float rate);

	/**
	* Set the frame rate at which to save GPS/INS in Hz.
	* \param rate The desired frame rate in Hz
	*/
	void SetGpsFrameRate(float rate);

	/**
	* Set the frame rate at which to save CAN messages in Hz.
	* \param rate The desired frame rate in Hz
	*/
	void SetCanBusRate(float rate);

	/**
	* Tell the simulation to label the saved data or not
	* \param label If true, saved data will be labeled
	*/
	void SetSaveLabeled(bool label) {
		save_labeled_ = label;
	}

	/**
	* Set a prefix that will be used for all the saved files
	* \param prefix The file prefix
	*/
	void SetOutfilePrefix(std::string prefix) {
		save_file_prefix_ = prefix;
	}

	/**
	* Write output for slam algorithms from the planar lidar, 
	* gps, and vehicle speed and steering
	*/
	void WriteSlamOutput();

	/// Return a pointer to the environment of the halo car
	mavs::environment::Environment* GetEnvironment() { return &env_; }

	/**
	* Set if the simulation is interactive or not
	* \param interactive True if the sim is interactive, false if not
	*/
	void SetInteractive(bool interactive) {
		is_interactive_ = interactive;
	}

	/// Return the current state of the vehicle
	VehicleState GetVehicleState() {
		return veh_state_;
	}

	/// Return a pointer to the vehicle dynamics simulation
	vehicle::AutonomousVehicle* GetVehicle() {
		return &veh_;
	}

	/// Return a list of obstacles detected by the obstacle sensor
	std::vector<Obstacle> GetObstacles();

	/// Return an occupancy grid
	mavs::OccupancyGrid GetGrid();

	/// Get the top lidar point cloud in PointCloud2 format
	mavs::PointCloud2 GetTopLidar() {
		mavs::PointCloud2 pc = top_lidar_.GetPointCloud2();
		return pc;
	}

	/// Get the left lidar point cloud in PointCloud2 format
	mavs::PointCloud2 GetLeftLidar() {
		mavs::PointCloud2 pc = left_lidar_.GetPointCloud2();
		return pc;
	}

	/// Get the right lidar point cloud in PointCloud2 format
	mavs::PointCloud2 GetRightLidar() {
		mavs::PointCloud2 pc = right_lidar_.GetPointCloud2();
		return pc;
	}

	/// Get the top left camera image
	mavs::Image GetTopLeftImage() {
		return top_left_camera_.GetRosImage();
	}

	/// Get the top right camera image
	mavs::Image GetTopRightImage() {
		return top_right_camera_.GetRosImage();
	}

	/// Get the rear left camera image
	mavs::Image GetRearLeftImage() {
		return rear_left_camera_.GetRosImage();
	}

	/// Get the rear right camera image
	mavs::Image GetRearRightImage() {
		return rear_right_camera_.GetRosImage();
	}

	/// Get the left fisheye camera image
	mavs::Image GetLeftFisheyeImage() {
		return left_fisheye_.GetRosImage();
	}

	/// Get the right fisheye camera image
	mavs::Image GetRightFisheyeImage() {
		return right_fisheye_.GetRosImage();
	}

	/// Get the return from the Piksi in NavSatFix format
	mavs::NavSatFix GetPiksiNavSatFix();

	/// Get Odometry from the Piksi
	mavs::Odometry GetOdometry(){
		return piksi_.GetOdometryMessage();
	}

	/** 
	* Add dynamic actor, can be "hmmwv" or "forester"
	* \param actor_name Specify which actor to add
	*/
	//void AddActor(std::string actor_name);

	/// Call this to use a chrono simulation for the vehicle
	void UseChronoVehicle(std::string vehicle_file) {
		use_chrono_vehicle_ = true;
		chrono_vehicle_file_ = vehicle_file;
	}

	/**
	 * Set aspects of the ray-tracing that may influence performance
	 * \param labeling Set to true to produce labeled data
	 * \param spectral Set to true to get spectral (rather than RGB) reflectances
	 * \param surface_textures Set to true to use procedurally generated surface textures 
	 */
	void SetRayTracingInfo(bool labeling, bool spectral, bool surface_textures);

private:

	//render a snapshot from the current pose
	std::string GetSaveFileName();
	void RenderSnapshot(bool with_vehicle);
	bool render_snapshot_noveh_;
	bool render_snapshot_;
	void SaveNormals();

	//simulation variables
	bool is_interactive_;
	bool display_;
	bool scene_loaded_;
	float elapsed_time_;
	int num_steps_;
	float dt_;
	float throttle_;
	float steering_;
	float braking_;
	VehicleState veh_state_;

	//data logging
	void SetLogDirectory();
	void SaveSensorData();
	bool logging_;
	bool write_slam_output_;
	std::string log_file_directory_;
	bool log_directory_set_;
	bool save_labeled_;
	void AssignLogDirectory();

	//Interactive Control window
	cimg_library::CImgDisplay disp_;
	cimg_library::CImg<float> disp_image_;
	sensor::camera::RgbCamera control_window_;
	void PrintState();
	//sensor::camera::RgbCamera control_window_;

	//Initialization Functions
	void InitializeSensors();
	void InitializeVehicle();
	void InitializeEnvironment();
	void InitializeHeadlights();
	//void AddActorsToScene();
	void SetSensorOffsets();

	//Update functions
	void GetKeyboardInput();
	void UpdateSensors();
	void UpdateActors();

	//vehicle and environment
	
#ifdef USE_CHRONO
	vehicle::ChronoWheeledJson chrono_car_;
#endif
	//vehicle::FullCar full_car_;
	vehicle::Rp3dVehicle full_car_;

	bool use_chrono_vehicle_;
	std::string chrono_vehicle_file_;
	vehicle::AutonomousVehicle veh_;
	environment::Environment env_;
	mavs::raytracer::embree::EmbreeTracer scene_;

	// Halo sensors
	//sensor::lidar::MEight top_lidar_;
	//sensor::lidar::MEight left_lidar_;
	//sensor::lidar::MEight right_lidar_;
	sensor::lidar::OusterOS2 top_lidar_;
	sensor::lidar::OusterOS2 left_lidar_;
	sensor::lidar::OusterOS2 right_lidar_;
	sensor::lidar::PlanarLidar planar_lidar_;
	sensor::ObjectDetector object_detector_;
	sensor::camera::RccbCamera top_left_camera_;
	sensor::camera::RccbCamera top_right_camera_;
	sensor::camera::RccbCamera rear_left_camera_;
	sensor::camera::RccbCamera rear_right_camera_;
	sensor::camera::RgbCamera driver_camera_;
	sensor::camera::FisheyeCamera left_fisheye_;
	sensor::camera::FisheyeCamera right_fisheye_;
	mavs::sensor::radar::Radar radar_;
	sensor::lidar::LidarTools pc_analyzer_;
	sensor::ins::SwiftnavPiksi piksi_;
	sensor::ogd::OccupancyGridDetector ogd_;

	//sensor toggles
	bool top_lidar_on_;
	bool left_lidar_on_;
	bool right_lidar_on_;
	bool top_left_camera_on_;
	bool top_right_camera_on_;
	bool rear_left_camera_on_;
	bool rear_right_camera_on_;
	bool driver_camera_on_;
	bool radar_on_;
	bool left_fisheye_on_;
	bool right_fisheye_on_;
	bool piksi_on_;
	bool planar_lidar_on_;
	bool ogd_on_;
	bool object_detector_on_;
	bool render_shadows_;
	int num_image_frames_generated_;
	int image_frame_steps_;
	int lidar_frame_steps_;
	int gps_frame_steps_;
	int can_frame_steps_;

	//headlights
	int right_headlight_id_;
	int left_headlight_id_;
	bool headlights_on_;

	//actors
	std::vector<int> actor_ids_;

	//environment toggles
	float rain_rate_;
	int current_hour_;
	int current_minute_;
	float turbidity_;
	float cloud_cover_;
	float snow_rate_;
	float fog_k_;

	// data for using an ANVEL replay file
	void LoadPoses();
	std::vector<mavs::Pose> poses_;
	bool using_poses_;
	bool poses_loaded_;
	bool log_frame_rate_;
	bool dust_on_;
	int pose_num_;

	//default location of MAVS data
	std::string mavs_data_path_;
	std::string save_file_prefix_;

	//vehicle intial position and orientation
	glm::vec3 veh_init_pos_;
	glm::quat veh_init_ori_;
	glm::vec3 veh_goal_;
	float dist_to_goal_;
	float GetSteeringAngle();

	//combined point cloud output
	void WriteCombinedPointCloud();
	bool write_combined_;
	std::string combined_name_;
	float lidar_blanking_dist_;

	//current driving request from vehicle
	nvidia::VehicleReq veh_req_;
};
} //namespace halo
} //namespace mavs

#endif

#endif 