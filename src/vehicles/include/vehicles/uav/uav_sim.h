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
* \class UavSim
*
* Class that loads in a MAVS scene, UAV mesh, and integrates with the UAV physics sim
* to enable a flight simulation.
*
* \author Chris Goodin
*
* \date 11/18/2024
*/

#ifndef MAVS_UAV_SIM_H
#define MAVS_UAV_SIM_H
#ifdef None
#undef None
#endif

// project includes
#include "vehicles/uav/uav.h"
#include "vehicles/uav/uav_controller.h"
#include "vehicles/uav/flight_control.h"
//#include "mavs_uav_sim/map_viewer.h"
// c++ includes
#include <string>
// mavs includes
#include <sensors/mavs_sensors.h>
#include <sensors/camera/lwir_camera.h>
#include <sensors/camera/ortho_camera.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#ifdef None
#undef None
#endif

namespace mavs {
namespace vehicle {
struct AnimationInit {
	glm::vec2 position;
	float heading;
	float velocity;
};

class UavSim {
public:
	UavSim();

	~UavSim();

	void LoadSimulation(std::string infile, mavs::raytracer::embree::EmbreeTracer* scene, mavs::environment::Environment* env);

	void Update(mavs::environment::Environment* env, float dt);

	bool IsActive() { return (flight_camera_.DisplayOpen() || num_steps_ == 0); }

	void AddUgvWaypoints(std::vector<glm::vec2> ugv_wp) { ugv_waypoints_ = ugv_wp; }

	void SetUgvPath(std::vector<glm::vec2> ugv_path, bool los_to_ugv);

	Uav* GetUav() { return &uav_; }

	std::vector<glm::vec2> GetFilteredWaypoints();

	void LoadAnimations(mavs::raytracer::embree::EmbreeTracer* scene, mavs::environment::Environment* env);

	UavController* GetController() { return &controller_; }

	mavs::Image GetCurrentImage();

	//sensor_msgs::msg::Image GetBlurredImage(int blur_radius);

	void SetStatus(std::string status) { status_ = status; }

	void SetCameraOn(bool on) { use_camera_ = on; }

	bool IsCameraOn() const { return use_camera_; }

	void SetRenderDebug(bool rdb) { render_debug_ = rdb; }

	void SetDesiredAltitude(float des_alt, float min_hgt);

	bool Crashed();

	bool ImageUpdated() { return image_updated_; }

	void SetControllerActive(bool cont_act) { controller_active_ = cont_act; }

private:
	FlightControl GetControlsFromWindow();
	void UpdateReadout();
	void UpdateSensors(mavs::environment::Environment* env, float dt);
	std::vector<glm::vec2> FillInWaypointsVec2(std::vector<glm::vec2> wp_in, float spacing);
	//void UpdateWaypoints();
	void UpdateDesiredAltitude(mavs::environment::Environment* env);
	std::vector<glm::vec2> GenerateWaypoints(float x, float y, float r, float theta_step_deg);
	std::string status_;
	bool save_data_;
	Uav uav_;
	UavController controller_;
	bool controller_active_;
	bool use_lwir_;
	mavs::sensor::camera::RgbCamera flight_camera_;
	mavs::sensor::camera::RgbCamera rgb_camera_;
	mavs::sensor::camera::LwirCamera lwir_camera_;
	bool image_updated_;
	//mavs::sensor::camera::RgbCamera lwir_camera_;
	//vis::MapViewer map_;
	bool use_camera_;
	bool render_debug_;
	bool use_gimbal_;
	bool use_waypoints_;
	//bool show_map_;
	int num_steps_;
	std::vector<glm::vec2> flightpath_;
	std::vector<glm::vec2> ugv_waypoints_;
	std::vector<glm::vec2> filtered_ugv_waypoints_;
	std::vector<glm::vec2> ugv_path_;
	float min_height_;
	float desired_altitude_;
	float terrain_elev_;
	float dwell_radius_;
	bool los_to_ugv_;

	std::vector<AnimationInit> anim_inits_;
	//std::vector<mavs::raytracer::Animation> animations_;
}; // class UavSim
} // namespace vehicle
} //namespace mavs

#endif // include guard