/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifdef USE_EMBREE
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <simulation/halo/halo_car.h>

#include <iomanip>

#include <mavs_core/data_path.h>
#include <mavs_core/math/utils.h>
#include <tinyfiledialogs.h>
#include <mavs_core/pose_readers/anvel_vprp_reader.h>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vehicles/controllers/pid_controller.h>
#ifdef USE_OMP
#include <omp.h>
#endif

namespace mavs {
namespace halo {

HaloCar::HaloCar() {
	scene_loaded_ = false;
	top_lidar_on_ = false;
	left_lidar_on_ = false;
	right_lidar_on_ = false;
	top_left_camera_on_ = false;
	top_right_camera_on_ = false;
	rear_left_camera_on_ = false;
	rear_right_camera_on_ = false;
	driver_camera_on_ = false;
	left_fisheye_on_ = false;
	right_fisheye_on_ = false;
	log_frame_rate_ = false;
	radar_on_ = false;
	piksi_on_ = false;
	planar_lidar_on_ = false;
	object_detector_on_ = false;
	write_combined_ = false;
	is_interactive_ = false;
	//raining_ = false;
	rain_rate_ = 0.0f;
	snow_rate_ = 0.0f;
	cloud_cover_ = 0.0f;
	turbidity_ = 2.0f;
	fog_k_ = 0.0f;
	headlights_on_ = true;
	logging_ = false;
	save_labeled_ = false;
	log_directory_set_ = false;
	render_shadows_ = true;
	using_poses_ = false;
	poses_loaded_ = false;
	display_ = true;
	write_slam_output_ = false;
	render_snapshot_ = false;
	render_snapshot_noveh_ = false;
	use_chrono_vehicle_ = false;
	pose_num_ = 0;
	image_frame_steps_ = 3;
	lidar_frame_steps_ = 10;
	gps_frame_steps_ = 1;
	can_frame_steps_ = 1;
	num_image_frames_generated_ = 0;
	lidar_blanking_dist_ = 0.0f;
	veh_init_pos_ = glm::vec3(0.0f, 0.0f, 0.0f);
	veh_init_ori_ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	dist_to_goal_ = 100.0f;
	veh_goal_ = glm::vec3(100.0f, 0.0f, 0.0f);
	dust_on_ = false;
	save_file_prefix_ = "";

	elapsed_time_ = 0.0f;
	num_steps_ = 0;
	dt_ = (float)(1.0 / 100.0); // 100 Hz

	InitializeEnvironment();

	InitializeHeadlights();

	InitializeSensors();
	SetSensorOffsets();

	mavs::MavsDataPath data_path;
	mavs_data_path_ = data_path.GetPath();

	throttle_ = 0.0f;
	steering_ = 0.0f;
	braking_ = 0.0f;
}

void HaloCar::SetImageFrameRate(float rate) {
	image_frame_steps_ = (int)((1.0f / rate) / 0.01);
}

void HaloCar::SetLidarFrameRate(float rate) {
	lidar_frame_steps_ = (int)((1.0f / rate) / 0.01);
}

void HaloCar::SetGpsFrameRate(float rate) {
	gps_frame_steps_ = (int)((1.0f / rate) / 0.01);
}

void HaloCar::SetCanBusRate(float rate) {
	can_frame_steps_ = (int)((1.0f / rate) / 0.01);
}

void HaloCar::LoadPoses(std::string posefile) {
	AnvelVprpReader reader;
	poses_ = reader.Load(posefile, 1);
	poses_loaded_ = true;
}

void HaloCar::LoadPoses() {
	char const * lTheOpenFileName;
	char const * lFilterPatterns[2] = { "*.vprp" };
	std::string fp = mavs_data_path_;
	fp.append("waypoints/");
	lTheOpenFileName = tinyfd_openFileDialog(
		"Select the ANVEL replay file",
		fp.c_str(),
		1,
		lFilterPatterns,
		NULL,
		0);
	if (lTheOpenFileName) {
		std::string posefile(lTheOpenFileName);
		std::replace(posefile.begin(), posefile.end(), '\\', '/');
		AnvelVprpReader reader;
		poses_ = reader.Load(posefile, 1);
		poses_loaded_ = true;
	}
	else {
		std::cerr << "Poses not loaded, continue driving manually" << std::endl;
	}
}

void HaloCar::LoadScene() {
	char const * lTheOpenFileName;
	char const * lFilterPatterns[2] = { "*.json" };
	std::string fp = mavs_data_path_;
	fp.append("scenes/");
	lTheOpenFileName = tinyfd_openFileDialog(
		"Select the MAVS Scene File",
		fp.c_str(),
		1,
		lFilterPatterns,
		NULL,
		0);
	std::string scenefile(lTheOpenFileName);
	std::replace(scenefile.begin(), scenefile.end(), '\\', '/');
	LoadScene(scenefile);
}

void HaloCar::LoadScene(std::string scenefile) {
	scene_.Load(scenefile);

	env_.SetRaytracer(&scene_);
	//AddActorsToScene();
	scene_loaded_ = true;
}

void HaloCar::SetRayTracingInfo(bool labeling, bool spectral, bool surface_textures){
	if (labeling){
		scene_.TurnOnLabeling();
	}
	else {
		scene_.TurnOffLabeling();
	}
	if (spectral){
		scene_.TurnOnSpectral();
	}
	else {
		scene_.TurnOffSpectral();
	}
	if (surface_textures){
		scene_.TurnOnSurfaceTextures();
	}
	else {
		scene_.TurnOffSurfaceTextures();
	}
}

/*void HaloCar::AddActorsToScene() {
	std::string actor_file = mavs_data_path_;
	actor_file.append("/actors/actors/forester_actor.json");
	actor_ids_ = env_.LoadActors(actor_file);
}*/

void HaloCar::InitializeEnvironment() {
	env_.SetLocalOrigin(33.4503998, 88.8183872, 102.0); //Starkville, MS
	current_hour_ = 12;
	current_minute_ = 0;
	env_.SetDateTime(2018, 11, 1, current_hour_, current_minute_, 0, 6); // Nov 1, 2018, noon
	env_.SetWind(5.0f, 2.5f);
	env_.SetTurbidity(turbidity_);
}

void HaloCar::InitializeHeadlights() {
	// Add headlights to the Halo Car
	mavs::environment::Light headlight;
	headlight.type = 2; //spotlight
	//headlight.angle = (float)(mavs::kDegToRad * 10.0f);
	headlight.angle = (float)(mavs::kDegToRad * 20.0f);
	headlight.cutoff_distance = 100.0f; //meters
	//headlight.color = glm::vec3(0.6f, 0.5f, 0.25f);
	//headlight.color = glm::vec3(1.2f, 1.0f, 0.5f);
	headlight.color = glm::vec3(2.4f, 2.0f, 1.0f);
	headlight.decay = 2.0f;
	right_headlight_id_ = env_.AddLight(headlight);
	left_headlight_id_ = env_.AddLight(headlight);
}

void HaloCar::InitializeVehicle() {
	std::string vehicle_file = mavs_data_path_;
	vehicle::Vehicle *veh;
#ifdef USE_CHRONO
	if (use_chrono_vehicle_) {
		chrono_car_.Load(chrono_vehicle_file_);
		veh = &chrono_car_;
	}
	else {
		vehicle_file.append("/vehicles/rp3d_vehicles/forester_2017_rp3d.json");
		full_car_.Load(vehicle_file);
		veh = &full_car_;
	}
#else
	vehicle_file.append("/vehicles/rp3d_vehicles/forester_2017_rp3d.json");
	full_car_.Load(vehicle_file);
	veh = &full_car_;
#endif

	veh_.SetVehicle(veh);
	float h = 1.0f + env_.GetGroundHeight(veh_init_pos_.x, veh_init_pos_.y);
	veh_.GetVehicle()->SetPosition(veh_init_pos_.x, veh_init_pos_.y, h);
	veh_.GetVehicle()->SetOrientation(veh_init_ori_.w, veh_init_ori_.x, veh_init_ori_.y, veh_init_ori_.z);
	//veh_.SetTimeStep(0.01);
	//veh_.GetVehicle()->SetCgHeight(1.5);
	//throttle_ = 0.0f;
	steering_ = 0.0f;
}

void HaloCar::InitializeSensors() {
	top_lidar_.SetName("Top_Lidar");
	left_lidar_.SetName("Left_Lidar");
	right_lidar_.SetName("Right_Lidar");
	top_lidar_.SetBlankingDist(3.0f);
	left_lidar_.SetBlankingDist(2.0f);
	right_lidar_.SetBlankingDist(2.0f);

	planar_lidar_.SetScanProperties(-90.0f, 90.0f, 0.25f);
	planar_lidar_.SetName("Front_Planar_Lidar");

	object_detector_.SetScanProperties(-180.0f, 180.0f, 0.25f);
	object_detector_.SetMaxRange(30.0f);
	object_detector_.SetName("Object_Detector");

	double thirty_hz = 1.0 / 30.0;
	driver_camera_.SetEnvironmentProperties(&env_);
	driver_camera_.Initialize(810, 540, 0.0225f, 0.015f, 0.009f);
	driver_camera_.SetAntiAliasing("oversampled");
	driver_camera_.SetPixelSampleFactor(3);
	driver_camera_.SetTimeStep(thirty_hz);
	driver_camera_.SetName("Driver_Camera");

	top_left_camera_.SetEnvironmentProperties(&env_);
	top_left_camera_.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	top_left_camera_.SetTimeStep(thirty_hz);
	top_left_camera_.SetName("Top_Left_Camera");
	top_left_camera_.SetRaindropsOnLens(true);

	top_right_camera_.SetEnvironmentProperties(&env_);
	top_right_camera_.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	top_right_camera_.SetTimeStep(thirty_hz);
	top_right_camera_.SetName("Top_Right_Camera");
	top_right_camera_.SetRaindropsOnLens(true);

	rear_left_camera_.SetEnvironmentProperties(&env_);
	rear_left_camera_.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	rear_left_camera_.SetTimeStep(thirty_hz);
	rear_left_camera_.SetName("Rear_Left_Camera");
	rear_left_camera_.SetRaindropsOnLens(true);

	rear_right_camera_.SetEnvironmentProperties(&env_);
	rear_right_camera_.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	rear_right_camera_.SetTimeStep(thirty_hz);
	rear_right_camera_.SetName("Rear_Right_Camera");
	rear_right_camera_.SetRaindropsOnLens(true);

	left_fisheye_.SetEnvironmentProperties(&env_);
	//left_fisheye_.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	left_fisheye_.Initialize(256, 256, 0.025f, 0.025f, 0.008f);
	left_fisheye_.SetTimeStep(thirty_hz);
	left_fisheye_.SetName("Left_Fisheye");

	right_fisheye_.SetEnvironmentProperties(&env_);
	//right_fisheye_.Initialize(256, 256, 0.0035f, 0.0035f, 0.0035f);
	right_fisheye_.Initialize(256, 256, 0.025f, 0.025f, 0.008f);
	right_fisheye_.SetTimeStep(thirty_hz);
	right_fisheye_.SetName("Right_Fisheye");

	right_fisheye_.SetEnvironmentProperties(&env_); right_fisheye_.SetEnvironmentProperties(&env_);
	control_window_.Initialize(384, 384, 0.0035f, 0.0035f, 0.0035f);
	control_window_.SetTimeStep(thirty_hz);
	control_window_.SetName("Control_Window");
	control_window_.SetGamma(0.65f);
}

void HaloCar::RenderSnapshot(bool with_vehicle) {
	sensor::camera::RgbCamera camera;
	camera.Initialize(1620, 1080, 0.0225f, 0.015f, 0.009f);
	camera.SetEnvironmentProperties(&env_);
	camera.SetAntiAliasing("oversampled");
	camera.SetPixelSampleFactor(5);
	camera.SetName("Snap_Camera");
	camera.SetRaindropsOnLens(true);
	glm::vec3 dc_offset(2.0f, 0.0f, 1.5f);
	glm::quat dc_orient(1.0f, 0.0f, 0.0f, 0.0f);
	if (with_vehicle) {
		dc_offset = glm::vec3(-10.0f, 0.0f, 1.0f);
		dc_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	}
	camera.SetRelativePose(dc_offset, dc_orient);
	camera.SetPose(veh_state_);
	camera.Update(&env_, 0.03);
	std::string ofname =  GetSaveFileName();
	camera.SaveImage(ofname);
}

void HaloCar::SetSensorOffsets() {
	glm::vec3 control_offset(-10.0f, 0.0f, 1.5f);
	glm::quat control_orient(1.0f, 0.0f, 0.0f, 0.0f);
	//glm::vec3 control_offset(0.0f, 0.0f, 10.0f);
	//glm::quat control_orient(0.7071f, 0.0f, 0.7071f, 0.0f);
	control_window_.SetRelativePose(control_offset, control_orient);

	glm::vec3 dc_offset(0.0f, 0.5f, 0.0f);
	glm::quat dc_orient(1.0f, 0.0f, 0.0f, 0.0f);
	driver_camera_.SetRelativePose(dc_offset, dc_orient);

	glm::vec3 tl_offset(1.35f, 0.5f, 0.85f);
	glm::quat tl_orient(1.0f, 0.0f, 0.0f, 0.0f);
	top_left_camera_.SetRelativePose(tl_offset, tl_orient);

	glm::vec3 tr_offset(1.35f, -0.5f, 0.85f);
	glm::quat tr_orient(1.0f, 0.0f, 0.0f, 0.0f);
	top_right_camera_.SetRelativePose(tr_offset, tr_orient);

	glm::vec3 rl_offset(-1.75f, 0.5f, 0.75f);
	glm::quat rl_orient(0.0f, 0.0f, 0.0f, 1.0f);
	rear_left_camera_.SetRelativePose(rl_offset, rl_orient);

	glm::vec3 rr_offset(-1.75f, -0.5f, 0.75f);
	glm::quat rr_orient(0.0f, 0.0f, 0.0f, 1.0f);
	rear_right_camera_.SetRelativePose(rr_offset, rr_orient);

	glm::vec3 lfish_offset(1.1f, 0.9f, 0.15f);
	glm::quat lfish_orient(0.7071f, 0.0f, 0.0f, 0.7071f);
	left_fisheye_.SetRelativePose(lfish_offset, lfish_orient);

	glm::vec3 rfish_offset(1.1f, -0.9f, 0.15f);
	glm::quat rfish_orient(0.7071f, 0.0f, 0.0f, -0.7071f);
	right_fisheye_.SetRelativePose(rfish_offset, rfish_orient);

	//glm::vec3 toplidar_offset(0.0f, 0.0f, 0.5f);
	glm::vec3 toplidar_offset(0.0f, 0.0f, 1.1f);
	glm::quat toplidar_orient(1.0f, 0.0f, 0.0f, 0.0f);
	top_lidar_.SetRelativePose(toplidar_offset, toplidar_orient);

	glm::vec3 radar_offset(2.0f, 0.0f, 0.5f);
	radar_.SetRelativePose(radar_offset, toplidar_orient);

	object_detector_.SetRelativePose(toplidar_offset, toplidar_orient);

	glm::vec3 planar_lidar_offset(1.6f, 0.0f, 0.25f);
	planar_lidar_.SetRelativePose(planar_lidar_offset, toplidar_orient);

	float to_rot = (float)(mavs::kDegToRad*87.0);
	//float fix_angle = (float)(mavs::kDegToRad*34.7);
	float fix_angle = (float)(mavs::kDegToRad*(-2.5));
	glm::vec3 left_pos(2.087, 0.779, 0.189);
	glm::vec3 right_pos(2.087, -0.779, 0.189);
	glm::vec3 left_x(cos(fix_angle), sin(fix_angle), 0.0);
	glm::vec3 right_x(cos(-fix_angle), sin(-fix_angle), 0.0);
	glm::quat left_ang = glm::angleAxis(-to_rot, left_x); 
	glm::quat right_ang = glm::angleAxis(to_rot, right_x); 
	left_lidar_.SetRelativePose(left_pos, left_ang);
	right_lidar_.SetRelativePose(right_pos, right_ang);
}

void HaloCar::SetWriteCombinedPointCloud(std::string basename, float blanking) {
	write_combined_ = true;
	combined_name_ = basename;
	lidar_blanking_dist_ = blanking;
}

void HaloCar::WriteCombinedPointCloud() {
	//puts the point clouds in the global frame
	if (num_steps_ % 10 == 0) {
		std::vector<glm::vec4> points;
		if (top_lidar_on_) {
			std::vector<glm::vec4> top_points = top_lidar_.GetRegisteredPointsXYZI();
			std::vector<float> top_dists = top_lidar_.GetDistances();
			for (int i = 0; i < (int)top_points.size(); i++) {
				if (top_dists[i] > lidar_blanking_dist_ && top_points[i].w>0.0f) {
					top_points[i].w = 100.0f * top_points[i].w;
					points.push_back(top_points[i]);
				}
			}
		}
		if (right_lidar_on_) {
			std::vector<glm::vec4> right_points = right_lidar_.GetRegisteredPointsXYZI();
			std::vector<float> right_dists = right_lidar_.GetDistances();
			for (int i = 0; i < (int)right_points.size(); i++) {
				if (right_dists[i] > lidar_blanking_dist_ && right_points[i].w>0.0f) {
					right_points[i].w = 100.0f * right_points[i].w;
					points.push_back(right_points[i]);
				}
			}
		}
		if (left_lidar_on_) {
			std::vector<glm::vec4> left_points = left_lidar_.GetRegisteredPointsXYZI();
			std::vector<float> left_dists = left_lidar_.GetDistances();
			for (int i = 0; i < (int)left_points.size(); i++) {
				if (left_dists[i] > lidar_blanking_dist_ && left_points[i].w > 0.0f) {
					left_points[i].w = 100.0f * left_points[i].w;
					points.push_back(left_points[i]);
				}
			}
		}
		/*
		glm::mat3 R_top = top_lidar_.GetOrientationMatrix();
		glm::vec3 off_top = top_lidar_.GetPosition(); 
		if (top_lidar_on_) {
			std::vector<glm::vec4> top_points = top_lidar_.GetPointsXYZI();
			for (int i = 0; i < top_points.size(); i++) {
				glm::vec3 p(top_points[i].x, top_points[i].y, top_points[i].z);
				p = (R_top*p) + off_top;
				points.push_back(glm::vec4(p,top_points[i].w));
			}
		}
		if (left_lidar_on_) {
			std::vector<glm::vec4> left_points = left_lidar_.GetPointsXYZI();
			glm::mat3 R_left = left_lidar_.GetOrientationMatrix();
			glm::vec3 off_left = left_lidar_.GetPosition();
			for (int i = 0; i < left_points.size(); i++) {
				glm::vec3 p(left_points[i].x, left_points[i].y, left_points[i].z);
				glm::vec3 p_prime = off_left + (R_left * p);
				points.push_back(glm::vec4(p_prime, left_points[i].w));
			}
		}
		if (right_lidar_on_) {
			std::vector<glm::vec4> right_points = right_lidar_.GetPointsXYZI();
			glm::mat3 R_right = right_lidar_.GetOrientationMatrix();
			glm::vec3 off_right = right_lidar_.GetPosition();
			for (int i = 0; i < right_points.size(); i++) {
				glm::vec3 p(right_points[i].x, right_points[i].y, right_points[i].z);
				glm::vec3 p_prime = off_right + (R_right * p);
				points.push_back(glm::vec4(p_prime, right_points[i].w));
			}
		}
		*/
		std::string fname = combined_name_ + "_" + mavs::utils::ToString(num_steps_,5) + ".pcd";
		pc_analyzer_.WriteCloudToPcd(points, veh_state_.pose.position, veh_state_.pose.quaternion, fname);
		//std::string fname = combined_name_ + "_" + mavs::utils::ToString(num_steps_, 5) + ".txt";
		//pc_analyzer_.WriteCloudToText(points, fname);
	}
}

/*void HaloCar::AddActor(std::string actor_name) {
	std::string actor_file = mavs_data_path_;
	if (actor_name == "hmmwv") {
		actor_file.append("/actors/hmmwv_actor.json");
	}
	else if (actor_name == "forester") {
		actor_file.append("/actors/forester_actor.json");
	}
	env_.LoadActors(actor_file);
}*/

mavs::Pose HaloCar::GetSensorRelativePose(int sens_num) {
	mavs::Pose pose;
	glm::mat3 ori_mat;
	glm::vec3 offset;
	if (sens_num == 0) {
		ori_mat = piksi_.GetGps()->GetRelativeOrientationMatrix();
		offset = piksi_.GetGps()->GetOffset();
	}
	else if (sens_num == 1) {
		ori_mat = top_left_camera_.GetRelativeOrientationMatrix();
		offset = top_left_camera_.GetOffset();
	}
	else if (sens_num == 2) {
		ori_mat = top_right_camera_.GetRelativeOrientationMatrix();
		offset = top_right_camera_.GetOffset();
	}
	else if (sens_num == 3) {
		ori_mat = rear_left_camera_.GetRelativeOrientationMatrix();
		offset = rear_left_camera_.GetOffset();
	}
	else if (sens_num == 4) {
		ori_mat = rear_right_camera_.GetRelativeOrientationMatrix();
		offset = rear_right_camera_.GetOffset();
	}
	else if (sens_num == 5) {
		ori_mat = left_fisheye_.GetRelativeOrientationMatrix();
		offset = left_fisheye_.GetOffset();
	}
	else if (sens_num == 6) {
		ori_mat = right_fisheye_.GetRelativeOrientationMatrix();
		offset = right_fisheye_.GetOffset();
	}
	else if (sens_num == 7) {
		ori_mat = top_lidar_.GetRelativeOrientationMatrix();
		offset = top_lidar_.GetOffset();
	}
	else if (sens_num == 8) {
		ori_mat = left_lidar_.GetRelativeOrientationMatrix();
		offset = left_lidar_.GetOffset();
	}
	else if (sens_num == 9) {
		ori_mat = right_lidar_.GetRelativeOrientationMatrix();
		offset = right_lidar_.GetOffset();
	}
	else if (sens_num == 10) {
		ori_mat = planar_lidar_.GetRelativeOrientationMatrix();
		offset = planar_lidar_.GetOffset();
	}
	else if (sens_num == 11) {
		ori_mat = object_detector_.GetRelativeOrientationMatrix();
		offset = object_detector_.GetOffset();
	}
	else if (sens_num == 12) {
		ori_mat = ogd_.GetRelativeOrientationMatrix();
		offset = ogd_.GetOffset();
	}
	glm::quat q(ori_mat);
	pose.quaternion = q;
	pose.position = offset;
	return pose;
}

void HaloCar::TurnOnSensor(int sens_num) {
	if (sens_num == 0) {
		piksi_on_ = true;
	}
	else if (sens_num == 1) {
		top_left_camera_on_ = true;
	}
	else if (sens_num == 2) {
		top_right_camera_on_ = true;
	}
	else if (sens_num == 3) {
		rear_left_camera_on_ = true;
	}
	else if (sens_num == 4) {
		rear_right_camera_on_ = true;
	}
	else if (sens_num == 5) {
		left_fisheye_on_ = true;
	}
	else if (sens_num == 6) {
		right_fisheye_on_ = true;
	}
	else if (sens_num == 7) {
		top_lidar_on_ = true;
	}
	else if (sens_num == 8) {
		left_lidar_on_ = true;
	}
	else if (sens_num == 9) {
		right_lidar_on_ = true;
	}
	else if (sens_num == 10) {
		planar_lidar_on_ = true;
	}
	else if (sens_num == 11) {
		object_detector_on_ = true;
	}
	else if (sens_num == 12) {
		ogd_on_ = true;
	}
}

void HaloCar::SaveNormals() {
	mavs::sensor::camera::RgbCamera cam;
	glm::vec3 offset(2.0f, 0.0f, 1.5f);
	glm::quat orient(1.0f, 0.0f, 0.0f, 0.0f);
	cam.SetRelativePose(offset, orient);
	cam.SetPose(veh_state_);
	cam.Update(&env_, 0.03);
	cam.SaveNormalsToText();
}

void HaloCar::GetKeyboardInput() {
	//Get Driving commands
	throttle_ = 0.0f;
	steering_ = 0.0f;
	braking_ = 0.0f;
	if (disp_.is_keyW()) {
		throttle_ = 1.0f;
		if (veh_req_.nVehReq<2000)veh_req_.nVehReq += 1;
	}
	else {
		if (veh_req_.nVehReq > 0)veh_req_.nVehReq -= 1;
	}
	if (disp_.is_keyS()) {
		//throttle_ = -1.0f;
		braking_ = 1.0f;
		if (veh_req_.nVehReq>0)veh_req_.nVehReq -= 1;
	}
	if (disp_.is_keyA()) {
		steering_ = 1.0f;
		veh_req_.SteeringAngleReq += 1;
	}
	else if (disp_.is_keyD()) {
		steering_ = -1.0f;
		veh_req_.SteeringAngleReq -= 1;
	}
	else {
		if (veh_req_.SteeringAngleReq > 0) {
			veh_req_.SteeringAngleReq -= 1;
		}
		else if (veh_req_.SteeringAngleReq < 0) {
			veh_req_.SteeringAngleReq += 1;
		}
	}

	//Lidar Toggles
	if (disp_.is_key7()) {
		top_lidar_on_ = !top_lidar_on_;
	}
	if (disp_.is_key8()) {
		left_lidar_on_ = !left_lidar_on_;
	}
	if (disp_.is_key9()) {
		right_lidar_on_ = !right_lidar_on_;
	}

	//Camera toggles
	if (disp_.is_keyB()) {
		driver_camera_on_ = !driver_camera_on_;
	}
	if (disp_.is_keyZ()) {
		radar_on_ = !radar_on_;
	}
	if (disp_.is_key1()) {
		top_left_camera_on_ = !top_left_camera_on_;
	}
	if (disp_.is_key2()) {
		top_right_camera_on_ = !top_right_camera_on_;
	}
	if (disp_.is_key3()) {
		rear_left_camera_on_ = !rear_left_camera_on_;
	}
	if (disp_.is_key4()) {
		rear_right_camera_on_ = !rear_right_camera_on_;
	}
	if (disp_.is_key5()) {
		left_fisheye_on_ = !left_fisheye_on_;
	}
	if (disp_.is_key6()) {
		right_fisheye_on_ = !right_fisheye_on_;
	}

	//piksi toggle
	if (disp_.is_key0()) {
		piksi_on_ = !piksi_on_;
	}

	//environment Toggles
	if (disp_.is_keyG()) {
		//raining_ = !raining_;
		rain_rate_ += 0.5f;
		if (rain_rate_ > 10.0f) {
			rain_rate_ = 0.0f;
		}
		env_.SetRainRate(rain_rate_);
	}
	if (disp_.is_keyT()) {
		turbidity_ += 1.0f;
		if (turbidity_ > 10.0f) {
			turbidity_ = 2.0f;
		}
		env_.SetTurbidity(turbidity_);
	}
	if (disp_.is_keyF()) {
		fog_k_ += 1.0f;
		if (fog_k_ > 10.0f) {
			fog_k_ = 0.0f;
		}
		//env_.SetFog((2.0f*fog_k_*1.0E-3f));
		env_.SetFog((5.0f*fog_k_*1.0E-3f));
		//env_.SetFog((fog_k_*1.0E-4f));
	}
	if (disp_.is_keyJ()) {
		SaveNormals();
	}
	if (disp_.is_keyY()) {
		log_frame_rate_ = !log_frame_rate_;
	}

	if (disp_.is_keyV()) {
		snow_rate_ += 2.5f;
		if (snow_rate_ > 25.0f) {
			snow_rate_ = 0.0f;
		}
		if (snow_rate_ > 0.0f) {
			env_.SetTemperature(-1.0f);
		}
		env_.SetSnowRate(snow_rate_);
	}

	if (disp_.is_keyH()) {
		current_minute_ += 15;
		if (current_minute_ == 60) {
			current_hour_ += 1;
			current_hour_ = current_hour_ % 24;
			current_minute_ = 0;
		}
		env_.SetDateTime(2018, 11, 1, current_hour_, current_minute_, 0, 6); // Nov 1, 2018, noon
	}
	if (disp_.is_keyM()) {
		render_shadows_ = !render_shadows_;
		driver_camera_.SetRenderShadows(render_shadows_);
		top_left_camera_.SetRenderShadows(render_shadows_);
		top_right_camera_.SetRenderShadows(render_shadows_);
		rear_left_camera_.SetRenderShadows(render_shadows_);
		rear_right_camera_.SetRenderShadows(render_shadows_);
		left_fisheye_.SetRenderShadows(render_shadows_);
		right_fisheye_.SetRenderShadows(render_shadows_);
		control_window_.SetRenderShadows(render_shadows_);
	}
	if (disp_.is_keyK()) {
		headlights_on_ = !headlights_on_;
		if (headlights_on_) {
			InitializeHeadlights();
		}
		else {
			std::vector<int> light_ids;
			light_ids.push_back(left_headlight_id_);
			light_ids.push_back(right_headlight_id_);
			env_.RemoveLights(light_ids);
		}
	}
	if (disp_.is_keyR()) {
		render_snapshot_ = true;
	}
	if (disp_.is_keyU()) {
		render_snapshot_noveh_ = true;
	}

	//Save Data toggle
	if (disp_.is_keyL()) {
		logging_ = !logging_;
		if (logging_ && !log_directory_set_) {
			SetLogDirectory();
			log_directory_set_ = true;
		}
	}

	//Use poses
	if (disp_.is_keyP()) {
		using_poses_ = !using_poses_;
		if (using_poses_ && !poses_loaded_) {
			LoadPoses();
		}
	}

	//turn on dust
	if (disp_.is_keyN()) {
		if (!dust_on_ ) {
			mavs::environment::ParticleSystem dust;
			dust.Dust();
			env_.AddParticleSystem(dust);
			env_.AssignParticleSystemToActor(0, 0);
			dust_on_ = true;
		}
	}

	//change cloud cover
	if (disp_.is_keyC()) {
		cloud_cover_ += 0.1f;
		if (cloud_cover_ >= 1.1f)cloud_cover_ = 0.0f;
		env_.SetCloudCover(cloud_cover_);
	}

	top_left_camera_.SetEnvironmentProperties(&env_);
	top_right_camera_.SetEnvironmentProperties(&env_);
	rear_left_camera_.SetEnvironmentProperties(&env_);
	rear_right_camera_.SetEnvironmentProperties(&env_);
	driver_camera_.SetEnvironmentProperties(&env_);
	control_window_.SetEnvironmentProperties(&env_);
}

void HaloCar::TurnOnLogging(std::string out_directory){
	logging_ = true;
	log_file_directory_ = out_directory;
	log_directory_set_ = true;
	AssignLogDirectory();
}

void HaloCar::AssignLogDirectory() {
	top_lidar_.SetSaveDirectory(log_file_directory_);
	left_lidar_.SetSaveDirectory(log_file_directory_);
	right_lidar_.SetSaveDirectory(log_file_directory_);
	top_left_camera_.SetSaveDirectory(log_file_directory_);
	top_right_camera_.SetSaveDirectory(log_file_directory_);
	rear_left_camera_.SetSaveDirectory(log_file_directory_);
	rear_right_camera_.SetSaveDirectory(log_file_directory_);
	left_fisheye_.SetSaveDirectory(log_file_directory_);
	right_fisheye_.SetSaveDirectory(log_file_directory_);
}

void HaloCar::SetLogDirectory() {
	save_labeled_ = tinyfd_messageBox("Save Labeled Data?",
		"Labled and Raw Data [yes] / Raw Only [no]?",
		"yesno", "question", 0);
	if (save_labeled_)scene_.TurnOnLabeling();
	char const * lTheSelectFolderName;
	lTheSelectFolderName = tinyfd_selectFolderDialog(
		"Select directory to save sensor output", NULL);
	if (lTheSelectFolderName) {
		log_file_directory_ = std::string(lTheSelectFolderName);
		log_directory_set_ = true;
		std::replace(log_file_directory_.begin(), log_file_directory_.end(), '\\', '/');
		log_file_directory_.append("/");
		AssignLogDirectory();
	}
	else {
		std::cerr << "Warning, no log directory selected, data will not be logged." << std::endl;
	}
}

std::string HaloCar::GetSaveFileName() {
	std::string fname = "output.bmp";
	char const * lTheSelectFileName;
	lTheSelectFileName = tinyfd_saveFileDialog("Select file to save", "output.bmp", 0,NULL,"Output file name");
	//lTheSelectFileName = tinyfd_saveFileDialog(
	//	"Select file to save", NULL);
	if (lTheSelectFileName) {
		fname = std::string(lTheSelectFileName);
	}
	return fname;
}

void HaloCar::WriteSlamOutput() {
	write_slam_output_ = true;
	TurnOnSensor(10); // The planar lidar
	TurnOnSensor(0); //gps/ INS
}

void HaloCar::SaveSensorData() {
	std::string outbase = log_file_directory_ + save_file_prefix_ + mavs::utils::ToString(num_steps_, 5);
	if (num_steps_ % lidar_frame_steps_ == 0) {
		if (top_lidar_on_) {
			top_lidar_.WriteHaloOutput("top_lidar", true,elapsed_time_);
			if (save_labeled_) {
				//top_lidar_.SaveRaw();
				top_lidar_.AnnotateFrame(&env_, true);
				std::vector<mavs::sensor::lidar::labeled_point> points = top_lidar_.GetLabeledPoints();
				std::string fname = outbase;
				fname.append("top_lidar");
				pc_analyzer_.AnalyzeCloud(points, fname, num_steps_, (mavs::sensor::lidar::Lidar *)&top_lidar_);
			}
		}
		if (left_lidar_on_) {
			left_lidar_.WriteHaloOutput("left_lidar",true,elapsed_time_);
			if (save_labeled_) {
				//left_lidar_.SaveRaw();
				left_lidar_.AnnotateFrame(&env_, true);
				std::vector<mavs::sensor::lidar::labeled_point> points = left_lidar_.GetLabeledPoints();
				std::string fname = outbase;
				fname.append("left_lidar");
				pc_analyzer_.AnalyzeCloud(points, fname, num_steps_, (mavs::sensor::lidar::Lidar *)&left_lidar_);
			}
		}
		if (right_lidar_on_) {
			right_lidar_.WriteHaloOutput("right_lidar",true,elapsed_time_);
			if (save_labeled_) {
				//right_lidar_.SaveRaw();
				right_lidar_.AnnotateFrame(&env_, true);
				std::vector<mavs::sensor::lidar::labeled_point> points = right_lidar_.GetLabeledPoints();
				std::string fname = outbase;
				fname.append("right_lidar");
				pc_analyzer_.AnalyzeCloud(points, fname, num_steps_, (mavs::sensor::lidar::Lidar *)&right_lidar_);
			}
		}
	} //save lidar at 10 Hz

	//piksi runs at 100 Hz by default
	if (piksi_on_ && num_steps_%gps_frame_steps_ ==0) {
		piksi_.SetTimeStamp(elapsed_time_);
		piksi_.AppendStateToSbp((log_file_directory_+"piksi.sbp"));
		//piksi_.WriteStateToSbp(outbase);
		//piksi_.PrintState();
	}

	if (num_steps_%can_frame_steps_ == 0) {
		veh_.AppendFeedbackToFile((log_file_directory_ + "veh_feedback.can"));
		veh_.SetTimeStamp(elapsed_time_);
	}
	if (num_steps_ % image_frame_steps_ == 0) { //Cameras run at 30 hz
		if (driver_camera_on_) {
			driver_camera_.SaveImage((outbase + "_driver_cam.bmp"));
			if (save_labeled_) {
				driver_camera_.AnnotateFrame(&env_, true);
				std::string seg_out = outbase;
				seg_out.append("_driver_segment.bmp");
				std::string csv_out = outbase;
				csv_out.append("_driver_labels.csv");
				driver_camera_.SaveSegmentedImage(seg_out);
				driver_camera_.SaveSemanticAnnotationsCsv(csv_out);
			}
		}
		if (top_right_camera_on_) {
			top_right_camera_.SaveImage((outbase+"_top_right.bmp"));
			if (save_labeled_) {
				top_right_camera_.AnnotateFrame(&env_, true);
				std::string seg_out = outbase;
				seg_out.append("_top_right_segment.bmp");
				std::string csv_out = outbase;
				csv_out.append("_top_right_labels.csv");
				top_right_camera_.SaveSegmentedImage(seg_out);
				top_right_camera_.SaveSemanticAnnotationsCsv(csv_out);
			}
		}
		if (top_left_camera_on_) {
			top_left_camera_.SaveImage((outbase + "_top_left.bmp"));
			if (save_labeled_) {
				top_left_camera_.AnnotateFrame(&env_, true);
				std::string seg_out = outbase;
				seg_out.append("_top_left_segment.bmp");
				std::string csv_out = outbase;
				csv_out.append("_top_left_labels.csv");
				top_left_camera_.SaveSegmentedImage(seg_out);
				top_left_camera_.SaveSemanticAnnotationsCsv(csv_out);
			}
		}
		if (rear_right_camera_on_) {
			rear_right_camera_.SaveImage((outbase + "_rear_right.bmp"));
			if (save_labeled_) {
				rear_right_camera_.AnnotateFrame(&env_, true);
				std::string seg_out = outbase;
				seg_out.append("_rear_right_segment.bmp");
				std::string csv_out = outbase;
				csv_out.append("_rear_right_labels.csv");
				rear_right_camera_.SaveSegmentedImage(seg_out);
				rear_right_camera_.SaveSemanticAnnotationsCsv(csv_out);
			}
		}
		if (rear_left_camera_on_) {
			rear_left_camera_.SaveImage((outbase + "_rear_left.bmp"));
			if (save_labeled_) {
				rear_left_camera_.AnnotateFrame(&env_, true);
				std::string seg_out = outbase;
				seg_out.append("_rear_left_segment.bmp");
				std::string csv_out = outbase;
				csv_out.append("_rear_left_labels.csv");
				rear_left_camera_.SaveSegmentedImage(seg_out);
				rear_left_camera_.SaveSemanticAnnotationsCsv(csv_out);
			}
		}
		if (left_fisheye_on_) {
			left_fisheye_.SaveRaw();
			if (save_labeled_) {
				left_fisheye_.AnnotateFrame(&env_, true);
				std::string seg_out = outbase;
				seg_out.append("_left_fisheye_segment.bmp");
				std::string csv_out = outbase;
				csv_out.append("_left_fisheye_labels.csv");
				left_fisheye_.SaveSegmentedImage(seg_out);
				left_fisheye_.SaveSemanticAnnotationsCsv(csv_out);
			}
		}
		if (right_fisheye_on_) {
			right_fisheye_.SaveRaw();
			if (save_labeled_) {
				right_fisheye_.AnnotateFrame(&env_, true);
				std::string seg_out = outbase;
				seg_out.append("_right_fisheye_segment.bmp");
				std::string csv_out = outbase;
				csv_out.append("_right_fisheye_labels.csv");
				right_fisheye_.SaveSegmentedImage(seg_out);
				right_fisheye_.SaveSemanticAnnotationsCsv(csv_out);
			}
		}
	} // save cameras at 30 Hz

	if (write_slam_output_ && (num_steps_ % 20 == 0)) { //5 hz
		planar_lidar_.WriteUnregisteredPointsToText((outbase+"_lidar.txt"));
		std::string pos_file = outbase + "pos_output.txt";
		std::ofstream fout(pos_file.c_str());
		//glm::vec3 lla = piksi_.GetGps()->GetRecieverPositionLLA();
		//glm::vec3 utm = piksi_.GetGps()->GetRecieverPositionUTM();
		glm::vec3 enu = piksi_.GetGps()->GetRecieverPositionENU();
		float steering = veh_.GetVehicle()->GetSteeringAngle();
		glm::vec3 vel = veh_.GetVehicle()->GetState().twist.linear;
		float speed = (float)sqrt(vel.x*vel.x + vel.y*vel.y);
		fout << "Time Easting Northing Speed Steering" << std::endl;
		//fout << std::setprecision(8)<<(elapsed_time_ / 1.0E-3) << " " << utm.x << " " << utm.y << " " << speed << " " << steering << std::endl;
		fout << std::setprecision(8) << (elapsed_time_ / 1.0E-3) << " " << veh_.GetVehicle()->GetState().pose.position.x << " " << veh_.GetVehicle()->GetState().pose.position.y << " " << speed << " " << steering << std::endl;
		fout.close();
	}
}

mavs::NavSatFix HaloCar::GetPiksiNavSatFix() {
	mavs::NavSatFix fix;
	glm::vec3 lla = piksi_.GetGps()->GetRecieverPositionLLA();
	fix.latitude = lla.x;
	fix.longitude = lla.y;
	fix.altitude = lla.z;
	mavs::NavSatStatus status;
	status.service = status.SERVICE_GPS;
	if (piksi_.GetGps()->GetNumSignals() >= 4) {
		status.status = status.STATUS_FIX;
	}
	else {
		status.status = status.STATUS_NO_FIX;
	}
	fix.status = status;
	return fix;
}

void HaloCar::UpdateSensors() {
	
	if ((display_ && num_steps_ % 5 == 0) ||
		(num_steps_ % lidar_frame_steps_ == 0 && (top_lidar_on_ || left_lidar_on_ || right_lidar_on_)) ||
		(piksi_on_ && num_steps_ % gps_frame_steps_ == 0) ||
		(num_steps_ % 20 == 0 && planar_lidar_on_) ||
		(num_steps_ % 10 == 0 && ogd_on_) ||
		(num_steps_ % 10 == 0 && object_detector_on_) ||
		(num_steps_ % image_frame_steps_ == 0 && (driver_camera_on_ || top_right_camera_on_ || top_left_camera_on_ || rear_left_camera_on_ || rear_right_camera_on_
			|| left_fisheye_on_ || right_fisheye_on_))
		) {
		UpdateActors();
	}

	//update control window at 20 Hz
	if (display_) {
		if (num_steps_ % 5 == 0) {
			control_window_.SetPose(veh_state_);
			control_window_.Update(&env_, 0.05);
			if (is_interactive_) {
				disp_image_ = control_window_.GetCurrentImage();
			}
			else {
				control_window_.Display();
			}
		}
	}

	if (num_steps_ % lidar_frame_steps_ == 0) {
		if (top_lidar_on_) {
			top_lidar_.SetPose(veh_state_);
			top_lidar_.Update(&env_, 0.1);
			if (is_interactive_) {
				top_lidar_.Display();
				//top_lidar_.DisplayLidarCamera(&env_);
			}
		}
		if (left_lidar_on_) {
			left_lidar_.SetPose(veh_state_);
			left_lidar_.Update(&env_, 0.1);
			if (is_interactive_)left_lidar_.Display();
		}
		if (right_lidar_on_) {
			right_lidar_.SetPose(veh_state_);
			right_lidar_.Update(&env_, 0.1);
			if (is_interactive_)right_lidar_.Display();
		}
	} // 10 Hz stuff

	if (piksi_on_ && num_steps_ % gps_frame_steps_ == 0) {
		piksi_.SetPose(veh_state_);
		piksi_.Update(&env_, 0.01);
		if (is_interactive_)piksi_.Display();
	}

	// radar
	if (num_steps_ % 20 == 0 && radar_on_) {
		radar_.SetPose(veh_state_);
		radar_.Update(&env_, 0.2);
		if (is_interactive_)radar_.Display();
	}

	// obstacle detector
	if (num_steps_ % 20 == 0 && planar_lidar_on_) {
		planar_lidar_.SetPose(veh_state_);
		planar_lidar_.Update(&env_, 0.2);
	}

	//occupancy grid detector
	if (num_steps_ % 10 == 0 && ogd_on_) {
		ogd_.SetPose(veh_state_);
		ogd_.Update(&env_, 0.2);
	}

	if (num_steps_ % 10 == 0 && object_detector_on_) {
		object_detector_.SetPose(veh_state_);
		object_detector_.Update(&env_, 0.1);
		object_detector_.AnnotateFrame(&env_,false);
		if (is_interactive_)object_detector_.Display();
	}

	if (num_steps_ % image_frame_steps_ == 0) {
		if (driver_camera_on_) {
			num_image_frames_generated_++;
			driver_camera_.SetPose(veh_state_);
			driver_camera_.Update(&env_, 0.03);
			if (is_interactive_)driver_camera_.Display();
		}
		if (top_right_camera_on_) {
			num_image_frames_generated_++;
			top_right_camera_.SetPose(veh_state_);
			top_right_camera_.Update(&env_, 0.03);
			if (is_interactive_)top_right_camera_.Display();
		}
		if (top_left_camera_on_) {
			num_image_frames_generated_++;
			top_left_camera_.SetPose(veh_state_);
			top_left_camera_.Update(&env_, 0.03);
			if (is_interactive_)top_left_camera_.Display();
		}
		if (rear_right_camera_on_) {
			num_image_frames_generated_++;
			rear_right_camera_.SetPose(veh_state_);
			rear_right_camera_.Update(&env_, 0.03);
			if (is_interactive_)rear_right_camera_.Display();
		}
		if (rear_left_camera_on_) {
			num_image_frames_generated_++;
			rear_left_camera_.SetPose(veh_state_);
			rear_left_camera_.Update(&env_, 0.03);
			if (is_interactive_)rear_left_camera_.Display();
		}
		if (left_fisheye_on_) {
			num_image_frames_generated_++;
			left_fisheye_.SetPose(veh_state_);
			left_fisheye_.Update(&env_, 0.03);
			if (is_interactive_)left_fisheye_.Display();
		}
		if (right_fisheye_on_) {
			num_image_frames_generated_++;
			right_fisheye_.SetPose(veh_state_);
			right_fisheye_.Update(&env_, 0.03);
			if (is_interactive_)right_fisheye_.Display();
		}
	} //30 Hz cameras
}

mavs::OccupancyGrid HaloCar::GetGrid() {
	mavs::OccupancyGrid grid;
	grid.info.height = 0;
	grid.info.width = 0;
	if (ogd_on_) {
		grid = ogd_.GetGrid();
	}
	return grid;
}

std::vector<Obstacle> HaloCar::GetObstacles() {
	std::vector<Obstacle> obstacles;
	if (object_detector_on_) {
		obstacles = object_detector_.GetObstacles();
		/*
		std::map<int, mavs::sensor::Annotation> anno = object_detector_.GetObjectAnnotations();
		std::map<int, mavs::sensor::Annotation>::iterator it;
		float thresh = 1.0E12f;
		for (it = anno.begin(); it != anno.end(); it++) {
			glm::vec3 ll = it->second.GetLLCorner();
			glm::vec3 ur = it->second.GetURCorner();
			if (ll.x<thresh && ll.x>-thresh && ll.y<thresh && ll.y>-thresh &&
				ur.x<thresh && ur.x>-thresh && ur.y<thresh && ur.y>-thresh) {
				glm::vec3 center = 0.5f*(ll + ur);
				glm::vec3 size = ur - ll;
				Obstacle obs;
				obs.x = center.x;
				obs.y = center.y;
				obs.height = ur.z;
				obs.radius = 0.5f*(float)sqrt(size.x*size.x + size.y*size.y);
				if (obs.radius>0.0f)obstacles.push_back(obs);
			}
		}
		*/
	}
	return obstacles;
}

void HaloCar::UpdateActors() {
	env_.SetActorPosition(0, veh_state_.pose.position, veh_state_.pose.quaternion);

	if (headlights_on_) {
		glm::vec3 veh_lt = veh_.GetVehicle()->GetLookTo();
		glm::vec3 veh_ls = veh_.GetVehicle()->GetLookSide();
		glm::vec3 veh_lu = veh_.GetVehicle()->GetLookUp();
		glm::vec3 veh_pos = glm::vec3(veh_state_.pose.position.x, veh_state_.pose.position.y, veh_state_.pose.position.z);
		//glm::vec3 left_light_pos = veh_pos + 1.6f*veh_lt + 0.75f*veh_ls;
		//glm::vec3 right_light_pos = veh_pos + 1.6f*veh_lt - 0.75f*veh_ls;
		glm::vec3 left_light_pos = veh_pos + 3.0f*veh_lt + 0.75f*veh_ls;
		glm::vec3 right_light_pos = veh_pos + 3.0f*veh_lt - 0.75f*veh_ls;
		float rot_ang = 0.1f; //(6 degrees)
		glm::vec3 left_lt = glm::rotate(veh_lt, rot_ang, veh_lu);
		glm::vec3 right_lt = glm::rotate(veh_lt, -rot_ang, veh_lu);
		env_.MoveLight(left_headlight_id_, left_light_pos, left_lt);
		env_.MoveLight(right_headlight_id_, right_light_pos, right_lt);
	}
}

void HaloCar::PrintState() {
	float yellow[] = { 255.0f,255.0f,0.0f };
	float green[] = { 0.0f,255.0f,0.0f };
	float red[] = { 255.0f,0.0f,0.0f };
	std::string cmd_1 = "Drive the vehicle with the W-A-S-D keys. ";
	std::string cmd_2 = "Press 7 to toggle the top lidar.";
	std::string cmd_3 = "Press 8 to toggle the left lidar.";
	std::string cmd_4 = "Press 9 to toggle the right lidar.";
	std::string cmd_5 = "Press 1 to toggle the top left camera.";
	std::string cmd_6 = "Press 2 to toggle the top right camera.";
	std::string cmd_7 = "Press 3 to toggle the rear left camera.";
	std::string cmd_8 = "Press 4 to toggle the rear right camera.";
	std::string cmd_9 = "Press 5 to toggle the left fisheye cameras.";
	std::string cmd_10 = "Press 6 to toggle the right fisheye cameras.";
	std::string cmd_11 = "Press G to increase rain rate, current rate: " + mavs::utils::ToString(rain_rate_) + " mm/h";
	std::string cmd_12 = "Press H to advance time by 1 hour, current time: ";
	if (current_hour_ < 12) {
		int ct = current_hour_;
		if (ct == 0)ct = 12;
		cmd_12.append((mavs::utils::ToString(ct)+":"+mavs::utils::ToString(current_minute_,2)));
		cmd_12.append(" AM");
	}
	else {
		int ct = current_hour_;
		if (ct != 12) ct = current_hour_ - 12;
		cmd_12.append((mavs::utils::ToString(ct)+":"+mavs::utils::ToString(current_minute_,2)));
		cmd_12.append(" PM");
	}
	std::string cmd_13 = "Press K to toggle headlights.";
	std::string cmd_14 = "Press M to toggle shadow rendering.";
	std::string cmd_15 = "Press L to toggle data logging for active sensors.";
	std::string cmd_16 = "Press P to load a .vprp for the vehicle to follow.";
	std::string cmd_17 = "Press 0 to toggle the SwiftNav Piksi.";
	std::string cmd_18 = "Press T to increase atmospheric turbidity: ";
	cmd_18 = cmd_18 + mavs::utils::ToString(turbidity_);
	std::string cmd_19 = "Press B to toggle driver view cam";
	std::string cmd_20 = "Press N to turn on dust";
	std::string cmd_21 = "Press C to increase cloud cover: ";
	cmd_21.append(mavs::utils::ToString(cloud_cover_, 1));
	std::string cmd_22 = "Press V to increase snow rate: ";
	cmd_22.append(mavs::utils::ToString(snow_rate_, 1));
	std::string cmd_23 = "Press F to increase fog density: ";
	cmd_23.append(mavs::utils::ToString(fog_k_, 1));

	disp_image_.draw_text(15, 5, cmd_1.c_str(), yellow);
	disp_image_.draw_text(15, 15, cmd_2.c_str(), yellow);
	disp_image_.draw_text(15, 25, cmd_3.c_str(), yellow);
	disp_image_.draw_text(15, 35, cmd_4.c_str(), yellow);
	disp_image_.draw_text(15, 45, cmd_5.c_str(), yellow);
	disp_image_.draw_text(15, 55, cmd_6.c_str(), yellow);
	disp_image_.draw_text(15, 65, cmd_7.c_str(), yellow);
	disp_image_.draw_text(15, 75, cmd_8.c_str(), yellow);
	disp_image_.draw_text(15, 85, cmd_9.c_str(), yellow);
	disp_image_.draw_text(15, 95, cmd_10.c_str(), yellow);
	disp_image_.draw_text(15, 105, cmd_11.c_str(), yellow);
	disp_image_.draw_text(15, 115, cmd_12.c_str(), yellow);
	disp_image_.draw_text(15, 125, cmd_13.c_str(), yellow);
	disp_image_.draw_text(15, 135, cmd_14.c_str(), yellow);
	disp_image_.draw_text(15, 145, cmd_15.c_str(), yellow);
	disp_image_.draw_text(15, 155, cmd_16.c_str(), yellow);
	disp_image_.draw_text(15, 165, cmd_17.c_str(), yellow);
	disp_image_.draw_text(15, 175, cmd_18.c_str(), yellow);
	disp_image_.draw_text(15, 185, cmd_19.c_str(), yellow);
	disp_image_.draw_text(15, 195, cmd_20.c_str(), yellow);
	disp_image_.draw_text(15, 205, cmd_21.c_str(), yellow);
	disp_image_.draw_text(15, 215, cmd_22.c_str(), yellow);
	disp_image_.draw_text(15, 225, cmd_23.c_str(), yellow);
	if (top_lidar_on_) {
		disp_image_.draw_circle(5, 20, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 20, 3, red, 1.0);
	}
	if (left_lidar_on_) {
		disp_image_.draw_circle(5, 30, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 30, 3, red, 1.0);
	}
	if (right_lidar_on_) {
		disp_image_.draw_circle(5, 40, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 40, 3, red, 1.0);
	}
	if (top_left_camera_on_) {
		disp_image_.draw_circle(5, 50, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 50, 3, red, 1.0);
	}
	if (top_right_camera_on_) {
		disp_image_.draw_circle(5, 60, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 60, 3, red, 1.0);
	}
	if (rear_left_camera_on_) {
		disp_image_.draw_circle(5, 70, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 70, 3, red, 1.0);
	}
	if (rear_right_camera_on_) {
		disp_image_.draw_circle(5, 80, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 80, 3, red, 1.0);
	}
	if (left_fisheye_on_) {
		disp_image_.draw_circle(5, 90, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 90, 3, red, 1.0);
	}
	if (right_fisheye_on_) {
		disp_image_.draw_circle(5, 100, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 100, 3, red, 1.0);
	}

	//environment toggles
	if (headlights_on_) {
		disp_image_.draw_circle(5, 130, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 130, 3, red, 1.0);
	}
	if (render_shadows_) {
		disp_image_.draw_circle(5, 140, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 140, 3, red, 1.0);
	}
	//data log toggle
	if (logging_) {
		disp_image_.draw_circle(5, 150, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 150, 3, red, 1.0);
	}
	if (using_poses_) {
		disp_image_.draw_circle(5, 160, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 160, 3, red, 1.0);
	}
	//piksi sensor
	if (piksi_on_) {
		disp_image_.draw_circle(5, 170, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 170, 3, red, 1.0);
	}
	//driver camera
	if (driver_camera_on_) {
		disp_image_.draw_circle(5, 190, 3, green, 1.0);
	}
	else {
		disp_image_.draw_circle(5, 190, 3, red, 1.0);
	}

	disp_.display(disp_image_);
}

void HaloCar::RunInteractive() {
	is_interactive_ = true;
//#ifdef USE_OMP
	//double t0 = omp_get_wtime();
//#endif
	if (scene_loaded_) {
		InitializeVehicle();
		while (true) {
#ifdef USE_OMP
			double t1 = omp_get_wtime();
#endif
			//Update the vehicle
			if (num_steps_ % 5 == 0)GetKeyboardInput();
			if (!using_poses_) {
				veh_.GetVehicle()->Update(&env_, throttle_, steering_, braking_, dt_);
				//veh_.SetRequests(veh_req_);
				//veh_.Update(&env_);
				veh_state_ = veh_.GetVehicle()->GetState();
			}
			else {
				veh_state_.pose.position = poses_[pose_num_].position;
				veh_state_.pose.quaternion = poses_[pose_num_].quaternion;
				pose_num_++;
				if (pose_num_ >= (int)poses_.size())pose_num_ = 0;
			}
			
			//UpdateActors();
			UpdateSensors();

			if (render_snapshot_) {
				RenderSnapshot(true);
				render_snapshot_ = false;
			}
			if (render_snapshot_noveh_) {
				RenderSnapshot(false);
				render_snapshot_noveh_ = false;
			}
			if (logging_) SaveSensorData();

			if (num_steps_ % 5 == 0)PrintState();

			env_.AdvanceTime(dt_);

			elapsed_time_ += dt_;
			num_steps_++;
			if (disp_.is_closed()) {
				break;
			}
#ifdef USE_OMP
			double dwall = omp_get_wtime() - t1;
			/*if (dwall < (double)dt_) {
				int msleep = (int)(1000 * (dt_ - dwall));
				mavs::utils::sleep_milliseconds(msleep);
			}*/
			//double t3 = omp_get_wtime();
			if (log_frame_rate_)std::cout << "Ratio of sim to real = " << dwall << " " << dwall / dt_ << std::endl;
#endif

		}
	}
	else {
		std::cerr << "ERROR, scene not loaded for simulation " << std::endl;
		return;
	}
} //RunInteractive

float HaloCar::GetSteeringAngle() {
	glm::vec2 pos(veh_state_.pose.position.x, veh_state_.pose.position.y);
	glm::vec3 lt = veh_.GetVehicle()->GetLookTo();
	glm::vec2 ltt(lt.x, lt.y);
	ltt = ltt / glm::length(ltt);
	glm::vec2 to_goal(veh_goal_.x - pos.x, veh_goal_.y - pos.y);
	dist_to_goal_ = glm::length(to_goal);
	to_goal = to_goal / dist_to_goal_;
	float angle_diff = -glm::orientedAngle(to_goal, ltt);
	float steering_angle = (float)(angle_diff*mavs::kRadToDeg) / 25.0f;
	steering_angle = std::min(std::max(-0.8f, steering_angle), 0.8f);
	return steering_angle;
}

void HaloCar::Run() {
	Run(INT_MAX);
}

void HaloCar::Run(int nframes) {
	is_interactive_ = false;
	mavs::vehicle::PidController speed_control;
	if (scene_loaded_) {
		InitializeVehicle();
		if (!display_) {
			std::cout << "Simulating vehicle ";
		}
		while (true && num_image_frames_generated_<nframes) { 
			
			if (!using_poses_) {
				veh_.GetVehicle()->Update(&env_, 0.5f, steering_, 0.0f, dt_);
				veh_state_ = veh_.GetVehicle()->GetState();
				steering_ = GetSteeringAngle();
			}
			else {
				veh_goal_ = poses_[pose_num_].position;
				steering_ = GetSteeringAngle();
				veh_req_.SteeringAngleReq = (int)(steering_ / 0.1);
				veh_req_.nVehReq = 50; //5 mph
				veh_.SetRequests(veh_req_);
				//veh_.Update(&env_);
				float requested_speed = 2.5f;
				if (elapsed_time_ > 1.0)requested_speed = 2.5f;
				speed_control.SetSetpoint(requested_speed);
				glm::vec3 velocity = veh_.GetVehicle()->GetState().twist.linear;
				float actual_speed = (float)sqrt(velocity.x*velocity.x + velocity.y*velocity.y);
				float throttle = (float)speed_control.GetControlVariable(actual_speed, 0.01f);
				veh_.GetVehicle()->Update(&env_, throttle, steering_, 0.0f, dt_);
				veh_state_ = veh_.GetVehicle()->GetState();
				float dist = (float)glm::length(veh_state_.pose.position - poses_[pose_num_].position);
				if (dist<4.0)pose_num_++;
				if (pose_num_ >= (int)poses_.size())break;
			}

			//UpdateActors();
			UpdateSensors();

			if (logging_) SaveSensorData();

			if (write_combined_)WriteCombinedPointCloud();

			env_.AdvanceTime(dt_);
			elapsed_time_ += dt_;
			
			//if (dist_to_goal_<5.0f)break;
			num_steps_++;
			if (!display_ && num_steps_%20==0) {
				std::cout << "."<<std::flush;
				if (num_steps_ % 100 == 0) {
					float frac = 100.0f*(1.0f*pose_num_/(1.0f*poses_.size()));
					std::cout << " " << frac << "% " << std::flush;
				}
			}
		}
	}
	else {
		std::cerr << "ERROR, scene not loaded for simulation " << std::endl;
		return;
	}
	if (!display_) {
		std::cout << std::endl;
	}
}


void HaloCar::Step(float throttle, float steering) {
	if (!scene_loaded_) {
		std::cerr << "ERROR, scene not loaded for simulation " << std::endl;
		return;
	}

	if (num_steps_ == 0) InitializeVehicle();

//#ifdef USE_OMP
//	double t0 = omp_get_wtime();
//#endif

	veh_.GetVehicle()->Update(&env_, throttle, steering, 0.0f, dt_);
	veh_state_ = veh_.GetVehicle()->GetState();
	steering_ = GetSteeringAngle();

	//UpdateActors();
	UpdateSensors();

	env_.AdvanceTime(dt_);

	if (logging_) SaveSensorData();

	elapsed_time_ += dt_;
	if (display_ && num_steps_ % 10 == 0) {
		control_window_.Display();
	}
	num_steps_++;

//#ifdef USE_OMP
	//double t1 = omp_get_wtime();
	//std::cout<<"MAVS sim/real = "<<((t1-t0)/dt_)<<std::endl;
//#endif
}

} //namespace halo
} //namespace mavs

#endif 