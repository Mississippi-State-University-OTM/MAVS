// library header definition
#include "mavs_matlab_interface.h"
// mavs includes
#include "mavs_core/data_path.h"
#include "sensors/mavs_sensors.h"
#include "sensors/camera/ortho_camera.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"

#if defined(_WIN32) || defined(WIN32)
#define EXPORT_CMD __declspec(dllexport)
#else
#define EXPORT_CMD 
#endif

namespace mavs {
namespace matlab {

//global data
std::vector<mavs::sensor::lidar::Lidar*> lidars;
static bool CheckLidarNum(int lidar_num) {
	if (lidars.size() <= 0) {
		std::cerr << "ERROR: REQUESTED LIDAR NUM " << lidar_num << ", but no lidars have been created." << std::endl;
		return false;
	}
	else if (lidar_num < 0 || lidar_num >= (int)lidars.size()) {
		std::cerr << "ERROR: REQUESTED LIDAR NUM " << lidar_num << ", must be in range 0-" << (lidars.size() - 1) << std::endl;
		return false;
	}
	else if (lidars[lidar_num] == NULL) {
		std::cerr << "ERROR: REQUESTED LIDAR NUM " << lidar_num << ", but it has been deleted " << std::endl;
		return false;
	}
	else {
		return true;
	}
}

std::vector<mavs::sensor::camera::Camera*> cameras;
static bool CheckCameraNum(int camera_num) {
	if (cameras.size() <= 0) {
		std::cerr << "ERROR: REQUESTED CAMERA NUM " << camera_num << ", but no cameras have been created." << std::endl;
		return false;
	}
	else if (camera_num < 0 || camera_num >= (int)cameras.size()) {
		std::cerr << "ERROR: REQUESTED CAMERA NUM " << camera_num << ", must be in range 0-" << (cameras.size() - 1) << std::endl;
		return false;
	}
	else if (cameras[camera_num] == NULL) {
		std::cerr << "ERROR: REQUESTED CAMERA NUM " << camera_num << ", but it has been deleted " << std::endl;
		return false;
	}
	else {
		return true;
	}
}

std::vector<mavs::raytracer::embree::EmbreeTracer *> scenes;
std::vector<mavs::environment::Environment *> environments;
static bool CheckSceneNum(int scene_num) {
	if (scenes.size() <= 0) {
		std::cerr << "ERROR: REQUESTED SCENE NUM " << scene_num << ", but no scenes have been created." << std::endl;
		return false;
	}
	else if (scene_num < 0 || scene_num >= scenes.size()) {
		std::cerr << "ERROR: REQUESTED SCENE NUM " << scene_num << ", must be in range 0-" << (scenes.size() - 1) << std::endl;
		return false;
	}
	else if (scenes[scene_num] == NULL) {
		std::cerr << "ERROR: REQUESTED SCENE NUM " << scene_num << ", but it has been deleted " << std::endl;
		return false;
	}
	else {
		return true;
	}
}

std::vector<mavs::vehicle::Rp3dVehicle*> vehicles;
static bool CheckVehicleNum(int veh_num) {
	if (vehicles.size() <= 0) {
		std::cerr << "ERROR: REQUESTED VEHICLE NUM " << veh_num << ", but no vehicles have been created." << std::endl;
		return false;
	}
	else if (veh_num < 0 || veh_num >= (int)vehicles.size()) {
		std::cerr << "ERROR: REQUESTED VEHICLE NUM " << veh_num << ", must be in range 0-" << (vehicles.size() - 1) << std::endl;
		return false;
	}
	else if (vehicles[veh_num] == NULL) {
		std::cerr << "ERROR: REQUESTED VEHICLE NUM " << veh_num << ", but it has been deleted " << std::endl;
		return false;
	}
	else {
		return true;
	}
}


//------ General MAVS functions -------------------------------
EXPORT_CMD std::string GetMavsDataPath() {
	mavs::MavsDataPath dp;
	return dp.GetPath();
}

//----------- Scene functions -----------------------------------
EXPORT_CMD int LoadMavsScene(std::string scene_file) {
	mavs::raytracer::embree::EmbreeTracer* scene = new mavs::raytracer::embree::EmbreeTracer;
	scene->Load(scene_file);
	scenes.push_back(scene);
	mavs::environment::Environment* env = new mavs::environment::Environment;
	env->SetRaytracer(scenes.back());
	environments.push_back(env);
	return (int)scenes.size() - 1;
}

EXPORT_CMD void ClearMavsScene(int scene_num) {
	if (CheckSceneNum(scene_num)) {
		delete scenes[scene_num];
		scenes[scene_num] = NULL;
		delete environments[scene_num];
		environments[scene_num] = NULL;
	}
}

EXPORT_CMD void UpdateMavsEnvironment(int scene_num, float dt) {
	if (CheckSceneNum(scene_num)) {
		environments[scene_num]->AdvanceTime(dt);
	}
}

EXPORT_CMD void SetRainRate(int scene_num, float rain_rate) {
	if (CheckSceneNum(scene_num)) {
		environments[scene_num]->SetRainRate(rain_rate);
	}
}
EXPORT_CMD void SetFog(int scene_num, float fog) {
	if (CheckSceneNum(scene_num)) {
		environments[scene_num]->SetFog(fog);
	}
}
EXPORT_CMD void SetSnowRate(int scene_num, float snow_rate) {
	if (CheckSceneNum(scene_num)) {
		environments[scene_num]->SetSnowRate(snow_rate);
	}
}
EXPORT_CMD void SetTurbidity(int scene_num, float turbid){
	if (CheckSceneNum(scene_num)) {
		environments[scene_num]->SetTurbidity(turbid);
	}
}
EXPORT_CMD void SetHour(int scene_num, float hour) {
	if (CheckSceneNum(scene_num)) {
		mavs::environment::DateTime dt = environments[scene_num]->GetDateTime();
		environments[scene_num]->SetDateTime(dt.year, dt.month, dt.day, (int)floor(hour), dt.minute, dt.second, dt.time_zone);
	}
}
EXPORT_CMD void SetCloudCover(int scene_num, float cloud_cover_frac) {
	if (CheckSceneNum(scene_num)) {
		environments[scene_num]->SetCloudCover(cloud_cover_frac);
	}
}

EXPORT_CMD void SetTerrainProperties(int scene_num, std::string soil_type, float soil_strength){
	if (CheckSceneNum(scene_num)) {
		environments[scene_num]->SetGlobalSurfaceProperties(soil_type, soil_strength);
	}
}

//------------ Lidar functions ------------------------------
EXPORT_CMD int CreateMavsLidar(std::string model) {
	mavs::sensor::lidar::Lidar* lidar;
	float rep_rate = 10.0f;
	if (model == "M8") {
		lidar = new mavs::sensor::lidar::MEight(rep_rate);
	}
	else if (model == "HDL-64E") {
		lidar = new mavs::sensor::lidar::Hdl64ESimple(rep_rate);
	}
	else if (model == "HDL-32E") {
		lidar = new mavs::sensor::lidar::Hdl32E;
	}
	else if (model == "VLP-16") {
		lidar = new mavs::sensor::lidar::Vlp16(rep_rate);
	}
	else if (model == "AnvelApiLidar") {
		lidar = new mavs::sensor::lidar::AnvelApiLidar(rep_rate);
	}
	else if (model == "FourPi") {
		lidar = new mavs::sensor::lidar::FourPiLidar;
	}
	else if (model == "LMS-291") {
		lidar = new mavs::sensor::lidar::Lms291_S05;
	}
	else if (model == "OS1") {
		lidar = new mavs::sensor::lidar::OusterOS1;
	}
	else if (model == "OS1-16") {
		lidar = new mavs::sensor::lidar::OusterOS1_16;
	}
	else if (model == "OS2") {
		lidar = new mavs::sensor::lidar::OusterOS2;
	}
	else if (model == "RS32") {
		lidar = new mavs::sensor::lidar::Rs32;
	}
	else {
		lidar = new mavs::sensor::lidar::Hdl32E;
	}
	lidar->SetPose(glm::vec3(0.0f, 0.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	lidars.push_back(lidar);
	return (int)lidars.size() - 1;
}

EXPORT_CMD void ClearMavsLidar(int lidar_num) {
	if (CheckLidarNum(lidar_num)) {
		delete lidars[lidar_num];
		lidars[lidar_num] = NULL;
	}
}

EXPORT_CMD void UpdateMavsLidar(int lidar_num, int env_num) {
	if (CheckLidarNum(lidar_num) && CheckSceneNum(env_num)) {
		lidars[lidar_num]->Update(environments[env_num], 0.1f);
	}
}

EXPORT_CMD void DisplayMavsLidar(int lidar_num) {
	if (CheckLidarNum(lidar_num)) {
		lidars[lidar_num]->Display();
	}
}

EXPORT_CMD std::vector<float> GetLidarPose(int lidar_num) {
	std::vector<float> pf;
	if (CheckLidarNum(lidar_num)) {
		mavs::Pose pose = lidars[lidar_num]->GetPose();
		pf.push_back(pose.position.x);
		pf.push_back(pose.position.y);
		pf.push_back(pose.position.z);
		pf.push_back(pose.quaternion.w);
		pf.push_back(pose.quaternion.x);
		pf.push_back(pose.quaternion.y);
		pf.push_back(pose.quaternion.z);
	}
	return pf;
}

EXPORT_CMD void SetLidarPose(int lidar_num, float px, float py, float pz, float ow, float ox, float oy, float oz) {
	if (CheckLidarNum(lidar_num)) {
		lidars[lidar_num]->SetPose(glm::vec3(px, py, pz), glm::quat(ow, ox, oy, oz));
	}
}

EXPORT_CMD void SetLidarOffset(int lidar_num, float px, float py, float pz, float ow, float ox, float oy, float oz) {
	if (CheckLidarNum(lidar_num)) {
		lidars[lidar_num]->SetRelativePose(glm::vec3(px, py, pz), glm::quat(ow, ox, oy, oz));
	}
}

EXPORT_CMD std::vector<float> GetLidarPointsXyzi(int lidar_num) {
	std::vector<float> points;
	if (CheckLidarNum(lidar_num)) {
		std::vector<glm::vec4> lp = lidars[lidar_num]->GetPointsXYZI();
		int np = (int)lp.size();
		points.resize(4*lp.size(), 0.0f);
		for (int i = 0; i < np; i++) {
			points[i] = lp[i].x;
			points[i + np] = lp[i].y;
			points[i + 2 * np] = lp[i].z;
			points[i + 3 * np] = lp[i].w;
		}
	}
	return points;
}

EXPORT_CMD std::vector<float> GetLidarPointsX(int lidar_num) {
	std::vector<float> points_x;
	if (CheckLidarNum(lidar_num)) {
		std::vector<glm::vec3> lp = lidars[lidar_num]->GetPoints();
		points_x.resize(lp.size(), 0.0f);
		for (int i = 0; i < (int)lp.size(); i++) {
			points_x[i] = lp[i].x;
		}
	}
	return points_x;
}

EXPORT_CMD std::vector<float> GetLidarPointsY(int lidar_num) {
	std::vector<float> points_y;
	if (CheckLidarNum(lidar_num)) {
		std::vector<glm::vec3> lp = lidars[lidar_num]->GetPoints();
		points_y.resize(lp.size(), 0.0f);
		for (int i = 0; i < (int)lp.size(); i++) {
			points_y[i] = lp[i].y;
		}
	}
	return points_y;
}

EXPORT_CMD std::vector<float> GetLidarPointsZ(int lidar_num) {
	std::vector<float> points_z;
	if (CheckLidarNum(lidar_num)) {
		std::vector<glm::vec3> lp = lidars[lidar_num]->GetPoints();
		points_z.resize(lp.size(), 0.0f);
		for (int i = 0; i < (int)lp.size(); i++) {
			points_z[i] = lp[i].z;
		}
	}
	return points_z;
}

EXPORT_CMD int GetLidarNumPoints(int lidar_num) {
	int num_points = 0;
	if (CheckLidarNum(lidar_num)) {
		num_points = lidars[lidar_num]->GetNumPoints();
	}
	return num_points;
}

//------------ Camera functions ------------------------------
EXPORT_CMD int CreateMavsCamera() {
	mavs::sensor::camera::RgbCamera* camera = new mavs::sensor::camera::RgbCamera;
	camera->SetPose(glm::vec3(0.0f, 0.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	cameras.push_back(camera);
	return (int)cameras.size() - 1;
}

EXPORT_CMD int CreateMavsOrthoCamera() {
	mavs::sensor::camera::OrthoCamera* camera = new mavs::sensor::camera::OrthoCamera;
	camera->SetPose(glm::vec3(0.0f, 0.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	cameras.push_back(camera);
	return (int)cameras.size() - 1;
}

EXPORT_CMD void FreeCamera(int camera_num) {
	if (CheckCameraNum(camera_num)) {
		cameras[camera_num]->FreePose();
	}
}

EXPORT_CMD void InitializeMavsCamera(int camera_num, int nx, int ny, float fa_x, float fa_y, float flen) {
	if (CheckCameraNum(camera_num)) {
		cameras[camera_num]->Initialize(nx, ny, fa_x, fa_y, flen);
	}
}

EXPORT_CMD std::vector<float> GetCameraImage(int camera_num) {
	std::vector<float> image;
	if (CheckCameraNum(camera_num)) {
		float *imbuff = cameras[camera_num]->GetImageBuffer();
		int buff_size = cameras[camera_num]->GetBufferSize();
		image.resize(buff_size, 0.0f);
		for (int i = 0; i < buff_size; i++) {
			image[i] = imbuff[i];
		}
	}
	return image;
}

EXPORT_CMD std::vector<int> GetImageDimensions(int camera_num) {
	std::vector<int> dim;
	dim.resize(2, 0);
	if (CheckCameraNum(camera_num)) {
		dim[0] = cameras[camera_num]->GetWidth();
		dim[1] = cameras[camera_num]->GetHeight();
	}
	return dim;
}
EXPORT_CMD std::vector<float> GetDrivingCommandFromCamera(int camera_num) {
	std::vector<float> dc;
	dc.resize(3, 0.0f);
	if (CheckCameraNum(camera_num)) {
		std::vector<bool> dc_l = cameras[camera_num]->GetKeyCommands();
		if (dc_l[1]) {
			dc[0] = 0.0f;
			dc[1] = 1.0f;
		}
		else if (dc_l[0]) {
			dc[0] = 1.0f;
			dc[1] = 0.0f;
		}
		if (dc_l[2]) {
			dc[2] = 1.0f;
		}
		else if (dc_l[3]) {
			dc[2] = -1.0f;
		}
	}
	return dc;
}

EXPORT_CMD void ClearMavsCamera(int camera_num) {
	if (CheckCameraNum(camera_num)) {
		delete cameras[camera_num];
		cameras[camera_num] = NULL;
	}
}

EXPORT_CMD void UpdateMavsCamera(int camera_num, int env_num) {
	if (CheckCameraNum(camera_num) && CheckSceneNum(env_num)) {
		cameras[camera_num]->Update(environments[env_num], 0.1f);
	}
}

EXPORT_CMD void DisplayMavsCamera(int camera_num) {
	if (CheckCameraNum(camera_num)) {
		cameras[camera_num]->Display();
	}
}

EXPORT_CMD bool IsCameraDisplayOpen(int camera_num) {
	if (CheckCameraNum(camera_num)) {
		return cameras[camera_num]->DisplayOpen();
	}
	else {
		return false;
	}
}

EXPORT_CMD std::vector<float> GetCameraPose(int camera_num) {
	std::vector<float> pf;
	if (CheckCameraNum(camera_num)) {
		mavs::Pose pose = cameras[camera_num]->GetPose();
		pf.push_back(pose.position.x);
		pf.push_back(pose.position.y);
		pf.push_back(pose.position.z);
		pf.push_back(pose.quaternion.w);
		pf.push_back(pose.quaternion.x);
		pf.push_back(pose.quaternion.y);
		pf.push_back(pose.quaternion.z);
	}
	return pf;
}

EXPORT_CMD void SetCameraPose(int camera_num, float px, float py, float pz, float ow, float ox, float oy, float oz) {
	if (CheckCameraNum(camera_num)) {
		cameras[camera_num]->SetPose(glm::vec3(px, py, pz), glm::quat(ow, ox, oy, oz));
	}
}

EXPORT_CMD void SetCameraOffset(int camera_num, float px, float py, float pz, float ow, float ox, float oy, float oz) {
	if (CheckCameraNum(camera_num)) {
		cameras[camera_num]->SetRelativePose(glm::vec3(px, py, pz), glm::quat(ow, ox, oy, oz));
	}
}

// ------------- Vehicle functions -------------------------------//
EXPORT_CMD int LoadMavsVehicle(std::string veh_file) {
	mavs::vehicle::Rp3dVehicle* veh = new mavs::vehicle::Rp3dVehicle;
	veh->Load(veh_file);
	vehicles.push_back(veh);
	return (int)vehicles.size() - 1;
}

EXPORT_CMD void SetMavsVehiclePose(int veh_num, float px, float py, float heading) {
	if (CheckVehicleNum(veh_num)) {
		vehicles[veh_num]->SetPosition(px, py, 0.0f);
		vehicles[veh_num]->SetOrientation(cosf(0.5f*heading), 0.0f, 0.0f, sinf(0.5f*heading));
	}
}

EXPORT_CMD void ClearMavsVehicle(int veh_num) {
	if (CheckVehicleNum(veh_num)) {
		delete vehicles[veh_num];
		vehicles[veh_num] = NULL;
	}
}

EXPORT_CMD void UpdateMavsVehicle(int veh_num, int env_num, float throttle, float steering, float braking, float dt) {
	if (CheckVehicleNum(veh_num) && CheckSceneNum(env_num)) {
		vehicles[veh_num]->Update(environments[env_num], throttle, steering, braking, dt);
	}
}

EXPORT_CMD std::vector<float> GetMavsVehiclePose(int veh_num) {
	std::vector<float> pose;
	pose.resize(13, 0.0f);
	if (CheckVehicleNum(veh_num)) {
		VehicleState state = vehicles[veh_num]->GetState();
		pose[0] = state.pose.position.x;
		pose[1] = state.pose.position.y;
		pose[2] = state.pose.position.z;
		pose[3] = state.pose.quaternion.w;
		pose[4] = state.pose.quaternion.x;
		pose[5] = state.pose.quaternion.y;
		pose[6] = state.pose.quaternion.z;
		pose[7] = state.twist.linear.x;
		pose[8] = state.twist.linear.y;
		pose[9] = state.twist.linear.z;
		pose[10] = state.twist.angular.x;
		pose[11] = state.twist.angular.y;
		pose[12] = state.twist.angular.z;
	}
	return pose;
}

} // namespace mavs
} // namespace matlab
