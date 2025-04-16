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
/**
* \file mavs_c_defs.h
*
* C wrappers around MAVS classes and methods
* Can be used to interface with python, matlab, etc
* through the MAVS dll.
* see: http://www.auctoris.co.uk/2017/04/29/calling-c-classes-from-python-with-ctypes/
*
*/
#ifndef USE_MPI
#include <raytracers/raytracer.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include <simulation/simulation.h>
#include <mavs_core/environment/environment.h>
#include <mavs_core/pose_readers/waypoints.h>
#include <mavs_core/plotting/map_viewer.h>
#include <sensors/sensor.h>
#include <sensors/imu/mems_sensor.h>
#include <mavs_core/terrain_generator/random_scene.h>
#include <mavs_core/plotting/mavs_plotting.h>
#include "vehicles/controllers/pure_pursuit_controller.h"
#include <simulation/ortho_viewer.h>
#include <simulation/rp3d_vehicle_viewer.h>
#include "raytracers/mtl_viewer.h"
#include <mavs_core/terrain_generator/grd_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) || defined(WIN32)
#define EXPORT_CMD __declspec(dllexport)
#else
#define EXPORT_CMD 
#endif

	//--- Embree Raytracer interfaces --------------------------//
#ifdef USE_EMBREE
EXPORT_CMD mavs::raytracer::embree::EmbreeTracer* NewEmbreeScene(void);

EXPORT_CMD void WriteEmbreeSceneStats(mavs::raytracer::embree::EmbreeTracer* rt, char* directory);

EXPORT_CMD void LoadEmbreeScene(mavs::raytracer::embree::EmbreeTracer* rt, char* fname);

EXPORT_CMD void LoadEmbreeSceneWithRandomSeed(mavs::raytracer::embree::EmbreeTracer* rt, char* fname);

EXPORT_CMD void DeleteEmbreeScene(mavs::raytracer::embree::EmbreeTracer* rt);

EXPORT_CMD float GetSurfaceHeight(mavs::raytracer::embree::EmbreeTracer* rt, float x, float y);

EXPORT_CMD float * PutWaypointsOnGround(mavs::raytracer::embree::EmbreeTracer* rt, mavs::Waypoints * wp);

EXPORT_CMD mavs::raytracer::embree::EmbreeTracer* CreateGapScene(float w, float l, float roughmag,
	float mr, char* basename, float plant_density, char* ecofile, char* output_dir, float gap_width, float gap_depth, float gap_angle_radians);

EXPORT_CMD mavs::raytracer::embree::EmbreeTracer* CreateSceneFromRandom(float w, float l, float lm, float hm,
	float mr, float tw, float wb, float track, char* path_type, char * roughness_type, char* basename, float plant_density,
	std::vector<glm::vec4> * potholes, char* ecofile, char* output_dir);

EXPORT_CMD void SetEnvironmentScene(mavs::environment::Environment* env, mavs::raytracer::embree::EmbreeTracer* scene);

// ------ MAVS animations -------------------------------------//
EXPORT_CMD void SetAnimationPositionInScene(mavs::environment::Environment* env, int anim_id, float x, float y, float heading);

EXPORT_CMD mavs::raytracer::Animation* NewMavsAnimation();

EXPORT_CMD void DeleteMavsAnimation(mavs::raytracer::Animation *anim);

EXPORT_CMD void LoadMavsAnimation(mavs::raytracer::Animation *anim, char *path_to_meshes, char *framefile_name);

EXPORT_CMD void LoadAnimationPathFile(mavs::raytracer::Animation *anim, char *path_file);

EXPORT_CMD int AddAnimationToScene(mavs::raytracer::embree::EmbreeTracer* rt, mavs::raytracer::Animation *anim);

EXPORT_CMD void SetMavsAnimationScale(mavs::raytracer::Animation *anim, float scale);

EXPORT_CMD void SetMavsAnimationRotations(mavs::raytracer::Animation *anim, bool y_to_x, bool y_to_z);

EXPORT_CMD void SetMavsAnimationSpeed(mavs::raytracer::Animation *anim, float speed);

EXPORT_CMD void SetMavsAnimationPosition(mavs::raytracer::Animation *anim, float x, float y);

EXPORT_CMD void SetMavsAnimationHeading(mavs::raytracer::Animation *anim, float heading);

EXPORT_CMD void ViewRp3dDebug(char* input_file_name);
#endif //use EMBREE

EXPORT_CMD mavs::terraingen::GridSurface *LoadDem(char* input_file_name, bool interp_no_data, bool recenter);

EXPORT_CMD void DownsampleDem(mavs::terraingen::GridSurface *surface, int dsfac);

EXPORT_CMD void DisplayDem(mavs::terraingen::GridSurface *surface);

EXPORT_CMD void ExportDemToObj(mavs::terraingen::GridSurface *surface, char* obj_name);

EXPORT_CMD void ExportDemToEsriAscii(mavs::terraingen::GridSurface *surface, char* ascii_name);

EXPORT_CMD std::vector<glm::vec2> * NewPointList2D();

EXPORT_CMD void DeletePointList2D(std::vector<glm::vec2> * list);

EXPORT_CMD void AddPointToList2D(std::vector<glm::vec2> * list, float x, float y);

EXPORT_CMD void DeletePointList4D(std::vector<glm::vec4> * list);

EXPORT_CMD void AddPointToList4D(std::vector<glm::vec4> * list, float w, float x, float y, float z);

EXPORT_CMD void TurnOnMavsSceneLabeling(mavs::raytracer::Raytracer* rt);
EXPORT_CMD void TurnOffMavsSceneLabeling(mavs::raytracer::Raytracer* rt);

//--- MAVS Plotting utility ---------------------------------------------//
EXPORT_CMD mavs::utils::Mplot* NewMavsPlotter(void);

EXPORT_CMD void DeleteMavsPlotter(mavs::utils::Mplot* plot);

EXPORT_CMD void PlotColorMatrix(mavs::utils::Mplot* plot, int width, int height, float *data);

EXPORT_CMD void PlotGrayMatrix(mavs::utils::Mplot* plot, int width, int height, float *data);

EXPORT_CMD void PlotTrajectory(mavs::utils::Mplot* plot, int np, float *x, float *y);

EXPORT_CMD void AddPlotToTrajectory(mavs::utils::Mplot* plot, int np, float *x, float *y);

// mavs map utilities
EXPORT_CMD mavs::utils::MapViewer* NewMavsMapViewer(float llx, float lly, float urx, float ury, float res);

EXPORT_CMD void DeleteMavsMapViewer(mavs::utils::MapViewer* map);

EXPORT_CMD void UpdateMap(mavs::utils::MapViewer* map, mavs::environment::Environment* env);

EXPORT_CMD void AddWaypointsToMap(mavs::utils::MapViewer* map, float* x, float* y, int nwp);

EXPORT_CMD void AddCircleToMap(mavs::utils::MapViewer* map, float cx, float cy , float radius);

EXPORT_CMD void AddLineToMap(mavs::utils::MapViewer* map, float p0x, float p0y, float p1x, float p1y);

EXPORT_CMD bool MapIsOpen(mavs::utils::MapViewer* map);

//--- Environment methods -----------------------------------------------//
EXPORT_CMD mavs::environment::Environment* NewMavsEnvironment(void);

EXPORT_CMD void DeleteMavsEnvironment(mavs::environment::Environment* env);

EXPORT_CMD void AdvanceEnvironmentTime(mavs::environment::Environment* env, float dt);

EXPORT_CMD float * GetSceneDensity(mavs::environment::Environment* env, float llx, float lly, float llz, float urx, float ury, float urz, float res);

EXPORT_CMD float * GetAnimationPosition(mavs::environment::Environment* env, int anim_num);

EXPORT_CMD void FreeEnvironmentScene(mavs::environment::Environment* env);

EXPORT_CMD int AddActorToEnvironment(mavs::environment::Environment* env, char* fname, bool auto_update);

EXPORT_CMD void SetActorPosition(mavs::environment::Environment* env, int actor_id,
	float position[3], float orientation[4]);

EXPORT_CMD void AddDustToActor(mavs::environment::Environment* env, int actor_id);

EXPORT_CMD void AddDustToActorColor(mavs::environment::Environment* env, int actor_id, float cr, float cg, float cb);

EXPORT_CMD void AddDustToEnvironment(mavs::environment::Environment* env, float px, float py, float pz, float vx, float vy, float vz, float dust_rate, float dustball_size, float velocity_randomization);

EXPORT_CMD void UpdateParticleSystems(mavs::environment::Environment* env, float dt);

EXPORT_CMD void SetRainRate(mavs::environment::Environment* env, float rain_rate);

EXPORT_CMD void SetSnowRate(mavs::environment::Environment* env, float snow_rate);

/// from 0-1
EXPORT_CMD void SetSnowAccumulation(mavs::environment::Environment* env, float snow_accum);

EXPORT_CMD void SetTurbidity(mavs::environment::Environment* env, float turbidity);

EXPORT_CMD void SetAlbedo(mavs::environment::Environment* env, float a_r, float a_g, float a_b);

EXPORT_CMD void SetWind(mavs::environment::Environment* env, float w_x, float w_y);

EXPORT_CMD void SetFog(mavs::environment::Environment* env, float fogginess);

EXPORT_CMD void SetCloudCover(mavs::environment::Environment* env, float cloud_cover);

EXPORT_CMD void SetTerrainProperties(mavs::environment::Environment* env, char *soil_type, float soil_strength);

EXPORT_CMD void SetTime(mavs::environment::Environment* env, int hour);

EXPORT_CMD void TurnSkyOnOff(mavs::environment::Environment* env, bool sky_on);

EXPORT_CMD void SetSkyColor(mavs::environment::Environment* env, float r, float g, float b);

EXPORT_CMD void SetSunColor(mavs::environment::Environment* env, float r, float g, float b);

EXPORT_CMD void SetSunLocation(mavs::environment::Environment* env, float azimuth_degrees, float zenith_degrees);

EXPORT_CMD void SetSunSolidAngle(mavs::environment::Environment* env, float solid_angle_degrees);

EXPORT_CMD void SetTimeSeconds(mavs::environment::Environment* env, int hour, int minute, int seconds);

EXPORT_CMD void SetDate(mavs::environment::Environment* env, int year, int month, int day);

EXPORT_CMD int GetNumberOfObjectsInEnvironment(mavs::environment::Environment* env);

EXPORT_CMD char * GetObjectName(mavs::environment::Environment* env, int object_id);

EXPORT_CMD float * GetObjectBoundingBox(mavs::environment::Environment* env, int object_id);

EXPORT_CMD void SetLocalOrigin(mavs::environment::Environment* env, double lat, double lon, double alt);

//--- Vehicle model constructors ------------//
EXPORT_CMD mavs::vehicle::Vehicle* NewMavsRp3dVehicle();

EXPORT_CMD void LoadMavsRp3dVehicle(mavs::vehicle::Vehicle* car, char* fname);

EXPORT_CMD void SetMavsRp3dVehicleReloadVis(mavs::vehicle::Vehicle* car, bool reload);

EXPORT_CMD float * GetMavsVehicleTirePositionAndOrientation(mavs::vehicle::Vehicle* veh, int tire_num);

EXPORT_CMD void SetRp3dExternalForce(mavs::vehicle::Vehicle* veh, float fx, float fy, float fz);

EXPORT_CMD float * GetRp3dLookTo(mavs::vehicle::Vehicle* veh);

EXPORT_CMD float GetRp3dVehicleTireDeflection(mavs::vehicle::Vehicle* veh, int i);

EXPORT_CMD void SetRp3dTerrain(mavs::vehicle::Vehicle* veh, char *soil_type, float soil_strength, char *terrain_type, float terrain_param1, float terrain_param2);

EXPORT_CMD void SetRp3dGravity(mavs::vehicle::Vehicle* veh, float gx, float gy, float gz);

EXPORT_CMD float GetRp3dTireNormalForce(mavs::vehicle::Vehicle* veh, int tire_id);

EXPORT_CMD float * GetRp3dTireForces(mavs::vehicle::Vehicle* veh, int tire_id);

EXPORT_CMD float GetRp3dTireSlip(mavs::vehicle::Vehicle* veh, int tire_id);

EXPORT_CMD float GetRp3dTireAngularVelocity(mavs::vehicle::Vehicle* veh, int tire_id);

EXPORT_CMD float GetRp3dTireSteeringAngle(mavs::vehicle::Vehicle* veh, int tire_id);

EXPORT_CMD float GetRp3dLatAccel(mavs::vehicle::Vehicle* veh);

EXPORT_CMD float GetRp3dLonAccel(mavs::vehicle::Vehicle* veh);

EXPORT_CMD void SetRp3dUseDrag(mavs::vehicle::Vehicle* veh, bool use_drag);

EXPORT_CMD float * GetMavsVehicleFullState(mavs::vehicle::Vehicle* veh);

//#ifdef USE_CHRONO
EXPORT_CMD mavs::vehicle::Vehicle* NewChronoVehicle();

EXPORT_CMD void LoadChronoVehicle(mavs::vehicle::Vehicle* car, char* fname);

EXPORT_CMD float GetChronoTireNormalForce(mavs::vehicle::Vehicle* veh, int tire_id);
//#endif

EXPORT_CMD void DeleteMavsVehicle(mavs::vehicle::Vehicle* veh);

EXPORT_CMD void UpdateMavsVehicle(mavs::vehicle::Vehicle* veh, mavs::environment::Environment* env, float throttle, float steering, float brake, float dt);

EXPORT_CMD float * GetMavsVehiclePosition(mavs::vehicle::Vehicle* veh);

EXPORT_CMD float * GetMavsVehicleVelocity(mavs::vehicle::Vehicle* veh);

EXPORT_CMD float GetMavsVehicleHeading(mavs::vehicle::Vehicle* veh);

EXPORT_CMD float GetMavsVehicleSpeed(mavs::vehicle::Vehicle* veh);

EXPORT_CMD float * GetMavsVehicleOrientation(mavs::vehicle::Vehicle* veh);

EXPORT_CMD void SetMavsVehiclePosition(mavs::vehicle::Vehicle* veh, float x, float y, float z);

EXPORT_CMD void SetMavsVehicleHeading(mavs::vehicle::Vehicle* veh, float theta);

EXPORT_CMD void MoveHeadlights(mavs::environment::Environment* env, mavs::vehicle::Vehicle* veh, float front_offset, float headlight_width, int left_id, int right_id);

EXPORT_CMD int AddPointLight(mavs::environment::Environment* env, float cr, float cg, float cb, float px, float py, float pz);

EXPORT_CMD int AddSpotLight(mavs::environment::Environment* env, float cr, float cg, float cb, float px, float py, float pz, float dx, float dy, float dz, float angle);

EXPORT_CMD void MoveLight(mavs::environment::Environment* env, int id, float px, float py, float pz, float dx, float dy, float dz);

EXPORT_CMD int * AddHeadlightsToVehicle(mavs::environment::Environment* env, mavs::vehicle::Vehicle* veh, float front_offset, float headlight_width);

EXPORT_CMD void SetMavsPathTracerCameraNormalization(mavs::sensor::Sensor* sens, char* norm_type);

EXPORT_CMD void SetMavsPathTracerFixPixels(mavs::sensor::Sensor* sens, bool fix);

//--------------- Vehicle controller funcions ---------------------------//
EXPORT_CMD mavs::vehicle::PurePursuitController* NewVehicleController();

EXPORT_CMD void DeleteVehicleController(mavs::vehicle::PurePursuitController* controller);

EXPORT_CMD void SetLooping(mavs::vehicle::PurePursuitController* controller);

EXPORT_CMD void SetControllerDesiredSpeed(mavs::vehicle::PurePursuitController* controller, float speed);

EXPORT_CMD void SetControllerSpeedParams(mavs::vehicle::PurePursuitController* controller, float kp, float ki, float kd);

EXPORT_CMD void SetControllerWheelbase(mavs::vehicle::PurePursuitController* controller, float wb);

EXPORT_CMD void SetControllerMaxSteeringAngle(mavs::vehicle::PurePursuitController* controller, float max_angle);

EXPORT_CMD void SetControllerMinLookAhead(mavs::vehicle::PurePursuitController* controller, float min_la);

EXPORT_CMD void SetControllerMaxLookAhead(mavs::vehicle::PurePursuitController* controller, float max_la);

EXPORT_CMD void SetControllerSteeringScale(mavs::vehicle::PurePursuitController* controller, float k);

EXPORT_CMD float * GetControllerDrivingCommand(mavs::vehicle::PurePursuitController* controller, float dt);

EXPORT_CMD void SetControllerDesiredPath(mavs::vehicle::PurePursuitController* controller, float *x, float *y, int np);

EXPORT_CMD void SetControllerVehicleState(mavs::vehicle::PurePursuitController* controller, float px, float py, float speed, float heading);

//--- Sensor base class functions ----------------------//
EXPORT_CMD void SetMavsSensorPose(mavs::sensor::Sensor* sens, float position[3], float orientation[4]);

EXPORT_CMD void SetMavsSensorRelativePose(mavs::sensor::Sensor* sens, float position[3], float orientation[4]);

EXPORT_CMD float * GetSensorPose(mavs::sensor::Sensor* sens);

EXPORT_CMD void DeleteMavsSensor(mavs::sensor::Sensor* sens);

EXPORT_CMD void SaveMavsSensorAnnotation(mavs::sensor::Sensor* sens, mavs::environment::Environment* env, char* ofname);

EXPORT_CMD void AnnotateMavsSensorFrame(mavs::sensor::Sensor* sens, mavs::environment::Environment* env);

EXPORT_CMD void SaveMavsCameraAnnotationFull(mavs::sensor::Sensor* sens, mavs::environment::Environment* env, char* ofname);

EXPORT_CMD void UpdateMavsSensor(mavs::sensor::Sensor* sens, mavs::environment::Environment* env, float dt);

EXPORT_CMD void DisplayMavsSensor(mavs::sensor::Sensor* sens);

EXPORT_CMD void SetPointCloudColorType(mavs::sensor::Sensor* sens, char* type);

EXPORT_CMD void DisplayMavsLidarPerspective(mavs::sensor::Sensor* sens, int im_width, int im_height);

EXPORT_CMD void SaveMavsSensorRaw(mavs::sensor::Sensor* sens);

EXPORT_CMD void SaveMavsLidarImage(mavs::sensor::Sensor* sens, char* fname);

EXPORT_CMD void SaveProjectedMavsLidarImage(mavs::sensor::Sensor* sens, char* fname);

EXPORT_CMD void SaveMavsCameraImage(mavs::sensor::Sensor* sens, char* fname);

EXPORT_CMD void SetMavsCameraShadows(mavs::sensor::Sensor* sens, bool shadows);

EXPORT_CMD void SetMavsCameraBlur(mavs::sensor::Sensor* sens, bool blur);

EXPORT_CMD void SetMavsCameraTargetBrightness(mavs::sensor::Sensor* sens, float target_brightness);

EXPORT_CMD void SetMavsCameraAntiAliasingFactor(mavs::sensor::Sensor* sens, int fac);

EXPORT_CMD void SetMavsCameraElectronics(mavs::sensor::Sensor* sens, float gamma, float gain);

EXPORT_CMD void SetMavsCameraLensDrops(mavs::sensor::Sensor* sens, bool onlens);

EXPORT_CMD void SetMavsCameraTempAndSaturation(mavs::sensor::Sensor* sens, float temp, float sat);

EXPORT_CMD void ConvertToRccb(mavs::sensor::Sensor* sens);

EXPORT_CMD float * GetCameraBuffer(mavs::sensor::Sensor* sens);

EXPORT_CMD int GetCameraBufferSize(mavs::sensor::Sensor* sens);

EXPORT_CMD int GetCameraBufferWidth(mavs::sensor::Sensor* sens);

EXPORT_CMD int GetCameraBufferHeight(mavs::sensor::Sensor* sens);

EXPORT_CMD int GetCameraBufferDepth(mavs::sensor::Sensor* sens);

EXPORT_CMD void FreeCamera(mavs::sensor::Sensor* sens);

EXPORT_CMD float * GetDrivingCommandFromCamera(mavs::sensor::Sensor* sens);

EXPORT_CMD void AddRainToExistingImage(char* image_name, float rain_rate, bool drops_on_lens);

EXPORT_CMD void AddRainToExistingImageRho(char* image_name, float rain_rate, float rho, bool drops_on_lens);

EXPORT_CMD mavs::sensor::Sensor* NewMavsRgbCameraDimensions(int num_hor,
	int num_vert, float focal_array_width, float focal_array_height,
	float focal_length);

EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCamera(char* type, int num_rays, int max_depth, float rr);

EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCameraExplicit(int nx, int ny, float h_size, float v_size, float flen, float gamma, int num_rays, int max_depth, float rr);

/*EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCameraLowRes(int num_rays, int max_depth, float rr);

EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCameraHighRes(int num_rays, int max_depth, float rr);

EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCameraHalfHighRes(int num_rays, int max_depth, float rr);*/

//--- Oak-D Stereo Camera sensor functions -----------------------------------------------//
EXPORT_CMD mavs::sensor::Sensor* NewMavsOakDCamera();

EXPORT_CMD float* GetOakDDepthBuffer(mavs::sensor::Sensor* sens);

EXPORT_CMD float* GetOakDImageBuffer(mavs::sensor::Sensor* sens);

EXPORT_CMD int GetOakDDepthBufferSize(mavs::sensor::Sensor* sens);

EXPORT_CMD int GetOakDImageBufferSize(mavs::sensor::Sensor* sens);

EXPORT_CMD mavs::sensor::Sensor* GetOakDCamera(mavs::sensor::Sensor* sens);

EXPORT_CMD float GetOakDMaxRangeCm(mavs::sensor::Sensor* sens);

EXPORT_CMD void SetOakDMaxRangeCm(mavs::sensor::Sensor* sens, float max_range_cm);

EXPORT_CMD void SetOakDCameraDisplayType(mavs::sensor::Sensor* sens, char* display_type);

//--- Zed2i Stereo Camera sensor functions -----------------------------------------------//
EXPORT_CMD mavs::sensor::Sensor* NewMavsZed2iCamera();

EXPORT_CMD float* GetZed2iDepthBuffer(mavs::sensor::Sensor* sens);

EXPORT_CMD float* GetZed2iImageBuffer(mavs::sensor::Sensor* sens);

EXPORT_CMD int GetZed2iDepthBufferSize(mavs::sensor::Sensor* sens);

EXPORT_CMD int GetZed2iImageBufferSize(mavs::sensor::Sensor* sens);

EXPORT_CMD mavs::sensor::Sensor* GetZed2iCamera(mavs::sensor::Sensor* sens);

EXPORT_CMD float GetZed2iMaxRangeCm(mavs::sensor::Sensor* sens);

EXPORT_CMD void SetZed2iMaxRangeCm(mavs::sensor::Sensor* sens, float max_range_cm);

EXPORT_CMD void SetZed2iCameraDisplayType(mavs::sensor::Sensor* sens, char* display_type);

EXPORT_CMD bool CameraDisplayOpen(mavs::sensor::Sensor* sens);

//--- Constructors for specific sensors ------------------------------//
EXPORT_CMD mavs::sensor::Sensor* NewMavsRgbCamera();

EXPORT_CMD mavs::sensor::Sensor* NewMavsLwirCamera(int nx, int ny, float dx, float dy, float flen);

EXPORT_CMD void LoadLwirThermalData(mavs::sensor::Sensor *sens, char* fname);

EXPORT_CMD mavs::sensor::Sensor* NewMavsCameraModel(char* model_name);

//--- RedEdge camera functions ---------------------------------------------///
EXPORT_CMD mavs::sensor::Sensor* NewMavsRedEdge();

EXPORT_CMD void SaveRedEdge(mavs::sensor::Sensor* sensor, char* fname);

EXPORT_CMD void SaveRedEdgeBands(mavs::sensor::Sensor* sensor, char* fname);

EXPORT_CMD void SaveRedEdgeFalseColor(mavs::sensor::Sensor* sensor, int band1, int band2, int band3, char* fname);

EXPORT_CMD void DisplayRedEdge(mavs::sensor::Sensor* sensor);

//--- MEMS functions -----------------------------------------------/////
EXPORT_CMD mavs::sensor::imu::MemsSensor* NewMavsMems(char* memstype);

EXPORT_CMD void SetMemsMeasurmentRange(mavs::sensor::imu::MemsSensor* sens, float range);

EXPORT_CMD void SetMemsResolution(mavs::sensor::imu::MemsSensor* sens, float res);

EXPORT_CMD void SetMemsConstantBias(mavs::sensor::imu::MemsSensor* sens, float bias_x, float bias_y, float bias_z);

EXPORT_CMD void SetMemsNoiseDensity(mavs::sensor::imu::MemsSensor* sens, float nd_x, float nd_y, float nd_z);

EXPORT_CMD void SetMemsBiasInstability(mavs::sensor::imu::MemsSensor* sens, float bi_x, float bi_y, float bi_z);

EXPORT_CMD void SetMemsAxisMisalignment(mavs::sensor::imu::MemsSensor* sens, float misalign_x, float misalign_y, float misalign_z);

EXPORT_CMD void SetMemsRandomWalk(mavs::sensor::imu::MemsSensor* sens, float rw_x, float rw_y, float rw_z);

EXPORT_CMD void SetMemsTemperatureBias(mavs::sensor::imu::MemsSensor* sens, float tb_x, float tb_y, float tb_z);

EXPORT_CMD void SetMemsTemperatureScaleFactor(mavs::sensor::imu::MemsSensor* sens, float tsf_x, float tsf_y, float tsf_z);

EXPORT_CMD void SetMemsAccelerationBias(mavs::sensor::imu::MemsSensor* sens, float ab_x, float ab_y, float ab_z);

EXPORT_CMD float * MemsUpdate(mavs::sensor::imu::MemsSensor* sens, float input_x, float input_y, float input_z, float temperature, float sample_rate);
//------ Done with MEMS functions -------------------------------------------------------------------------------------------------------------
//--- Radar functions -----------------------------------------------////

EXPORT_CMD mavs::sensor::Sensor* NewMavsRadar();

EXPORT_CMD void SetRadarMaxRange(mavs::sensor::Sensor* sens, float max_range);

EXPORT_CMD void SetRadarFieldOfView(mavs::sensor::Sensor* sens, float fov, float res_degrees);

EXPORT_CMD void SetRadarSampleResolution(mavs::sensor::Sensor* sens, float res_degrees);

EXPORT_CMD void SaveMavsRadarImage(mavs::sensor::Sensor* sens, char* fname);

EXPORT_CMD float * GetRadarReturnLocations(mavs::sensor::Sensor* sens);

EXPORT_CMD float * GetRadarTargets(mavs::sensor::Sensor* sens);

EXPORT_CMD int GetRadarNumTargets(mavs::sensor::Sensor* sens);

//--- RTK functions ---------------------------------------------------////
EXPORT_CMD mavs::sensor::Sensor* NewMavsRtk();

EXPORT_CMD void SetRtkError(mavs::sensor::Sensor* sens, float error);

EXPORT_CMD void SetRtkDroputRate(mavs::sensor::Sensor* sens, float droput_rate);

EXPORT_CMD void SetRtkWarmupTime(mavs::sensor::Sensor* sens, float warmup_time);

EXPORT_CMD float * GetRtkPosition(mavs::sensor::Sensor* sens);

EXPORT_CMD float * GetRtkOrientation(mavs::sensor::Sensor* sens);

EXPORT_CMD void WriteMavsLidarToColorizedCloud(mavs::sensor::Sensor* sens, char* fname);

EXPORT_CMD float GetChamferDistance(int npc1, float* pc1x, float* pc1y, float* pc1z, int npc2, float* pc2x, float* pc2y, float* pc2z);

EXPORT_CMD void SetMavsSensorVelocity(mavs::sensor::Sensor* sens, float vx, float vy, float vz);

EXPORT_CMD void WriteMavsLidarToLabeledCloud(mavs::sensor::Sensor* sens, char* fname);

EXPORT_CMD void WriteMavsLidarToLabeledPcdWithNormals(mavs::sensor::Sensor* sens, char* fname);

EXPORT_CMD void WriteMavsLidarToLabeledPcd(mavs::sensor::Sensor* sens, char* fname);

EXPORT_CMD void WriteMavsLidarToPcd(mavs::sensor::Sensor* sens, char* fname);

EXPORT_CMD int GetMavsLidarNumberPoints(mavs::sensor::Sensor* sens);

EXPORT_CMD float * GetMavsLidarRegisteredPoints(mavs::sensor::Sensor* sens);

EXPORT_CMD float * GetMavsLidarUnRegisteredPointsXYZI(mavs::sensor::Sensor* sens);

EXPORT_CMD float * GetMavsLidarUnRegisteredPointsXYZIL(mavs::sensor::Sensor* sens);

EXPORT_CMD void AddPointsToImage(mavs::sensor::Sensor* lidar, mavs::sensor::Sensor* camera);

EXPORT_CMD void AnalyzeCloud(mavs::sensor::Sensor* sens, char* fname, int frame_num, bool display);

EXPORT_CMD mavs::sensor::Sensor* NewMavsLidar(char* senstype);

EXPORT_CMD mavs::sensor::Sensor* MavsLidarSetScanPattern(float horiz_fov_low, float horiz_fov_high, float horiz_resolution,
	float vert_fov_low, float vert_fov_high, float vert_resolution);

EXPORT_CMD mavs::Waypoints * LoadAnvelReplayFile(char* fname);

EXPORT_CMD mavs::Waypoints * LoadWaypointsFromJson(char* fname);

EXPORT_CMD void DeleteMavsWaypoints(mavs::Waypoints * wp);

EXPORT_CMD void SaveWaypointsAsJson(mavs::Waypoints *wp, char* fname);

EXPORT_CMD int GetNumWaypoints(mavs::Waypoints * wp);

EXPORT_CMD float * GetWaypoint(mavs::Waypoints * wp, int wpnum);

//-----  Commands for creating a top down viewer --------
EXPORT_CMD mavs::OrthoViewer * CreateOrthoViewer();

EXPORT_CMD void DeleteOrthoViewer(mavs::OrthoViewer *viewer);

EXPORT_CMD void UpdateOrthoViewerWaypoints(mavs::OrthoViewer *viewer, mavs::environment::Environment* env, mavs::Waypoints *wp);

EXPORT_CMD void UpdateOrthoViewer(mavs::OrthoViewer *viewer, mavs::environment::Environment* env);

EXPORT_CMD void DisplayOrtho(mavs::OrthoViewer *viewer);

EXPORT_CMD void SaveOrtho(mavs::OrthoViewer *viewer, char *fname);

EXPORT_CMD float * GetOrthoBuffer(mavs::OrthoViewer *viewer);

EXPORT_CMD int GetOrthoBufferSize(mavs::OrthoViewer *viewer);

// -------------- Rp3d Vehicle Viewer ---------------------------------------------------//
EXPORT_CMD mavs::Rp3dVehicleViewer * CreateRp3dViewer();

EXPORT_CMD void DeleteRp3dViewer(mavs::Rp3dVehicleViewer *viewer);

EXPORT_CMD void Rp3dViewerLoadVehicle(mavs::Rp3dVehicleViewer *viewer, char *vehfile);

EXPORT_CMD void Rp3dViewerDisplay(mavs::Rp3dVehicleViewer *viewer, bool show_debug);

EXPORT_CMD void Rp3dViewerUpdate(mavs::Rp3dVehicleViewer *viewer, bool show_debug);

EXPORT_CMD float * Rp3dViewerGetSideImage(mavs::Rp3dVehicleViewer *viewer);

EXPORT_CMD float * Rp3dViewerGetFrontImage(mavs::Rp3dVehicleViewer *viewer);

EXPORT_CMD int Rp3dViewerGetSideImageSize(mavs::Rp3dVehicleViewer *viewer);

EXPORT_CMD int Rp3dViewerGetFrontImageSize(mavs::Rp3dVehicleViewer *viewer);

EXPORT_CMD void Rp3dViewerSaveSideImage(mavs::Rp3dVehicleViewer *viewer, char *fname);

EXPORT_CMD void Rp3dViewerSaveFrontImage(mavs::Rp3dVehicleViewer *viewer, char *fname);

// ------ Commands for the material viewer -------------------
EXPORT_CMD mavs::raytracer::MtlViewer * CreateMaterialViewer();

EXPORT_CMD void DeleteMaterialViewer(mavs::raytracer::MtlViewer *viewer);

EXPORT_CMD void UpdateMaterialViewer(mavs::raytracer::MtlViewer *viewer);

EXPORT_CMD void ResetMaterialViewer(mavs::raytracer::MtlViewer *viewer, float kd_r, float kd_g, float kd_b);

EXPORT_CMD void LoadMaterialViewerMesh(mavs::raytracer::MtlViewer *viewer, char* meshfile);

EXPORT_CMD int GetMaterialViewerNumMats(mavs::raytracer::MtlViewer *viewer);

EXPORT_CMD char * GetMaterialViewerMatName(mavs::raytracer::MtlViewer *viewer, int id);

EXPORT_CMD char * GetMaterialViewerSpectrumName(mavs::raytracer::MtlViewer *viewer, int id);

EXPORT_CMD float * GetMaterialViewerMaterial(mavs::raytracer::MtlViewer *viewer, int id);

#ifdef __cplusplus
}
#endif


#endif
