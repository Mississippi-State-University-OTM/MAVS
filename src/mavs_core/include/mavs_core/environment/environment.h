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
 * \class Environment
 *
 * The environment class specifies things that all simulation entities need
 * to know like the coordinate system, time of day, etc.
 *
 * \author Chris Goodin
 *
 * \date 1/4/2018
 */
#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <raytracers/raytracer.h>
#include <raytracers/bounding_box.h>
#include <mavs_core/environment/solar_position.h>
#include <mavs_core/environment/clouds.h>
#include <ArHosekSkyModel.h>
#include <FastNoise.h>
#include <mavs_core/environment/particle_system/particle_system.h>
#include <mavs_core/environment/actors/actor.h>
#include <mavs_core/environment/snow/snow.h>
#include <mavs_core/environment/fog/fog.h>
#include <mavs_core/coordinate_systems/coord_conversions.h>

namespace mavs{
namespace environment{

struct SkyState {
	ArHosekSkyModelState* state[3];
};


/** 
 * Artificial light source (not sun). A light may be one of 2 types
 * 1 = point source
 * 2 = spotlight/headlight
 */
struct Light{
  Light(){
    type = 1;
    cutoff_distance = 22.63f;
    color.x = 255.0f;
    color.y = 255.0f;
    color.z = 255.0f;
    direction.x = 0.0f;
    direction.y = 0.0f;
    direction.z = -1.0f;
    decay = 2.0f;
    angle = 0.2f;
		radius = 0.1f;
  }
  glm::vec3 color;
  glm::vec3 position;
  glm::vec3 direction;
  unsigned short int type;
  float cutoff_distance;
  float decay;
  float angle;
	float radius;
	bool is_active;
};
 
class Environment {
public:
	/// Constructor for Environment object
	Environment();

	/// Destructor for Environment object
	~Environment();

	/**
	 * Load an environment input file.
	 * \param input_file Full path to the json input file
	 */
	void Load(std::string input_file);

	/**
	 * Set the origin of the local coordinate system in lat-long-altitude.
	 * \param lat Latitude in decimal degrees
	 * \param lon Longitude in decimal degrees (Westing = negative)
	 * \param alt Altitude in meters
	 */
	void SetLocalOrigin(double lat, double lon, double alt);

	/**
	 * Returns the origin of the local coordinate system in lat-long-altitude.
	 */
	coordinate_system::LLA GetLocalOrigin() { return local_origin_; }

	/**
	 * Returns the rgb value of the sky from 0 to 255
	 * \param direction The normalized viewing direction, in local ENU coords.
	 * \param solid_angle The solid angle along the view direction
	 */
	glm::vec3 GetSkyRgb(glm::vec3 direction, float solid_angle);

	/**
	* Returns the rgb value of the sky averaged over the hemisphere
	*/
	glm::vec3 GetAvgSkyRgb();

	/// Return the RGB from the sun
	glm::vec3 GetSunColor();

	/// Returns the local albedo of the scene from 0 to 1 in RGB.
	glm::vec3 GetAlbedoRgb();

	/**
	 * Sets the local albedo for the RGB bands. Albedo ranges [0,1]
	 * \param red_alb Local albedo in the red band.
	 * \param green_alb Local albedo in the green band.
	 * \param blue_alb Local albedo in the blue band
	 */
	void SetAlbedoRgb(double red_alb, double green_alb, double blue_alb);

	/**
	 * Set the atmospheric turbidity, which is a measure of aerosal content.
	 * Turbidity = 2 is exceptionally clear.
	 * Turbidity = 10 is exceptionally smoggy / hazy.
	 * \param turbidity Turbidity of the atmosphere, [2,10].
	 */
	void SetTurbidity(double turbidity);

	/**
	 * Set the date and time of the experiment.
	 * \param year 4 digit year
	 * \param month Month from 1 to 12
	 * \param day Day from 1 to 31
	 * \param hour Hour from 0 to 23 in local time
	 * \param minute Minute from 0 to 59
	 * \param second Second from 0 to 59
	 * \param time_zone Local time zone, no daylight savings (CST=6)
	 */
	void SetDateTime(int year, int month, int day, int hour, int minute,
		int second, int time_zone);

	/**
	 * Set the date and time of the experiment.
	 * \param date_time MAVS DateTime structure
	 */
	void SetDateTime(DateTime date_time);

	/// Return the current date/time structure
	DateTime GetDateTime() {
		return date_time_;
	}

	/// Returns normalized vector pointing to the sun in local ENU coordinates.
	glm::vec3 GetSolarDirection();

	/**
	 * Sets the ray tracer to be used in the simulation, which must conform to
	 * the interface defined in raytracer.h
	 * \param tracer Pointer to the raytracer to be used.
	 */
	void SetRaytracer(raytracer::Raytracer *tracer);

	void FreeRaytracer() {
		delete scene_;
	}

	/**
	* Returns pointer to the ray tracer to be used in the simulation
	*/
	raytracer::Raytracer * GetScene() {
		return scene_;
	}

	/**
	 * Returns the closest intersection in the scene.
	 * \param origin Point in ENU of the origin of the ray.
	 * \param direction Normalized direction of the ray in ENU.
	 */
	raytracer::Intersection GetClosestIntersection(glm::vec3 origin, glm::vec3 direction);

	/**
	* Returns the closest intersection to the scene surface.
	* \param origin Point in ENU of the origin of the ray.
	* \param direction Normalized direction of the ray in ENU.
	*/
	raytracer::Intersection GetClosestTerrainIntersection(glm::vec3 origin, glm::vec3 direction);

	/**
	 * Returns the closest intersection in the scene.
	 * The last parameter is the return value
	 * \param origin Point in ENU of the origin of the ray.
	 * \param direction Normalized direction of the ray in ENU.
	 * \param inter The returned intersection
	 */
	void GetClosestIntersection(glm::vec3 origin, glm::vec3 direction, raytracer::Intersection &inter);

	/**
	 * Returns true if the ray intersects any object in the scene.
	 * \param origin Point in ENU of the origin of the ray.
	 * \param direction Normalized direction of the ray in ENU.
	 */
	bool GetAnyIntersection(glm::vec3 origin, glm::vec3 direction) {
		return scene_->GetAnyIntersection(origin, direction);
	}

	/**
	 * Add a spotlight to the scene. Returns the ID of the added light
	 * \param color The RGB color of the spotlight (0-255)
	 * \param position The position of the spotlight in word ENU coordinates.
	 * \param direction The direction of the spotlight in world coordinates.
	 * \param spot_angle The opening angle of the spotlight in degrees.
	 */
	int AddSpotlight(glm::vec3 color, glm::vec3 position, glm::vec3 direction,
		float spot_angle);

	/**
	 * Add a point light source to the scene. Returns the ID of the light.
	 * \param color The RGB color of the spotlight (0-255)
	 * \param position The position of the spotlight in word ENU coordinates.
	 */
	int AddPointlight(glm::vec3 color, glm::vec3 position);

	/**
	* Add a light to the scene. Returns the ID of the light.
	* \param light The Light to add
	*/
	int AddLight(Light light);

	/**
	* Remove a vector list of lights by integer ID
	* \param ids The ID #'s of the lights to remove
	*/
	void RemoveLights(std::vector<int> ids);

	/**
	* Move light with ID to new orientation
	* \param light_id The ID number of the light to move
	* \param position The new position of the light in word ENU coordinates.
	* \param direction The direction of the light in world coordinates.
	*/
	void MoveLight(int light_id, glm::vec3 position, glm::vec3 direction);

	/// Return the current number of lights, not including the sun
	size_t GetNumLights() { return lights_.size(); }

	/// Return the i^th light
	Light GetLight(int i) { return lights_[i]; }

	void SetLightActive(int light_id, bool active){
		if (light_id >= 0 && light_id < lights_.size()) {
			lights_[light_id].is_active = active;
		}
	}

	Light *GetPointerToLight(int light_id) {
		if (light_id >= 0 && light_id < lights_.size()) {
			return &lights_[light_id];
		}
		else {
			return NULL;
		}
	}

	/// Return a pointer to the particle systems
	ParticleSystem* GetParticleSystems() {
		if (particle_systems_.size() > 0) {
			return &particle_systems_[0];
		}
		else {
			return NULL;
		}
	}

	/// Tell the number of particle systems in the environment
	size_t GetNumParticleSystems() { return particle_systems_.size(); }

	/**
	 * Add a particle system to the environment
	 * \param psys The particle system to add.
	 */
	void AddParticleSystem(ParticleSystem psys) {
		particle_systems_.push_back(psys);
		int n = (int)particle_systems_.size() - 1;
		particle_system_to_actor_map_[n] = -1;
	}

	void UnsetActorUpdate(int actor_num) {
		if (actor_num >= 0 && actor_num < (int)actors_.size()) {
			actors_[actor_num].SetAutoUpdate(false);
		}
	}

	/// Rerturn the current number of actors
	int GetNumberOfActors() {
		return (int)actors_.size();
	}

	void AssignParticleSystemToActor(int psysnum, int actor_num) {
		particle_system_to_actor_map_[psysnum] = actor_num;
	}

	/**
	* Advance particle systems in time
	* \param dt The time step, in seconds
	*/
	void AdvanceParticleSystems(float dt);

	/**
	* Add a dynamic actors to the environment
	* \param actor_file The json file with actor info.
	*/
	std::vector<int> LoadActors(std::string actor_file);

	/**
	* Add a single actor to the environment based on supplied parameters
	* \param meshfile The mesh file associated with the actor
	* \param y_to_z Rotate the mesh?
	* \param x_to_y Rotate the mesh?
	* \param y_to_x Rotate the mesh?
	* \param offset Offset of the mesh w.r.t. actor position
	* \param scale Scale factor fo the actor mesh
	*/
	std::vector<int> AddActor(std::string meshfile, bool y_to_z, bool x_to_y, bool y_to_x, glm::vec3 offset, glm::vec3 scale);

	/**
	* Set the position of an actor with the specified ID
	* \param id ID number of the actor to set
	* \param pos New position of the actor in local ENU coordinates
	* \param orient New orientation of the actor in local ENU coordinates
	*/
	void SetActorPosition(int id, glm::vec3 pos, glm::quat orient);

	/**
	* Set the position of an actor with the specified ID
	* \param id ID number of the actor to set
	* \param pos New position of the actor in local ENU coordinates
	* \param orient New orientation of the actor in local ENU coordinates
	* \param commit_scene Set to false if you don't want to recommit scene on update
	*/
	void SetActorPosition(int id, glm::vec3 pos, glm::quat orient, bool commit_scene);

	/**
	* Set the position of an actor with the specified ID. Inclusion of dt
	* also calculates the velocity
	* \param id ID number of the actor to set
	* \param pos New position of the actor in local ENU coordinates
	* \param orient New orientation of the actor in local ENU coordinates
	* \param dt The time step of the change in position, in seconds
	*/
	void SetActorPosition(int id, glm::vec3 pos, glm::quat orient, float dt);

	/**
	* Set the position of an actor with the specified ID. Inclusion of dt
	* also calculates the velocity
	* \param id ID number of the actor to set
	* \param pos New position of the actor in local ENU coordinates
	* \param orient New orientation of the actor in local ENU coordinates
	* \param dt The time step of the change in position, in seconds
	* \param commit_scene Set to false if you don't want to recommit scene on update
	*/
	void SetActorPosition(int id, glm::vec3 pos, glm::quat orient, float dt, bool commit_scene);

	/**
	* Set the actor velocity in local ENU coordinates
	* \param id ID number of the actor to set
	* \param vel New velocity of the actor in local ENU coordinates
	*/
	void SetActorVelocity(int id, glm::vec3 vel);

  /** 
   * Advance environment time by dt seconds. Will move the particle systems and
   * other animations. Does not move the sun/moon
   * \param dt The time step to advance in seconds
   */
  void AdvanceTime(float dt);

	/**
	* Get the ground height in global coordinates at
	* a given position.
	* \param x X-coordinate in global frame
	* \param y Y-coordinate in global frame
	*/
	float GetGroundHeight(float x, float y);

	/**
	* Get the ground height and normal
	* in global coordinates at a given position.
	* \param x X-coordinate in global frame
	* \param y Y-coordinate in global frame
	*/
	glm::vec4 GetGroundHeightAndNormal(float x, float y);

	/**
	* Tell the position of actor number act_num
	* If act_num is not a valid ID, then it will
	* return (0,0,0).
	* \param act_num The number of the actor for which to get position
	*/
	glm::vec3 GetActorPosition(int act_num);

	/**
	* Tell the position of an animation number anim_num
	* If anim_num is not a valid ID, then it will
	* return (0,0,0).
	* \param anim_num The number of the animation for which to get position.
	*/
	glm::vec3 GetAnimationPosition(int anim_num) { return scene_->GetAnimationPosition(anim_num); }

	/**
	 * Return a pointer to a given animation. Returns a null pointer if the animation id is not valid.
	 * \param anim_num The number of the animation to get.
	 */
	raytracer::Animation* GetAnimation(int anim_num){return scene_->GetAnimation(anim_num); }

	/**
	* Set the position and heading of the animation.
	* \param anim_id The ID number of the animation to set.
	* \param x The x-coordinate in local ENU.
	* \param y The y-coordinate in local ENU.
	* \param heading Heading, in radians relative to +X.
	*/
	void SetAnimationPosition(int anim_id, float x, float y, float heading) {
		scene_->SetAnimationPosition(anim_id, x, y, heading);
	}

	/**
	* Return the name of an object with the given ID
	* \param id ID number of the desired object
	*/
	std::string GetObjectName(int id) {
		return scene_->GetObjectName(id);
	}

	/**
	* Return the orientation of an object with the given ID
	* \param id ID number of the desired object
	*/
	glm::quat GetObjectOrientation(int id) {
		return scene_->GetObjectOrientation(id);
	}

	/**
	* Return bounding box of an object with the given ID
	* \param id ID number of the desired object
	*/
	raytracer::BoundingBox GetObjectBoundingBox(int id);

	/// Return the number of objects in the scene
	int GetNumberObjects() {
		return scene_->GetNumberObjects();
	}

	/**
	* Get the number of a given label
	* \param label_name The name of the label
	*/
	int GetLabelNum(std::string label_name) {
		return scene_->GetLabelNum(label_name);
	}

	/**
	* Get the color associated with the label
	* \param label_name The name of the label
	*/
	glm::vec3 GetLabelColor(std::string label_name) {
		return scene_->GetLabelColor(label_name);
	}

	/**
	* Get the semantic label of a given mesh
	* \param mesh_name The name of the mesh
	*/
	std::string GetLabel(std::string mesh_name) {
		return scene_->GetLabel(mesh_name);
	}

	/// Return true if it's raining
	bool IsRaining() {
		return is_raining_;
	}

	/// Return the rain rate in mm/h
	float GetRainRate() {
		return rain_rate_;
	}

	/**
	* Set the rain rate in mm/h
	* \param rate The rain rate
	*/
	void SetRainRate(float rate);

	/// Return the windspeed and direction as a vector
	glm::vec2 GetWind() {
		return wind_;
	}

	/**
	* Set the horizontal windspeed and direction
	* \param wx The wind speed (m/s) in the x / easting direction
	* \param wy The wind speed (m/s) in the y / northing direction
	*/
	void SetWind(float wx, float wy) {
		wind_ = glm::vec2(wx,wy);
	}

	/**
	* Set the cloud cover as a fraction of the total sky
	* \param cover_fraction The fractional coverage from 0-1;
	*/
	void SetCloudCover(float cover_fraction);

	/**
	* Set the color of the clouds in (r,g,b)
	* \param r The red color [0,255]
	* \param g The green color [0,255]
	* \param b The blue color [0,255]
	*/
	void SetCloudColor(float r, float g, float b) {
		clouds_.SetCloudColor(r, g, b);
	}

	/**
	* Set the environment temperature in Celsius
	* Default is 20 degrees
	* \param temp Temperature in degrees Celsius
	*/
	void SetTemperature(float temp) {
		temperature_ = temp;
	}

	/**
	* Set the snow rate, default is 0
	* Heavy snow = 25 mm/h
	* \param rate Snow rate in mm/h
	*/
	void SetSnowRate(float rate) {
		snow_rate_ = rate;
		if (snow_rate_ > 0.0) {
			is_snowing_ = true;
			snow_.Initialize(rate, temperature_);
		}
		if (snow_rate_ <= 0.0)is_snowing_ = false;
	}

	/**
	* Set the snow accumlation amount 
	* The number is in relative units from 0 to 1. 
	* 0 means no snow on the ground, 1 means ground is
	* completely covered with snow.
	* \param accum The accumulation parameter
	*/
	void SetSnowAccumulation(float accum) {
		snow_accum_fac_ = std::max(0.0f, std::min(1.0f, accum));
	}

	/// Return the snow accumlation parameter, which ranges from 0 to 1
	float GetSnowAccumulation() { return snow_accum_fac_; }

	/// Returns true if snowing
	bool IsSnowing() { return is_snowing_; }

	/// Returns the snow rate in mm/h
	float GetSnowRate() { return snow_rate_; }

	/// Return a pointer to the snow cloud
	Snow * GetSnow() { return &snow_; }

	/// Returns the temperature in celsius
	float GetTemperature() { return temperature_; }

	/**
	* Set the global fog absorption parameter
	* \param k The fog absorption parameter
	*/
	void SetFog(float k) {
		fog_k_ = k;
		fog_.SetK(k);
	}

	/// Return true if fog is enabled
	bool IsFoggy() { return (fog_k_ > 0.0); }

	/// Get the global fog absorption parameter
	float GetFogK(glm::vec3 p0, glm::vec3 p1) { return fog_.GetK(p0,p1); }
	Fog * GetFog() { return &fog_; }

	/**
	* Set the global surface properties
	* \param surface_type The surface type, can be "dry", "wet", "snow", "ice", "clay", or "sand"
	* \param RCI The cone index in pascals
	*/
	void SetGlobalSurfaceProperties(std::string surface_type, float RCI) {
		surface_type_ = surface_type;
		rci_ = RCI;
	}

	/// Return the global surface type
	std::string GetSurfaceType() { return surface_type_; }

	/// Return the global surface index in Pascals
	float GetSurfaceRCI() { return rci_; }

	/// Turn off the sky model, sky will be black
	void TurnOffSky() { sky_on_ = false; }

	/// Turn (back) on sky model, it is on by default
	void TurnOnSky() { sky_on_ = true; }

	/// Return true if the sky rendering is turned on
	bool IsSkyOn() { return sky_on_; }

	/// Return the vegetation density on a 3D grid, as a filling fraction of the cell
	std::vector< std::vector <std::vector<float> > > GetVegDensityOnGrid(glm::vec3 ll, glm::vec3 ur, float res);

	/**
	* Set the azimuth and zenith angle of the sun relative to your position, in degrees
	* \param azimuth_degrees Degrees clockwise of North
	* \param zenith_degrees Degrees off vertical
	*/
	void SetSunPosition(float azimuth_degrees, float zenith_degrees);

	/**
	* Set the sky to a constant color.
	* This will turn off the earth atmosphere model
	* \param r Red channel, 0.0-255.0
	* \param g Green channel, 0.0-255.0
	* \param b Blue channel, 0.0-255.0
	*/
	void SetSkyConstantColor(float r, float g, float b);

	/**
	* Set the sun to a constant color.
	* This will turn off the earth atmosphere model
	* \param r Red channel, 0.0-255.0
	* \param g Green channel, 0.0-255.0
	* \param b Blue channel, 0.0-255.0
	*/
	void SetSunConstantColor(float r, float g, float b);

	/**
	* Set the solid angle the sun subtends in the sky
	* \param solid_angle_degrees Sun solid angle in degrees
	*/
	void SetSunSolidAngle(float solid_angle_degrees) { sun_solid_angle_ = (float)kDegToRad*solid_angle_degrees; }

 private:
	 // surface properties
	 std::string surface_type_;
	 float rci_;
	 bool sky_on_;
	 glm::vec3 sky_constant_color_;
	 glm::vec3 sun_constant_color_;
	 float sun_solid_angle_;
  DateTime date_time_;

  coordinate_system::CoordinateConverter coord_converter_;
  coordinate_system::LLA local_origin_;
  raytracer::Raytracer *scene_;

  SolarPosition solar_position_;
  glm::vec3 sun_direction_;
	glm::vec3 avg_sky_col_;
  
  std::vector<Light> lights_;

	Clouds clouds_;
	Fog fog_;
	float fog_k_;
	float temperature_;
	float snow_rate_;
	bool is_snowing_;
	Snow snow_;
	float snow_accum_fac_;
	FastNoise snow_noise_func_;
	void GetSnowAccumulation(glm::vec3 origin, glm::vec3 direction, raytracer::Intersection &inter);

	double moon_sun_intens_ratio_;
	std::vector<SkyState> rgb_sky_;
	std::vector<SkyState> rgb_night_sky_;
	bool is_night_;
	glm::vec3 GetAvgSkyRgbNoClouds();
	glm::vec3 no_cloud_avg_rgb_;
  //ArHosekSkyModelState* rgb_sky_[3];
	//ArHosekSkyModelState* rgb_night_sky_[3];
	//bool day_sky_allocated_;
	//bool night_sky_allocated_;

  double albedo_[3];
  double turbidity_;
	bool sky_initialized_;
	bool solar_direction_calculated_;

	glm::vec2 wind_;
	float rain_rate_;
	float rain_scattering_coeff_;
	bool is_raining_;

	DateTime old_date_;

  std::vector<ParticleSystem> particle_systems_;
	std::vector<actor::Actor> actors_;
	std::unordered_map<int, int> particle_system_to_actor_map_;
	std::string data_path_;

	void FreeSkyModelStates();
	void InitializeSkyStates();

	void WindTime(float secs);
};

} //namespace environment
} //namespace mavs

#endif
