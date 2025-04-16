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
// The rapidjson includes must be first because of 
// a weird Bool typedef
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <fstream>
#include <iostream>
#include <limits>
#include <algorithm>

#include <mavs_core/environment/environment.h>
#include <mavs_core/math/constants.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>

namespace mavs{
namespace environment{

Environment::Environment(){
	//These need to be called in the correct order
	// or some values won't be initialized
	//SetAlbedoRgb(0.1,0.1,0.1);
	//SetDateTime(2004,6,5,14,0,0,6);
	//SetLocalOrigin(32.3033, 90.8742,73.152);
	snow_noise_func_.SetFrequency(100.0);
	surface_type_ = "dry";
	sky_on_ = true;
	sky_constant_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
	sun_constant_color_ = glm::vec3(255.0f, 255.0f, 255.0f);
	sun_solid_angle_ = 6.8E-5f;
	rci_ = 250.0f;
	sky_initialized_ = false;
	solar_direction_calculated_ = false;
	is_raining_ = false;
	is_night_ = false;
	albedo_[0] = 0.1;
	albedo_[1] = 0.1;
	albedo_[2] = 0.1;
	date_time_.year = 2004;
	date_time_.month = 6;
	date_time_.day = 5;
	date_time_.hour = 14;
	date_time_.minute = 0;
	date_time_.second = 0;
	date_time_.time_zone = 6;
	old_date_ = date_time_;
	temperature_ = 20.0f; //degrees celsius
	is_snowing_ = false;
	snow_rate_ = 0.0f;
	snow_accum_fac_ = 0.0f;

	fog_k_ = 0.0f;
	local_origin_.latitude = 32.3033;
  	local_origin_.longitude = 90.8742;
  	local_origin_.altitude = 73.152;
	solar_position_.SetDateTime(date_time_);
	solar_position_.SetLocalLatLong(local_origin_.latitude, local_origin_.longitude);
	glm::vec3 solar_direction_ = GetSolarDirection();
	SetTurbidity(2.5);

	//day_sky_allocated_ = false;
	//night_sky_allocated_ = false;
	rain_scattering_coeff_ = 1.0;
	wind_ = glm::vec2(0.0f, 0.0f);
	avg_sky_col_ = glm::vec3(0.0f, 0.0f, 0.0f);
	moon_sun_intens_ratio_ = 1.62E-6;
	MavsDataPath dp;
	data_path_ = dp.GetPath();
	data_path_.append("/");
	InitializeSkyStates();
	scene_ = NULL;
}

Environment::~Environment() {
	FreeSkyModelStates();	
}

void Environment::SetRaytracer(raytracer::Raytracer *tracer) {
	scene_ = tracer;
	//surface_type_ = tracer->GetSurfaceMaterialType();
	//rci_ = tracer->GetSuraceConeIndex();
}

void Environment::SetTurbidity(double turbidity) { 
	turbidity_ = turbidity; 
	InitializeSkyStates();
}

void Environment::FreeSkyModelStates() {
	if (rgb_sky_.size()>0) {
		for (unsigned int i = 0; i < 3; i++) {
			arhosekskymodelstate_free(rgb_sky_[0].state[i]);
		}
		rgb_sky_.pop_back();
	}
	if (rgb_night_sky_.size()>0) {
		for (unsigned int i = 0; i < 3; i++) {
			arhosekskymodelstate_free(rgb_night_sky_[0].state[i]);
		}
		rgb_night_sky_.pop_back();
	}
	sky_initialized_ = false;
	//day_sky_allocated_ = false;
	//night_sky_allocated_ = false;
}

void Environment::InitializeSkyStates() {
	
	FreeSkyModelStates();
	double solar_elevation = (float)(mavs::kPi_2 - acos(sun_direction_.z));
	if (solar_elevation > 0.0f && !is_night_) {
		if (solar_elevation < 0.07)solar_elevation = 0.07;
		sky_initialized_ = true;
		SkyState rgb;
		rgb_sky_.push_back(rgb);
		for (int i = 0; i < 3; i++) {
			rgb_sky_[0].state[i] = arhosek_rgb_skymodelstate_alloc_init(turbidity_, albedo_[i],
				solar_elevation);
		}
	}
	else {
		sky_initialized_ = false;
		//save for next time
		double lunar_elevation = (90.0*kDegToRad - acos(sun_direction_.z));
		if (lunar_elevation < 0.07)lunar_elevation = 0.07;
		SkyState rgb;
		rgb_night_sky_.push_back(rgb);
		float moon_phase_frac = solar_position_.GetLunarIntensityFraction();
		for (int i = 0; i < 3; i++) {
			rgb_night_sky_[0].state[i] = arhosekskymodelstate_alienworld_alloc_init(lunar_elevation,
				moon_phase_frac*moon_sun_intens_ratio_, 4100.0, turbidity_, albedo_[i]);
		}
	}
	no_cloud_avg_rgb_ = GetAvgSkyRgbNoClouds();
}

void Environment::SetDateTime(DateTime date_time) {
	date_time_.millisecond = date_time.millisecond;
	SetDateTime(date_time.year, date_time.month, date_time.day, date_time.hour, date_time.minute, date_time.second, date_time.time_zone);
}

void Environment::SetDateTime(int year, int month, int day, int hour, int minute,
	int second, int time_zone) {
	date_time_.year = year;
	date_time_.month = month;
	date_time_.day = day;
	date_time_.hour = hour;
	date_time_.minute = minute;
	date_time_.second = second;
	date_time_.time_zone = time_zone;
	solar_position_.SetDateTime(date_time_);
	solar_direction_calculated_ = false;
	glm::vec3 solar_direction_ = GetSolarDirection();
	SetTurbidity(turbidity_);
}

void Environment::WindTime(float sec) {
	std::vector<int> days_per_month{ 31,28,31,30,31,30,31,31,30,31,30,31 };
	if (date_time_.year % 4 == 0)days_per_month[1] = 29;
	int num_sec = (int)floor(sec); // number of seconds to wind
	int num_msec = (int)(1000 * sec - 1000*floor(sec)); // number of milli seconds to wind

	// update milliseconds;
	int newms = date_time_.millisecond + num_msec;
	if (newms > 999) {
		date_time_.millisecond = newms - 1000;
		date_time_.second += 1;
	}
	else {
		date_time_.millisecond = newms;
	}
	date_time_.second += num_sec;
	while (date_time_.second >= 60) {
		date_time_.second -= 60;
		date_time_.minute++;

		if (date_time_.minute >= 60) {
			date_time_.minute = 0;
			date_time_.hour++;

			if (date_time_.hour >= 24) {
				date_time_.hour = 0;
				date_time_.day += 1;
				int month_idx = date_time_.month - 1;
				if (date_time_.day > days_per_month[month_idx]) {
					date_time_.day = 1;
					date_time_.month += 1;
					if (date_time_.month > 12) {
						date_time_.year += 1;
						date_time_.month = 1;
					}
				}
			}
		}
	}
}

void Environment::Load(std::string input_file){
	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	if (d.HasMember("LocalOrigin")) {
		if (d["LocalOrigin"].HasMember("Latitude") &&
			d["LocalOrigin"].HasMember("Longitude") &&
			d["LocalOrigin"].HasMember("Altitude")) {
			double lat = d["LocalOrigin"]["Latitude"].GetDouble();
			double lon = d["LocalOrigin"]["Longitude"].GetDouble();
			double alt = d["LocalOrigin"]["Altitude"].GetDouble();
			SetLocalOrigin(lat, lon, alt);
		}
	}

	if (d.HasMember("DateTime")) {
		if (d["DateTime"].HasMember("Year") &&
			d["DateTime"].HasMember("Month") &&
			d["DateTime"].HasMember("Day") &&
			d["DateTime"].HasMember("Hour") &&
			d["DateTime"].HasMember("Minute") &&
			d["DateTime"].HasMember("Second") &&
			d["DateTime"].HasMember("TimeZone")) {
			int year = d["DateTime"]["Year"].GetInt();
			int month = d["DateTime"]["Month"].GetInt();
			int day = d["DateTime"]["Day"].GetInt();
			int hour = d["DateTime"]["Hour"].GetInt();
			int minute = d["DateTime"]["Minute"].GetInt();
			int second = d["DateTime"]["Second"].GetInt();
			int timezone = d["DateTime"]["TimeZone"].GetInt();
			SetDateTime(year, month, day, hour, minute, second, timezone);
		}
	}

	if (d.HasMember("AtmosphericTurbidity")) {
		double turbidity = d["AtmosphericTurbidity"].GetDouble();
		SetTurbidity(turbidity);
	}

	if (d.HasMember("Rain Rate")) {
		rain_rate_ = d["Rain Rate"].GetFloat();
		SetRainRate(rain_rate_);
		is_raining_ = true;
	}

	if (d.HasMember("Wind")) {
		wind_.x = d["Wind"][0].GetFloat();
		wind_.y = d["Wind"][1].GetFloat();
	}

	if (d.HasMember("LocalAlbedo")) {
		if (d["LocalAlbedo"].HasMember("Red") &&
			d["LocalAlbedo"].HasMember("Green") &&
			d["LocalAlbedo"].HasMember("Blue")) {
			double r = d["LocalAlbedo"]["Red"].GetDouble();
			double g = d["LocalAlbedo"]["Green"].GetDouble();
			double b = d["LocalAlbedo"]["Blue"].GetDouble();
			SetAlbedoRgb(r, g, b);
		}
	}
}


void Environment::GetSnowAccumulation(glm::vec3 origin, glm::vec3 direction, raytracer::Intersection &inter) {
	if (glm::dot(direction, inter.normal) > 0.0f)inter.normal = -inter.normal;
	glm::vec3 p = origin + inter.dist*direction + 0.0001f*inter.normal;
	glm::vec3 snow_col(1.0f, 1.0f, 1.0f);

	float f_inc = std::min(1.0f,std::max(inter.normal.z,0.0f) + 0.4f*(float)(1.0f + snow_noise_func_.GetPerlin(p.x, p.y)));
	glm::vec3 v_snow(wind_.x, wind_.y, -1.0f);
	
	raytracer::Intersection snow_inter = scene_->GetClosestIntersection(p, -v_snow);
	float f_e = 1.0f;
	if (snow_inter.dist>0.0f) f_e = 1.0f-(float)exp(-snow_inter.dist); 
	float sa = snow_accum_fac_ * (f_inc*f_e);
	inter.color = sa * snow_col + (1.0f-sa) * inter.color;
	inter.material.kd = sa * snow_col + (1.0f-sa) * inter.material.kd;
	inter.material.ka = inter.material.kd;
	inter.material.ks = glm::vec3(0.0f, 0.0f, 0.0f);
	inter.material.ns = 1.0f;
	return;
}

raytracer::Intersection Environment::GetClosestIntersection(glm::vec3 origin, glm::vec3 direction) {
	raytracer::Intersection inter = scene_->GetClosestIntersection(origin, direction);
	if (snow_accum_fac_ > 0.0f && inter.dist>0.0f) {
		GetSnowAccumulation(origin, direction, inter);
	}
	return inter;
}

raytracer::Intersection Environment::GetClosestTerrainIntersection(glm::vec3 origin, glm::vec3 direction) {
	raytracer::Intersection inter = scene_->GetClosestTerrainIntersection(origin, direction);
	return inter;
}

void Environment::GetClosestIntersection(glm::vec3 origin, glm::vec3 direction, raytracer::Intersection &inter) {
	scene_->GetClosestIntersection(origin, direction, inter);
	if (snow_accum_fac_ > 0.0f && inter.dist>0.0f) {
		GetSnowAccumulation(origin, direction, inter);
	}
	return;
}


void Environment::SetAlbedoRgb(double red_alb, double green_alb, double blue_alb) {
	albedo_[0] = red_alb;
	albedo_[1] = green_alb;
	albedo_[2] = blue_alb;
}

glm::vec3 Environment::GetAlbedoRgb(){
  glm::vec3 albedo(albedo_[0],albedo_[1],albedo_[2]);
  return albedo;
}

void Environment::SetRainRate(float rate) {
	rain_rate_ = rate;
	if (rain_rate_ > 0.0f) {
		is_raining_ = true;
		//rain_scattering_coeff_ = exp(-0.075f*rain_rate_);
		//rain_scattering_coeff_ = exp(-0.125f*rain_rate_);
		rain_scattering_coeff_ = 1.0f;
		//turbidity_ = mavs::math::clamp(0.32f*rate + 2.0f, 2.0f, 10.0f);
		//SetTurbidity(turbidity_);
	}
}

void Environment::SetSkyConstantColor(float r, float g, float b) {
	sky_constant_color_ = glm::vec3(r, g, b);
	sky_on_ = false;
}

void Environment::SetSunConstantColor(float r, float g, float b) {
	sun_constant_color_ = glm::vec3(r, g, b);
	sky_on_ = false;
}

glm::vec3 Environment::GetSkyRgb(glm::vec3 direction, float solid_angle){
	if (sky_on_) {
		double theta = acos(direction.z);
		if (theta >= mavs::kPi_2)theta = (float)(mavs::kPi_2 - 0.001);
		double gamma = acos(dot(direction, sun_direction_));
		if (std::isnan(gamma))gamma = 0.0;
		glm::vec3 skycolor;
		if (sky_initialized_) { //day time
			for (int i = 0; i < 3; i++) {
				skycolor[i] = (float)(arhosek_tristim_skymodel_radiance(rgb_sky_[0].state[i], theta, gamma, i));
			}
		}
		else { //night_time
			for (int i = 0; i < 3; i++) {
				skycolor[i] = (float)(arhosek_tristim_skymodel_radiance(rgb_night_sky_[0].state[i], theta, gamma, i));
			}
			float l = solar_position_.GetStarBrightness(direction, solid_angle);
			if (l > 0) {
				float intens = (float)(0.1*l / 1.1E-7);
				skycolor = skycolor + glm::vec3(intens, intens, intens);
			}
		}

		//float cloudcover = clouds_.GetCloudDensity(direction);

		glm::vec3 cloudcover = clouds_.GetCloudColor(direction);
		if (glm::length(cloudcover) > 0.0f) {
			float brightness = glm::length(no_cloud_avg_rgb_);
			skycolor = brightness * cloudcover;
		}
		// assumes that any scattered light from the rain makes it back into the sky
		//skycolor = (1.0f + rain_scattering_coeff_)*skycolor;

		return skycolor;
	}
	else {
		float gamma = acosf(dot(direction, sun_direction_));
		glm::vec3 skycolor = sky_constant_color_;
		if (gamma <= sun_solid_angle_)skycolor = sun_constant_color_;
		return skycolor;
	}
}

void Environment::SetCloudCover(float cover_fraction) {
	clouds_.SetCloudCoverFraction(cover_fraction);
}

glm::vec3 Environment::GetAvgSkyRgbNoClouds() {
	float frac = clouds_.GetCloudCoverFraction();
	clouds_.SetCloudCoverFraction(0.0);
	glm::vec3 col = GetAvgSkyRgb();
	clouds_.SetCloudCoverFraction(frac);
	return col;
}

glm::vec3 Environment::GetAvgSkyRgb() {
	if (sky_on_) {
		float solar_elevation = (float)(90.0*kDegToRad - acos(sun_direction_.z));
		float ang_step = (float)(10.0f*mavs::kDegToRad);
		float theta = ang_step;
		float max_theta = (float)(90.0f*mavs::kDegToRad);
		float max_phi = (float)(180.0f*mavs::kDegToRad);
		float nsteps = 0.0f;
		while (theta <= max_theta) {
			float phi = (float)(ang_step - 180.0f*mavs::kDegToRad);
			while (phi <= max_phi) {
				glm::vec3 d(cos(phi)*sin(theta), sin(phi)*sin(theta), cos(theta));
				avg_sky_col_ += GetSkyRgb(d, 0.0776f);
				nsteps += 1.0f;
				phi += ang_step;
			}
			theta += ang_step;
		}
		avg_sky_col_ = avg_sky_col_ / nsteps;
		return avg_sky_col_;
	}
	else {
		return sky_constant_color_;
	}
}

glm::vec3 Environment::GetSunColor() {
	if (sky_on_) {
		glm::vec3 direction = GetSolarDirection();
		glm::vec3 suncolor = GetSkyRgb(direction, 0.0001f);

		if (is_raining_) {
			//suncolor = rain_scattering_coeff_ * suncolor;
		}
		float d = clouds_.GetCloudDensity(direction);
		suncolor = suncolor * (1.0f - d);
		return suncolor;
	}
	else {
		return sun_constant_color_;
	}
}

void Environment::SetLocalOrigin(double lat, double lon, double alt){
  local_origin_.latitude = lat;
  local_origin_.longitude = lon;
  local_origin_.altitude = alt;
	solar_position_.SetLocalLatLong(lat, lon);
}

glm::vec3 Environment::GetSolarDirection(){
  glm::vec3 direction;
	if (!(date_time_ == old_date_) || !solar_direction_calculated_) {
		HorizontalCoordinate solpos =
			solar_position_.GetSolarPosition();
		float sz = (float)sin(solpos.zenith);
		//direction.x = (float)cos(solpos.azimuth)*sz;
		//direction.y = (float)sin(solpos.azimuth)*sz;
		// ctg, 4/16/25, Kodai noticed these were wrong
		direction.y = (float)cos(-solpos.azimuth) * sz;
		direction.x = (float)sin(-solpos.azimuth) * sz;
		direction.z = (float)cos(solpos.zenith);
		//save for next time
		sun_direction_ = direction;
		solar_direction_calculated_ = true;
		double solar_elevation = (float)(mavs::kPi_2 - acos(sun_direction_.z));
		if  (solar_elevation<0.0f){
			HorizontalCoordinate moonpos = solar_position_.GetLunarPosition();
			float sz = (float)sin(moonpos.zenith);
			direction.x = (float)cos(moonpos.azimuth)*sz;
			direction.y = (float)sin(moonpos.azimuth)*sz;
			direction.z = (float)cos(moonpos.zenith);
			sky_initialized_ = false;
			//save for next time
			sun_direction_ = direction;
			is_night_ = true;
			solar_direction_calculated_ = true;
		}
		else {
			is_night_ = false;
		}
  }
  else { // we've already calculated solar direction
    direction = sun_direction_;
  }
  old_date_ = date_time_;
  return direction;
}

void Environment::SetSunPosition(float azimuth_degrees, float zenith_degrees) {
	// zenith of zero = straight overhead
	// azimuth of zero = north
	// azimuth of 90 degrees = east
	float zen = (float)(zenith_degrees*kDegToRad);
	float az = (float)(azimuth_degrees*kDegToRad);
	
	sun_direction_ = glm::vec3(sinf(zen)*sinf(az), sinf(zen)*cosf(az), cosf(zen));
	solar_direction_calculated_ = true;
}

float Environment::GetGroundHeight(float x, float y) {
	if (scene_ == NULL) {
		return 0.0f;
	}
	float z = scene_->GetSurfaceHeight(x, y);
	return z;
}

glm::vec4 Environment::GetGroundHeightAndNormal(float x, float y) {
	if (scene_ == NULL) {
		return glm::vec4(0.0f, 0.0f, 1.0f, 0.0);
	}
	glm::vec4 z = scene_->GetSurfaceHeightAndNormal(x, y);
	if (z.z < 0.0)z = -1.0f*z;
	return z;
}

std::vector<int> Environment::AddActor(std::string meshfile, bool y_to_z, bool x_to_y, bool y_to_x, glm::vec3 offset, glm::vec3 scale) {

	std::vector<int> actor_nums = scene_->AddActor(meshfile, y_to_z, x_to_y, y_to_x, offset, scale);
	actor::Actor actor;
	actor.SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
	actor.SetOffset(offset.x, offset.y, offset.z);
	actor.SetScale(scale.x, scale.y, scale.z);
	actors_.push_back(actor);

	if (actors_.size() != actor_nums.size()) {
		std::cerr << "ERROR: There was an error loading the actors, "
			<< " The ray tracer loaded " << actor_nums.size()
			<< " actors, but the environment has " << actors_.size()
			<< " actors." << std::endl;
		exit(1);
	}

	for (int i = 0; i < (int)actors_.size(); i++) {
		actors_[i].SetId(actor_nums[i]);
	}
	return actor_nums;
}

std::vector<int> Environment::LoadActors(std::string actor_file) {
	std::vector<int> actor_nums = scene_->AddActors(actor_file);
	actor::Actor actor;

	FILE* fp = fopen(actor_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	if (d.HasMember("Actors")) {
		for (unsigned int m = 0; m<d["Actors"].Capacity(); m++) {

			if (d["Actors"][m].HasMember("Locked to Ground")) {
				bool locked = d["Actors"][m]["Locked to Ground"].GetBool();
				if (locked)actor.LockToGround();
			}

			if (d["Actors"][m].HasMember("Instances")) {
				int num_inst = d["Actors"][m]["Instances"].Capacity();
				for (int mi = 0; mi<num_inst; mi++) {
					// Get the paths for the actors
					if (d["Actors"][m]["Instances"][mi].HasMember("Initial Position")) {
						glm::vec3 pos;
						pos.x = d["Actors"][m]["Instances"][mi]["Initial Position"][0].GetFloat();
						pos.y = d["Actors"][m]["Instances"][mi]["Initial Position"][1].GetFloat();
						pos.z = d["Actors"][m]["Instances"][mi]["Initial Position"][2].GetFloat();
						actor.SetPosition(pos);
					}
					if (d["Actors"][m]["Instances"][mi].HasMember("Waypoints File")) {
						//std::string wpfile = d["Actors"][m]["Instances"][mi]["Waypoints File"].GetString();
						std::string wpfile = data_path_;
						wpfile.append(d["Actors"][m]["Instances"][mi]["Waypoints File"].GetString());
						actor.LoadPath(wpfile);
					}
					// Get the offsests for the actors
					if (d["Actors"][m]["Instances"][mi].HasMember("Offset")) {
						actor.SetOffset(d["Actors"][m]["Instances"][mi]["Offset"][0].GetFloat(),
							d["Actors"][m]["Instances"][mi]["Offset"][1].GetFloat(),
							d["Actors"][m]["Instances"][mi]["Offset"][2].GetFloat());
					}
					if (d["Actors"][m]["Instances"][mi].HasMember("Scale")) {
						actor.SetScale(d["Actors"][m]["Instances"][mi]["Scale"][0].GetFloat(),
							d["Actors"][m]["Instances"][mi]["Scale"][1].GetFloat(),
							d["Actors"][m]["Instances"][mi]["Scale"][2].GetFloat());
					}
					if (d["Actors"][m]["Instances"][mi].HasMember("Speed")) {
						actor.SetSpeed(d["Actors"][m]["Instances"][mi]["Speed"].GetFloat());
					}
				} //loop over instances
			} // if instanced
			else {
				std::cerr << "ERROR: No instances listed for Actor "
					<< m << std::endl;
				exit(1);
			}
			actors_.push_back(actor);
		}
	}

	if (actors_.size() != actor_nums.size()) {
		std::cerr << "ERROR: There was an error loading the actors, "
			<< " The ray tracer loaded " << actor_nums.size()
			<< " actors, but the environment has " << actors_.size()
			<< " actors." << std::endl;
		exit(1);
	}
	
	for (int i = 0; i < (int)actors_.size(); i++) {
		actors_[i].SetId(actor_nums[i]);
	}
	return actor_nums;
}

int Environment::AddSpotlight(glm::vec3 color, glm::vec3 position, glm::vec3 direction,
	float spot_angle) {
	Light light;
	light.color = color;
	light.position = position;
	light.direction = direction;
	light.type = 2;
	light.decay = 1.0;
	light.angle = (float)mavs::kDegToRad*spot_angle;
	float maxel = -1.0E6;
	for (int i = 0; i < 3; i++)if (color[i] > maxel)maxel = color[i];
	light.cutoff_distance = pow(maxel / 0.5f, 1.0f / light.decay);
	lights_.push_back(light);
	return (int)(lights_.size() - 1);
}

int Environment::AddPointlight(glm::vec3 color, glm::vec3 position) {
	Light light;
	light.color = color;
	light.position = position;
	light.type = 1;
	light.decay = 2.0;
	float maxel = -1.0E6;
	for (int i = 0; i < 3; i++)if (color[i] > maxel)maxel = color[i];
	light.cutoff_distance = pow(maxel / 0.5f, 1.0f / light.decay);
	lights_.push_back(light);
	return (int)(lights_.size() - 1);
}

int Environment::AddLight(Light light) {
	lights_.push_back(light);
	return (int)(lights_.size() - 1);
}

void Environment::MoveLight(int light_id, glm::vec3 position, glm::vec3 direction) {
	if (light_id >= 0 && light_id < (int)lights_.size()) {
		lights_[light_id].position = position;
		lights_[light_id].direction = direction;
	}
}

void Environment::RemoveLights(std::vector<int> ids) {
	std::vector<Light> newlights;
	for (int i = 0; i < (int)lights_.size(); i++) {
		bool remove = false;
		for (int j = 0; j < (int)ids.size(); j++) {
			if (i ==  ids[j]) remove = true;
		}
		if (!remove)newlights.push_back(lights_[i]);
	}
	lights_ = newlights;
}

void Environment::SetActorPosition(int id, glm::vec3 pos, glm::quat orient) {
	SetActorPosition(id, pos, orient, true);
}

void Environment::SetActorPosition(int id, glm::vec3 pos, glm::quat orient, bool commit_scene) {
	SetActorPosition(id, pos, orient, 1.0f, commit_scene);
}

void Environment::SetActorVelocity(int id, glm::vec3 vel) {
	if (id >= 0 && id < (int)actors_.size()) {
		actors_[id].SetVelocity(vel);
		scene_->SetActorVelocity(actors_[id].GetId(), vel);
	}
}

void Environment::SetActorPosition(int id, glm::vec3 pos, glm::quat orient, float dt) {
	SetActorPosition(id, pos, orient, dt, true);
}

void Environment::SetActorPosition(int id, glm::vec3 pos, glm::quat orient, float dt, bool commit_scene) {
	if (id >= 0 && id < (int)actors_.size()) {
		glm::vec3 old_position = actors_[id].GetPosition();
		actors_[id].SetVelocity((pos - old_position) / dt);
		actors_[id].SetPose(pos, orient);
		pos += actors_[id].GetOffset();
		if (actors_[id].IsLockedToGround()) {
			pos.z = scene_->GetSurfaceHeight(pos.x, pos.y) + actors_[id].GetZOffset();
			pos.z += actors_[id].GetZOffset();
		}
		scene_->UpdateActor(pos, actors_[id].GetOrientation(),
			actors_[id].GetScale(), actors_[id].GetId(), commit_scene);
		actors_[id].SetAutoUpdate(false);
	}
}

void Environment::AdvanceParticleSystems(float dt) {
	//advance the particle systems
	for (int p = 0; p<(int)particle_systems_.size(); p++) {
		if (particle_system_to_actor_map_[p] >= 0) {
			int actor_num = particle_system_to_actor_map_[p];
			if (actor_num >= 0 && actor_num < actors_.size()) {
				glm::vec3 pos = actors_[actor_num].GetPosition();
				particle_systems_[p].SetSourceCenter(pos);
				glm::vec3 vel = actors_[actor_num].GetVelocity();
				particle_systems_[p].SetInitialVelocity(vel.x, vel.y, vel.z);
			}
		}
		particle_systems_[p].Advance(dt);
	}
}

void Environment::AdvanceTime(float dt){
	AdvanceParticleSystems(dt);
	snow_.Update(dt, glm::vec3(wind_.x, wind_.y,0.0f));
	snow_accum_fac_ = std::max(0.0f,std::min(1.0f, snow_accum_fac_ + dt * snow_rate_ / 60.0f));
	//advance actors
	for (int a = 0; a < (int)actors_.size(); a++) {
		if (actors_[a].GetAutoUpdate()) {
			glm::vec3 old_pos = actors_[a].GetPosition();;
			actors_[a].Update(dt);
			glm::vec3 pos = actors_[a].GetPosition();
			actors_[a].SetVelocity((pos - old_pos) / dt);
			if (actors_[a].IsLockedToGround())
				pos.z = scene_->GetSurfaceHeight(pos.x, pos.y);
			pos += actors_[a].GetOffset();
			scene_->UpdateActor(pos, actors_[a].GetOrientation(),
				actors_[a].GetScale(), actors_[a].GetId());
		}
	}
	if (scene_) scene_->UpdateAnimations(dt);
	WindTime(dt);
	SetDateTime(date_time_);
}

glm::vec3 Environment::GetActorPosition(int act_num) {
	glm::vec3 p(0.0, 0.0, 0.0);
	if (act_num >= 0 && act_num < (int)actors_.size()) {
		p = actors_[act_num].GetPosition();
	}
	return p;
}

std::vector< std::vector <std::vector<float> > > Environment::GetVegDensityOnGrid(glm::vec3 ll, glm::vec3 ur, float res) {
	float half_res = 0.5f*res;
	int num_sample_rays = std::max(10,(int)std::floor(100*res*res));

	std::vector <std::vector < std::vector<float> > > grid;
	int nx = (int)ceil((ur.x - ll.x) / res);
	int ny = (int)ceil((ur.y - ll.y) / res);
	int nz = (int)ceil((ur.z - ll.z) / res);
	if (nx <= 0 || ny <= 0 || nz <= 0) return grid;

	std::vector<glm::vec3> directions,look_to;
	directions.resize(6);
	look_to.resize(6);
	directions[0] = glm::vec3(half_res, 0.0f, 0.0f);
	directions[1] = glm::vec3(-half_res, 0.0f, 0.0f);
	directions[2] = glm::vec3(0.0f, half_res, 0.0f);
	directions[3] = glm::vec3(0.0f, -half_res, 0.0f);
	directions[4] = glm::vec3(0.0f, 0.0f, half_res);
	directions[5] = glm::vec3(0.0f, 0.0f, -half_res);
	for (int i = 0; i < 6; i++) {
		look_to[i] = -directions[i] / half_res;
	}

	grid = mavs::utils::Allocate3DVector(nx, ny, nz, 0.0f);
#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
	for (int i = 0; i < nx; i++) {
		float x = ll.x + (i + 0.5f) * res;
		for (int j = 0; j < ny; j++) {
			float y = ll.y + (j + 0.5f) * res;
			for (int k = 0; k < nz; k++) {
				float z = ll.z + (k + 0.5f) * res;
				float num_hits = 0.0f;
				glm::vec3 cp(x, y, z);
				for (int np = 0; np < num_sample_rays; np++) {
					int dirindex = mavs::math::rand_in_range(0, 5);
					glm::vec3 p = cp + directions[dirindex];
					if (dirindex == 0 || dirindex == 1) {
						p = p + glm::vec3(0.0f, mavs::math::rand_in_range(-half_res, half_res), mavs::math::rand_in_range(-half_res, half_res));
					}
					else if (dirindex == 2 || dirindex == 3) {
						p = p + glm::vec3(mavs::math::rand_in_range(-half_res, half_res), 0.0f, mavs::math::rand_in_range(-half_res, half_res));
					}
					else {
						p = p + glm::vec3(mavs::math::rand_in_range(-half_res, half_res), mavs::math::rand_in_range(-half_res, half_res), 0.0f);
					}
					mavs::raytracer::Intersection inter = GetClosestIntersection(p, look_to[dirindex]);
				
					if (inter.dist > 0.0f && inter.dist<=res && inter.object_id!=-99) {
						num_hits += 1.0f;
					}
				}
				grid[i][j][k] = num_hits / (float)num_sample_rays;
			} //k
		} // j
	} // i
	return grid;
}

//std::string Environment::GetObjectName(int id) {
//	return scene_->GetObjectName(id);
//}

//glm::quat Environment::GetObjectOrientation(int id) {
//	return scene_->GetObjectOrientation(id);
//}

raytracer::BoundingBox Environment::GetObjectBoundingBox(int id) {
	raytracer::BoundingBox bb;
	bb = scene_->GetObjectBoundingBox(id);
	return bb;
}
  
} //namespace environment
} //namespace mavs
