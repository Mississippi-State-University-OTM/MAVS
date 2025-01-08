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

// class defintion
#include "vehicles/uav/uav_sim.h"
// mavs includes
#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

namespace mavs {
namespace vehicle {
static rapidjson::Document LoadJsonDocument(std::string input_file) {
	if (!mavs::utils::file_exists(input_file)) {
		std::cerr << "ERROR: Requested vehicle input file, " << input_file << ", which does not exist. EXITING!" << std::endl;
		exit(97);
	}

	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	return d;
};

void UavSim::LoadSimulation(std::string input_file, mavs::raytracer::embree::EmbreeTracer* scene, mavs::environment::Environment* env) {

	rapidjson::Document doc = LoadJsonDocument(input_file);

	if (doc.HasMember("Save Images")) {
		save_data_ = doc["Save Images"].GetBool();
	}

	/*if (doc.HasMember("Show Map")) {
		show_map_ = doc["Show Map"].GetBool();
	}
	glm::vec2 llc(-2000.0f, -2000.0f);
	glm::vec2 urc(2000.0f, 2000.0f);
	float pixdim = 5.0f;

	if (doc.HasMember("Map")) {
		if (doc["Map"].HasMember("Lower Left Corner")) {
			if (doc["Map"]["Lower Left Corner"].Capacity() > 1) {
				llc.x = doc["Map"]["Lower Left Corner"][0].GetFloat();
				llc.y = doc["Map"]["Lower Left Corner"][1].GetFloat();
			}
		}
		if (doc["Map"].HasMember("Upper Right Corner")) {
			if (doc["Map"]["Upper Right Corner"].Capacity() > 1) {
				urc.x = doc["Map"]["Upper Right Corner"][0].GetFloat();
				urc.y = doc["Map"]["Upper Right Corner"][1].GetFloat();
			}
		}
		if (doc["Map"].HasMember("Pixel Size")) {
			pixdim = doc["Map"]["Pixel Size"].GetFloat();
		}
	}

	map_.Init(llc, urc, pixdim);*/

	std::string scene_in = "";
	if (doc.HasMember("MAVS Scene File")) {
		scene_in = doc["MAVS Scene File"].GetString();
	}
	else {
		std::cerr << "ERROR: No \"MAVS Scene File\" entry in json file. Exiting." << std::endl;
		exit(98);
	}

	std::string actor_in = "";
	if (doc.HasMember("MAVS Actor File")) {
		actor_in = doc["MAVS Actor File"].GetString();
	}
	else {
		std::cerr << "ERROR: No \"MAVS Actor File\" entry in json file. Exiting." << std::endl;
		exit(96);
	}



	mavs::MavsDataPath mdp;
	std::string mavs_data_path = mdp.GetPath();
	std::string scene_file(mavs_data_path + "/scenes/" + scene_in);

	scene->Load(scene_file);

	env->SetRaytracer(scene);

	env->LoadActors(mavs_data_path + "/actors/actors/" + actor_in);

	glm::vec3 init_pos(0.0f, 0.0f, 250.0f);
	if (doc.HasMember("Initial Position")) {
		if (doc["Initial Position"].Capacity() == 3) {
			for (int i = 0; i < 3; i++) {
				init_pos[i] = doc["Initial Position"][i].GetFloat();
			}
		}
	}

	float init_airspeed = 15.0f; //m/s
	if (doc.HasMember("Initial Airspeed")) {
		init_airspeed = doc["Initial Airspeed"].GetFloat();
	}

	float init_heading = 0.0f; //radians
	if (doc.HasMember("Initial Heading")) {
		init_heading = doc["Initial Heading"].GetFloat();
	}

	if (doc.HasMember("People")) {
		for (int i = 0; i < (int)doc["People"].Capacity(); i++) {
			if (doc["People"][i].HasMember("Position") && doc["People"][i].HasMember("Heading") &&
				doc["People"][i].HasMember("Speed")) {
				if (doc["People"][i]["Position"].Capacity() == 2) {
					AnimationInit anim_init;
					anim_init.position.x = doc["People"][i]["Position"][0].GetFloat();
					anim_init.position.y = doc["People"][i]["Position"][1].GetFloat();
					anim_init.heading = doc["People"][i]["Heading"].GetFloat();
					anim_init.velocity = doc["People"][i]["Speed"].GetFloat();
					anim_inits_.push_back(anim_init);
				}
			}
		}
	}

	if (doc.HasMember("Camera")) {
		use_camera_ = false;
		if (doc["Camera"].HasMember("Active")) {
			use_camera_ = doc["Camera"]["Active"].GetBool();
		}
		if (doc["Camera"].HasMember("Use Gimbal")) {
			use_gimbal_ = doc["Camera"]["Use Gimbal"].GetBool();
		}
		if (doc["Camera"].HasMember("LWIR")) {
			use_lwir_ = doc["Camera"]["LWIR"].GetBool();
		}
		if (doc["Camera"].HasMember("Resolution") &&
			doc["Camera"].HasMember("Image Plane Size") &&
			doc["Camera"].HasMember("Focal Length")
			) {
			if (doc["Camera"]["Resolution"].Capacity() == 2 &&
				doc["Camera"]["Image Plane Size"].Capacity() == 2) {
				int nx = doc["Camera"]["Resolution"][0].GetInt();
				int ny = doc["Camera"]["Resolution"][1].GetInt();
				float px = doc["Camera"]["Image Plane Size"][0].GetFloat();
				float py = doc["Camera"]["Image Plane Size"][1].GetFloat();
				float flen = doc["Camera"]["Focal Length"].GetFloat();
				if (use_lwir_) {
					lwir_camera_.Initialize(nx, ny, px, py, flen);
					std::string thermal_file(mavs_data_path + "/sims/thermal_sims/utah_desert.json");
					lwir_camera_.LoadThermalData(thermal_file);
					glm::vec3 sensor_offset(1.0f, 0.0f, -2.5f);
					glm::quat sensor_orient(0.7071f, 0.0f, 0.7071f, 0.0f);
					if (use_gimbal_)sensor_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
					lwir_camera_.SetRelativePose(sensor_offset, sensor_orient);
					lwir_camera_.SetAntiAliasing("oversampled");
					lwir_camera_.SetPixelSampleFactor(3);
				}
				else {
					rgb_camera_.Initialize(nx, ny, px, py, flen);
					glm::vec3 sensor_offset(1.0f, 0.0f, -2.5f);
					glm::quat sensor_orient(0.7071f, 0.0f, 0.7071f, 0.0f);
					if (use_gimbal_)sensor_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
					rgb_camera_.SetRelativePose(sensor_offset, sensor_orient);
					rgb_camera_.SetElectronics(0.75f, 1.0f);
					rgb_camera_.SetAntiAliasing("oversampled");
					rgb_camera_.SetPixelSampleFactor(3);
				}
			}
		}
		else {
			std::cerr << "ERROR: CAMERA BLOCK WAS MISSING RESOLUTION OR LENS INFO. EXITING." << std::endl;
			exit(49);
		}
	}

	if (doc.HasMember("Uav")) {
		if (doc["Uav"].HasMember("Max Velocity")) {
			float max_vel = doc["Uav"]["Max Velocity"].GetFloat();
			uav_.SetMaxVelocity(max_vel);
		}
		if (doc["Uav"].HasMember("Mass")) {
			float mass = doc["Uav"]["Mass"].GetFloat();
			uav_.SetMass(mass);
		}
		if (doc["Uav"].HasMember("Wing Area")) {
			float wing_area = doc["Uav"]["Wing Area"].GetFloat();
			uav_.SetWingArea(wing_area);
		}
		if (doc["Uav"].HasMember("Max Roll Rate (rad/s)")) {
			float mrr = doc["Uav"]["Max Roll Rate (rad/s)"].GetFloat();
			uav_.SetMaxRollRate(mrr);
		}
		if (doc["Uav"].HasMember("Max Thrust")) {
			float max_t = doc["Uav"]["Max Thrust"].GetFloat();
			uav_.SetMaxThrust(max_t);
		}
		if (doc["Uav"].HasMember("Lift Coefficient")) {
			float cl = doc["Uav"]["Lift Coefficient"].GetFloat();
			uav_.SetLiftCoeff(cl);
		}
		if (doc["Uav"].HasMember("Drag Coefficient")) {
			float cd = doc["Uav"]["Drag Coefficient"].GetFloat();
			uav_.SetDragCoeff(cd);
		}
	}

	controller_active_ = false;
	if (doc.HasMember("Controller")) {
		if (doc["Controller"].HasMember("Active")) {
			controller_active_ = doc["Controller"]["Active"].GetBool();
		}
		if (doc["Controller"].HasMember("Desired Speed")) {
			float des_speed = doc["Controller"]["Desired Speed"].GetFloat();
			controller_.SetDesiredSpeed(des_speed);
		}
		if (doc["Controller"].HasMember("Desired Altitude")) {
			desired_altitude_ = doc["Controller"]["Desired Altitude"].GetFloat();
			controller_.SetDesiredAltitude(desired_altitude_);
		}
		if (doc["Controller"].HasMember("Min Height")) {
			min_height_ = doc["Controller"]["Min Height"].GetFloat();
		}
		if (doc["Controller"].HasMember("Roll Limit Degrees")) {
			float roll_lim = doc["Controller"]["Roll Limit Degrees"].GetFloat();
			controller_.SetMaxRollRadians(3.14159265358979f * roll_lim / 180.0f);
		}
		if (doc["Controller"].HasMember("Dwell Radius")) {
			dwell_radius_ = doc["Controller"]["Dwell Radius"].GetFloat();
		}
		if (doc["Controller"].HasMember("Waypoints")) {
			std::vector<glm::vec2> waypoints;
			for (int i = 0; i < (int)doc["Controller"]["Waypoints"].Capacity(); i++) {
				if (doc["Controller"]["Waypoints"][i].Capacity() > 1) {
					glm::vec2 wp;
					wp.x = doc["Controller"]["Waypoints"][i][0].GetFloat();
					wp.y = doc["Controller"]["Waypoints"][i][1].GetFloat();
					waypoints.push_back(wp);
				}
				else {
					std::cerr << "ERROR: Waypoints must be listed as an array of x-y pairs" << std::endl;
					exit(42);
				}
			}
			if (waypoints.size() > 1) {
				use_waypoints_ = true;
				std::vector<glm::vec2> new_wp = FillInWaypointsVec2(waypoints, 10.0f);
				controller_.SetWaypoints(new_wp);
			}
		}
		if (doc["Controller"].HasMember("Speed Control")) {
			if (doc["Controller"]["Speed Control"].HasMember("P") &&
				doc["Controller"]["Speed Control"].HasMember("I") &&
				doc["Controller"]["Speed Control"].HasMember("D")) {
				float p = doc["Controller"]["Speed Control"]["P"].GetFloat();
				float i = doc["Controller"]["Speed Control"]["I"].GetFloat();
				float d = doc["Controller"]["Speed Control"]["D"].GetFloat();
				controller_.SetSpeedControllerParams(p, i, d);
			}
		}
		if (doc["Controller"].HasMember("Altitude Control")) {
			if (doc["Controller"]["Altitude Control"].HasMember("P") &&
				doc["Controller"]["Altitude Control"].HasMember("I") &&
				doc["Controller"]["Altitude Control"].HasMember("D")) {
				float p = doc["Controller"]["Altitude Control"]["P"].GetFloat();
				float i = doc["Controller"]["Altitude Control"]["I"].GetFloat();
				float d = doc["Controller"]["Altitude Control"]["D"].GetFloat();
				controller_.SetAltitudeControllerParams(p, i, d);
			}
		}
		if (doc["Controller"].HasMember("Roll Control")) {
			if (doc["Controller"]["Roll Control"].HasMember("Max Lookahead")) {
				float maxla = doc["Controller"]["Roll Control"]["Max Lookahead"].GetFloat();
				controller_.SetMaxLookahead(maxla);
			}
			if (doc["Controller"]["Roll Control"].HasMember("Min Lookahead")) {
				float minla = doc["Controller"]["Roll Control"]["Min Lookahead"].GetFloat();
				controller_.SetMinLookahead(minla);
			}
			if (doc["Controller"]["Roll Control"].HasMember("Steering Coeff")) {
				float k = doc["Controller"]["Roll Control"]["Steering Coeff"].GetFloat();
				controller_.SetSteeringCoeff(k);
			}
			if (doc["Controller"]["Roll Control"].HasMember("Goal Threshold")) {
				float gt = doc["Controller"]["Roll Control"]["Goal Threshold"].GetFloat();
				controller_.SetGoalThresh(gt);
			}
		}
	}

	uav_.SetPosition(init_pos.x, init_pos.y, init_pos.z);
	uav_.SetAirspeed(init_airspeed);
	uav_.SetHeadingRadians(init_heading);
	env->SetActorPosition(0, uav_.GetPosition(), glm::quat(cosf(0.5f * init_heading), 0.0f, 0.0f, sinf(0.5f * init_heading)));

	flight_camera_.Initialize(512, 384, 0.0035f, 0.002625f, 0.0035f);
	//flight_camera_.SetRelativePose(glm::vec3(-10.0f, 0.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	float angle = 15.0f * 3.14159f / 180.0f;
	flight_camera_.SetRelativePose(glm::vec3(-10.0f, 0.0f, 4.0f), glm::quat(cosf(0.5f * angle), 0.0f, sinf(0.5f * angle), 0.0f));
	flight_camera_.SetElectronics(0.75f, 1.0f);
}

void UavSim::LoadAnimations(mavs::raytracer::embree::EmbreeTracer* scene, mavs::environment::Environment* env) {
	mavs::MavsDataPath mdp;
	std::string mavs_data_path = mdp.GetPath();
	std::string frame_list(mavs_data_path + "/scenes/meshes/animations/GenericWalk/walk_frames.txt");
	std::string path_to_frames(mavs_data_path + "/scenes/meshes/animations/GenericWalk/");
	//animations_.resize(anim_inits_.size());
	for (int i = 0; i < (int)anim_inits_.size(); i++) {
		mavs::raytracer::Animation animation;
		animation.SetPathToMeshes(path_to_frames);
		animation.LoadFrameList(frame_list);
		animation.SetBehavior("straight");
		float z = env->GetGroundHeight(anim_inits_[i].position.x, anim_inits_[i].position.y);
		animation.SetPosition(glm::vec3(anim_inits_[i].position.x, anim_inits_[i].position.y, z + 0.01f));
		animation.SetHeading(anim_inits_[i].heading);
		animation.SetRotateYToX(true);
		animation.SetSpeed(anim_inits_[i].velocity);
		scene->AddAnimation(animation);
	}
}

std::vector<glm::vec2> UavSim::FillInWaypointsVec2(std::vector<glm::vec2> wp_in, float spacing) {
	if (wp_in.size() <= 1)return wp_in;
	std::vector<glm::vec2> wp_out;
	wp_out.push_back(wp_in[0]);

	glm::vec2 lt = wp_in[1] - wp_in[0];
	lt = lt / glm::length(lt);

	int current_wp = 0;
	int np = 0;
	while (current_wp < (int)wp_in.size() - 1) {
		glm::vec2 current_point = wp_in[current_wp] + (np * spacing) * lt;
		float d = glm::length(current_point - wp_in[current_wp + 1]);
		if (d <= spacing) {
			wp_out.push_back(wp_in[current_wp + 1]);
			current_wp++;
			np = 0;
			lt = wp_in[current_wp + 1] - wp_in[current_wp];
			lt = lt / glm::length(lt);
		}
		else {
			wp_out.push_back(current_point);
			np++;
		}
	}
	return wp_out;
}

} // namespace vehicle
} //namespace mavs