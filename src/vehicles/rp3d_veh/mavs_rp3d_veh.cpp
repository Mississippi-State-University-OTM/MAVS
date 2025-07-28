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
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <mavs_core/math/utils.h>
#include <numeric>
#include <glm/gtc/quaternion.hpp>

namespace mavs {
namespace vehicle {

Rp3dVehicle::Rp3dVehicle() {
	// solver parameters
	max_dt_ = 1.0f / 1000.0f;

	chassis_drag_coeff_ = 0.3f;

	auto_commit_animations_ = true;
	
	rp3d::PhysicsWorld::WorldSettings settings;
	settings.defaultVelocitySolverNbIterations = 20;
	settings.defaultPositionSolverNbIterations = 15;
	settings.isSleepingEnabled = false;
	settings.gravity = rp3d::Vector3(0.0, 0.0, -9.806);

	world_ = physics_common_.createPhysicsWorld(settings);
	world_->setNbIterationsVelocitySolver(20); //default is 10 
	world_->setNbIterationsPositionSolver(15); //default is 5

	external_force_ = rp3d::Vector3(0.0f, 0.0f, 0.0f);

	//chassis parameters
	cg_offset_ = 0.25f;
	cg_lateral_offset_ = 0.0f;
	cg_long_offset_ = 0.0f;
	chassis_dimensions_ = rp3d::Vector3(2.4f, 2.0f, 0.5f);
	sprung_mass_ = 1200.0f;
	torque_mass_scale_factor_ = 1.0f;

	skid_steered_ = false;
	max_governed_speed_ = 100000.0f; //m/s

	calculate_drag_forces_ = true;

	animate_tires_ = false;
	load_visualization_ = true;
	vehicle_id_num_ = 0;
}

Rp3dVehicle::~Rp3dVehicle() {
	physics_common_.destroyPhysicsWorld(world_);
}

void Rp3dVehicle::SetTerrainProperties(std::string soil_type, float soil_strength, std::string height_function, std::vector<float> height_args) {
	for (int i = 0; i < running_gear_.size(); i++) {
		running_gear_[i].GetTire()->SetTerrainHeightFunction(height_function, height_args);
		running_gear_[i].GetTire()->SetSoilType(soil_type, soil_strength);
	}
}

float Rp3dVehicle::GetPercentDeflectionOfTire(int i) {
	float d = 0.0f;
	if (i >= 0 && i < running_gear_.size()) {
		d = 100.0f*running_gear_[i].GetTire()->GetCurrentDeflection();
	}
	return d;
}

void Rp3dVehicle::Load(std::string input_file) {

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

	if (d.HasMember("Skid Steered")) {
		skid_steered_ = d["Skid Steered"].GetBool();
	}

	if (d.HasMember("Initial Pose")) {
		if (d["Initial Pose"].HasMember("Position")) {
			if (d["Initial Pose"]["Position"].Capacity() == 3) {
				current_state_.pose.position.x = d["Initial Pose"]["Position"][0].GetFloat();
				current_state_.pose.position.y = d["Initial Pose"]["Position"][1].GetFloat();
				current_state_.pose.position.z = d["Initial Pose"]["Position"][2].GetFloat();
			}
		}
		if (d["Initial Pose"].HasMember("Orientation")) {
			if (d["Initial Pose"]["Orientation"].Capacity() == 4) {
				current_state_.pose.quaternion.w = d["Initial Pose"]["Orientation"][0].GetFloat();
				current_state_.pose.quaternion.x = d["Initial Pose"]["Orientation"][1].GetFloat();
				current_state_.pose.quaternion.y = d["Initial Pose"]["Orientation"][2].GetFloat();
				current_state_.pose.quaternion.z = d["Initial Pose"]["Orientation"][3].GetFloat();
			}
		}
	}

	if (d.HasMember("Chassis")) {
		if (d["Chassis"].HasMember("Sprung Mass")) {
			sprung_mass_ = d["Chassis"]["Sprung Mass"].GetFloat();
		}
		// scale factor used to account for lacking steering torque on vehicles < 10 kg
		torque_mass_scale_factor_ = 5000.0f*expf(-sprung_mass_/3.0f) + 1.0f;
		//torque_mass_scale_factor_ = 3000.0f*exp(-sprung_mass_) + 1.0f;
		if (d["Chassis"].HasMember("CG Offset")) {
			cg_offset_ = d["Chassis"]["CG Offset"].GetFloat();
		}

		if (d["Chassis"].HasMember("CG Lateral Offset")) {
			cg_lateral_offset_ = d["Chassis"]["CG Lateral Offset"].GetFloat();
		}

		if (d["Chassis"].HasMember("CG Longitudinal Offset")) {
			cg_long_offset_ = d["Chassis"]["CG Longitudinal Offset"].GetFloat();
		}

		if (d["Chassis"].HasMember("Dimensions")) {
			if (d["Chassis"]["Dimensions"].Capacity() == 3) {
				chassis_dimensions_.x = d["Chassis"]["Dimensions"][0].GetFloat();
				chassis_dimensions_.y = d["Chassis"]["Dimensions"][1].GetFloat();
				chassis_dimensions_.z = d["Chassis"]["Dimensions"][2].GetFloat();
			}
		}
		if (d["Chassis"].HasMember("Drag Coefficient")) {
			chassis_drag_coeff_ = d["Chassis"]["Drag Coefficient"].GetFloat();
		}
	}

	if (d.HasMember("Powertrain")) {
		if (d["Powertrain"].HasMember("Final Drive Ratio")) {
			powertrain_.SetFinalDriveRatio(d["Powertrain"]["Final Drive Ratio"].GetFloat());
		}
		if (d["Powertrain"].HasMember("Speed Governor")) {
			max_governed_speed_ = d["Powertrain"]["Speed Governor"].GetFloat();
		}
		if (d["Powertrain"].HasMember("Max Engine Torque")) {
			powertrain_.SetMaxTorque(d["Powertrain"]["Max Engine Torque"].GetFloat());
		}
		if (d["Powertrain"].HasMember("Max Engine Rpm")) {
			powertrain_.SetMaxRpm(d["Powertrain"]["Max Engine Rpm"].GetFloat());
		}
		if (d["Powertrain"].HasMember("Max Braking Torque")) {
			powertrain_.SetMaxBrakingTorque(d["Powertrain"]["Max Braking Torque"].GetFloat());
		}
		if (d["Powertrain"].HasMember("Idle Rpm")) {
			powertrain_.SetIdleRpm(d["Powertrain"]["Idle Rpm"].GetFloat());
		}
	}
	
	num_tire_slices_ = 3;
	dtheta_slice_ = 2.5f;
	if (d.HasMember("Radial Spring Tire Model")) {
		if (d["Radial Spring Tire Model"].HasMember("Num Slices")) {
			num_tire_slices_ = d["Radial Spring Tire Model"]["Num Slices"].GetInt();

		}
		if (d["Radial Spring Tire Model"].HasMember("Theta Step")) {
			dtheta_slice_ = d["Radial Spring Tire Model"]["Theta Step"].GetFloat();
		}
	}

	//load all the axles
	if (d.HasMember("Axles")) {
		if (d["Axles"].Capacity() >= 2) {
			for (int i = 0; i < (int)d["Axles"].Capacity(); i++) {
				rp3d_axle axle;
				if (d["Axles"][i].HasMember("Tire")) {
					if (d["Axles"][i]["Tire"].HasMember("Spring Constant")) {
						axle.tire.spring_constant = d["Axles"][i]["Tire"]["Spring Constant"].GetFloat();
					}
					else {
						std::cerr << "WARNING: No spring constant given for axle " << i << std::endl;
						axle.tire.spring_constant = 470000.0f;
					}
					if (d["Axles"][i]["Tire"].HasMember("Damping Constant")) {
						axle.tire.damping_constant = d["Axles"][i]["Tire"]["Damping Constant"].GetFloat();
					}
					else {
						std::cerr << "WARNING: No damping constant given for axle " << i << std::endl;
						axle.tire.damping_constant = 300000.0;
					}
					if (d["Axles"][i]["Tire"].HasMember("Radius")) {
						axle.tire.radius = d["Axles"][i]["Tire"]["Radius"].GetFloat();
					}
					else {
						std::cerr << "WARNING: No radius given for axle " << i << std::endl;
						axle.tire.radius = 0.5f;
					}
					if (d["Axles"][i]["Tire"].HasMember("Width")) {
						axle.tire.width = d["Axles"][i]["Tire"]["Width"].GetFloat();
					}
					else {
						std::cerr << "WARNING: No width given for axle " << i << std::endl;
						axle.tire.width = 0.25f;
					}
					if (d["Axles"][i]["Tire"].HasMember("Section Height")) {
						axle.tire.section_height = d["Axles"][i]["Tire"]["Section Height"].GetFloat();
					}
					if (d["Axles"][i]["Tire"].HasMember("Viscous Friction Coefficient")) {
						axle.tire.viscous_fric = d["Axles"][i]["Tire"]["Viscous Friction Coefficient"].GetFloat();
					}
					else {
						std::cerr << "WARNING: No section  height given for axle " << i << std::endl;
						axle.tire.section_height = 0.25f;
					}
					/*
					if (d["Axles"][i]["Tire"].HasMember("Rolling Resistance Coeff")) {
						axle.tire.rolling_resist_coeff = d["Axles"][i]["Tire"]["Rolling Resistance Coeff"].GetFloat();
					}
					else {
						std::cerr << "WARNING: No rolling resistance given for tire " << i << std::endl;
						axle.tire.rolling_resist_coeff = 0.04f;
					}
					*/
					if (d["Axles"][i]["Tire"].HasMember("High Slip Crossover Angle")) {
						axle.tire.slip_crossover_angle = d["Axles"][i]["Tire"]["High Slip Crossover Angle"].GetFloat();
					}
					else {
						std::cerr << "WARNING: No slip crossover angle given for tire " << i << std::endl;
						axle.tire.slip_crossover_angle = 5.0f;
					}
				} // tire data
				//remainder of axle data
				if (d["Axles"][i].HasMember("Longitudinal Offset")) {
					axle.long_offset = d["Axles"][i]["Longitudinal Offset"].GetFloat();
				}
				else {
					if (i == 0) {
						axle.long_offset = 1.2f;
					}
					else if (i == 1) {
						axle.long_offset = -1.4f;
					}
					else {
						axle.long_offset = -1.4f - 1.0f*(i - 1);
					}
				}
				axle.lat_offset = 0.75f;
				if (d["Axles"][i].HasMember("Track Width")) {
					axle.lat_offset = 0.5f*d["Axles"][i]["Track Width"].GetFloat();
				}
				axle.spring_constant = 37044.0f;
				if (d["Axles"][i].HasMember("Spring Constant")) {
					axle.spring_constant = d["Axles"][i]["Spring Constant"].GetFloat();
				}
				axle.damping_constant = 1058.4f;
				if (d["Axles"][i].HasMember("Damping Constant")) {
					axle.damping_constant = d["Axles"][i]["Damping Constant"].GetFloat();
				}
				axle.spring_length = 0.3f;
				if (d["Axles"][i].HasMember("Spring Length")) {
					axle.spring_length = d["Axles"][i]["Spring Length"].GetFloat();
				}
				axle.spring_length += cg_offset_;

				axle.max_steer_angle = 5.0f;
				if (d["Axles"][i].HasMember("Max Steer Angle")) {
					axle.max_steer_angle = d["Axles"][i]["Max Steer Angle"].GetFloat();
				}
				axle.unsprung_mass = 60.0f;
				if (d["Axles"][i].HasMember("Unsprung Mass")) {
					axle.unsprung_mass = d["Axles"][i]["Unsprung Mass"].GetFloat();
				}
				axle.steered = true;
				if (d["Axles"][i].HasMember("Steered")) {
					axle.steered = d["Axles"][i]["Steered"].GetBool();
				}
				axle.powered = true;
				if (d["Axles"][i].HasMember("Powered")) {
					axle.powered = d["Axles"][i]["Powered"].GetBool();
				}
				axles_.push_back(axle);
			} // loop over axles
		}
		else {
			std::cerr << "ERROR: LESS THAN 2 AXLES LISTED IN VEHICLE FILE " << input_file << ", EXITING" << std::endl;
			exit(99);
		}
	}
	else {
		std::cerr << "ERROR: NO AXLES LISTED IN VEHICLE INPUT FILE " << input_file << ", EXITING" << std::endl;
		exit(99);
	}

	//load the associated animation
	if (d.HasMember("Mesh")) {
		anim_.file = "misc/cube.obj";
		if (d["Mesh"].HasMember("File")) {
			anim_.file = d["Mesh"]["File"].GetString();
		}
		anim_.x_to_y = false;
		if (d["Mesh"].HasMember("Rotate X to Y")) {
			anim_.x_to_y = d["Mesh"]["Rotate X to Y"].GetBool();
		}
		anim_.y_to_x = false;
		if (d["Mesh"].HasMember("Rotate Y to X")) {
			anim_.y_to_x = d["Mesh"]["Rotate Y to X"].GetBool();
		}
		anim_.y_to_z = false;
		if (d["Mesh"].HasMember("Rotate Y to Z")) {
			anim_.y_to_z = d["Mesh"]["Rotate Y to Z"].GetBool();
		}
		anim_.offset = rp3d::Vector3(0.0f, 0.0f, 0.0f);
		if (d["Mesh"].HasMember("Offset")) {
			if (d["Mesh"]["Offset"].Capacity() == 3) {
				anim_.offset.x = d["Mesh"]["Offset"][0].GetFloat();
				anim_.offset.y = d["Mesh"]["Offset"][1].GetFloat();
				anim_.offset.z = d["Mesh"]["Offset"][2].GetFloat();
			}
		}
		anim_.scale = rp3d::Vector3(1.0f, 1.0f, 1.0f);
		if (d["Mesh"].HasMember("Scale")) {
			if (d["Mesh"]["Scale"].Capacity() == 3) {
				anim_.scale.x = d["Mesh"]["Scale"][0].GetFloat();
				anim_.scale.y = d["Mesh"]["Scale"][1].GetFloat();
				anim_.scale.z = d["Mesh"]["Scale"][2].GetFloat();
			}
		}
	}
	else {
		//std::cerr << "WARNING: NO ANIMATION INFO WAS LISTED, USING DEFAULT" << std::endl;
		anim_.file = "misc/cube.obj";
		anim_.x_to_y = false;
		anim_.y_to_x = false;
		anim_.y_to_z = false;
		anim_.offset = rp3d::Vector3(0.0f, 0.0f, 0.0f);
		anim_.scale = rp3d::Vector3(1.0f, 1.0f, 1.0f);
	}

	//load the associated tires animation
	if (d.HasMember("Tire Mesh")) {
		animate_tires_ = true;
		tire_anim_.file = "misc/cube.obj";
		if (d["Tire Mesh"].HasMember("File")) {
			tire_anim_.file = d["Tire Mesh"]["File"].GetString();
		}
		tire_anim_.x_to_y = false;
		if (d["Tire Mesh"].HasMember("Rotate X to Y")) {
			tire_anim_.x_to_y = d["Tire Mesh"]["Rotate X to Y"].GetBool();
		}
		tire_anim_.y_to_x = false;
		if (d["Tire Mesh"].HasMember("Rotate Y to X")) {
			tire_anim_.y_to_x = d["Tire Mesh"]["Rotate Y to X"].GetBool();
		}
		tire_anim_.y_to_z = false;
		if (d["Tire Mesh"].HasMember("Rotate Y to Z")) {
			tire_anim_.y_to_z = d["Tire Mesh"]["Rotate Y to Z"].GetBool();
		}
		tire_anim_.offset = rp3d::Vector3(0.0f, 0.0f, 0.0f);
		if (d["Tire Mesh"].HasMember("Offset")) {
			if (d["Tire Mesh"]["Offset"].Capacity() == 3) {
				tire_anim_.offset.x = d["Tire Mesh"]["Offset"][0].GetFloat();
				tire_anim_.offset.y = d["Tire Mesh"]["Offset"][1].GetFloat();
				tire_anim_.offset.z = d["Tire Mesh"]["Offset"][2].GetFloat();
			}
		}
		tire_anim_.offset.y = tire_anim_.offset.y + cg_lateral_offset_;
		tire_anim_.scale = rp3d::Vector3(1.0f, 1.0f, 1.0f);
		if (d["Tire Mesh"].HasMember("Scale")) {
			if (d["Tire Mesh"]["Scale"].Capacity() == 3) {
				tire_anim_.scale.x = d["Tire Mesh"]["Scale"][0].GetFloat();
				tire_anim_.scale.y = d["Tire Mesh"]["Scale"][1].GetFloat();
				tire_anim_.scale.z = d["Tire Mesh"]["Scale"][2].GetFloat();
			}
		}
	}
	else {
		//std::cerr << "WARNING: NO ANIMATION INFO WAS LISTED, USING DEFAULT" << std::endl;
		tire_anim_.file = "misc/cube.obj";
		tire_anim_.x_to_y = false;
		tire_anim_.y_to_x = false;
		tire_anim_.y_to_z = false;
		tire_anim_.offset = rp3d::Vector3(0.0f, 0.0f, 0.0f);
		tire_anim_.scale = rp3d::Vector3(0.00001f, 0.00001f, 0.00001f);
	}

}

void Rp3dVehicle::AddAxle(rp3d_axle axle) {
	
	mavs_rp3d::MavsTire tire(axle.unsprung_mass, axle.tire.radius, axle.tire.width, axle.tire.section_height, axle.tire.spring_constant);
	tire.SetViscousFrictionCoeff(axle.tire.viscous_fric);
	tire.SetLateralCrossoverAngle(axle.tire.slip_crossover_angle);

	tire.SetNumSlices(num_tire_slices_);
	tire.SetDthetaSlice(dtheta_slice_);

	//running_gear_.push_back(mavs_rp3d::Suspension(&chassis_, world_, tire, axle.long_offset, axle.lat_offset, axle.spring_length, axle.spring_constant, axle.damping_constant, axle.max_steer_angle));
	//running_gear_.push_back(mavs_rp3d::Suspension(&chassis_, &physics_common_, world_, tire, axle.long_offset, axle.lat_offset, axle.spring_length, axle.spring_constant, axle.damping_constant, axle.max_steer_angle));
	running_gear_.push_back(mavs_rp3d::Suspension(&chassis_, &physics_common_, world_, tire, axle.long_offset, 
		(axle.lat_offset-cg_lateral_offset_), axle.spring_length, axle.spring_constant, axle.damping_constant, axle.max_steer_angle));
	int n = (int)running_gear_.size() - 1;
	running_gear_[n].GetTire()->SetId(n);
	running_gear_[n].SetSteered(axle.steered);
	running_gear_[n].SetPowered(axle.powered);
	//running_gear_[n].GetTire()->SetLeft(true);
	running_gear_[n].GetTire()->SetLeft(false);
	//running_gear_.push_back(mavs_rp3d::Suspension(&chassis_, &physics_common_, world_, tire, axle.long_offset, -axle.lat_offset, axle.spring_length, axle.spring_constant, axle.damping_constant, axle.max_steer_angle));
	running_gear_.push_back(mavs_rp3d::Suspension(&chassis_, &physics_common_, world_, tire, axle.long_offset, 
		(-axle.lat_offset-cg_lateral_offset_), axle.spring_length, axle.spring_constant, axle.damping_constant, axle.max_steer_angle));
	n = (int)running_gear_.size() - 1;
	running_gear_[n].SetSteered(axle.steered);
	running_gear_[n].SetPowered(axle.powered);
	//running_gear_[n].GetTire()->SetLeft(false);
	running_gear_[n].GetTire()->SetLeft(true);
	running_gear_[n].GetTire()->SetId(n);
}

void Rp3dVehicle::Init(environment::Environment *env) {
	float tire_radius = axles_[0].tire.radius;
	float spring_length = axles_[0].spring_length;
	// Set the vehicle on the ground
	float h = env->GetGroundHeight((float)current_state_.pose.position.x, (float)current_state_.pose.position.y);
	//float z = h + spring_length + cg_offset_ + 2.0f*tire_radius;
	float z = h + spring_length + cg_offset_ + tire_radius + 0.75f;
	rp3d::Vector3 pos((float)current_state_.pose.position.x + cg_long_offset_, (float)current_state_.pose.position.y, z);
	
	// find the current ground normal and vehicle yaw
	glm::vec4 h_and_n = env->GetGroundHeightAndNormal((float)current_state_.pose.position.x, (float)current_state_.pose.position.y);
	glm::vec3 ground_normal(h_and_n.x, h_and_n.y, h_and_n.z);
	ground_normal = glm::normalize(ground_normal);
	
	// adjust the pitch of the vehicle to align with the ground
	glm::mat3 R = mavs::math::GetRotationMatrixFromVectors(GetLookUp(),ground_normal);
	glm::quat new_ori(R * glm::mat3(GetOrientation()));

	current_state_.pose.quaternion = new_ori;
	glm::vec3 new_look_up = GetLookUp();

	//rp3d::Quaternion quat((float)current_state_.pose.quaternion.x, (float)current_state_.pose.quaternion.y, (float)current_state_.pose.quaternion.z, (float)current_state_.pose.quaternion.w);
	rp3d::Quaternion quat(new_ori.x, new_ori.y, new_ori.z, new_ori.w);
	quat.normalize();

	rp3d::Transform init_trans(pos, quat);
	chassis_.Init(&physics_common_, world_, init_trans, cg_offset_, chassis_dimensions_.x, chassis_dimensions_.y, chassis_dimensions_.z, sprung_mass_);

	for (int i = 0; i < axles_.size(); i++) {
		AddAxle(axles_[i]);
	}

	current_tire_forces_.resize(running_gear_.size());
	
	//int current_num_objs = env->GetNumberObjects()-2; // the -2 accounts for the surface object
	if (load_visualization_) {
		std::vector<int> actor_num = env->AddActor(anim_.file, anim_.y_to_z, anim_.x_to_y, anim_.y_to_x, glm::vec3(anim_.offset.x, anim_.offset.y, anim_.offset.z), glm::vec3(anim_.scale.x, anim_.scale.y, anim_.scale.z));
		vehicle_id_num_ = env->GetNumberOfActors() - 1;// actor_num.back(); // -current_num_objs;
	}
	else {
		if (animate_tires_) {
			vehicle_id_num_ = env->GetNumberOfActors() - 5;
		}
		else {
			vehicle_id_num_ = env->GetNumberOfActors() - 1;
		}
	}
	int current_tire_id = vehicle_id_num_ + 1;
	if (animate_tires_){
		for (int i = 0; i < 2*axles_.size(); i++) {
			if (load_visualization_){
				std::vector<int> tire_num = env->AddActor(tire_anim_.file, tire_anim_.y_to_z, tire_anim_.x_to_y, tire_anim_.y_to_x, glm::vec3(tire_anim_.offset.x, tire_anim_.offset.y, tire_anim_.offset.z), glm::vec3(tire_anim_.scale.x, tire_anim_.scale.y, tire_anim_.scale.z));
				tire_id_nums_.push_back(env->GetNumberOfActors() - 1);
			}
			else {
				//tire_id_nums_.push_back(tire_num.back()-current_num_objs);
				tire_id_nums_.push_back(current_tire_id);
				current_tire_id++;
			}
		}
	}
	
}

void Rp3dVehicle::CalculateWheelTorques(float current_velocity, float throttle, float brake, float steering) {
	for (int i = 0; i < running_gear_.size(); i++) {
		float torque = 0.0f;
		if (!skid_steered_) {
			torque = powertrain_.CalculateWheelTorque(current_velocity, throttle, brake, running_gear_[i].GetTire()->GetRadius(), running_gear_[i].IsPowered());
		}
		else { 
			// model to mix throttle with steering for skid steered vehicle
			float throt_steer = 0.0f;
			if (i % 2 == 0) { // left side
				throt_steer = (1.0f - fabs(steering))*throttle - steering * (1.0f - 0.5f*throttle);
			}
			else { // right side
				throt_steer = (1.0f - fabs(steering))*throttle + steering * (1.0f - 0.5f*throttle);
			}
			torque = powertrain_.CalculateWheelTorque(current_velocity, throt_steer, brake, running_gear_[i].GetTire()->GetRadius(), running_gear_[i].IsPowered());
		}
		running_gear_[i].SetCurrentAppliedTorque(torque);
	}
}

void Rp3dVehicle::CalculateSteeringAngles(float dt, float steering_factor) {
	float max_steering_rate = 0.785f; //3.14f; // Max change of pi radians per second
	float max_ds = dt * max_steering_rate;
	for (int i = 0; i < running_gear_.size(); i++) {
		if (running_gear_[i].IsSteered()) {
			float steering = steering_factor * running_gear_[i].GetMaxSteeringAngle();
			float ds = steering - running_gear_[i].GetCurrentSteeringAngle();
			float absds = fabsf(ds);
			if (absds > max_ds)ds = max_ds * (ds / absds);
			float new_steering = running_gear_[i].GetCurrentSteeringAngle() + ds;
			new_steering = std::min(running_gear_[i].GetMaxSteeringAngle(), std::max(-running_gear_[i].GetMaxSteeringAngle(), new_steering));
			running_gear_[i].SetCurrentSteeringAngle(new_steering);
		}
	}
}

float Rp3dVehicle::GetLongVelSlope() {
	const auto n = time_trace_.size();
	const auto s_x = std::accumulate(time_trace_.begin(), time_trace_.end(), 0.0);
	const auto s_y = std::accumulate(long_vel_trace_.begin(), long_vel_trace_.end(), 0.0);
	const auto s_xx = std::inner_product(time_trace_.begin(), time_trace_.end(), time_trace_.begin(), 0.0);
	const auto s_xy = std::inner_product(time_trace_.begin(), time_trace_.end(), long_vel_trace_.begin(), 0.0);
	const auto a = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
	return a;
}

float Rp3dVehicle::GetLongitudinalAcceleration() {
	glm::vec3 la = current_state_.accel.linear;
	glm::vec3 lv = current_state_.twist.linear;
	glm::vec3 lt = GetLookTo();
	float long_accel = glm::dot(la, lt);
	if (long_vel_trace_.size() >= 3) {
		long_accel = GetLongVelSlope();
	}
	return long_accel;
}

void Rp3dVehicle::SetState(VehicleState veh_state) {
	current_state_ = veh_state;
	chassis_.SetPositionOrientation(current_state_.pose.position.x, current_state_.pose.position.y, current_state_.pose.position.z, current_state_.pose.quaternion.w, current_state_.pose.quaternion.x, current_state_.pose.quaternion.y, current_state_.pose.quaternion.z);
	chassis_.SetLinearAngularVelocity(current_state_.twist.linear.x, current_state_.twist.linear.y, current_state_.twist.linear.z, current_state_.twist.angular.x, current_state_.twist.angular.y, current_state_.twist.angular.z);
}

static glm::vec3 ComputeAngularVelocity(const glm::quat& q1, const glm::quat& q2, float dt) {

	glm::quat q_rel = q2*glm::conjugate(q1);
	q_rel = glm::normalize(q_rel);

	// Convert to axis-angle
	float angle = 2.0f * acosf(q_rel.w);
	float sin_half_angle = sqrtf(1.0f - q_rel.w * q_rel.w);
	glm::vec3 axis(0.0f, 0.0f, 0.0f);
	if (sin_half_angle > 1.0E-10f) {
		axis = glm::vec3(q_rel.x, q_rel.y, q_rel.z) / sin_half_angle;
	}
	glm::vec3 omega = (angle / dt) * axis;
	return omega;
}

void Rp3dVehicle::Update(environment::Environment *env, float throttle, float steer, float brake, float dt) {
	//Initialize the vehicle on the first time step
	if (local_sim_time_ <= 0.0) {
		Init(env);
	}
	float dt_in = dt;
	int nsteps = 1;
	if (dt > max_dt_) {
		nsteps = (int)roundf(dt / max_dt_);
		dt = max_dt_;
	}
	chassis_.GetBody()->setIsActive(true);
	for (int i = 0; i < nsteps; i++) {
		world_->update(dt);
		float longitudinal_velocity = chassis_.GetBody()->getLinearVelocity().dot(chassis_.GetLookTo());
		if (longitudinal_velocity > max_governed_speed_)throttle = 0.0; // set the speed governor
		CalculateWheelTorques(longitudinal_velocity, throttle, brake, steer);
		if (!skid_steered_)CalculateSteeringAngles(dt, steer);
		ApplyGroundForces(env, dt, throttle, brake, steer);
		ApplySuspensionForces();
		if (calculate_drag_forces_) ApplyDragForces(longitudinal_velocity);
		chassis_.GetBody()->applyForceToCenterOfMass(external_force_);
		chassis_.GetBody()->applyForceAtWorldPosition(extern_force_at_point_, extern_force_application_point_);
	}

	float delta_t = dt * nsteps;


	long_vel_trace_.push_back(glm::dot((glm::vec3)current_state_.twist.linear, GetLookTo()));
	if (long_vel_trace_.size() > 50)long_vel_trace_.erase(long_vel_trace_.begin());
	time_trace_.push_back(local_sim_time_);
	if (time_trace_.size() > 50)time_trace_.erase(time_trace_.begin());

	current_state_.accel.linear.x = (chassis_.GetBody()->getLinearVelocity().x - current_state_.twist.linear.x) / delta_t;
	current_state_.accel.linear.y = (chassis_.GetBody()->getLinearVelocity().y - current_state_.twist.linear.y) / delta_t;
	current_state_.accel.linear.z = (chassis_.GetBody()->getLinearVelocity().z - current_state_.twist.linear.z) / delta_t;
	current_state_.accel.angular.x = (chassis_.GetBody()->getAngularVelocity().x - current_state_.twist.angular.x) / delta_t;
	current_state_.accel.angular.y = (chassis_.GetBody()->getAngularVelocity().y - current_state_.twist.angular.y) / delta_t;
	current_state_.accel.angular.z = (chassis_.GetBody()->getAngularVelocity().z - current_state_.twist.angular.z) / delta_t;
	
	glm::quat old_orientation = current_state_.pose.quaternion;
	current_state_.pose.position.x = chassis_.GetPosition().x;
	current_state_.pose.position.y = chassis_.GetPosition().y;
	current_state_.pose.position.z = chassis_.GetPosition().z;
	current_state_.pose.quaternion.w = chassis_.GetOrientation().w;
	current_state_.pose.quaternion.x = chassis_.GetOrientation().x;
	current_state_.pose.quaternion.y = chassis_.GetOrientation().y;
	current_state_.pose.quaternion.z = chassis_.GetOrientation().z;
	current_state_.twist.linear.x = chassis_.GetBody()->getLinearVelocity().x;
	current_state_.twist.linear.y = chassis_.GetBody()->getLinearVelocity().y;
	current_state_.twist.linear.z = chassis_.GetBody()->getLinearVelocity().z;
	current_state_.twist.angular = ComputeAngularVelocity(old_orientation, current_state_.pose.quaternion, dt_in);

	// update the actor position in the environment
	env->SetActorPosition(vehicle_id_num_, current_state_.pose.position, current_state_.pose.quaternion, auto_commit_animations_);
	env->SetActorVelocity(vehicle_id_num_, current_state_.twist.linear);
	if (animate_tires_) {
		for (int tn = 0; tn < (int)tire_id_nums_.size(); tn++) {
			env->SetActorPosition(tire_id_nums_[tn], GetTirePosition(tn), GetTireOrientation(tn), auto_commit_animations_);
			env->SetActorVelocity(tire_id_nums_[tn], current_state_.twist.linear);
		}
	}
	
	local_sim_time_ += delta_t;
}

void Rp3dVehicle::ApplyDragForces(float velocity) {
	float c_d = chassis_drag_coeff_; // 0.3f;
	float c_r = 0.00004f;
	float frontal_area = chassis_dimensions_.z*chassis_dimensions_.y; 
	float rho = 1.23f;
	float f_a = 0.5f*rho*frontal_area*c_d*velocity*velocity;
	float f_r = chassis_.GetBody()->getMass()*9.81f*c_r*velocity;
	rp3d::Vector3 look_to = chassis_.GetLookTo();
	rp3d::Vector3 drag_force = (-f_a - f_r)*look_to;
	chassis_.GetBody()->applyForceToCenterOfMass(drag_force);
}

void Rp3dVehicle::ApplySuspensionForces() {
	rp3d::Vector3 chass_vel = chassis_.GetBody()->getLinearVelocity();
	rp3d::Vector3 chass_pos = chassis_.GetPosition();
	rp3d::Matrix3x3 rot_mat = chassis_.GetOrientation().getMatrix();
	rp3d::Vector3 look_up = rot_mat.getColumn(2);
	look_up.normalize();
	for (int i = 0; i < running_gear_.size(); i++) {
		rp3d::Vector3 t_vel = running_gear_[i].GetBody()->getLinearVelocity();
		rp3d::Vector3 rel_vel = t_vel - chass_vel;
		rp3d::Vector3 t_pos = running_gear_[i].GetPosition();
		float v = -rel_vel.dot(look_up);
		float x = -running_gear_[i].GetTranslation();
		float f = -running_gear_[i].GetSpringConstant()*x - running_gear_[i].GetDampingConstant()*v;
		rp3d::Vector3 force = rp3d::decimal(f)*look_up;
		chassis_.GetBody()->applyForceAtWorldPosition(force, t_pos);
		running_gear_[i].GetBody()->applyForceToCenterOfMass(-force);
	}
}

void Rp3dVehicle::ApplyGroundForces(environment::Environment *env, float dt, float throttle, float brake, float steering) {
	for (int i = 0; i < running_gear_.size(); i++) {
		rp3d::Vector3 force = running_gear_[i].GetTire()->Update(env, dt, running_gear_[i].GetBody()->getTransform(), running_gear_[i].GetBody()->getLinearVelocity(), running_gear_[i].GetCurrentWheelTorque(), running_gear_[i].GetCurrentSteeringAngle());
		running_gear_[i].GetBody()->applyForceToCenterOfMass(force);
		if (skid_steered_) {
			// this section gives smoother turning for skid-steered vehicles
			rp3d::Vector3 r = running_gear_[i].GetBody()->getTransform().getPosition() - chassis_.GetBody()->getTransform().getPosition();
			// the factor of 15 is tuned parameter to give natural motion to the skid-steered vehicle
			rp3d::Vector3 torque = torque_mass_scale_factor_*15.0f*r.cross(force);
			chassis_.GetBody()->applyTorque(torque);
		}
		else {
			// this section gives smoother turning for skid-steered vehicles
			rp3d::Vector3 r = running_gear_[i].GetBody()->getTransform().getPosition() - chassis_.GetBody()->getTransform().getPosition();
			// the torque_mass_scale_factor_ = 1 for vehicles >> 10 kg
			rp3d::Vector3 torque = torque_mass_scale_factor_*r.cross(force);
			chassis_.GetBody()->applyTorque(torque);
		}
		current_tire_forces_[i] = glm::vec3(force.x, force.y, force.z);
	}
}

glm::vec3 Rp3dVehicle::GetTireForces(int tire_id) {
	glm::vec3 force(0.0f, 0.0f, 0.0f);
	if (tire_id >= 0 && tire_id < (int)current_tire_forces_.size()) {
		force = current_tire_forces_[tire_id];
	}
	return force;
}

float Rp3dVehicle::GetTireAngularVelocity(int tire_id) {
	float angular_velocity = 0.0f; 
	if (tire_id >= 0 && tire_id < (int)running_gear_.size()) {
		angular_velocity = running_gear_[tire_id].GetTire()->GetAngularVelocity();
	}
	return angular_velocity;
}


float Rp3dVehicle::GetTireSlip(int tire_id) {
	float slip = 0.0f;
	if (tire_id >= 0 && tire_id < (int)running_gear_.size()) {
		slip = running_gear_[tire_id].GetTire()->GetSlip();
	}
	return slip;
}

float Rp3dVehicle::GetTireSteeringAngle(int tire_id) {
	float angle = 0.0f;
	if (tire_id >= 0 && tire_id < (int)running_gear_.size()) {
		angle = running_gear_[tire_id].GetCurrentSteeringAngle();
	}
	return angle;
}


}// namespace vehicle
}// namespace mavs