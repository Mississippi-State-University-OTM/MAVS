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
// these have to be first because of conflicting typedefs
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
// class definition
#include <vehicles/chrono/chrono_wheeled_json.h>
// mavs utilities
#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>

namespace mavs {
namespace vehicle {

ChronoWheeledJson::ChronoWheeledJson() {
	time_ = 0.0;
	vehicle_ = NULL;
	terrain_ = NULL;
	powertrain_ = NULL;
	max_dt_ = 2.5E-3;
	current_state_.pose.position.x = 0.0; 
	current_state_.pose.position.y = 0.0;
	current_state_.pose.position.z = 1.5;
	current_state_.pose.quaternion.w = 1.0;
	current_state_.pose.quaternion.x = 0.0;
	current_state_.pose.quaternion.y = 0.0;
	current_state_.pose.quaternion.z = 0.0;
	driver_inputs_.m_throttle = 0.0;
	driver_inputs_.m_steering = 0.0;
	driver_inputs_.m_braking = 0.0;
}

ChronoWheeledJson::~ChronoWheeledJson() {
	if (vehicle_ != NULL) {
		delete vehicle_;
		vehicle_ = NULL;
	}
	if (terrain_ != NULL) {
		delete terrain_;
		terrain_ = NULL;
	}
	if (powertrain_ != NULL) {
		//delete powertrain_;
		powertrain_ = NULL;
	}
}

void ChronoWheeledJson::GetInputFiles(std::string input_file) {
	FILE* fp = fopen(input_file.c_str(), "rb");
	if (fp == NULL) {
		std::cerr << "Could not open file " << input_file << std::endl;
	}
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	MavsDataPath mavs_data_path;
	std::string data_path = mavs_data_path.GetChronoPath(); 
	chrono::vehicle::SetDataPath(data_path);

	if (d.HasMember("Vehicle File")) {
		vehicle_file_ = d["Vehicle File"].GetString();
	}
	else {
		std::cerr << "Vehicle File not provided, using default." << std::endl;
#if defined(_WIN32) || defined(WIN32)
		vehicle_file_ = "hmmwv\\vehicle\\HMMWV_Vehicle.json";
#else
		vehicle_file_ = "hmmwv/vehicle/HMMWV_Vehicle.json";
#endif
	}

	if (d.HasMember("Terrain File")) {
		terrain_file_ = d["Terrain File"].GetString();
	}
	else {
		std::cerr << "Terrain File not provided, using default." << std::endl;
#if defined(_WIN32) || defined(WIN32)
		terrain_file_ = "terrain\\RigidPlane.json";
#else
		terrain_file_ = "terrain/RigidPlane.json";
#endif
	}

	if (d.HasMember("Powertrain File")) {
		powertrain_file_ = d["Powertrain File"].GetString();
	}
	else {
		std::cerr << "Powertrain File not provided, using default." << std::endl;
#if defined(_WIN32) || defined(WIN32)
		powertrain_file_ = "generic\\powertrain\\SimplePowertrain.json";
#else
		powertrain_file_ = "generic/powertrain/SimplePowertrain.json";
#endif
	}

	if (d.HasMember("Tire File")) {
		tire_file_ = d["Tire File"].GetString();
	}
	else {
		std::cerr << "Tire File not provided, using default." << std::endl;
#if defined(_WIN32) || defined(WIN32)
		tire_file_ = "generic\\tire\\RigidTire.json";
#else
		tire_file_ = "generic/tire/RigidTire.json";
#endif
	}
}

void ChronoWheeledJson::Load(std::string input_file) {

	GetInputFiles(input_file);

	chrono::ChVector<> initLoc(current_state_.pose.position.x,
		current_state_.pose.position.y,
		current_state_.pose.position.z);
	chrono::ChQuaternion<> initRot(current_state_.pose.quaternion.w,
		current_state_.pose.quaternion.x,
		current_state_.pose.quaternion.y,
		current_state_.pose.quaternion.z);

	MavsDataPath mavs_data_path;

	vehicle_ = new chrono::vehicle::WheeledVehicle((mavs_data_path.GetChronoPath()+vehicle_file_), chrono::ChContactMethod::NSC);

	vehicle_->Initialize(chrono::ChCoordsys<>(initLoc, initRot));
	vehicle_->GetChassis()->SetFixed(false);

	terrain_ = new chrono::vehicle::RigidTerrain(vehicle_->GetSystem(),chrono::vehicle::GetDataFile(terrain_file_));
	terrain_->Initialize();

	powertrain_ = chrono::vehicle::ReadPowertrainJSON(chrono::vehicle::GetDataFile(powertrain_file_));
	vehicle_->InitializePowertrain(powertrain_);

	for (auto& axle : vehicle_->GetAxles()) {
		for (auto& wheel : axle->GetWheels()) {
			auto tire = chrono::vehicle::ReadTireJSON(chrono::vehicle::GetDataFile(tire_file_));
			vehicle_->InitializeTire(tire, wheel, chrono::vehicle::VisualizationType::MESH);
		}
	}

} // Load

/*//Copied from ChTire.cpp in chrono library
static TireKinematicState CalculateKinematics(const chrono::vehicle::WheelState& state, double tire_radius, glm::vec3 terrain_normal) {
	// Wheel normal (expressed in global frame)
	chrono::ChVector<> wheel_normal = state.rot.GetYaxis();

	// Terrain normal at wheel location (expressed in global frame)
	chrono::ChVector<> Z_dir(terrain_normal.x, terrain_normal.y, terrain_normal.z);
	// terrain.GetNormal(state.pos.x(), state.pos.y());

	// Longitudinal (heading) and lateral directions, in the terrain plane
	chrono::ChVector<> X_dir = Vcross(wheel_normal, Z_dir);
	X_dir.Normalize();
	chrono::ChVector<> Y_dir = Vcross(Z_dir, X_dir);

	// Tire reference coordinate system
	chrono::ChMatrix33<> rot;
	rot.Set_A_axis(X_dir, Y_dir, Z_dir);
	chrono::ChCoordsys<> tire_csys(state.pos, rot.Get_A_quaternion());

	// Express wheel linear velocity in tire frame
	chrono::ChVector<> V = tire_csys.TransformDirectionParentToLocal(state.lin_vel);
	// Express wheel normal in tire frame
	chrono::ChVector<> n = tire_csys.TransformDirectionParentToLocal(wheel_normal);

	// Slip angle
	double abs_Vx = std::abs(V.x());
	double zero_Vx = 1e-6;

	TireKinematicState out_state;

	out_state.slip_angle = (abs_Vx > zero_Vx) ? std::atan(V.y() / abs_Vx) : 0;

	// Longitudinal slip
	out_state.slip = (abs_Vx > zero_Vx) ? -(V.x() - state.omega * tire_radius) / abs_Vx : 0;

	// Camber angle
	out_state.camber_angle = std::atan2(n.z(), n.y());
	return out_state;
}*/


glm::vec3 ChronoWheeledJson::GetTirePosition(int i) {
	chrono::ChVector<> pos;
	if (i == 0) {
		pos	= vehicle_->GetWheel(0, chrono::vehicle::VehicleSide::LEFT)->GetPos();
	}
	else if (i == 1) {
		pos = vehicle_->GetWheel(0, chrono::vehicle::VehicleSide::RIGHT)->GetPos();
	}
	else if (i == 2) {
		pos = vehicle_->GetWheel(1, chrono::vehicle::VehicleSide::LEFT)->GetPos();
	}
	else if (i == 3) {
		pos = vehicle_->GetWheel(1, chrono::vehicle::VehicleSide::RIGHT)->GetPos();
	}

	glm::vec3 tire_pos(pos.x(), pos.y(), pos.z());
	return tire_pos;
}

glm::vec3 ChronoWheeledJson::GetTireTerrainForces(int i) {
	chrono::ChVector<> force;
	if (i == 0) {
		force = vehicle_->GetWheel(0, chrono::vehicle::VehicleSide::LEFT)->GetTire()->ReportTireForce(terrain_).force;
	}
	else if (i == 1) {
		force = vehicle_->GetWheel(0, chrono::vehicle::VehicleSide::RIGHT)->GetTire()->ReportTireForce(terrain_).force;
	}
	else if (i == 2) {
		force = vehicle_->GetWheel(1, chrono::vehicle::VehicleSide::LEFT)->GetTire()->ReportTireForce(terrain_).force;
	}
	else if (i == 3) {
		force = vehicle_->GetWheel(1, chrono::vehicle::VehicleSide::RIGHT)->GetTire()->ReportTireForce(terrain_).force;
	}

	glm::vec3 f(force.x(), force.y(), force.z());
	return f;
}

glm::quat ChronoWheeledJson::GetTireOrientation(int i) {
	chrono::ChQuaternion<> rot;
	if (i == 0) {
		rot = vehicle_->GetWheel(0, chrono::vehicle::VehicleSide::LEFT)->GetState().rot;
	}
	else if (i == 1) {
		rot = vehicle_->GetWheel(0, chrono::vehicle::VehicleSide::RIGHT)->GetState().rot;
	}
	else if (i == 2) {
		rot = vehicle_->GetWheel(1, chrono::vehicle::VehicleSide::LEFT)->GetState().rot;
	}
	else if (i == 3) {
		rot = vehicle_->GetWheel(1, chrono::vehicle::VehicleSide::RIGHT)->GetState().rot;
	}
	glm::quat tire_rot(rot.e0(), rot.e1(), rot.e2(), rot.e3());
	return tire_rot;
}

void ChronoWheeledJson::Update(mavs::environment::Environment *env, float throttle, float steer, float brake, float dt) {
	
	int num_steps = 1;
	double step_size = dt;
	if (dt > max_dt_) {
		num_steps = (int)ceil(dt / max_dt_);
		step_size = max_dt_;
	}

	throttle = mavs::math::clamp(throttle, 0.0f, 1.0f);
	float max_dsteer = 2.0f*dt;
	float max_dthrottle = 0.5f*dt;
	// The next lines ensure smooth changes to the throttle and steering
	if (throttle > 0.0) {
		float tdiff = throttle - driver_inputs_.m_throttle;
		if (tdiff <= 0.0) {
			driver_inputs_.m_throttle = throttle;
		}
		else {
			if (tdiff > max_dthrottle)tdiff = max_dthrottle;
			driver_inputs_.m_throttle += tdiff;
		}
	}
	else {
		driver_inputs_.m_throttle = 0.0;
	}
	if (std::fabs(steer) > 0) {
		float sdiff = steer - driver_inputs_.m_steering;
		if (std::fabs(sdiff) > max_dsteer)sdiff = mavs::math::get_sign(sdiff)*max_dsteer;
		driver_inputs_.m_steering += sdiff;
	}
	else {
		driver_inputs_.m_steering = 0.75*driver_inputs_.m_steering;
	}
	driver_inputs_.m_braking = brake;

	for (int step_count = 0; step_count < num_steps; step_count++) {

		double time = vehicle_->GetSystem()->GetChTime();
		vehicle_->Synchronize(time, driver_inputs_, *terrain_);
		terrain_->Synchronize(time);

		// Advance simulation for one timestep for all modules
		vehicle_->Advance(step_size);
		terrain_->Advance(step_size);

	} // for step count

	//Copy all the data over to the MAVS structures;
	current_steering_radians_ = 0.0f; // need to figure out how to extract this from chrono
	chrono::ChVector<> pos = vehicle_->GetVehiclePos();
	std::shared_ptr<chrono::vehicle::ChChassis> chassis = vehicle_->GetChassis();
	
	chrono::ChQuaternion<> rot = vehicle_->GetVehicleRot();
	glm::quat mavs_rot((float)rot.e0(), (float)rot.e1(), (float)rot.e2(), (float)rot.e3());
	current_state_.pose.position.x = pos.x();
	current_state_.pose.position.y = pos.y();
	current_state_.pose.position.z = pos.z();
	current_state_.pose.quaternion = mavs_rot;
	chrono::ChVector<> vel = vehicle_->GetVehiclePointVelocity(vehicle_->GetVehiclePos());
	current_state_.twist.linear.x = vel.x(); 
	current_state_.twist.linear.y = vel.y(); 
	current_state_.twist.linear.z = vel.z(); 
	local_sim_time_ = time_;
	if (log_data_) WriteState();
	updated_ = true;
}

} //namespace vehicle
} //namespace mavs