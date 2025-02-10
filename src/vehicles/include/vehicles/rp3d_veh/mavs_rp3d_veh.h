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
* \class Rp3dVehicle
*
* Class that implements a vehicle using the 
* ReactPhysics3D physics engine.
* https://www.reactphysics3d.com/
* https://github.com/DanielChappuis/reactphysics3d?
*
* \author Chris Goodin
*
* \date 9/13/2019
*/
#ifndef MAVS_RP3D_VEHICLE
#define MAVS_RP3D_VEHICLE
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <iostream>
#include <vector>
#ifdef None
#undef None
#endif
#include <vehicles/rp3d_veh/mavs_rp3d_chassis.h>
#include <vehicles/rp3d_veh/mavs_rp3d_suspension.h>
#include <vehicles/rp3d_veh/mavs_powertrain.h>
#include <reactphysics3d/reactphysics3d.h>
#include <vehicles/vehicle.h>
#include <rapidjson/document.h>

namespace mavs {
namespace vehicle {

/**
* Structure holding tire data prior to initialization.
*/
struct rp3d_tire {
	float spring_constant;
	float damping_constant;
	//float rolling_resist_coeff;
	float radius;
	float width;
	float section_height;
	float slip_crossover_angle;
	float viscous_fric;
};

/**
* Structure holding axle data prior to initialization.
*/
struct rp3d_axle {
	rp3d_tire tire;
	float long_offset;
	float lat_offset;
	float spring_constant;
	float damping_constant;
	float spring_length;
	float max_steer_angle;
	float unsprung_mass;
	bool powered;
	bool steered;
};

/**
* Structure holding animation data prior to initialization.
*/
struct rp3d_anim {
	std::string file;
	bool y_to_z;
	bool x_to_y;
	bool y_to_x;
	rp3d::Vector3 offset;
	rp3d::Vector3 scale;
};

class Rp3dVehicle : public Vehicle {
public:
	/// Constructor
	Rp3dVehicle();

	/// Destructor
	~Rp3dVehicle();

	/// Copy constructor
	Rp3dVehicle(const Rp3dVehicle &veh) {
		//world_ = new rp3d::DynamicsWorld(rp3d::Vector3(0.0f, 0.0f, -9.81f));
		//physics_common_ = new rp3d::PhysicsCommon;
		world_ = physics_common_.createPhysicsWorld();
		world_->setNbIterationsVelocitySolver(15); //default is 10 
		world_->setNbIterationsPositionSolver(8); //default is 5
		max_dt_ = veh.max_dt_;
		calculate_drag_forces_ = veh.calculate_drag_forces_;
		chassis_drag_coeff_ = veh.chassis_drag_coeff_;
		skid_steered_ = veh.skid_steered_;
		max_governed_speed_ = veh.max_governed_speed_;
		sprung_mass_ = veh.sprung_mass_;
		cg_offset_ = veh.cg_offset_;
		cg_lateral_offset_ = veh.cg_lateral_offset_;
		cg_long_offset_ = veh.cg_long_offset_;
		chassis_dimensions_ = veh.chassis_dimensions_;
		auto_commit_animations_ = veh.auto_commit_animations_;
		animate_tires_ = veh.animate_tires_;
		dtheta_slice_ = veh.dtheta_slice_;
		load_visualization_ = veh.load_visualization_;
		num_tire_slices_ = veh.num_tire_slices_;
		torque_mass_scale_factor_ = veh.torque_mass_scale_factor_;
		vehicle_id_num_ = veh.vehicle_id_num_;
	}

	/**
	* Load a vehicle input file
	* \param input_file Full path to the vehicle input file
	*/
	void Load(std::string input_file);

	/**
	* Method to update the state of the vehicle.
	* Inherited from MAVS vehicle base class.
	* \param env Reference to the current environment.
	* \param throttle Commanded throttle from 0 to 1.
	* \param steer Commanded steering angle (radians).
	* \param dt The time step for the update
	*/
	void Update(environment::Environment *env, float throttle, float steer, float brake, float dt);

	/**
	* Set the maximum internal timestep. If a larger step is 
	* requested in the Update method, it will subdivided into smaller steps.
	* Default is 0.005 seconds.
	* \param dt The maximum time step in seconds
	*/
	void SetMaxDt(float dt) { max_dt_ = dt; }

	/**
	* Manually set the state of the vehicle in cartesian ENU coordinates.
	*/
	void SetState(VehicleState veh_state);

	/**
	* Set the soil type and strength. Available soil types are
	* 'snow', 'ice', 'wet', 'sand', 'clay', 'paved'
	* The soil strength param is in RCI and is only used when the type is 'clay' or 'sand'
	* AND
	* Set the terrain height function. The available terrain types are
	* 'flat', 'sloped', 'sine', and 'rough'. The second argument is a list of
	* parameters for the height model.
	* flat: args[0] = terrain height
	* sloped: args[0] = fractional slope (1 = 45 degrees)
	* sine: args[0] = wavelength in meters, args[1] = magnitude of oscillation
	* rough: args[0] = wavelength of roughness in meters, args[1] = magnitude of roughness, in meters
	* \param soil_type The soil type, see above
	* \param soil_strength The soil strength, see above
	* \param height_function The height model used, see above
	* \param height_args Parameters for the height model, see above
	*/
	void SetTerrainProperties(std::string soil_type, float soil_strength, std::string height_function, std::vector<float> height_args);

	/**
	* Get the percent deflection of the ith tired, numbered left to right, front to rear
	* Returns zero if the tire index is out of range
	* Index starts at zero from front driver side, 1 for front passenger side, and so on
	* \param i The tire index
	*/
	float GetPercentDeflectionOfTire(int i);

	/**
	* Set the magnitude of gravity in the simulation.
	* Should be a positive number, the direction will 
	* be straight down in ENU coordinates.
	* \param g The acceleration due to gravity in m/s^2
	*/
	void SetGravity(float g) {
		rp3d::Vector3 grav(0.0f, 0.0f, -g);
		world_->setGravity(grav);
	}

	/**
	* Set the direction and magnitude of gravity in the simulation.
	* These will be in ENU coordinates.
	* Could be used for driving on a sphere where gravity always 
	* points from your position in ENU to the sphere center.
	* \param x The x-acceleration due to gravity in m/s^2
	* \param y The y-acceleration due to gravity in m/s^2
	* \param z The z-acceleration due to gravity in m/s^2
	*/
	void SetGravity(float x, float y, float z) {
		rp3d::Vector3 grav(x, y, z);
		world_->setGravity(grav);
	}

	/// Get the chassis dimensions in meters, in vehicle coordinate frame
	glm::vec3 GetChassisDimensions() {
		glm::vec3 d(chassis_dimensions_.x, chassis_dimensions_.y, chassis_dimensions_.z);
		return d;
	}

	/// Return the number of tires on the vehicle
	int GetNumTires() { return (int)running_gear_.size(); }

	/// Get a pointer to the ith tire
	mavs_rp3d::MavsTire *GetTire(int i) {
		if (i >= 0 && i < running_gear_.size() && running_gear_.size()>0) {
			return running_gear_[i].GetTire();
		}
		else {
			return NULL;
		}
	}

	mavs_rp3d::Suspension *GetSuspension(int i) {
		if (i >= 0 && i < running_gear_.size() && running_gear_.size()>0) {
			return &running_gear_[i];
		}
		else {
			return NULL;
		}
	}

	/// Get position of the ith tire
	glm::vec3 GetTirePosition(int i) {
		if (i >= 0 && i < running_gear_.size() && running_gear_.size()>0) {
			return glm::vec3(running_gear_[i].GetPosition().x, running_gear_[i].GetPosition().y, running_gear_[i].GetPosition().z);
		}
		else {
			return glm::vec3(0.0f, 0.0f, 0.0f);
		}
	}

	glm::vec3 GetTrailerTirePosition(int i) {
		if (i >= 0 && i < trailer_running_gear_.size() && trailer_running_gear_.size()>0) {
			return glm::vec3(trailer_running_gear_[i].GetPosition().x, trailer_running_gear_[i].GetPosition().y, trailer_running_gear_[i].GetPosition().z);
		}
		else {
			return glm::vec3(0.0f, 0.0f, 0.0f);
		}
	}

	/// Get orientation of the ith tire
	glm::quat GetTireOrientation(int i) {
		if (i >= 0 && i < running_gear_.size() && running_gear_.size()>0) {
			rp3d::Quaternion tq = running_gear_[i].GetTire()->GetCurrentOrientation();
			glm::quat q(tq.w, tq.x, tq.y, tq.z);
			return q;
		}
		else {
			return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		}
	}

	glm::quat GetTrailerTireOrientation(int i) {
		if (i >= 0 && i < trailer_running_gear_.size() && trailer_running_gear_.size()>0) {
			rp3d::Quaternion tq = trailer_running_gear_[i].GetTire()->GetCurrentOrientation();
			glm::quat q(tq.w, tq.x, tq.y, tq.z);
			return q;
		}
		else {
			return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		}
	}

	/// Get the offset of the CG above the running gear
	float GetCgOffset() { return cg_offset_; }

	/// Get the offset of the CG above the running gear
	float GetCgLateralOffset() { return cg_lateral_offset_; }

	/// Get the longitudinal (+X) offset of the CG above the running gear
	float GetCgLongitudinalOffset() { return cg_long_offset_; }

	/// Get the distance from the front axle to cg, positive to the rear of vehicle
	float GetCgLongOffset() { return axles_[0].long_offset; }

	/// Return the name of the mesh file associated with the vehicle
	std::string GetMeshFile() { return anim_.file; }

	/**
	* Make the vehicle skid steered or not.
	* By default, skid_steered=false and vehicles are ackerman steered.
	* \param ss True if the vehicle is skid steered, false if not.
	*/
	void SetSkidSteered(bool ss) { skid_steered_ = ss; }

	/**
	* Return the current x-y-z forces on a tire in world coordinates.
	* \param tire_id The id of the tire to query.
	*/
	glm::vec3 GetTireForces(int tire_id);

	/**
	* Return the angular velocity of the tire in rad/s
	* \param tire_id The id of the tire to query.
	*/
	float GetTireAngularVelocity(int tire_id);

	/**
	* Return the slip
	* \param tire_id The id of the tire to query.
	*/
	float GetTireSlip(int tire_id);

	/**
	* Turn the drag force calculation on/off.
	* \param use_drag Set to "true" to apply drag forces, false to turn them off. True by default.
	*/
	void UseDragForces(bool use_drag) { calculate_drag_forces_ = use_drag; }

	/**
	* Add an external force on the vehicle.
	* Force is added directly to the CG
	* Units are Netwons, force direction is in world ENU
	* \param force The force in Newtons to add to the CG
	*/
	void SetExternalForceOnCg(glm::vec3 force) { 
		external_force_.x = force.x; 
		external_force_.y = force.y;
		external_force_.z = force.z;
	}

	void SetExternalForceAtPoint(glm::vec3 force, glm::vec3 point) {
		extern_force_at_point_ = reactphysics3d::Vector3(force.x, force.y, force.z);
		extern_force_application_point_ = reactphysics3d::Vector3(point.x, point.y, point.z);
	}

	/**
	* Return the current tire steering angle in radians
	* \param tire_id The id of the tire to query.
	*/
	float GetTireSteeringAngle(int tire_id);

	/**
	* If set to false, the visualization will not be reloaded 
	* when the vehicle is created. This is used when a vehicle
	* is being respawned in the terrain in different places.
	* It is true by default.
	* \param load_vis Set to false to refrain from reloading the meshes into the MAVS scene.
	*/
	void SetReloadVisualization(bool load_vis) {
		load_visualization_ = load_vis;
	}

	/**
	* Return the current longitudinal acceleration in vehicle coordinates
	* This method overwrites the base vehicle class
	*/
	float GetLongitudinalAcceleration();

	/**
	* If true, animations will automatically be recommitted to the embree scene.
	* This can slow down the sim if it is done too frequently with too many animations
	* Set to false to speed things up. Then you have to to manually recommit the animations
	* at some point. It is true by default
	* \param aca True to automatically commit animations, false if not.
	*/
	void SetAutoCommitAnimations(bool aca) { auto_commit_animations_ = aca; }

	/// Get a pointer to the chassis object
	mavs_rp3d::Chassis* GetChassis() { return &chassis_; }

	int GetVehicleIdNum() { return vehicle_id_num_; }

	void SetHitchPoint(float x, float y, float z) { hitch_point_ = glm::vec3(x, y, z); }

	glm::vec3 GetHitchPointInWorldCoordinates();

private:
	bool auto_commit_animations_;

	//Initialization Functions
	void Init(environment::Environment *env);
	void InitTrailer(environment::Environment* env);
	void AddAxle(rp3d_axle axle);
	void AddAxleToTrailer(rp3d_axle axle);

	float max_dt_;
	bool calculate_drag_forces_;
	bool skid_steered_;
	float max_governed_speed_;
	std::vector<glm::vec3> current_tire_forces_;

	// Vehicle dynamics elements
	mavs_rp3d::Chassis chassis_;
	std::vector<mavs_rp3d::Suspension> running_gear_;
	rp3d::PhysicsWorld* world_;
	rp3d::PhysicsCommon physics_common_;
	mavs_rp3d::MavsPowertrain powertrain_;

	// trailer data members ----------------------------------------------------//
	bool has_trailer_;
	mavs_rp3d::Chassis trailer_;
	glm::vec3 hitch_point_;
	void LoadTrailer(const rapidjson::Value& trailer_object);
	float trailer_sprung_mass_;
	float trailer_cg_offset_, trailer_cg_lateral_offset_, trailer_cg_long_offset_;
	float trailer_chassis_drag_coeff_;
	rp3d::Vector3 trailer_chassis_dimensions_;
	std::vector<rp3d_axle> trailer_axles_;
	rp3d_anim trailer_anim_;
	rp3d_anim trailer_tire_anim_;
	std::vector<mavs_rp3d::Suspension> trailer_running_gear_;
	int trailer_id_num_;
	std::vector<glm::vec3> current_trailer_tire_forces_;
	glm::vec3 trailer_offset_;
	std::vector<int> trailer_tire_id_nums_;
	//rp3d::HingeJoint* trailer_hitch_joint_;
	rp3d::FixedJoint* trailer_hitch_joint_;
	//----------------------------------------------------------------------------//
	

	// Update functions
	void CalculateWheelTorques(float current_velocity, float throttle, float brake, float steering);
	void CalculateSteeringAngles(float dt, float steering);
	void ApplySuspensionForces();
	void ApplyGroundForces(environment::Environment *env, float dt, float throttle, float brake, float steering);
	void ApplyDragForces(float velocity);

	//chassis parameters
	float sprung_mass_;
	float cg_offset_;
	float cg_lateral_offset_;
	float cg_long_offset_;
	rp3d::Vector3 chassis_dimensions_;
	float torque_mass_scale_factor_;
	float chassis_drag_coeff_;

	// Loaded params
	std::vector<rp3d_axle> axles_;
	//std::vector<rp3d_tire> tires_;
	rp3d_anim anim_;
	rp3d_anim tire_anim_;
	bool animate_tires_;
	int vehicle_id_num_;
	std::vector<int> tire_id_nums_;
	bool load_visualization_;

	float GetLongVelSlope();
	std::vector<float> time_trace_;
	std::vector<float> long_vel_trace_;

	rp3d::Vector3 external_force_;
	rp3d::Vector3 extern_force_at_point_;
	rp3d::Vector3 extern_force_application_point_;

	int num_tire_slices_;
	float dtheta_slice_;
};

}// namespace vehicle
}// namespace mavs

#endif
