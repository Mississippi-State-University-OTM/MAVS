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
* \class MavsTire
*
* Class for a MAVS tire to be used with the RP3D vehicle model
* Calculates normal force using a spring damper model.
*
* \author Chris Goodin
*
* \date 9/13/2019
*/
#ifndef MAVS_TIRE
#define MAVS_TIRE
#include <vehicles/rp3d_veh/radial_spring_tire.h>
#include <mavs_core/environment/environment.h>
#include <vehicles/rp3d_veh/mavs_rp3d_pacejka.h>
#include <reactphysics3d/reactphysics3d.h>
#include <FastNoise.h>

namespace mavs {
namespace vehicle {
namespace mavs_rp3d {

class MavsTire {
public:
	/// Construct a default MAVS tire
	MavsTire();

	/// Copy constructor
	MavsTire(const MavsTire &tire) {
		left_ = tire.left_;
		k_ = tire.k_;
		c_ = tire.c_;
		mass_ = tire.mass_;
		moment_ = tire.moment_;
		radius_ = tire.radius_;
		width_ = tire.width_;
		section_height_ = tire.section_height_;
		viscous_friction_coeff_ = tire.viscous_friction_coeff_;
		terrain_noise_ = tire.terrain_noise_;
		height_function_set_ = tire.height_function_set_;
		terrain_type_ = tire.terrain_type_;
		terrain_type_args_ = tire.terrain_type_args_;
		soil_type_set_ = tire.soil_type_set_;
		soil_type_ = tire.soil_type_;
		soil_strength_ = tire.soil_strength_;
		angular_velocity_ = tire.angular_velocity_;
		current_slip_ = tire.current_slip_;
		current_deflection_ = tire.current_deflection_;
		pacejka_ = tire.pacejka_;
		radial_spring_tire_ = tire.radial_spring_tire_;
		current_rotation_angle_ = tire.current_rotation_angle_;
		current_orientation_ = tire.current_orientation_;
		elapsed_time_ = tire.elapsed_time_;
		tire_id_ = tire.tire_id_;
		num_slices_ = tire.num_slices_;
		dtheta_slice_ = tire.dtheta_slice_;
		startup_ = tire.startup_;
	}

	/**
	* Construct a MAVS tire with specified parameters
	* \param tire_mass In kg
	* \param tire_radius Undeflected, in meters
	* \param tire_spring_constant Tire spring constant in N/m
	*/
	MavsTire(float tire_mass, float tire_radius, float tire_width, float tire_section_height, float tire_spring_constant);

	/// Return the current rotation angle of the tire
	float GetCurrentRotationAngle() {
		return current_rotation_angle_;
	}

	/// Return the current orientation as a quaternion
	rp3d::Quaternion GetCurrentOrientation() { return current_orientation_; }

	/**
	* Initalize a MAVS tire with specified parameters
	* \param tire_mass In kg
	* \param tire_radius Undeflected, in meters
	* \param tire_spring_constant Tire spring constant in N/m
	*/
	void Init(float tire_mass, float tire_radius, float tire_width, float tire_section_height, float tire_spring_constant);

	/// Get the mass of the tire in kg
	float GetMass() { return mass_; }

	/// Get the spring constant of the tire in N/m
	float GetSpringConstant() { return k_; }

	/// Get the damping constant of the tire in N*s/m
	float GetDampingConstant() { return c_; }

	/// Get the undeflected radius of the tire in meters
	float GetRadius() { return radius_; }

	/// Return the undeflected tire width in meters
	float GetWidth() { return width_; }

	/**
	* Set the tire spring constant
	* \param k Spring constant in N/m
	*/
	void SetSpringConstant(float k) {
		k_ = k;
		CalcDamping();
	}

	/**
	* Set the tire mass
	* \param mass In kg
	*/
	void SetMass(float mass) {
		mass_ = mass;
		CalcDamping();
	}

	/**
	* Set the undeflected tire radius
	* \param radius In meters
	*/
	void SetRadius(float radius) { radius_ = radius; }

	/**
	* Set the tire damping constant
	* \param damping Damping constant in N*s/m
	*/
	void SetDamping(float damping) { c_ = damping; }

	/**
	* Calculate the forces on the tire. 
	* Returns the forces on the tire in world coordinates
	* \param env Pointer to the current mavs environment.
	* \param dt The calcuation time step
	* \param tire_pose Transform indicating tire position and orientation in world coordinates
	* \param tire_velocity 3D tire velocity in world coordinates (m/s)
	* \param torque The torque applied to the tire, in N/m
	* \param steer_angle The steer angle of the tire, in radians
	*/
	rp3d::Vector3 Update(environment::Environment *env, float dt, rp3d::Transform tire_pose, rp3d::Vector3 tire_velocity, float torque, float steer_angle);

	/**
	* Set the terrain height function. The available terrain types are
	* 'flat', 'sloped', 'sine', and 'rough'. The second argument is a list of
	* parameters for the height model. 
	* flat: args[0] = terrain height
	* sloped: args[0] = fractional slope (1 = 45 degrees)
	* sine: args[0] = wavelength in meters, args[1] = magnitude of oscillation
	* rough: args[0] = wavelength of roughness in meters, args[1] = magnitude of roughness, in meters
	* \param height_model The height model used, see above
	* \param args Parameters for the height model, see above
	*/
	void SetTerrainHeightFunction(std::string height_model, std::vector<float> args);

	/**
	* Set the soil type and strength. Available soil types are
	* 'snow', 'ice', 'wet', 'sand', 'clay', 'paved'
	* The soil strength param is in RCI and is only used when the type is 'clay' or 'sand'
	* \param soil_type The soil type, see above
	* \param soil_strength The soil strength, see above
	*/
	void SetSoilType(std::string soil_type, float soil_strength);

	/**
	* Return the current fractional deflection of the tire
	*/
	float GetCurrentDeflection() { return current_deflection_; }

	/**
	* Set the viscous friction coefficient of the tire.
	* Default is 0.5.
	* \param vfc The viscous friction coefficient.
	*/
	void SetViscousFrictionCoeff(float vfc) { viscous_friction_coeff_ = vfc; }

	/*
	* Set the angle at which the lateral force "croses over".
	* \param ang Crossover angle in radians
	*/
	void SetLateralCrossoverAngle(float ang) { pacejka_.SetSlipAngleCrossover(ang); }

	/// Return the current angular velocity in rad/s
	float GetAngularVelocity() { return angular_velocity_; }

	/// Return the current slip
	float GetSlip() { return current_slip_; }

	/// Set to true if tire is on left (drivers) side
	void SetLeft(bool is_left) {
		left_ = is_left;
	}

	/**
	* Set the ID of the tire
	* \param id The id number of the tire.
	*/
	void SetId(int id) { tire_id_ = id; }

	/// Return the id of the tire
	int GetId() { return tire_id_; }

	/// Set the number of lateral slices for the spring tire model, min is 1
	void SetNumSlices(int ns) { num_slices_ = std::max(ns,1); }

	/// Set the angle separation  (degrees) between sample points on the radial spring tire, min is 0.1 degrees
	void SetDthetaSlice(float dts) { dtheta_slice_ = std::max(dts, 0.1f); }

private:
	// methods
	void CalcDamping();
	rp3d::Vector3 RodRot(rp3d::Vector3 v, rp3d::Vector3 k, float theta);
	float CalcSlip(float vx);
	float GetTerrainHeightFromFunction(float x, float y);

	// data
	bool left_;
	float k_;
	float c_;
	float mass_;
	float moment_;
	float radius_;
	float width_;
	float section_height_;
	float viscous_friction_coeff_;
	FastNoise terrain_noise_;
	bool height_function_set_;
	std::string terrain_type_;
	std::vector<float> terrain_type_args_;
	bool soil_type_set_;
	std::string soil_type_;
	float soil_strength_;
	float angular_velocity_;
	float current_slip_;
	float current_deflection_;
	double current_rotation_angle_;
	rp3d::Quaternion current_orientation_;
	MavsPacejka pacejka_;
	mavs::vehicle::radial_spring::Tire radial_spring_tire_;
	float elapsed_time_;
	int tire_id_;
	int num_slices_;
	float dtheta_slice_;
	bool startup_;
};

} // namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs

#endif