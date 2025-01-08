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
* \class Suspension
*
* Class for a MAVS suspension to be used with the RP3D vehicle model
* Calculates forces on vehicle chassis from suspension elements
*
* \author Chris Goodin
*
* \date 9/13/2019
*/
#ifndef MAVS_RP3D_SUSP
#define MAVS_RP3D_SUSP
#include <vehicles/rp3d_veh/mavs_rp3d_chassis.h>
#include <vehicles/rp3d_veh/mavs_tire.h>
#include <reactphysics3d/reactphysics3d.h>

namespace mavs {
namespace vehicle {
namespace mavs_rp3d {

class Suspension : public DynamicBody {
public:
	/// Create a blank suspension
	Suspension() {}

	/**
	* Create a suspension element attached to a chassis, explicitly specifying the damping constant
	*/
	Suspension(mavs_rp3d::Chassis* chassis, rp3d::PhysicsCommon* physics_common, rp3d::PhysicsWorld* world, MavsTire tire, float long_offset, float lat_offset, float spring_length, float spring_constant, float damping_constant, float max_steering_angle_degrees) {
		Init(chassis, physics_common, world, tire, long_offset, lat_offset, spring_length, spring_constant, damping_constant, max_steering_angle_degrees);
	}

	/// Get the current compression of the spring, in meters
	float GetTranslation() { return joint_->getTranslation(); }

	/// Get the suspension spring constant in N/m
	float GetSpringConstant() { return k_; }

	/// Get the suspension damping constant in N*s/m
	float GetDampingConstant() { return c_; }

	/// Get the spring rest length (uncompressed) in meters
	float GetSpringLength() { return spring_length_; }

	/// Get a pointer to the tire attached to the suspension element
	MavsTire* GetTire() { return &tire_; }

	/**
	* Set of the element is steered or not
	* \param steered True if steered, false if not steered
	*/
	void SetSteered(bool steered) { is_steered_ = steered; }

	/**
	* Set of the element is powered or not
	* \param powered True if powered, false if not powered
	*/
	void SetPowered(bool powered) { is_powered_ = powered; }

	/// Return true if the element is powered
	bool IsPowered() { return is_powered_; }

	/// Return true if the element is steered
	bool IsSteered() { return is_steered_; }

	/// Get the maximum steering angle of the element, in radians
	float GetMaxSteeringAngle() { return max_steering_angle_; }

	/// Get the current steering angle of the element, in radians
	float GetCurrentSteeringAngle() { return current_steering_angle_; }

	/**
	* Set the steering angle of the element, in radians
	* \param sa The steering angle in radians
	*/
	void SetCurrentSteeringAngle(float sa) { 
		current_steering_angle_ = sa; 
		current_steering_angle_ = std::min(std::max(current_steering_angle_, -max_steering_angle_), max_steering_angle_);
	}

	/**
	* Set the torque being appled to the wheel
	* \param torque The applied torque in N/m
	*/
	void SetCurrentAppliedTorque(float torque) { current_applied_torque_ = torque; }

	/// Return the torque currently being applied to the wheel, in N/m
	float GetCurrentWheelTorque() { return current_applied_torque_; }

private:
	void Init(mavs_rp3d::Chassis* chassis, rp3d::PhysicsCommon* physics_common, rp3d::PhysicsWorld* world, MavsTire tire, float long_offset, float lat_offset, float spring_length, float spring_constant, float damping_constant, float max_steering_angle_degrees) {
		is_steered_ = true;
		is_powered_ = true;
		max_steering_angle_ = max_steering_angle_degrees * 3.14159f / 180.0f;
		current_steering_angle_ = 0.0f;
		current_applied_torque_ = 0.0f;
		tire_ = tire;
		k_ = spring_constant;
		c_ = damping_constant;
		spring_length_ = spring_length;
		rp3d::Matrix3x3 rot_mat = chassis->GetOrientation().getMatrix();
		rp3d::Vector3 look_to = rot_mat.getColumn(0);
		rp3d::Vector3 look_side = rot_mat.getColumn(1);
		rp3d::Vector3 look_up = rot_mat.getColumn(2);
		rp3d::Vector3 position = chassis->GetPosition() + long_offset * look_to + lat_offset * look_side - spring_length * look_up;
		CreateBox(physics_common, world, rp3d::Vector3(0.1f, 0.1f, 0.1f), position, chassis->GetOrientation(), tire_.GetMass());
		
		//rp3d::Vector3 anchor_point = position + rp3d::Vector3(0.0f, 0.0f, spring_length);
		//slider joint
		//rp3d::SliderJointInfo jointInfo(chassis->GetBody(), body_, anchor_point, rp3d::Vector3(0.0f, 0.0f, 1.0f));
		rp3d::SliderJointInfo jointInfo(chassis->GetBody(), body_, position, look_up);
		jointInfo.isCollisionEnabled = false;
		jointInfo.isLimitEnabled = true;
		jointInfo.minTranslationLimit = -0.95*spring_length; 
		jointInfo.maxTranslationLimit = 0.95*spring_length; 
		joint_ = dynamic_cast<rp3d::SliderJoint*>(world->createJoint(jointInfo));
	}

	rp3d::SliderJoint* joint_;
	float k_;
	float c_;
	float spring_length_;
	MavsTire tire_;
	bool is_steered_;
	bool is_powered_;
	float max_steering_angle_;

	//state variables
	float current_steering_angle_;
	float current_applied_torque_;
};
} // namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs

#endif