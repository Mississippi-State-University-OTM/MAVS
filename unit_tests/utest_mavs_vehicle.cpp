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
* \file utest_mavs_vehcle.cpp
*
* Unit tests for the MAVS rp3d vehicle implementation.
* Uses the catch2 testing framework.
*
* \author Chris Goodin
*
* \date 4/27/2021
*/
#include "catch2/catch.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"

static const float delta = 1.0E-5f;

class UtestSpringDamper {
public:
	UtestSpringDamper() {
		rp3d::PhysicsWorld::WorldSettings settings;
		settings.defaultVelocitySolverNbIterations = 15;
		settings.defaultPositionSolverNbIterations = 8;
		settings.isSleepingEnabled = false;
		settings.gravity = rp3d::Vector3(0.0, 0.0, -9.806);
		world_ = physics_common_.createPhysicsWorld(settings);
		world_->setNbIterationsVelocitySolver(15); //default is 10 
		world_->setNbIterationsPositionSolver(10); //default is 5
		unit_orientation_ = rp3d::Matrix3x3::identity();
		spring_length_ = 1.0f;
		c_ = 1.0f;
		k_ = 100.0f;
		Initialize();
	}

	~UtestSpringDamper() {
		physics_common_.destroyPhysicsWorld(world_);
	}

	void Initialize() {
		CreateGround();
		CreateJoint();
	}

	void Simulate() {
		float dt = 0.001f;
		float elapsed_time = 0.0f;
		while (elapsed_time < 5.0f) {
			world_->update(dt);
			ApplySpringForces();
			elapsed_time += dt;
		}
	}

	float GetCurrentPosition() { return bouncer_.GetPosition().z; }

private:

	void ApplySpringForces() {
		float x = joint_->getTranslation();
		float v = bouncer_.GetBody()->getLinearVelocity().z;
		float force = -k_ * x - c_*v;
		rp3d::Vector3 look_up(0.0, 0.0, 1.0);
		rp3d::Vector3 vec = bouncer_.GetBody()->getTransform().getOrientation().getMatrix()*look_up;
		bouncer_.GetBody()->applyForceAtWorldPosition(force*vec, bouncer_.GetBody()->getTransform().getPosition()+x*vec);
	}

	void CreateGround() {
		ground_.CreateBox(&physics_common_, world_, rp3d::Vector3(10000.0f, 10000.0f,0.0001f), rp3d::Vector3(0.0, 0.0, 0.0), unit_orientation_, 1.0);
		ground_.GetBody()->enableGravity(false);
		ground_.GetBody()->setType(rp3d::BodyType::STATIC);
	}

	void CreateJoint() {
		bouncer_.CreateBox(&physics_common_, world_, rp3d::Vector3(0.1, 0.1, 0.1f), rp3d::Vector3(0.0, 0.0, spring_length_), unit_orientation_, 1.0);
		rp3d::SliderJointInfo joint_info(ground_.GetBody(), bouncer_.GetBody(), bouncer_.GetPosition(), rp3d::Vector3(0.0f, 0.0f, 1.0f));
		joint_info.isCollisionEnabled = false;
		joint_info.isLimitEnabled = true;
		joint_info.minTranslationLimit = -0.95f*spring_length_;
		joint_info.maxTranslationLimit = 0.95f*spring_length_;
		joint_ = dynamic_cast<rp3d::SliderJoint*>(world_->createJoint(joint_info));
	}

	float k_;
	float c_;
	float spring_length_;
	rp3d::PhysicsCommon physics_common_;
	rp3d::PhysicsWorld *world_;
	mavs::vehicle::mavs_rp3d::DynamicBody ground_;
	mavs::vehicle::mavs_rp3d::DynamicBody bouncer_;
	rp3d::Matrix3x3 unit_orientation_;
	rp3d::SliderJoint* joint_;
};


TEST_CASE("RP3D Vehicle", "[rp3d_vehicle]") {
	// define some initial test parameters
	float mass = 1.0f;
	rp3d::Vector3 dimensions = rp3d::Vector3(1.0, 1.0, 1.0);
	float g = -9.806f;
	float zstart = 1.0f;

	// create the physics world and set some solver settings
	rp3d::PhysicsCommon physics_common;
	rp3d::PhysicsWorld::WorldSettings settings;
	settings.defaultVelocitySolverNbIterations = 15;
	settings.defaultPositionSolverNbIterations = 8;
	settings.isSleepingEnabled = false;
	settings.gravity = rp3d::Vector3(0.0, 0.0, g);
	rp3d::PhysicsWorld *world = physics_common.createPhysicsWorld(settings);
	world->setNbIterationsVelocitySolver(15); //default is 10 
	world->setNbIterationsPositionSolver(10); //default is 5

	// create a rigid body of mass 1.0 and dimensions 1x1x1
	rp3d::Matrix3x3 orientation;
	orientation = rp3d::Matrix3x3::identity();
	rp3d::Transform transform = rp3d::Transform(rp3d::Vector3(0.0, 0.0, zstart), orientation);
	rp3d::RigidBody* body = world->createRigidBody(transform);
	body->setMass(mass);
	rp3d::BoxShape* box = physics_common.createBoxShape(0.5*dimensions);
	rp3d::Transform align_transform = rp3d::Transform::identity();
	rp3d::Collider* shape_ = body->addCollider(box, align_transform);
	float mass_density = mass / (dimensions.x*dimensions.y*dimensions.z);
	shape_->getMaterial().setMassDensity(mass_density);

	// simulate the body in free fall, check against analytic solution
	float max_time = 1.0f;
	float current_time = 0.0f;
	float dt = 0.001f;
	float delta_free_fall = 0.0f;
	while (current_time <= max_time) {
		world->update(dt);
		rp3d::Vector3 position = body->getTransform().getPosition();
		current_time += dt;
		float zcalc = zstart + 0.5*g*current_time*current_time;
		delta_free_fall = std::fabs(zcalc - position.z);
	}

	// create two mavs_rp3d dynamic bodies attached with a slider joint
	// make the top one fixed
	float spring_length = 4.0f;
	rp3d::Vector3 position2(10.0, 0.0, spring_length);
	rp3d::Vector3 position1(10.0, 0.0, 0.5*spring_length);
	mavs::vehicle::mavs_rp3d::DynamicBody body1, body2;
	body1.CreateBox(&physics_common, world, dimensions, position1, orientation, mass);
	body2.CreateBox(&physics_common, world, dimensions, position2, orientation, mass);
	body2.GetBody()->enableGravity(false);
	body2.GetBody()->setType(rp3d::BodyType::STATIC);
	rp3d::SliderJointInfo jointInfo (body1.GetBody(), body2.GetBody(), position2, rp3d::Vector3(0.0f, 0.0f, 1.0f));
	jointInfo.isCollisionEnabled = false;
	jointInfo.isLimitEnabled = true;
	jointInfo.minTranslationLimit = 0.95f*spring_length;
	jointInfo.maxTranslationLimit = -0.95f*spring_length;
	rp3d::SliderJoint* joint = dynamic_cast<rp3d::SliderJoint*>(world->createJoint(jointInfo));
	current_time = 0.0f;
	while (current_time <= max_time) {
		world->update(dt);
		current_time += dt;
	}
	float delta_joint = std::fabs(std::fabs(body1.GetPosition().z -position1.z) - 0.95f*spring_length);

	// create two mavs rp3d dynamic bodies attached by a slider joint and a spring
	// compare to analytic solution for harmonic motion
	float k = 100.0f; // spring constant
	float omega = (float)sqrt(k / mass);
	rp3d::Vector3 position3(20.0, 0.0, spring_length);
	rp3d::Vector3 position4(20.0, 0.0, 0.5*spring_length);
	mavs::vehicle::mavs_rp3d::DynamicBody body3, body4;
	body3.CreateBox(&physics_common, world, dimensions, position3, orientation, mass);
	body4.CreateBox(&physics_common, world, dimensions, position4, orientation, mass);
	body3.GetBody()->enableGravity(false);
	body3.GetBody()->setType(rp3d::BodyType::STATIC);
	rp3d::SliderJointInfo jointInfo2(body3.GetBody(), body4.GetBody(), position4, rp3d::Vector3(0.0f, 0.0f, 1.0f));
	jointInfo2.isCollisionEnabled = false;
	jointInfo2.isLimitEnabled = false;
	rp3d::SliderJoint* joint2 = dynamic_cast<rp3d::SliderJoint*>(world->createJoint(jointInfo2));
	// simulate the spring system and compare to analytical solution
	current_time = 0.0f;
	float zpos = position4.z;
	float zvel = 0.0f;
	float a_old = 0.0f;
	while (current_time <= max_time) {
		world->update(dt);
		float x = body4.GetPosition().z - position4.z;
		float f = -k * x;
		rp3d::Vector3 force(0.0f, 0.0f, f);
		body4.GetBody()->applyForceToCenterOfMass(force);
		float a = f / mass + g;
		zpos += zvel * dt + 0.5f*a*dt*dt;
		zvel += 0.5*dt *(a + a_old);
		a_old = a;
		current_time += dt;
	}
	float delta_spring = std::fabs(std::fabs(body4.GetPosition().z - zpos));

	// create a spring-damper system and simulate it
	UtestSpringDamper suspension;
	suspension.Simulate();
	float delta_spring_damper = std::fabs(0.909495-suspension.GetCurrentPosition());

	// Do unit test checks
	REQUIRE(delta_free_fall<0.01f);
	REQUIRE(delta_joint < delta);
	REQUIRE(delta_spring < 0.01f);
	REQUIRE(delta_spring_damper < 0.01f);

}
