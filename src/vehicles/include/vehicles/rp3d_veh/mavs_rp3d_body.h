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
* \class DynamicBody
*
* Class for a MAVS dynamic body to be usied in a vehicle simulation
* Automatically attached collision/mass elements to rigid bodies and holds
* on to all the necessary pointers.
*
* \author Chris Goodin
*
* \date 9/13/2019
*/
#ifndef MAVS_RP3D_BODY
#define MAVS_RP3D_BODY
#include <reactphysics3d/reactphysics3d.h>

namespace mavs {
namespace vehicle {
namespace mavs_rp3d {

class DynamicBody {
public:
	/// Create an empty dynamic body
	DynamicBody() {}
	
	/**
	* Add a cuboid body to an rp3d Dynamics world
	* \param world Pointer to the rp3d dynamics world
	* \param dimensions Length, width, and height of the box in meters
	* \param position Position in world coordinates
	* \param orientation Orientation in world coordinates
	* \param mass Mass of the box in kg
	*/
	//void CreateBox(rp3d::DynamicsWorld* world, rp3d::Vector3 dimensions, rp3d::Vector3 position, rp3d::Quaternion orientation, float mass) {
	void CreateBox(rp3d::PhysicsCommon* physics_common, rp3d::PhysicsWorld* world, rp3d::Vector3 dimensions, rp3d::Vector3 position, rp3d::Quaternion orientation, float mass) {
		rp3d::Vector3 half_dimensions = 0.5f*dimensions;
		transform_ = rp3d::Transform(position, orientation);
		body_ = world->createRigidBody(transform_);
		body_->setMass(mass);
		box_ = physics_common->createBoxShape(half_dimensions);
		rp3d::Transform align_transform = rp3d::Transform::identity();
		shape_ = body_->addCollider(box_, align_transform);
		float mass_density = mass / (dimensions.x*dimensions.y*dimensions.z);
		shape_->getMaterial().setMassDensity(mass_density);
	}

	/// Get the current position of the body in world coordinates
	rp3d::Vector3 GetPosition() { return body_->getTransform().getPosition(); }

	/// Get the current orientation of the body in world coordinates
	rp3d::Quaternion GetOrientation() { return body_->getTransform().getOrientation(); }

	/// Return a pointer to the RP3D rigid body of the box
	rp3d::RigidBody* GetBody() { return body_; }

	/// Return the current "Look To" vector of the box
	rp3d::Vector3 GetLookTo() {
		rp3d::Matrix3x3 rot_mat = body_->getTransform().getOrientation().getMatrix();
		return rot_mat.getColumn(0);
	}

	/// Return a pointer to the RP3D shape of the collision box
	rp3d::Collider* GetShape() { return shape_; }

	void SetPositionOrientation(float px, float py, float pz, float qw, float qx, float qy, float qz) {
		rp3d::Vector3 position(px, py, pz);
		rp3d::Quaternion orientation(qw, qx, qy, qz);
		rp3d::Transform new_transform(position, orientation);
		body_->setTransform(new_transform);
		transform_ = new_transform;
	}

	void SetLinearAngularVelocity(float vx, float vy, float vz, float ax, float ay, float az) {
		reactphysics3d::Vector3 new_velocity(vx, vy, vz);
		body_->setLinearVelocity(new_velocity);
		reactphysics3d::Vector3 new_angvel(ax, ay, az);
		body_->setAngularVelocity(new_angvel);
	}

protected:
	rp3d::Transform transform_;
	rp3d::RigidBody* body_;
	rp3d::BoxShape* box_;
	rp3d::Collider* shape_;
};

} //namespace mavs_rp3d
} // namespace vehicle
} // namespace mavs

#endif