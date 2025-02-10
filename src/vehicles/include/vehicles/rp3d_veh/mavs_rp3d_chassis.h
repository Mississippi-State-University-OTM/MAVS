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
* \class Chassis
*
* Class for a MAVS chassis to be used with the RP3D vehicle model
* Holds info about the chassis body
*
* \author Chris Goodin
*
* \date 9/13/2019
*/
#ifndef MAVS_RP3D_CHASS
#define MAVS_RP3D_CHASS
#include <vehicles/rp3d_veh/mavs_rp3d_body.h>
#include <reactphysics3d/reactphysics3d.h>

namespace mavs {
namespace vehicle {
namespace mavs_rp3d {

class Chassis : public DynamicBody {
public:
	/// Create a default blank chassis
	Chassis() {
		cg_offset_ = 0.0f;
		mass_ = 1.0f;
	}

	/**
	* Initialize a chassis attached to an RP3D world
	* \param world Pointer to the RP3D dynamic world the chassis will belong to.
	* \param init_trans The transfrom specifying the initial position and orientation of the chassis in world coordinates
	* \param cg_offset The offset of the chassis CG from the top of the suspension, in meters
	* \param dx Length (longitudinal) of the chassis in meters
	* \param dy Width (lateral) of the chassis in meters
	* \param dz Height (vertical) of the chassis in meters
	* \param mass Mass of the chassis in kg
	*/
	void Init(rp3d::PhysicsCommon* physics_common, rp3d::PhysicsWorld* world, rp3d::Transform init_trans, float cg_offset, float dx, float dy, float dz, float mass) {
		cg_offset_ = cg_offset;
		CreateBox(physics_common, world, rp3d::Vector3(dx, dy, dz), init_trans.getPosition(), init_trans.getOrientation(), mass);
		mass_ = mass;
	}

	/// Return the vertical offset of the chassis CG from the suspension
	float GetCgOffset() { return cg_offset_; }

	float GetMass() { return mass_; }

private:
	float cg_offset_;
	float mass_;
};

} //namespace mavs_rp3d
}// namespace vehicle
}// namespace mavs

#endif