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
	Chassis() {}

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