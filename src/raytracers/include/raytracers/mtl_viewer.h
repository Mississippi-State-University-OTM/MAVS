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
* \class MtlViewer
*
* Class for loading all the materials associated with a 
* particular object, and methods for accessing the data
* associated with those materials.
* 
* \author Chris Goodin
*/
#include <raytracers/simple_tracer/simple_tracer.h>
#include <raytracers/material.h>
#include <raytracers/mesh.h>
#include <sensors/camera/rgb_camera.h>

namespace mavs {
namespace raytracer {

class MtlViewer {

public:
	/// Create a material viewer
	MtlViewer();

	/**
	* Load the mesh and associated materials
	* \param meshfile full path to the mesh to load
	*/
	void LoadMesh(std::string meshfile);

	/// Update the display when the material is changed
	void Update();

	/** 
	* Set the material to be rendered
	* \param material The MAVS material to be rendered
	*/
	void SetMaterial(Material material);

	/// Return the total number of materials in the mesh
	int GetNumMats();

	/**
	* Return the name of the material with ID
	* \param id The material id #
	*/
	char * GetMatName(int id);

	/**
	* Return the name of the spectrum with ID
	* \param id The material id #
	*/
	char * GetSpectrumName(int id);

	/**
	* Return the material with ID
	* \param id The material id #
	*/
	Material GetMaterial(int id);

private:

	mavs::sensor::camera::RgbCamera camera;
	mavs::raytracer::SimpleTracer scene;
	mavs::environment::Environment env;
	Mesh mesh_;
	Material material_;
	Sphere sphere_;
	std::vector<Material> avail_mats_;
	int prim_num_;
};

} // namespace raytracer
} //namespace mavs

