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
* \class SceneEditor
*
* Class with methods for interactively editing and viewing a scene
*
* \author Chris Goodin
*
* \date 2/28/2019
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <mavs_core/environment/environment.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include <mavs_core/math/polygon.h>
#include <sensors/camera/ortho_camera.h>
#include <sensors/camera/rgb_camera.h>
#include <sensors/annotation_colors.h>
#include <mavs_core/pose_readers/waypoints.h>

namespace mavs {
namespace utils {

/**
* Describes a vegetation distribution
*/
struct VegDistribution {
	/// The position of each plant
	std::vector<glm::vec2> positions;

	/// The name of the plant mesh
	std::string mesh_name;

	/// Polygon representing the distribution
	mavs::math::Polygon polygon;
};

class SceneEditor {
public:
	/**
	* Create a scene editor
	*/
	SceneEditor();

	/**
	* Load a mesh to be used as the surface
	* This MUST be called first for the scene editor to work.
	*/
	void LoadSurface(std::string infile);

	/**
	* Run the dynamic editing loop for adding meshes/viewing
	*/
	void RunEditLoop();

private:
	//methods
	glm::vec3 PixelToCoordinate(int i, int j);
	glm::vec2 CoordinateToPixel(glm::vec2 v);
	void DrawPolygons();
	void CalculateDimensions();
	void ViewScene();
	glm::mat3x4 GetRandomRotScale();
	void AddPolygon();
	void Zoom();
	void Unzoom();
	void DisplayPointCoordinates();
	void GetKeyboardInput();
	// Print the scene statistics to the terminal
	void GetSceneStats();
	void SaveScene();
	void DisplayInstructions();
	void LoadTrail();
	void SelectTrail();
	void SelectVegDistro();
	void GenerateEcosystem();
	void CopyPathsToTrails();
	//void SelectEcoFile();

	//data
	cimg_library::CImg<float> image_;
	cimg_library::CImgDisplay disp_;
	cimg_library::CImgDisplay disp_instruct_;
	glm::vec3 ll_;
	glm::vec3 ur_;
	glm::vec3 center_;
	glm::vec3 current_ll_;
	glm::vec3 current_ur_;
	glm::vec3 current_center_;
	float current_pixres_;
	mavs::sensor::camera::OrthoCamera cam_;
	mavs::environment::Environment env_;
	mavs::raytracer::embree::EmbreeTracer scene_;
	std::string surface_path_;
	std::string surface_file_;
	std::vector<VegDistribution> vegetation_;
	mavs::sensor::AnnotationColors colors_;
	std::vector<mavs::Waypoints> paths_;
	std::vector<mavs::Trail> trails_;
	glm::mat3x4 unit_rot_scale_;

	bool saved_;
	bool trail_loaded_;
	bool displayed_;
	bool ecosystem_loaded_;
};

}//namespace utils
} //namespace mavs

