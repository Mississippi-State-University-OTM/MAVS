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
//rapidjson includes
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
//class header
#include "scene_editor.h"
// c++ includes
#include <iostream>
// mavs includes
#include <mavs_core/environment/environment.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/math/polygon.h>
#include <sensors/camera/simple_camera.h>
#include <sensors/camera/rgb_camera.h>
#include <sensors/io/user_io.h>
#include <sensors/annotation_colors.h>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>
#include <mavs_core/terrain_generator/ecosystem.h>
#ifdef Bool
#undef Bool
#endif

namespace mavs {
namespace utils {

// global constants
static const float pixres_ = 0.25f;
static const int maxdim_ = 768;
static const glm::quat o1_(0.7071f, 0.0f, 0.0f, 0.7071f);
static const glm::quat o2_(0.7071f, 0.0f, 0.7071f, 0.0f);
static const glm::quat orientation_ = o1_ * o2_;
//static const glm::vec2 origin(-5.0f, 3.0f);
static const glm::vec2 origin(0.0f, 0.0f);
static const float yellow[3] = { 255.0f, 255.0f, 0.0f };
static const float green[3] = { 0.0f, 255.0f, 0.0f };
static const float blue[3] = { 0.0f, 0.0f, 255.0f };

SceneEditor::SceneEditor() {
	//--- Initialize global variable ---//
	unit_rot_scale_[0][0] = 1.0f; unit_rot_scale_[0][1] = 0.0f; unit_rot_scale_[0][2] = 0.0f;
	unit_rot_scale_[1][0] = 0.0f; unit_rot_scale_[1][1] = 1.0f; unit_rot_scale_[1][2] = 0.0f;
	unit_rot_scale_[2][0] = 0.0f; unit_rot_scale_[2][1] = 0.0f; unit_rot_scale_[2][2] = 1.0f;
	unit_rot_scale_[0][3] = 0.0f; unit_rot_scale_[1][3] = 0.0f; unit_rot_scale_[2][3] = 0.0f;
	saved_ = false;
	displayed_ = false;
	trail_loaded_ = false;
	ecosystem_loaded_ = false;
}

glm::vec3 SceneEditor::PixelToCoordinate(int i, int j) {
	int jj = image_.height() - j;
	float dx = current_pixres_ * i;
	float dy = current_pixres_ * jj;
	glm::vec3 v(current_ll_.x + dx, current_ll_.y + dy, 0.0f);
	return v;
}

glm::vec2 SceneEditor::CoordinateToPixel(glm::vec2 v) {
	int i = (int)((v.x - current_ll_.x) / current_pixres_);
	int jj = (int)((v.y - current_ll_.y) / current_pixres_);
	glm::vec2 p(i, image_.height() - jj);
	return p;
}

void SceneEditor::DisplayInstructions() {
	std::string instruct = "INSTRUCTIONS:";
	std::string zoom = "- Z to zoom in to selection.";
	std::string unzoom = "- U to zoom out / unzoom.";
	std::string point = "- C to select a point and print it's coordinates to the terminal.";
	std::string poly = "- P to select a polygon with the mouse and fill it with instances of a mesh.";
	std::string poly_more1 = "\t- Select points by clicking with the mouse.";
	std::string poly_more2 = "\t- Close the polygon by clicking on the first point.";
	std::string poly_more3 = "\t- Move in a clockwise direction.";
	std::string view = "- V to open a scene viewer window for the current scene.";
	std::string stats = "- I to print the current scene stats to the terminal.";
	std::string trail = "- T to load a trail from file.";
	std::string trail_user = "- K to create a trail with mouse.";
	std::string trail_more1 = "\t- Select points with the mouse.";
	std::string trail_more2 = "\t- Press Q when done selecting.";
	std::string ecosystem = "- E to grow ecosystem.";
	std::string eco_more1 = "\t- You'll be prompted to select the ecosystem file input.";
	std::string eco_more2 = "\t- Select the output file for the .vprp that will be created.";
	std::string eco_more3 = "\t- Set the relative plant density.";
	std::string eco_more4 = "\t- Set the output scene file name.";
	std::string veg_points = "- G to load vegetation datapoints for the scene.";
	std::string save = "- S to save the current scene to a MAVS json file.";
	std::string quit = "- Close the main window to quit, or press Ctrl+C in the terminal.";
	cimg_library::CImg<float> instructions(448, 384, 1, 3, 150.0f);
	int vp = 15;
	int dvp = 15;
	instructions.draw_text(15, vp, instruct.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, zoom.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, unzoom.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, point.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, poly.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, poly_more1.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, poly_more2.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, poly_more3.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, view.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, stats.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, trail.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, trail_user.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, trail_more1.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, trail_more2.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, ecosystem.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, eco_more1.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, eco_more2.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, eco_more3.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, eco_more4.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, veg_points.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, save.c_str(), yellow);
	vp += dvp;
	instructions.draw_text(15, vp, quit.c_str(), yellow);
	vp += dvp;
	if (disp_instruct_.is_closed()) {
		disp_instruct_.assign(instructions);
	}
	else {
		disp_instruct_.display(instructions);
	}
}

void SceneEditor::GetKeyboardInput() {
	//Display Instructions
	if (disp_.is_keyH() && !displayed_) {
		DisplayInstructions();
		displayed_ = true;
	}
	if (!disp_.is_keyH()) {
		displayed_ = false;
	}
	// Zoom to box
	if (disp_.is_keyZ()) {
		Zoom();
	}
	// Unzoom 
	if (disp_.is_keyU()) {
		Unzoom();
	}
	// Print coordinates to terminal
	if (disp_.is_keyC()) {
		DisplayPointCoordinates();
	}
	// Add a polygon to the scnee 
	if (disp_.is_keyP()) {
		AddPolygon();
	}
	//view the current scene
	if (disp_.is_keyV()) {
		ViewScene();
	}
	//print screen stats to terminal
	if (disp_.is_keyI()) {
		GetSceneStats();
	}
	//save the current scene
	if (disp_.is_keyS() && !saved_) {
			SaveScene();
			saved_ = true;
	}
	if (!disp_.is_keyS()){
		saved_ = false;
	}
	//Load trail
	if (disp_.is_keyT() && !trail_loaded_) {
		LoadTrail();
		trail_loaded_ = true;
	}
	if (!disp_.is_keyT()) {
		trail_loaded_ = false;
	}

	// Add a polygon to the scene 
	if (disp_.is_keyK()) {
		SelectTrail();
	}

	// Select empirical veg distribution file 
	if (disp_.is_keyG()) {
		SelectVegDistro();
	}

	// use current trails and surface to generate ecosystem
	if (disp_.is_keyE() && !ecosystem_loaded_) {
		GenerateEcosystem();
		ecosystem_loaded_ = true;
	}
}

void SceneEditor::LoadTrail() {
	std::string trail_file = mavs::io::GetInputFileName("Select trail input file", "*.json");
	if (trail_file.size() < 5) {
		return;
	}
	Waypoints trail;
	trail.Load(trail_file);
	paths_.push_back(trail);
}

void SceneEditor::CopyPathsToTrails() {
	trails_.resize(paths_.size());
	for (int pn = 0; pn < (int)paths_.size(); pn++) {
		trails_[pn].SetPath(paths_[pn]);
		trails_[pn].SetTrackWidth(0.25);
		trails_[pn].SetTrailWidth(5.0);
		trails_[pn].SetWheelbase(2.0);
		std::string save_prompt = "Name of trail " + mavs::utils::ToString(pn) + " .vprp file";
		std::string vprp_out = mavs::io::GetSaveFileName(save_prompt, "trail.vprp", "*.vprp");
		if (vprp_out.size() < 4) return;
		trails_[pn].SaveToAnvelVprp(vprp_out);
		trails_[pn].SetPathFile(vprp_out);
	}
}

void SceneEditor::GenerateEcosystem() {
	std::string eco_file = mavs::io::GetInputFileName("Select ecosystem file", "*.json");
	if (eco_file.size() < 4)return;
	CopyPathsToTrails();
	mavs::terraingen::Ecosystem ecosystem;
	ecosystem.Load(eco_file);
	mavs::terraingen::HeightMap heightmap;
	heightmap.CreateFromMesh(surface_file_);
	ecosystem.LoadTerrain(surface_file_);
	ecosystem.SetHeightMap(heightmap);
	for (int tn = 0; tn < (int)trails_.size(); tn++) {
		ecosystem.AddTrail(trails_[tn]);
	}
	float plant_density = mavs::io::GetUserNumericInput("Plant Density", "Select Plant density");
	ecosystem.SetGrowthFactor(plant_density);
	ecosystem.Simulate();
	std::string eco_out = mavs::io::GetSaveFileName("Name of output scene file", "scene.json", "*.json");
	if (eco_out.size() > 4) {
		ecosystem.SaveToMavsScene(eco_out);
	}
}

void SceneEditor::DrawPolygons() {
	for (int p = 0; p < (int)vegetation_.size(); p++) {
		int np = vegetation_[p].polygon.NumPoints();
		for (int i = 1; i < np; i++) {
			glm::ivec2 p0 = CoordinateToPixel(vegetation_[p].polygon.GetPoint(i - 1));
			glm::ivec2 p1 = CoordinateToPixel(vegetation_[p].polygon.GetPoint(i));
			glm::vec3 color = colors_.GetColor(p);
			image_.draw_line(p0.x, p0.y, p1.x, p1.y, (float *)&color, 1.0);
		}
		if (np >= 3) {
			glm::ivec2 p0 = CoordinateToPixel(vegetation_[p].polygon.GetPoint(np - 1));
			glm::ivec2 p1 = CoordinateToPixel(vegetation_[p].polygon.GetPoint(0));
			glm::vec3 color = colors_.GetColor(p);
			image_.draw_line(p0.x, p0.y, p1.x, p1.y, (float *)&color, 1.0);
		}
		for (int v = 0; v < (int)vegetation_[p].positions.size(); v++) {
			glm::ivec2 pv = CoordinateToPixel(vegetation_[p].positions[v]);
			glm::vec3 color = colors_.GetColor(p);
			image_.draw_circle(pv.x, pv.y, 1, (float *)&color, 1.0);
		}
	}
	for (int t = 0; t < (int)paths_.size(); t++) {
		for (int p = 0; p < (paths_[t].NumWaypoints()-1); p++) {
			glm::ivec2 p0 = CoordinateToPixel(paths_[t].GetWaypoint(p));
			glm::ivec2 p1 = CoordinateToPixel(paths_[t].GetWaypoint(p+1));
			image_.draw_line(p0.x, p0.y, p1.x, p1.y, (float *)&green, 1.0);
		}
	}

	glm::ivec2 origin_pix = CoordinateToPixel(origin);
	image_.draw_circle(origin_pix.x, origin_pix.y, 3, (float *)&blue, 1.0);

	std::string inst_str = "Press H to display help.";
	image_.draw_text(15, 15, inst_str.c_str(), yellow);
}

void SceneEditor::CalculateDimensions() {
	float dx = current_ur_.x - current_ll_.x;
	float dy = current_ur_.y - current_ll_.y;
	current_center_ = 0.5f*(current_ur_ + current_ll_);
	int nx = (int)ceil(dx / pixres_);
	int ny = (int)ceil(dy / pixres_);
	int nmax = std::max(nx, ny);
	if (nmax > maxdim_) {
		float scale = ((float)nmax) / ((float)maxdim_);
		current_pixres_ = pixres_ * scale;
		nx = (int)ceil(dx / current_pixres_);
		ny = (int)ceil(dy / current_pixres_);
	}
	glm::vec3 position(current_center_.x, current_center_.y, ur_.z + 1000.0f);
	cam_.Initialize(nx, ny, dx, dy, 1.0f);
	cam_.SetPose(position, orientation_);
	disp_.resize(cam_.GetWidth(), cam_.GetHeight());
}

glm::mat3x4 SceneEditor::GetRandomRotScale() {
	glm::mat3x4 rot_scale;
	glm::vec3 euler_angles;
	euler_angles.x = 0.0f;
	euler_angles.y = mavs::math::rand_in_range(0.0f, (float)mavs::k2Pi);
	euler_angles.z = 0.0f;
	glm::mat3x3 om = glm::orientate3(euler_angles);
	float s = mavs::math::rand_in_range(0.85f, 1.15f);
	for (int ii = 0; ii<3; ii++) {
		for (int jj = 0; jj<3; jj++) {
			rot_scale[ii][jj] = s*om[ii][jj];
		}
	}
	return rot_scale;
}

void SceneEditor::SaveScene() {
	if (paths_.size() > trails_.size())CopyPathsToTrails();

	std::string layer_file = mavs::io::GetInputFileName("Select surface layer file", "*.json");
	if (layer_file.size() < 4)return;

	std::string fname = mavs::io::GetSaveFileName("Select json output file", "scene.json", "*.json");
	if (fname.size() < 5) return;

	rapidjson::StringBuffer s;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
	writer.SetIndent(' ', 2);

	std::string mesh_path = mavs::utils::GetPathFromFile(surface_file_);
	mesh_path.append("/");
	std::string surface_mesh_nopath = surface_file_;
	mavs::utils::EraseSubString(surface_mesh_nopath, mesh_path);

	writer.StartObject(); {
		writer.Key("PathToMeshes");
		writer.String(mesh_path.c_str());

		writer.Key("UseFullPaths");
		writer.Bool(true);

		writer.Key("Object Labels");
		writer.String("labels.json");

		writer.Key("Layered Surface");
		writer.StartObject(); {
			writer.Key("Mesh");
			writer.String(surface_file_.c_str());
			if (trails_.size() > 0) {
				writer.Key("Trail");
				writer.StartObject(); {
					writer.Key("Trail Width");
					writer.Double(trails_[0].GetTrailWidth());
					writer.Key("Track Width");
					writer.Double(trails_[0].GetTrackWidth());
					writer.Key("Wheelbase");
					writer.Double(trails_[0].GetWheelbase());
					writer.Key("Path");
					writer.StartArray();
					for (int tn = 0; tn < trails_.size(); tn++) {
						writer.String(trails_[tn].GetPathFile().c_str());
					}
					writer.EndArray();
					writer.EndObject(); } //trail_object
			}
			writer.Key("Layers");
			writer.String(layer_file.c_str());
			writer.EndObject(); }

		//surface mesh array
		writer.Key("Surface Mesh");
		writer.StartArray(); {
			writer.StartObject(); {
				writer.Key("Mesh");
				//writer.String(surface_mesh_.GetName().c_str());
				writer.String(surface_file_.c_str());
				writer.Key("Rotate Y to Z");
				writer.Bool(false);
				writer.Key("YawPitchRoll");
				writer.StartArray();
				for (int i = 0; i < 3; i++)writer.Double(0.0);
				writer.EndArray();
				writer.Key("Position");
				writer.StartArray();
				for (int i = 0; i < 3; i++)writer.Double(0.0);
				writer.EndArray();
				writer.Key("Scale");
				writer.StartArray();
				for (int i = 0; i < 3; i++)writer.Double(1.0);
				writer.EndArray();
				writer.EndObject(); }
			writer.EndArray(); }
		// End surface mesh array

		// Array of objects
		writer.Key("Objects");
		writer.StartArray(); {
			/*
			// First write the surface
			writer.StartObject(); {
				writer.Key("Mesh");
				writer.String(surface_mesh_nopath.c_str());
				writer.Key("Instances");
				writer.StartArray(); {
					writer.StartObject(); {
						writer.Key("YawPitchRoll");
						writer.StartArray();
						for (int i = 0; i < 3; i++)writer.Double(0.0);
						writer.EndArray();
						writer.Key("Position");
						writer.StartArray();
						for (int i = 0; i < 3; i++)writer.Double(0.0);
						writer.EndArray();
						writer.Key("Scale");
						writer.StartArray();
						for (int i = 0; i < 3; i++)writer.Double(1.0);
						writer.EndArray();
						writer.EndObject(); }
					writer.EndArray(); }// of instances
				writer.EndObject(); }// surface object
			*/
			//now loop through veg object instances
			for (int ns = 0; ns < (int)vegetation_.size(); ns++) {
				writer.StartObject();
				writer.Key("Mesh");
				std::string pname = vegetation_[ns].mesh_name;
				mavs::utils::EraseSubString(pname, mesh_path);
				writer.String(pname.c_str());
				writer.Key("Random");
				writer.StartObject(); {
					writer.Key("Offset");
					writer.StartArray();
					writer.Double(0.0);
					writer.Double(0.0);
					writer.Double(0.0);
					writer.EndArray();
					writer.Key("Number");
					writer.Int((int)vegetation_[ns].positions.size());
					writer.Key("Polygon");
					writer.StartArray();
					for (int p = 0; p < vegetation_[ns].polygon.NumPoints(); p++) {
						writer.StartArray();
						writer.Double(vegetation_[ns].polygon.GetPoint(p).x);
						writer.Double(vegetation_[ns].polygon.GetPoint(p).y);
						writer.EndArray();
					}
					writer.EndArray();
					writer.Key("Scale");
					writer.StartArray();
					writer.Double(0.85);
					writer.Double(1.15);
					writer.EndArray();
					writer.EndObject(); } //Random object
				writer.EndObject();
			} //loop through polygons

			writer.EndArray(); }
		// done with array of objects

		writer.EndObject(); }
	std::string outstring = s.GetString();
	std::ofstream fout(fname.c_str());
	fout << outstring;
	fout.close();
} // SaveScene

void SceneEditor::ViewScene() {
	mavs::environment::Environment new_env;
	mavs::raytracer::embree::EmbreeTracer new_scene;
	mavs::raytracer::Mesh surface_mesh;
	surface_mesh.Load(surface_path_, surface_file_);
	int mesh_id;
	new_scene.AddMesh(surface_mesh, unit_rot_scale_, 0, 0, mesh_id);
	new_scene.SetSurfaceMesh(surface_mesh, unit_rot_scale_);
	for (int v = 0; v < vegetation_.size(); v++) {
		std::string path_to_mesh = mavs::utils::GetPathFromFile(vegetation_[v].mesh_name);
		path_to_mesh.append("/");
		mavs::raytracer::Mesh mesh;
		mesh.Load(path_to_mesh, vegetation_[v].mesh_name);
		for (int p = 0; p < (int)vegetation_[v].positions.size(); p++) {
			glm::vec2 pos = vegetation_[v].positions[p];
			float h = scene_.GetSurfaceHeight(pos.x, pos.y);
			glm::mat3x4 rot_scale = GetRandomRotScale();
			rot_scale[0][3] = pos.x;
			rot_scale[1][3] = pos.y;
			rot_scale[2][3] = h;
			int mesh_id;
			new_scene.AddMesh(mesh, rot_scale, p, 0,mesh_id);
		}
	}
	new_scene.CommitScene();
	new_scene.SetLoaded(true);
	new_env.SetRaytracer(&new_scene);
	mavs::sensor::camera::RgbCamera free_camera;
	free_camera.Initialize(384, 384, 0.0035f, 0.0035f, 0.0035f);
	free_camera.SetRenderShadows(false);
	free_camera.SetAntiAliasing("oversampled");
	free_camera.SetPixelSampleFactor(1);
	free_camera.SetEnvironmentProperties(&new_env);
	free_camera.FreePose();
	glm::vec3 fc_pos(center_.x, center_.y, ur_.z);
	glm::quat fc_orient(1.0f, 0.0f, 0.0f, 0.0f);
	free_camera.SetPose(fc_pos, fc_orient);
	free_camera.Update(&new_env, 0.1);
	free_camera.Display();
	while (free_camera.DisplayOpen()) {
		free_camera.Update(&new_env, 0.1);
		free_camera.Display();
	}
}

void SceneEditor::SelectVegDistro() {

}

void SceneEditor::SelectTrail() {
	bool finished = false;
	std::vector<glm::ivec2> uv_points;
	while (!finished) {
		switch (disp_.key()) {
		case cimg_library::cimg::keyQ: finished = true; break;
		}
		cimg_library::CImg <int> SelectedImageCoords = image_.get_select(disp_, 0, 0, true);
		glm::ivec2 pix;
		pix.x = SelectedImageCoords(0);
		pix.y = SelectedImageCoords(1);
		if (pix.x > 0 && pix.y > 0) {
			glm::vec3 p = PixelToCoordinate(pix.x, pix.y);
			image_.draw_circle(pix.x, pix.y, 2, (float *)&yellow, 1.0);
			if (uv_points.size() > 0) {
				image_.draw_line(uv_points.back().x, uv_points.back().y, pix.x, pix.y, (float *)&yellow, 1.0);
			}
			uv_points.push_back(pix);
		}
	}
	bool happy = mavs::io::GetUserBool("Happy with trail?", "Yes/No");
	if (happy) {
		mavs::Waypoints trail;
		for (int i = 0; i < (int)uv_points.size(); i++) {
			glm::vec3 p = PixelToCoordinate(uv_points[i].x, uv_points[i].y);
			glm::vec2 p2(p.x, p.y);
			trail.AddPoint(p2);
		}
		paths_.push_back(trail);
	}
}

void SceneEditor::AddPolygon() {
	std::vector<glm::vec2> poly_points;
	std::vector<glm::ivec2> uv_points;
	bool finished = false;
	while (!finished) {
		cimg_library::CImg <int> SelectedImageCoords = image_.get_select(disp_, 0, 0, true);
		glm::ivec2 pix;
		pix.x = SelectedImageCoords(0);
		pix.y = SelectedImageCoords(1);
		glm::vec3 p = PixelToCoordinate(pix.x, pix.y);
		poly_points.push_back(glm::vec2(p.x, p.y));
		image_.draw_circle(pix.x, pix.y, 2, (float *)&yellow, 1.0);
		if (uv_points.size() > 0) {
			image_.draw_line(uv_points.back().x, uv_points.back().y, pix.x, pix.y, (float *)&yellow, 1.0);
		}
		uv_points.push_back(pix);
		for (int i = 1; i < uv_points.size(); i++) {
			glm::ivec2 v = (uv_points[i] - uv_points[0]);
			float d = (float)sqrt(v.x*v.x + v.y*v.y);
			if (d < 3.5f) {
				finished = true;
				poly_points.pop_back();
			}
		}
	}
	if (poly_points.size() >= 3) {
		image_.draw_line(uv_points.back().x, uv_points.back().y, uv_points[0].x, uv_points[0].y, (float *)&yellow, 1.0);
		bool happy = mavs::io::GetUserBool("Happy with polygon?", "Yes/No");
		if (happy) {
			VegDistribution veggie;
			veggie.polygon = mavs::math::Polygon(poly_points);
			float density = mavs::io::GetUserNumericInput("Set vegetation density", "Desired density in #/m^2");
			veggie.mesh_name = mavs::io::GetInputFileName("Set the associated mesh", "*.obj");
			float area = veggie.polygon.GetArea();
			int num_to_add = (int)(density * area);
			std::vector<glm::vec2> veg_pos;
			for (int n = 0; n < num_to_add; n++) {
				glm::vec2 p = veggie.polygon.GetRandomInside();
				veggie.positions.push_back(p);
			}
			vegetation_.push_back(veggie);
		} //if happy
	}
	else {
		mavs::io::DisplayUserInfo("ERORR!", "Must select at least 3 point in polygon");
	}
}

void SceneEditor::Zoom() {
	cimg_library::CImg <int> SelectedImageCoords = image_.get_select(disp_, 2, 0, 0);
	int nx = SelectedImageCoords(3) - SelectedImageCoords(0);
	int ny = SelectedImageCoords(4) - SelectedImageCoords(1);
	if (nx > 0 && ny > 0) {
		glm::vec3 new_ll = PixelToCoordinate(SelectedImageCoords(0), SelectedImageCoords(1));
		glm::vec3 new_ur = PixelToCoordinate(SelectedImageCoords(3), SelectedImageCoords(4));
		current_ll_.x = new_ll.x;
		current_ll_.y = new_ur.y;
		current_ur_.x = new_ur.x;
		current_ur_.y = new_ll.y;
		CalculateDimensions();
		cam_.Update(&env_, 0.03);
	}
}

void SceneEditor::Unzoom() {
	current_ll_ = ll_;
	current_ur_ = ur_;
	current_center_ = center_;
	CalculateDimensions();
	cam_.Update(&env_, 0.03);
}

void SceneEditor::DisplayPointCoordinates() {
	cimg_library::CImg <int> SelectedImageCoords = image_.get_select(disp_, 0, 0, 0);
	int u = SelectedImageCoords(0);
	int v = SelectedImageCoords(1);
	glm::vec3 p = PixelToCoordinate(u, v);
	std::cout << "Selected Pixel " << u << " " << v << " " << p.x << " " << p.y << std::endl;
}

void SceneEditor::LoadSurface(std::string infile) {
	//--- Load the mesh, create the scene --- //
	surface_file_ = infile; // std::string(argv[1]);
	surface_path_ = mavs::utils::GetPathFromFile(surface_file_);
	surface_path_.append("/");
	mavs::raytracer::Mesh surface_mesh;
	surface_mesh.Load(surface_path_, surface_file_);
	int mesh_id;
	scene_.AddMesh(surface_mesh, unit_rot_scale_, 0, 0,mesh_id);
	scene_.CommitScene();
	scene_.SetLoaded(true);
	env_.SetRaytracer(&scene_);

	//--- Calculate initial dimensions of the scene ---//
	ur_ = scene_.GetUpperRightCorner();
	ll_ = scene_.GetLowerLeftCorner();
	center_ = 0.5f*(ur_ + ll_);
	current_ll_ = ll_;
	current_ur_ = ur_;
	current_center_ = center_;
	CalculateDimensions();

	//--- Render the initial display ---//
	cam_.Update(&env_, 0.03);
	disp_.set_title("Scene");
	image_ = cam_.GetCurrentImage();
	disp_.display(image_);
}

void SceneEditor::RunEditLoop(){
	//--- Perform the scene editing loop ---//
	while (!disp_.is_closed() ) {
		GetKeyboardInput();
		image_ = cam_.GetCurrentImage();
		DrawPolygons();
		disp_.display(image_);
	}
}

void SceneEditor::GetSceneStats() {
	int num_facets = 0;
	int num_plants = 0;
	mavs::raytracer::Mesh surface_mesh;
	surface_mesh.Load(surface_path_, surface_file_);
	num_facets += (int)surface_mesh.GetNumFaces();
	for (int v = 0; v < vegetation_.size(); v++) {
		std::string path_to_mesh = mavs::utils::GetPathFromFile(vegetation_[v].mesh_name);
		path_to_mesh.append("/");
		mavs::raytracer::Mesh mesh;
		mesh.Load(path_to_mesh, vegetation_[v].mesh_name);
		num_facets += (int)(mesh.GetNumFaces()*vegetation_[v].positions.size());
		num_plants += (int)vegetation_[v].positions.size();
	}
	std::cout << "Scene has " << num_facets << " triangles and " << (num_plants + 1) << " objects." << std::endl;
}

}//namespace utils
} //namespace mavs
