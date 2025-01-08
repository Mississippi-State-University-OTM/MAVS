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
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <mavs_core/terrain_generator/random_scene.h>

#include <iostream>

#include <mavs_core/terrain_generator/ecosystem.h>
#include <mavs_core/terrain_generator/grd_surface.h>
#include <mavs_core/terrain_generator/random_surface.h>
#include <raytracers/texture_layers/layered_surface.h>
#include <mavs_core/math/utils.h>
#include <tinyfiledialogs.h>

namespace mavs {
namespace terraingen {

RandomScene::RandomScene() {
	loaded_ = false;
	use_dem_ = false;
	use_vprp_ = false;
	gap_width_ = 0.0f;
	gap_depth_ = 0.0f;
	gap_angle_radians_ = 0.0f;
}

void RandomScene::CheckOutputDirectoryIsValid() {
	// First check if the requested directory exists
	if (!utils::path_is_valid(inputs_.output_directory)) {
		std::cerr << "ERROR: Asked to create scene in directory " << inputs_.output_directory << ", which does not exist." << std::endl;
		std::cout << "Select new output directory:" << std::endl;
		std::string message_string("Current output directory " + inputs_.output_directory + " is invalid.\n Press OK to select a new folder.");
		bool set_new = tinyfd_messageBox("Select new output directory for random scene.", message_string.c_str(), "ok", "info", 1);
		char const * lTheSelectFolderName;
		lTheSelectFolderName = tinyfd_selectFolderDialog("Select output directory", NULL);
		if (lTheSelectFolderName) {
			inputs_.output_directory = std::string(lTheSelectFolderName);
		}
	}
}

void RandomScene::SetInputs(RandomSceneInputs inputs) {
	inputs_ = inputs;
	CheckOutputDirectoryIsValid();
	loaded_ = true;
}

void RandomScene::Load(std::string infile) {
	if (!mavs::utils::file_exists(infile)) {
		std::cerr << "ERROR: Could not find file " << infile << std::endl;
		exit(1);
	}
	FILE* fp = fopen(infile.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	//RandomSceneInputs scene;

	if (d.HasMember("Output Directory")) {
		inputs_.output_directory = d["Output Directory"].GetString();
	}
	else {
		inputs_.output_directory = "./";
	}

	CheckOutputDirectoryIsValid();

	if (d.HasMember("DEM File")) {
		std::string df = d["DEM File"].GetString();
		if (df.length() > 3) {
			inputs_.dem_file = df;
			use_dem_ = true;
		}
	}

	if (d.HasMember("Terrain Width")) {
		inputs_.terrain_width = d["Terrain Width"].GetFloat();
	}
	else {
		inputs_.terrain_width = 10.0f;
	}

	if (d.HasMember("Terrain Length")) {
		inputs_.terrain_length = d["Terrain Length"].GetFloat();
	}
	else {
		inputs_.terrain_length = 10.0f;
	}
	bool potholes = false;
	if (d.HasMember("Potholes")) {
		potholes = d["Potholes"].GetBool();
	}
	float pothole_depth = 0.0f;
	if (d.HasMember("Pothole Depth")) {
		 float pothole_depth = d["Pothole Depth"].GetFloat();
	}
	float pothole_diameter = 0.0f;
	if (d.HasMember("Pothole Diameter")) {
		pothole_diameter = d["Pothole Diameter"].GetFloat();
	}

	if (d.HasMember("Low Freq Mag")) {
		inputs_.lo_mag = d["Low Freq Mag"].GetFloat();
	}
	else {
		inputs_.lo_mag = 10.0f;
	}

	if (d.HasMember("High Freq Mag")) {
		inputs_.hi_mag = d["High Freq Mag"].GetFloat();
	}
	else {
		inputs_.hi_mag = 0.1f;
	}

	if (d.HasMember("Mesh Resolution")) {
		inputs_.mesh_resolution = d["Mesh Resolution"].GetFloat();
	}
	else {
		inputs_.mesh_resolution = 0.25f;
	}

	if (d.HasMember("Path File")) {
		std::string vp = d["Path File"].GetString();
		if (vp.length() > 3) {
			inputs_.vprp_file = vp;
			use_vprp_ = true;
		}
	}

	if (d.HasMember("Trail Width")) {
		inputs_.trail_width = d["Trail Width"].GetFloat();
	}
	else {
		inputs_.trail_width = 20.0f;
	}

	if (d.HasMember("Track Width")) {
		inputs_.track_width = d["Track Width"].GetFloat();
	}
	else {
		inputs_.track_width = 0.6f;
	}

	if (d.HasMember("Wheelbase")) {
		inputs_.wheelbase = d["Wheelbase"].GetFloat();
	}
	else {
		inputs_.wheelbase = 1.25f;
	}

	if (d.HasMember("Plant Density")) {
		inputs_.plant_density = d["Plant Density"].GetFloat();
	}
	else {
		inputs_.plant_density = 1.0f;
	}

	if (d.HasMember("Base Name")) {
		inputs_.basename = d["Base Name"].GetString();
	}
	else {
		inputs_.basename = "ecosystem";
	}

	if (d.HasMember("Path Type")) {
		inputs_.path_type = d["Path Type"].GetString();
	}
	else {
		inputs_.path_type = "Ridges";
	}

	if (d.HasMember("Ecosystem File")) {
		inputs_.eco_file = d["Ecosystem File"].GetString();
	}
	else {
		std::cerr << "No ecosystem file given in json file, exiting " << std::endl;
		exit(2);
	}
	loaded_ = true;
}

void RandomScene::PrintInputs() {
	std::cout << inputs_.terrain_width << " " << inputs_.terrain_length << " " << inputs_.lo_mag << " " << inputs_.hi_mag << " " << inputs_.mesh_resolution << std::endl;
	std::cout << inputs_.trail_width << " " << inputs_.track_width << " " << inputs_.wheelbase << " " << inputs_.plant_density << std::endl;
	std::cout << inputs_.path_type << " " << inputs_.eco_file << " " << inputs_.output_directory << " " << inputs_.basename << std::endl;
	std::cout << inputs_.dem_file << " " << inputs_.vprp_file << std::endl;
	//std::cout << inputs_.potholes << " " << inputs_.pothole_depth << " " << inputs_.pothole_diameter << std::endl;
}

void RandomScene::Create() {
	if (!loaded_) {
		std::cerr << "WARNING: No scene was created because inputs haven't been loaded." << std::endl;
		return;
	}
	
	// create and save a surface
	std::string surfout = inputs_.output_directory;
	surfout.append("/");
	surfout.append(inputs_.basename);
	surfout.append("_surface");

	mavs::Trail trail;
	mavs::terraingen::HeightMap heightmap;
	if (use_dem_) {
		mavs::terraingen::GridSurface surface;
		surface.LoadAscFile(inputs_.dem_file);
		surface.Display();
		surface.RemoveNoDataCells();
		//surface.Smooth(1.5f);
		surface.Recenter();
		surface.WriteToPointsFile("points.txt");
		surface.WriteObj(surfout);
		if (use_vprp_) {
			trail.LoadPath(inputs_.vprp_file);
		}
		else {
			trail = surface.GetTrail(inputs_.path_type);
		}
		heightmap = surface.GetHeightMap();
	} // use DEM 
	else {
		mavs::terraingen::RandomSurface surface;
		//surface.SetMeshResolution(inputs_.mesh_resolution);
		surface.SetRoughnessParams(50.0f, inputs_.lo_mag, 2.0f, inputs_.hi_mag);
		//surface.SetCorners(-0.5*inputs_.terrain_width, -0.5*inputs_.terrain_length, 0.5*inputs_.terrain_width, 0.5*inputs_.terrain_length);

		if (inputs_.surface_rough_type == "variable") {
			surface.GenerateVariableRoughness(-0.5f*inputs_.terrain_width, -0.5f*inputs_.terrain_length, 0.5f*inputs_.terrain_width, 0.5f*inputs_.terrain_length, inputs_.mesh_resolution);
		}
		else if (inputs_.surface_rough_type == "gaussian") {
			surface.GenerateGaussianHeightMap(-0.5f*inputs_.terrain_width, -0.5f*inputs_.terrain_length, 0.5f*inputs_.terrain_width, 0.5f*inputs_.terrain_length, inputs_.mesh_resolution);
		}
		else if (inputs_.surface_rough_type == "gap") {
			surface.GenerateHeightmapWithGap(-0.5f*inputs_.terrain_width, -0.5f*inputs_.terrain_length, 0.5f*inputs_.terrain_width, 0.5f*inputs_.terrain_length, inputs_.mesh_resolution, gap_depth_, gap_width_, gap_angle_radians_);
		}
		else { // use perlin by default
			surface.GenerateHeightMap(-0.5f*inputs_.terrain_width, -0.5f*inputs_.terrain_length, 0.5f*inputs_.terrain_width, 0.5f*inputs_.terrain_length, inputs_.mesh_resolution);
		}

		if (use_vprp_) {
			trail.LoadPath(inputs_.vprp_file);
		}
		else {
			trail = surface.GetTrail(inputs_.path_type);
		}
		//if (inputs_.potholes.size()>0) {
		for (int p=0;p<inputs_.potholes.size();p++){
			surface.AddPotholeAtLocation(inputs_.potholes[p].location.x, inputs_.potholes[p].location.y, inputs_.potholes[p].diameter*0.5f, inputs_.potholes[p].depth);
			//surface.AddPotholes(inputs_.pothole_depth,inputs_.pothole_diameter,inputs_.number_potholes,inputs_.pothole_locations);
		}
		WritePotholeFile(inputs_.potholes, inputs_.output_directory+"/potholes_"+inputs_.basename+".txt");
		surface.WriteObj(surfout);
		heightmap = surface.GetHeightMap();
	} //don't use DEM
	

	// create and save trail from surface
	//mavs::Trail trail = surface.GetTrail(inputs_.path_type);
	trail.SetTrackWidth(inputs_.track_width);
	trail.SetTrailWidth(inputs_.trail_width);
	trail.SetWheelbase(inputs_.wheelbase);

	if (use_vprp_) {
		trail.SetPathFile(inputs_.vprp_file);
	}
	else {
		std::string pathout = inputs_.output_directory;
		pathout.append("/");
		pathout.append(inputs_.basename);
		pathout.append("_path.vprp");
		//std::string trail_name("temp_path.vprp");
		trail.SaveToAnvelVprp(pathout);
		trail.SetPathFile(pathout);
	}

	//create ecosystem
	mavs::terraingen::Ecosystem ecosystem;
	ecosystem.Load(inputs_.eco_file);
	//set heightmap, surface mesh, and trail of ecosystem
	std::string surfin = surfout;
	surfin.append(".obj");
	ecosystem.LoadTerrain(surfin);
	ecosystem.SetHeightMap(heightmap);

	ecosystem.SetTrail(trail);
	ecosystem.SetGrowthFactor(inputs_.plant_density);

	//simulate plant growth
	ecosystem.Simulate();
	// Save resulting scene to a file
	//set up file name
	std::string scene_out = inputs_.output_directory;
	scene_out.append("/");
	scene_out.append(inputs_.basename);
	std::string truth_out = scene_out;
	scene_out.append("_scene.json");
	//save output files

	std::string pot_outfile = inputs_.output_directory;
	pot_outfile.append("/potholes_");
	pot_outfile.append(inputs_.basename + ".txt");
	ecosystem.SetPotholeFile(pot_outfile);

	ecosystem.SaveToMavsScene(scene_out);
	truth_out.append("_truth.txt");
	ecosystem.SaveEcosystemToFile(truth_out);
	
}

void RandomScene::WritePotholeFile(std::vector<Pothole> potholes, std::string outfile) {
	std::ofstream fout(outfile.c_str());
	for (int i = 0; i < potholes.size(); i++) {
		fout << potholes[i].depth << " " << potholes[i].diameter << " " << potholes[i].location.x << " " << potholes[i].location.y << std::endl;
	}
	fout.close();
}

} //namespace terraingen
} //namespace mavs