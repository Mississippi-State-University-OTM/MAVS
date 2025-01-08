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
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
//#include "thirdparty/rapidjson/writer.h"

#include <mavs_core/terrain_generator/ecosystem.h>

#ifdef USE_OMP
#include <omp.h>
#endif

#include <iostream>
#include <fstream>
#include <limits>
#include <algorithm>

#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>
#include <mavs_core/pose_readers/anvel_vprp_reader.h>

// The following statement must be included because
// There is a libX11 header that defines Bool as a macro
// leading to conflict with rapijson writer.Bool
// see https://forum.openframeworks.cc/t/error-building-with-rapidjson/20251
#ifdef Bool
#undef Bool
#endif

namespace mavs {
namespace terraingen {

Ecosystem::Ecosystem() {
	// model parameters
	ground_clearance_ = 0.3f;
	use_textures_ = false;
	growth_factor_ = 1.0f;
	//other params
	grid_cell_size_ = 5.0f;
	nx_cells_ = 0;
	ny_cells_ = 0;
	terrain_loaded_ = false;
	sim_length_years_ = 15;
}

void Ecosystem::SetTrail(Trail &trail) {
	trails_.resize(1);
	trails_[0] = trail;
	InitializeTrail();
}

void Ecosystem::AddTrail(Trail &trail) {
	trails_.push_back(trail);
	InitializeTrail();
}

void Ecosystem::AddPlants(){
	float half_trail_width = 0.0f;
	if (trails_.size() > 0)half_trail_width = 0.5f*trails_[0].GetTrailWidth();
	for (int ns = 0; ns < (int)species_.size(); ns++) {
		float newfac = ecosys_area_ * growth_factor_;
		int nplants = species_[ns].GetNumNew(newfac);
		for (int i = 0; i < nplants; i++) {
			Plant p;
			p.SetSpecies(species_[ns]);
			glm::vec2 pos(math::rand_in_range(llc_.x, urc_.x),
				math::rand_in_range(llc_.y, urc_.y));
			bool on_tracks = false;
			for (int tn = 0; tn < (int)trails_.size(); tn++) {
				if (trails_[tn].IsPointInTracks(pos))on_tracks = true;
			}
			if (!on_tracks) {
				for (int tn = 0; tn < (int)trails_.size(); tn++) {
					float dp;
					if (trails_[tn].IsPointOnTrail(pos, half_trail_width,dp) && p.GetMaxHeight() > ground_clearance_) {
						p.SetMaxHeight(ground_clearance_);
						break;
					}
				}
				p.SetPosition(pos);
				plants_.push_back(p);
				p.Grow();
				plants_.back().SetHeight(math::rand_in_range(0.0f, p.GetHeight()));
			}
		}
	}
}

bool Ecosystem::PlantsOverlap(Plant *plant1, Plant *plant2) {
	float dist = glm::length((plant1->GetPosition() - plant2->GetPosition()));
	if (dist < (plant1->GetRadius() + plant2->GetRadius())) {
		return true;
	}
	else {
		return false;
	}
}

void Ecosystem::Step() {
	//----- First add new growth--------------
	AddPlants();

	//---- remove the dying plants ----------------
	std::vector<Plant> living_plants;
	for (int i = 0; i < (int)plants_.size(); i++) {
		if (plants_[i].IsAlive()) {
			living_plants.push_back(plants_[i]);
		}
	}
	int num_plants = (int)plants_.size();
	int num_living = (int)living_plants.size();
	int num_dead = num_plants - num_living;
	plants_.clear();
	plants_ = living_plants;

	//---- clear the existing cells ---------------
	for (int i = 0; i < nx_cells_; i++) {
		for (int j = 0; j < ny_cells_; j++) {
			cells_[i][j].clear();
		}
	}

	//---- grow the plants ------------------------
	for (int i = 0; i < (int)plants_.size(); i++) {
		glm::vec2 p_pos = plants_[i].GetPosition();
		int cx = (int)floor((p_pos.x - llc_.x) / grid_cell_size_);
		int cy = (int)floor((p_pos.y - llc_.y) / grid_cell_size_);
		if (cx >= 0 && cy >= 0 && cx < (int)cells_.size() && cy < (int)cells_[0].size()) {
			cells_[cx][cy].push_back(i);
			plants_[i].SetCell(cx, cy);
			plants_[i].Grow();
		}
	}

	//---- check for competing plants ------------
#ifdef USE_OMP  
	#pragma omp parallel for schedule(dynamic)
#endif 
	for (int i = 0; i < (int)plants_.size(); i++) {
		glm::ivec2 cell = plants_[i].GetCell();
		int xlo = std::max(0, cell.x-1);
		int xhi = std::min(nx_cells_ - 1, cell.x + 1);
		int ylo = std::max(0, cell.y - 1);
		int yhi = std::min(ny_cells_ - 1, cell.y + 1);
		for (int xi = xlo; xi <= xhi; xi++) {
			for (int yi = ylo; yi <= yhi; yi++) {
				for (int k = 0; k < (int)cells_[xi][yi].size(); k++) {
					int j = cells_[xi][yi][k];
					if (i != j) {
						if (PlantsOverlap(&plants_[i], &plants_[j])) {
							float ri = plants_[i].GetRadius();
							float rj = plants_[j].GetRadius();
							if (ri > rj) {
								plants_[j].Kill();
							}
							else {
								plants_[i].Kill();
							}
						}
					}
				}
			}
		}
	}
	// ---- done checking for competing plants --------
}

void Ecosystem::PruneRunts() {
	// find runts
	for (int i = 0; i < (int)plants_.size(); i++) {
		float prune_height = plants_[i].GetSpecies()->GetMinHeight();
		if (plants_[i].GetHeight() < prune_height) {
			plants_[i].Kill();
		}
	}

	//remove runts
	std::vector<Plant> living_plants;
	for (int i = 0; i < (int)plants_.size(); i++) {
		if (plants_[i].IsAlive()) {
			living_plants.push_back(plants_[i]);
		}
	}
	plants_.clear();
	plants_ = living_plants;
}

void Ecosystem::Simulate() {
	std::cout << "Ecosystem year ";
	for (int t = 0; t < sim_length_years_; t++) {
		std::cout<<t <<"... ";
		Step();
	}
	std::cout << std::endl;
	PruneRunts();
}

void Ecosystem::LoadTerrain(std::string terrain_file) {
	std::string file_path = utils::GetPathFromFile(terrain_file);
	file_path += "/";
	if (utils::file_exists(terrain_file)) {
		surface_mesh_.Load(file_path, terrain_file);
		surface_mesh_file_ = terrain_file;
		terrain_loaded_ = true;
	}
	else {
		std::cerr << "ERROR: Could not find terrain file " <<
			terrain_file << ", using flat terrain. " << std::endl;
	}
}

void Ecosystem::InitializeTrail() {
	if (trails_.size() > 0) {
		//path_.CreateFromAnvelVprp(path_file);
		float trail_width = trails_[0].GetTrailWidth();
		llc_ = surf_.GetLLCorner(); // trail_.GetPath()->GetLowerLeft();
		urc_ = surf_.GetURCorner(); // trail_.GetPath()->GetUpperRight();
		llc_.x = llc_.x - trail_width;
		llc_.y = llc_.y - trail_width;
		urc_.x = urc_.x + trail_width;
		urc_.y = urc_.y + trail_width;

		float lx = surf_.GetURCorner().x - surf_.GetLLCorner().x;
		float ly = surf_.GetURCorner().y - surf_.GetLLCorner().y; //urc_.y - llc_.y;
		ecosys_area_ = lx * ly;
		grid_cell_size_ = std::max(grid_cell_size_, 0.001f*0.001f*ecosys_area_);
		nx_cells_ = 1 + (int)ceil(lx / grid_cell_size_);
		ny_cells_ = 1 + (int)ceil(ly / grid_cell_size_);
		cells_.clear();
		std::vector<plant_list> column;
		column.resize(ny_cells_);
		cells_.resize(nx_cells_, column);
	}
}


void Ecosystem::Load(std::string input_file) {
	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	mavs::MavsDataPath data_path;
	std::string base_path = data_path.GetPath();
	base_path += "/";

	if (d.HasMember("Texture Inputs")) {
		texture_file_ = base_path + d["Texture Inputs"].GetString();
		//LoadTerrain(surface_mesh);
		use_textures_ = true;
	}
	else {
		std::cerr << "No Textures listed, surface will not have textures." << std::endl;
		return;
	}

	if (d.HasMember("Surface Mesh")) {
		//surface_mesh_file_ = d["Surface Mesh"].GetString();
		std::string surface_mesh_file_ = base_path + d["Surface Mesh"].GetString();
		LoadTerrain(surface_mesh_file_);
	}
	else {
		std::cerr << "ERROR: No surface listed for ecosystem, sim will not run" << std::endl;
		return;
	}

	if (d.HasMember("Simulation Length")) {
		sim_length_years_ = d["Simulation Length"].GetInt();
	}

	if (d.HasMember("Anvel Replay File")) {
		if (d["Anvel Replay File"].IsArray()) {
			for (int tn = 0; tn < (int)d["Anvel Replay File"].Capacity(); tn++) {
				Trail trail;
				std::string path_file = base_path + d["Anvel Replay File"][tn].GetString();
				trail.LoadPath(path_file);
				trails_.push_back(trail);
			}
		}
		else {
			Trail trail;
			std::string path_file = base_path + d["Anvel Replay File"].GetString();
			trail.LoadPath(path_file);
			trails_.push_back(trail);
		}
		
	}
	else {
		std::cerr << "ERROR: No path listed through ecosystem, sim will not run" << std::endl;
		return;
	}

	if (d.HasMember("PathToMeshes")) {
		std::string path_to_meshes = d["PathToMeshes"].GetString();
		SetPathToMeshes(path_to_meshes);
	}
	else {
		std::string path_to_meshes = base_path + "scenes/meshes/";
		SetPathToMeshes(path_to_meshes);
	}
	if (d.HasMember("Trail Properties") && trails_.size()>0) {
		const rapidjson::Value& trail = d["Trail Properties"];
		if (trail.HasMember("Tire Width")) {
			float width = trail["Tire Width"].GetFloat();
			trails_[0].SetTrackWidth(width);
		}
		if (trail.HasMember("Wheelbase")) {
			float wb = trail["Wheelbase"].GetFloat();
			trails_[0].SetWheelbase(wb);
		}
		if (trail.HasMember("Ground Clearance")) {
			float gc = trail["Ground Clearance"].GetFloat();
			SetGroundClearance(gc);
		}
		if (trail.HasMember("Trail Width")) {
			float tw = trail["Trail Width"].GetFloat();
			trails_[0].SetTrailWidth(tw);
		}
	}
	InitializeTrail();

	if (d.HasMember("Species")) {
		for (unsigned int i = 0; i < d["Species"].Capacity(); i++) {
			const rapidjson::Value& spec_obj = d["Species"][i];
			Species species;

			if (spec_obj.HasMember("Num New Per Area")) {
				float num = spec_obj["Num New Per Area"].GetFloat();
				species.SetNumNewPerArea(num);
			}

			if (spec_obj.HasMember("Growth Rate")) {
				float rate = spec_obj["Growth Rate"].GetFloat();
				species.SetGrowthRate(rate);
			}

			if (spec_obj.HasMember("Max Height")) {
				float maxh = spec_obj["Max Height"].GetFloat();
				species.SetMaxHeight(maxh);
			}

			if (spec_obj.HasMember("Height To Diameter Ratio")) {
				float ratio = spec_obj["Height To Diameter Ratio"].GetFloat();
				species.SetDiamToHeightRatio(ratio);
			}

			if (spec_obj.HasMember("Max Age")) {
				float max_age = spec_obj["Max Age"].GetFloat();
				species.SetMaxAge(max_age);
			}

			if (spec_obj.HasMember("Min Height")) {
				float min_height = spec_obj["Min Height"].GetFloat();
				species.SetMinHeight(min_height);
			}

			if (spec_obj.HasMember("Mesh Height")) {
				float mesh_height = spec_obj["Mesh Height"].GetFloat();
				species.SetDefaultHeight(mesh_height);
			}

			if (spec_obj.HasMember("Rotate Mesh")) {
				bool rotate = spec_obj["Rotate Mesh"].GetBool();
				species.SetRotateMesh(rotate);
			}

			if (spec_obj.HasMember("Mesh File")) {
				std::string mesh_name = spec_obj["Mesh File"].GetString();
				species.SetName(mesh_name);
				AddSpecies(species);
			}
			else {
				std::cerr << "ERROR: No mesh listed for species " << i << ", not adding ecosystem." << std::endl;
			}
		}
	}
}

void Ecosystem::SaveEcosystemToFile(std::string outfile) {
	std::ofstream fout(outfile.c_str());
	for (int i = 0; i < (int)plants_.size(); i++) {
		glm::vec2 p = plants_[i].GetPosition();
		fout << p.x << " " << p.y << " " << plants_[i].GetHeight()
			<< " " << species_num_[plants_[i].GetSpeciesName()] <<
			" "<< plants_[i].GetSpeciesName()<<
			std::endl;
	}
	fout.close();
}

float Ecosystem::GetTerrainHeightAtPoint(glm::vec2 point) {
	float z;
	if (terrain_loaded_) {
		z = surf_.GetHeightAtPoint(point);
		//z2 = surface_mesh_.GetMeshHeight(point);
	}
	else {
		z= 0.0f;
	}
	return z;
}

void Ecosystem::SaveToMavsScene(std::string outfile) {
	rapidjson::StringBuffer s;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
	writer.SetIndent(' ', 2);

	writer.StartObject(); {

		writer.Key("PathToMeshes");
		writer.String(path_to_meshes_.c_str());

		writer.Key("UseFullPaths");
		writer.Bool(true);

		writer.Key("Object Labels");
		writer.String("labels.json");

		// Texture layer information
		if (use_textures_) {
			writer.Key("Layered Surface");
			writer.StartObject(); {
				writer.Key("Mesh");
				writer.String(surface_mesh_file_.c_str());
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
				writer.String(texture_file_.c_str());
				

				writer.Key("Potholes");
				writer.String(pothole_file_.c_str());

				writer.EndObject();
			}
		}
		if (terrain_loaded_) {
			//surface mesh array
			writer.Key("Surface Mesh");
			writer.StartArray(); {
				writer.StartObject(); {
					writer.Key("Mesh");
					//writer.String(surface_mesh_.GetName().c_str());
					writer.String(surface_mesh_file_.c_str());
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
		}
		// Array of objects
		writer.Key("Objects");
		writer.StartArray(); {
			if (!use_textures_){
			// First write the surface
			writer.StartObject(); {
				writer.Key("Mesh");
				writer.String(surface_mesh_.GetName().c_str());
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
														 //done with surface object
		}
			//now loop through veg object instances
			for (int ns = 0; ns < (int)species_.size(); ns++) {
				writer.StartObject();
				writer.Key("Mesh");
				writer.String(species_[ns].GetName().c_str());
				if (species_[ns].GetRotate()) {
					writer.Key("Rotate Y to Z");
					writer.Bool(true);
				}
				std::vector<Plant> these_plants;
				for (int i = 0; i < (int)plants_.size(); i++) {
					if (plants_[i].GetSpeciesName() == species_[ns].GetName()) {
						these_plants.push_back(plants_[i]);
					}
				}
				writer.Key("Instances");
				writer.StartArray();
				for (int i = 0; i < (int)these_plants.size(); i++) {
					glm::vec2 pos2 = these_plants[i].GetPosition();
					float z = GetTerrainHeightAtPoint(pos2);
					float theta = math::rand_in_range(0.0f, (float)k2Pi);
					float scale = these_plants[i].GetHeight() / species_[ns].GetDefaultHeight();
					writer.StartObject();
					writer.Key("YawPitchRoll");
					writer.StartArray();
					writer.Double(0.0);
					writer.Double((double)theta);
					writer.Double(0.0);
					writer.EndArray();
					writer.Key("Position");
					writer.StartArray();
					writer.Double(pos2.x);
					writer.Double(pos2.y);
					writer.Double((double)z);
					writer.EndArray();
					writer.Key("Scale");
					writer.StartArray();
					for (int i = 0; i < 3; i++)writer.Double((double)scale);
					writer.EndArray();
					writer.EndObject();
				}
				writer.EndArray(); //array of instances
				writer.EndObject();
			} //loop through species

			writer.EndArray(); }
		// done with array of objects

		writer.EndObject(); }
	std::string outstring = s.GetString();
	std::ofstream fout(outfile.c_str());
	fout << outstring;
	fout.close();
}

} //namespace terraingen
} //namespace mavs