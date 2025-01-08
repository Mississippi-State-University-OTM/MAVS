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
#include <raytracers/embree_tracer/embree_tracer.h>

#include <cstdio>
#include <iostream>
#include <limits>
#include <ctime>

#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>
#include <embree3/rtcore_ray.h>

#include <mavs_core/math/utils.h>
#include <mavs_core/math/polygon.h>
#include <mavs_core/math/constants.h>
#include <mavs_core/terrain_generator/heightmap.h>
#include <mavs_core/data_path.h>
#include <mavs_core/pose_readers/trail.h>

#include <embree_tracer/embree_datatypes.h>

namespace mavs{
namespace raytracer{
namespace embree{
	
static void rtcIntersect(RTCScene scene, RTCRayHit &query) {
	RTCIntersectContext context;
	rtcInitIntersectContext(&context);
	query.hit.geomID = RTC_INVALID_GEOMETRY_ID;
	query.hit.primID = RTC_INVALID_GEOMETRY_ID;
	rtcIntersect1(scene, &context, &query);
	query.hit.Ng_x = -query.hit.Ng_x; 
	query.hit.Ng_y = -query.hit.Ng_y;
	query.hit.Ng_z = -query.hit.Ng_z;
}

EmbreeTracer::EmbreeTracer(){
	rtc_device_ = rtcNewDevice("threads=1");
	//rtc_device_ = rtcNewDevice("threads=0");
	rtc_scene_ = rtcNewScene(rtc_device_);
	surface_ = rtcNewScene(rtc_device_);
	layered_surface_ = rtcNewScene(rtc_device_);
	animation_scene_ = rtcNewScene(rtc_device_);
	num_dynamic_ = 0;
	num_facets_in_scene_ = 0;
	surface_loaded_ = false;
	labels_loaded_ = false;
	scene_loaded_ = false;
	num_meshes_ = 0;
	perform_labeling_ = false;
	use_spectral_ = false;
	use_surface_textures_ = true;
	use_full_paths_ = false;
	lower_left_corner_.x = std::numeric_limits<float>::infinity();
	lower_left_corner_.y = std::numeric_limits<float>::infinity();
	lower_left_corner_.z = std::numeric_limits<float>::infinity();
	upper_right_corner_.x = std::numeric_limits<float>::lowest();
	upper_right_corner_.y = std::numeric_limits<float>::lowest();
	upper_right_corner_.z = std::numeric_limits<float>::lowest();
	surface_material_ = "dry";
	surface_cone_index_ = 250.0f*6894.76f;
	inst_id_ = 0;
}

static glm::mat3 GetIdentity() {
	glm::mat3 rot;
	rot[0][0] = 1.0f; rot[0][1] = 0.0f; rot[0][2] = 0.0f;
	rot[1][0] = 0.0f; rot[1][1] = 1.0f; rot[1][2] = 0.0f;
	rot[2][0] = 0.0f; rot[2][1] = 0.0f; rot[2][2] = 1.0f;
	return rot;
}

static glm::mat3x4 GetRotFromEuler(glm::vec3 euler_angles) {
	glm::mat3x4 rot_scale;
	glm::mat3x3 om = orientate3(euler_angles);
	for (int ii = 0; ii < 3; ii++) {
		for (int jj = 0; jj < 3; jj++) {
			rot_scale[ii][jj] = om[ii][jj];
		}
	}
	rot_scale[0][3] = 0.0f; rot_scale[1][3] = 0.0f; rot_scale[2][3] = 0.0f;
	return rot_scale;
}

glm::mat3x4 EmbreeTracer::GetAffineIdentity() {
	glm::mat3x4 rot_scale;
	rot_scale[0][0] = 1.0f; rot_scale[0][1] = 0.0f; rot_scale[0][2] = 0.0f;
	rot_scale[1][0] = 0.0f; rot_scale[1][1] = 1.0f; rot_scale[1][2] = 0.0f;
	rot_scale[2][0] = 0.0f; rot_scale[2][1] = 0.0f; rot_scale[2][2] = 1.0f;
	rot_scale[0][3] = 0.0f; rot_scale[1][3] = 0.0f; rot_scale[2][3] = 0.0f;
	return rot_scale;
}

static glm::mat3x4 ScaleAffine(glm::mat3x4 rot_scale, float x_scale, float y_scale, float z_scale) {
	rot_scale[0][0] *= x_scale; rot_scale[0][1] *= y_scale; rot_scale[0][2] *= z_scale;
	rot_scale[1][0] *= x_scale; rot_scale[1][1] *= y_scale; rot_scale[1][2] *= z_scale;
	rot_scale[2][0] *= x_scale; rot_scale[2][1] *= y_scale; rot_scale[2][2] *= z_scale;
	return rot_scale;
}

static glm::mat3x4 SetAffineOffset(glm::mat3x4 rot_scale, float x_off, float y_off, float z_off) {
	rot_scale[0][3] = x_off; rot_scale[1][3] = y_off; rot_scale[2][3] = z_off;
	return rot_scale;
}

static glm::mat3x4 SetAffineOffset(glm::mat3x4 rot_scale, glm::vec3 offset) {
	rot_scale[0][3] = offset.x; rot_scale[1][3] = offset.y; rot_scale[2][3] = offset.z;
	return rot_scale;
}

EmbreeTracer::~EmbreeTracer(){
	for (int i = 0; i < (int)instanced_scenes_.size(); i++) {
		rtcReleaseScene(instanced_scenes_[i]);
	}
	rtcReleaseScene(rtc_scene_);
	rtcReleaseScene(surface_);
	rtcReleaseScene(animation_scene_);
	rtcReleaseScene(layered_surface_); // could this cause problems
	rtcReleaseDevice(rtc_device_);
}

EmbreeTracer::EmbreeTracer(const EmbreeTracer &scene){

}

void EmbreeTracer::LoadSemanticLabels(std::string input_file) {
	if (!utils::file_exists(input_file)) {
		std::cerr << "Warning: Could not find label file " << input_file << std::endl;
		return;
	}

	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	semantic_labels_["textured_surface"] = "ground"; 
	semantic_labels_["pothole"] = "pothole";
	std::map<std::string, int> semantic_label_nums;
	if (d.HasMember("Label Colors")) {
		for (unsigned int i = 0; i < d["Label Colors"].Capacity(); i++) {
			std::string lab_name = d["Label Colors"][i]["Label"].GetString();
			glm::vec3 color;
			color.x = d["Label Colors"][i]["Color"][0].GetFloat();
			color.y = d["Label Colors"][i]["Color"][1].GetFloat();
			color.z = d["Label Colors"][i]["Color"][2].GetFloat();
			label_colors_[lab_name] = color;
			if (d["Label Colors"][i].HasMember("Label Number")) {
				int ln = d["Label Colors"][i]["Label Number"].GetInt();
				semantic_label_nums[lab_name] = ln;
				label_nums_[lab_name] = ln;
			}
			else {
				semantic_label_nums[lab_name] = i;
			}
		}
	}

	if (d.HasMember("Labels")) {
		for (unsigned int i = 0; i < d["Labels"].Capacity(); i++) {
			if (d["Labels"][i].Capacity() == 2) {
				std::string obj_name = d["Labels"][i][0].GetString();
				std::string lab_name = d["Labels"][i][1].GetString();
				semantic_labels_[obj_name] = lab_name;
				label_nums_[lab_name] = semantic_label_nums[lab_name]; // i;
				labels_loaded_ = true;
			}
		}
	}
}

void EmbreeTracer::LoadVegDistribution(const rapidjson::Value& d) {
	bool write_check = false;
	if (d.HasMember("Write Check File")) {
		write_check = d["Write Check File"].GetBool();
	}
	std::vector<VegSpecies> species;
	if (d.HasMember("Species")) {
		for (unsigned int i = 0; i < d["Species"].Capacity(); i++) {
			//Mesh mesh;
			VegSpecies spec;
			spec.num_added = 0;
			if (d["Species"][i].HasMember("Rotated")) {
				spec.rotated = d["Species"][i]["Rotated"].GetBool();
			}
			if (d["Species"][i].HasMember("Name")) {
				spec.name = d["Species"][i]["Name"].GetString();
			}
			if (d["Species"][i].HasMember("Mesh")) {
				spec.meshfile = d["Species"][i]["Mesh"].GetString();
				std::string to_load = (file_path_ + spec.meshfile);
				if (!semantic_labels_.count(spec.meshfile))std::cerr << "WARNING: NO LABEL FOUND FOR MESH " << spec.meshfile << std::endl;
				spec.mesh.Load(file_path_, to_load);
				if (spec.rotated)spec.mesh.RotateYToZ();
			}
			else {
				std::cerr << "ERROR, no mesh file listed for species " << (i + 1) << " in scene file" << std::endl;
				exit(200);
			}

			if (d["Species"][i].HasMember("Mesh Height")) {
				spec.mesh_height = d["Species"][i]["Mesh Height"].GetFloat();
			}
			species.push_back(spec);
		}
	} // finished loading species info

	std::vector<VegDataPoint> veg_data_points;
	if (d.HasMember("Zones")) {
		for (unsigned int z = 0; z < d["Zones"].Capacity(); z++) {
			VegDataPoint zone;
			if (d["Zones"][z].HasMember("Coordinates")) {
				zone.x = d["Zones"][z]["Coordinates"][0].GetFloat();
				zone.y = d["Zones"][z]["Coordinates"][1].GetFloat();
			}
			if (d["Zones"][z].HasMember("Species")) {
				int nspec = (int)d["Zones"][z]["Species"].Capacity();
				
				if (nspec == species.size()) {
					for (int i = 0; i < nspec; i++) {
						VegSpecDescriptor desc;
						desc.density = d["Zones"][z]["Species"][i]["Density"].GetFloat();
						desc.min_diameter = d["Zones"][z]["Species"][i]["MinDiam"].GetFloat();
						desc.max_diameter = d["Zones"][z]["Species"][i]["MaxDiam"].GetFloat();
						zone.veg.push_back(desc);
					}
				}
				else {
					std::cerr << "Warning: incorrect number of species listed for veg measurment point " << z << " in scene file "<<std::endl;
					return;
				}
			} //loop over species
			veg_data_points.push_back(zone);
		} //loop over zones
	} // if has member "Zones"

	//Create a grid that holds the different vegetation densities
	float delta = 20.0f;
	int n_steps = (int)ceil((surface_ur_.y - surface_ll_.y) / delta);
	int e_steps = (int)ceil((surface_ur_.x - surface_ll_.x) / delta);
	int num_species = (int)species.size();
	if (e_steps == 0 || n_steps == 0)return;
	std::vector<std::vector<std::vector <float> > > density_map = mavs::utils::Allocate3DVector(e_steps, n_steps, num_species, 0.0f);
	std::vector<std::vector<std::vector <glm::vec2> > > diameter_map = mavs::utils::Allocate3DVector(e_steps, n_steps, num_species, glm::vec2(0.0f, 0.0f));
	for (int n = 0; n < n_steps; n++) {
		float northing = (0.5f + n)*delta + surface_ll_.y;
		for (int e = 0; e < e_steps; e++) {
			float easting = (0.5f + e)*delta + surface_ll_.x;
			for (int si = 0; si < species.size(); si++) {
				float dens_numerator = 0.0f;
				float min_diam = 1000.0f;
				float max_diam = 0.0f;
				float denominator = 0.0f;
				for (int i = 0; i < veg_data_points.size(); i++) {
					float ee = veg_data_points[i].x; 
					float nn = veg_data_points[i].y; 
					//float d = sqrt((ee - easting)*(ee - easting) + (nn - northing)*(nn - northing));
					float d = powf((ee - easting)*(ee - easting) + (nn - northing)*(nn - northing),3);
					if (d > 0.0f){
						dens_numerator += veg_data_points[i].veg[si].density/d; 
						if (veg_data_points[i].veg[si].min_diameter < min_diam && veg_data_points[i].veg[si].min_diameter!=0.0f)min_diam = veg_data_points[i].veg[si].min_diameter;
						if (veg_data_points[i].veg[si].max_diameter > max_diam)max_diam = veg_data_points[i].veg[si].max_diameter;
						denominator += 1.0f / d;
					}
					else {
						dens_numerator += std::numeric_limits<float>::max()*veg_data_points[i].veg[si].density;
						denominator += std::numeric_limits<float>::max();
					}
				}// loop over zones
				if (denominator > 0.0f) {
					density_map[e][n][si] = dens_numerator / denominator;
					diameter_map[e][n][si].x = min_diam;
					diameter_map[e][n][si].y = max_diam;
				}
				else {
					density_map[e][n][si] = 0.0f;
					diameter_map[e][n][si].x = 0.0f;
					diameter_map[e][n][si].y = 0.0f;
				}
			} //loop over tree types / species
		}
	}

	std::vector<VegCheck> veg_check;
	//place trees in cells
	float cell_area = delta * delta;
	for (int s = 0; s < species.size(); s++) {
		for (int i = 0; i < density_map.size(); i++) {
			float xlo = i * delta + surface_ll_.x;
			float xhi = (i + 1)*delta + surface_ll_.x;
			for (int j = 0; j < density_map[0].size(); j++) {
				float ylo = j * delta + surface_ll_.y;
				float yhi = (j + 1)*delta + surface_ll_.y;
				float numraw = density_map[i][j][s] * cell_area;
				float test_val = numraw - floor(numraw);
				float mc_val = mavs::math::rand_in_range(0.0f, 1.0f);
				int num = (int)floor(numraw);
				if (test_val > mc_val) {
					num = num + 1;
				}
				for (int mr = 0; mr < num; mr++) {
					//glm::mat3x4 rot_scale;
					glm::vec3 position;
					position.x = mavs::math::rand_in_range(xlo, xhi);
					position.y = mavs::math::rand_in_range(ylo, yhi);
					bool on_trail = false;
					for (int ls = 0; ls < layered_surfaces_.size(); ls++) {
						if (layered_surfaces_[ls].IsPointOnTrail(position.x, position.y)) {
							on_trail = true;
						}
					}
					if (!on_trail) {
						position.z = GetSurfaceHeight(position.x, position.y);
						glm::vec3 euler_angles;
						euler_angles.x = 0.0f;
						euler_angles.y = math::rand_in_range(0.0f, (float)k2Pi);
						euler_angles.z = 0.0f;
						glm::mat3x4 rot_scale = GetRotFromEuler(euler_angles);
						//trunk diameter in cm
						float trunk_diam = math::rand_in_range(diameter_map[i][j][s].x, diameter_map[i][j][s].y);
						//tree height in m;
						float ht = 0.5f*(1.3f + 0.6208f*trunk_diam - 0.00308f*trunk_diam*trunk_diam);
						float scale_fac = ht / species[s].mesh_height;
						rot_scale = ScaleAffine(rot_scale, scale_fac, scale_fac, scale_fac);
						glm::vec3 final_pos = position; // +offset;
						rot_scale = SetAffineOffset(rot_scale, final_pos);
						if (write_check) {
							VegCheck vc;
							vc.name = species[s].name;
							vc.x = final_pos.x;
							vc.y = final_pos.y;
							veg_check.push_back(vc);
						}
						int instId = AddMesh(species[s].mesh, rot_scale, species[s].num_added, RTC_BUILD_QUALITY_MEDIUM);
						species[s].num_added += 1;
					} // if not on trail
				} // for all random placements, mr
			} // j
		} // i
	} //species

	if (write_check) {
		//Loop through the test points and sample the vegetation, test if we got it right
		float cutoff_dist = 10.0f;
		std::vector<std::vector<int> > spec_count = mavs::utils::Allocate2DVector((int)veg_data_points.size(), (int)species.size(), 0);
		for (int i = 0; i < veg_data_points.size(); i++) {
			for (int j = 0; j < veg_check.size(); j++) {
				float d = (float)sqrt((veg_data_points[i].x - veg_check[j].x)*(veg_data_points[i].x - veg_check[j].x) + (veg_data_points[i].y - veg_check[j].y)*(veg_data_points[i].y - veg_check[j].y));
				if (d < cutoff_dist) {
					for (int k = 0; k < species.size(); k++) {
						if (veg_check[j].name == species[k].name) {
							spec_count[i][k]++;
							break;
						}
					}
				}
			}
		}
		std::ofstream fout("veg_distribution_check.txt");
		float check_area = 3.14159f*cutoff_dist*cutoff_dist;
		for (int i = 0; i < veg_data_points.size(); i++) {
			for (int k = 0; k < species.size(); k++) {
				int num_calc = (int)(check_area * veg_data_points[i].veg[k].density);
				fout << i << " " << k << " " << spec_count[i][k] << " " << num_calc << " " << spec_count[i][k] - num_calc << std::endl;
			}
		}
		fout.close();
		//Done with check calculation;
	}
}

void EmbreeTracer::LoadLayeredSurface(const rapidjson::Value& d) {
	mavs::MavsDataPath mavs_data_path;
	glm::mat3x4 rot_scale = GetAffineIdentity();

	if (d.HasMember("Scale")) {
		if (d["Scale"].Capacity() != 3) {
			std::cerr << "ERROR: Scale must have 3 elements." << std::endl;
			exit(1);
		}
		float sx = d["Scale"][0].GetFloat();
		float sy = d["Scale"][1].GetFloat();
		float sz = d["Scale"][2].GetFloat();
		rot_scale = ScaleAffine(rot_scale, sx, sy, sz);
	}

	if (d.HasMember("Mesh")) {
		Mesh mesh;
		std::string to_load;
		if (use_full_paths_) {
			to_load = d["Mesh"].GetString();
			if (!semantic_labels_.count(to_load))std::cerr << "WARNING: NO LABEL FOUND FOR MESH " << to_load << std::endl;
		}
		else {
			std::string meshfile = d["Mesh"].GetString();
			if (!semantic_labels_.count(meshfile))std::cerr << "WARNING: NO LABEL FOUND FOR MESH " << meshfile << std::endl;
			to_load = (file_path_ + meshfile);
		}
		mesh.Load(file_path_, to_load);;
		int surf_id = SetLayeredSurfaceMesh(mesh, rot_scale);
	}
	else if (d.HasMember("Heightmap")) {
		cimg_library::CImg<float> map_img;
		if (d["Heightmap"].HasMember("Map")) {
			std::string to_load;
			if (use_full_paths_) {
				to_load = d["Heightmap"]["Map"].GetString();
			}
			else {
				std::string meshfile = d["Heightmap"]["Map"].GetString();
				to_load = (file_path_ + meshfile);
			}
			map_img.load(to_load.c_str());
		}
		float scale = 0.1f;
		if (d["Heightmap"].HasMember("Scale")) {
			scale = d["Heightmap"]["Scale"].GetFloat();
		}
		float resolution = 1.0f;
		if (d["Heightmap"].HasMember("Resolution")) {
			resolution = d["Heightmap"]["Resolution"].GetFloat();
		}
		mavs::terraingen::HeightMap hm;
		hm.CreateFromImage(map_img, resolution, scale);
		std::string meshname("loaded_from_heightmap");
		hm.WriteObj(meshname, false, false);
		Mesh mesh;
		mesh.Load(file_path_, meshname+".obj");
		int surf_id = SetLayeredSurfaceMesh(mesh, rot_scale);
		int surf_mesh_id = SetSurfaceMesh(mesh, rot_scale);
		surface_ll_.x = -0.5f*map_img.width()*resolution;
		surface_ll_.y = -0.5f*map_img.width()*resolution;
		surface_ur_.x = 0.5f*map_img.width()*resolution;
		surface_ur_.y = 0.5f*map_img.width()*resolution;
	}
	Trail trail;
	std::vector<Trail> trails;
	if (d.HasMember("Trail")) {
		if (d["Trail"].HasMember("Trail Width")) {
			trail.SetTrailWidth(d["Trail"]["Trail Width"].GetFloat());
		}
		if (d["Trail"].HasMember("Track Width")) {
			trail.SetTrackWidth(d["Trail"]["Track Width"].GetFloat());
		}
		if (d["Trail"].HasMember("Wheelbase")) {
			trail.SetWheelbase(d["Trail"]["Wheelbase"].GetFloat());
		}
		if (d["Trail"].HasMember("Path")) {
			if (d["Trail"]["Path"].IsArray()) {
				for (int tnum = 0; tnum < (int)d["Trail"]["Path"].Capacity(); tnum++) {
					Trail this_trail = trail;
					std::string pose_path;
					if (use_full_paths_) {
						pose_path = d["Trail"]["Path"][tnum].GetString();
					}
					else {
						pose_path = mavs_data_path.GetPath();
						pose_path.append("/");
						pose_path.append(d["Trail"]["Path"][tnum].GetString());
					}
					this_trail.LoadPath(pose_path);
					trails.push_back(this_trail);
				}
			}
			else {
				std::string pose_path;
				if (use_full_paths_) {
					pose_path = d["Trail"]["Path"].GetString();
				}
				else {
					pose_path = mavs_data_path.GetPath();
					pose_path.append("/");
					pose_path.append(d["Trail"]["Path"].GetString());
				}
				trail.LoadPath(pose_path);
				trails.push_back(trail);
			}
		} // If has path
	}
	if (d.HasMember("Layers")) {
		LayeredSurface layers;
		std::string layer_file;
		if (use_full_paths_) {
			layer_file = d["Layers"].GetString();
		}
		else {
			layer_file = file_path_;
			layer_file.append(d["Layers"].GetString());
		}
		layers.LoadSurfaceTextures(file_path_, layer_file);
		//layers.SetTrail(trail);
		for (int tnum = 0; tnum < (int)trails.size(); tnum++) {
			layers.AddTrail(trails[tnum]);
		}
		layered_surfaces_.push_back(layers);
	}

	if (d.HasMember("Potholes")) {
		// loading from local only
		std::string pothole_file = d["Potholes"].GetString();
		std::ifstream potin(pothole_file.c_str());
		while (!potin.eof()) {
			float diam, rad, depth, x, y;
			potin >> depth >> diam >> x >> y;
			rad = diam * 0.5f;
			glm::vec4 hole(depth, rad, x, y);
			potholes_.push_back(hole);
		}
		potholes_.pop_back();
	}
}

void EmbreeTracer::LoadSurfaceMesh(const rapidjson::Value& d) {
	surface_ll_.x = std::numeric_limits<float>::max();
	surface_ll_.y = std::numeric_limits<float>::max();
	surface_ur_.x = std::numeric_limits<float>::lowest();
	surface_ur_.y = std::numeric_limits<float>::lowest();
	for (unsigned int surfnum = 0; surfnum < d.Capacity(); surfnum++) {
		if (d[surfnum].HasMember("Material")) {
			surface_material_ = d[surfnum]["Material"].GetString();
		}
		if (d[surfnum].HasMember("Cone Index")) {
			surface_cone_index_ = 6894.76f*d[surfnum]["Cone Index"].GetFloat();
		}
		bool y_to_z = false;
		if (d[surfnum].HasMember("Rotate Y to Z")) {
			y_to_z = d[surfnum]["Rotate Y to Z"].GetBool();
		}
		bool x_to_y = false;
		if (d[surfnum].HasMember("Rotate X to Y")) {
			x_to_y = d[surfnum]["Rotate X to Y"].GetBool();
		}
		bool y_to_x = false;
		if (d[surfnum].HasMember("Rotate Y to X")) {
			y_to_x = d[surfnum]["Rotate Y to X"].GetBool();
		}
		std::string meshfile;
		glm::vec3 mesh_center;
		Mesh mesh;
		if (d[surfnum].HasMember("Mesh")) {
			std::string to_load, path_to_surface;
			meshfile = d[surfnum]["Mesh"].GetString();
			if (!semantic_labels_.count(meshfile))std::cerr << "WARNING: NO LABEL FOUND FOR MESH " << meshfile << std::endl;
			if (use_full_paths_) {
				to_load = meshfile;
				path_to_surface = utils::GetPathFromFile(to_load);
			}
			else {
				to_load = (file_path_ + meshfile);
				path_to_surface = file_path_;
			}
			mesh.Load(path_to_surface, to_load);
			if (y_to_z)mesh.RotateYToZ();
			if (x_to_y)mesh.RotateXToY();
			if (y_to_x)mesh.RotateYToX();
			mesh_center = mesh.GetCenter();
		}
		glm::mat3x4 rot_scale;
		glm::vec3 position;
		if (d[surfnum].HasMember("YawPitchRoll")) {
			if (d[surfnum]["YawPitchRoll"].Capacity() != 3) {
				std::cerr << "ERROR: YawPitchRoll must contain 3 elements."
					<< std::endl;
				exit(1);
			}
			glm::vec3 euler_angles;
			euler_angles.x = d[surfnum]["YawPitchRoll"][0].GetFloat();
			euler_angles.y = d[surfnum]["YawPitchRoll"][1].GetFloat();
			euler_angles.z = d[surfnum]["YawPitchRoll"][2].GetFloat();
			rot_scale = GetRotFromEuler(euler_angles);
		}
		else {
			rot_scale = GetAffineIdentity();
		}
		if (d[surfnum].HasMember("Position")) {
			if (d[surfnum]["Position"].Capacity() == 3) {
				position.x = d[surfnum]["Position"][0]
					.GetFloat();
				position.y = d[surfnum]["Position"][1]
					.GetFloat();
				position.z = d[surfnum]["Position"][2]
					.GetFloat();
			}
		}
		else {
			position = glm::vec3(0.0f, 0.0f, 0.0f);
		}
		if (d[surfnum].HasMember("Scale")) {
			if (d[surfnum]["Scale"].Capacity() != 3) {
				std::cerr << "ERROR: Scale must have 3 elements." << std::endl;
				exit(1);
			}
			float sx = d[surfnum]["Scale"][0].GetFloat();
			float sy = d[surfnum]["Scale"][1].GetFloat();
			float sz = d[surfnum]["Scale"][2].GetFloat();
			rot_scale = ScaleAffine(rot_scale, sx, sy, sz);
		}

		if (d[surfnum].HasMember("Mesh")) {
			glm::vec3 offset = position; // -mesh_center;
			rot_scale = SetAffineOffset(rot_scale, offset);
			int surf_id = SetSurfaceMesh(mesh, rot_scale);
		}
		BoundingBox bb = mesh.GetBoundingBox();
		if (bb.GetLowerLeft().x < surface_ll_.x)surface_ll_.x = bb.GetLowerLeft().x;
		if (bb.GetLowerLeft().y < surface_ll_.y)surface_ll_.y = bb.GetLowerLeft().y;
		if (bb.GetUpperRight().x > surface_ur_.x)surface_ur_.x = bb.GetUpperRight().x;
		if (bb.GetUpperRight().y > surface_ur_.y)surface_ur_.y = bb.GetUpperRight().y;
	} // loop over surface meshes
} //LoadSurfaceMesh

void EmbreeTracer::LoadObjects(const rapidjson::Value& d) {
	for (unsigned int m = 0; m<d.Capacity(); m++) {
		bool y_to_z = false;
		if (d[m].HasMember("Rotate Y to Z")) {
			y_to_z = d[m]["Rotate Y to Z"].GetBool();
		}
		bool x_to_y = false;
		if (d[m].HasMember("Rotate X to Y")) {
			x_to_y = d[m]["Rotate X to Y"].GetBool();
		}
		bool y_to_x = false;
		if (d[m].HasMember("Rotate Y to X")) {
			y_to_x = d[m]["Rotate Y to X"].GetBool();
		}
		bool label_by_group = false;
		if (d[m].HasMember("Label By Group")) {
			label_by_group = d[m]["Label By Group"].GetBool();
		}
		bool smooth_normals = false;
		if (d[m].HasMember("Smooth Normals")) {
			smooth_normals = d[m]["Smooth Normals"].GetBool();
		}
		std::string meshfile;
		glm::vec3 mesh_center;
		std::string mtlfile = "";
		Mesh mesh;
		if (d[m].HasMember("Material")) {
			mtlfile = d[m]["Material"].GetString();
		}
		if (d[m].HasMember("Mesh")) {
			meshfile = d[m]["Mesh"].GetString();
			std::string to_load = (file_path_ + meshfile);
			if (!semantic_labels_.count(meshfile))std::cerr << "WARNING: NO LABEL FOUND FOR MESH " << meshfile << std::endl;
			//mesh.Load(file_path_, to_load);
			mesh.Load(file_path_, to_load,mtlfile);
			if (y_to_z)mesh.RotateYToZ();
			if (x_to_y)mesh.RotateXToY();
			if (y_to_x)mesh.RotateYToX();
			if (smooth_normals)mesh.CalculateNormals();
			if (label_by_group)mesh.SetLabelByGroup(true);
			mesh_center = mesh.GetCenter();
		}
		if (d[m].HasMember("Instances")) {
			int num_inst = d[m]["Instances"].Capacity();
			for (int mi = 0; mi<num_inst; mi++) {
				glm::mat3x4 rot_scale;
				glm::vec3 position;
				if (d[m]["Instances"][mi].HasMember("YawPitchRoll")) {
					if (d[m]["Instances"][mi]["YawPitchRoll"].Capacity() != 3) {
						std::cerr << "ERROR: YawPitchRoll must contain 3 elements."
							<< std::endl;
						exit(1);
					}
					glm::vec3 euler_angles;
					euler_angles.x = d[m]["Instances"][mi]["YawPitchRoll"][0].GetFloat();
					euler_angles.y = d[m]["Instances"][mi]["YawPitchRoll"][1].GetFloat();
					euler_angles.z = d[m]["Instances"][mi]["YawPitchRoll"][2].GetFloat();
					rot_scale = GetRotFromEuler(euler_angles);
				}
				else {
					rot_scale = GetAffineIdentity();
				}
				if (d[m]["Instances"][mi].HasMember("Position")) {
					if (d[m]["Instances"][mi]["Position"].Capacity() == 3) {
						position.x = d[m]["Instances"][mi]["Position"][0]
							.GetFloat();
						position.y = d[m]["Instances"][mi]["Position"][1]
							.GetFloat();
						position.z = d[m]["Instances"][mi]["Position"][2]
							.GetFloat();
					}
				}
				else {
					position = glm::vec3(0.0f, 0.0f, 0.0f);
				}
				if (d[m]["Instances"][mi].HasMember("Scale")) {
					if (d[m]["Instances"][mi]["Scale"].Capacity() != 3) {
						std::cerr << "ERROR: Scale must have 3 elements." << std::endl;
						exit(1);
					}
					float sx = d[m]["Instances"][mi]["Scale"][0].GetFloat();
					float sy = d[m]["Instances"][mi]["Scale"][1].GetFloat();
					float sz = d[m]["Instances"][mi]["Scale"][2].GetFloat();
					rot_scale = ScaleAffine(rot_scale, sx, sy, sz);
				}

				if (d[m].HasMember("Mesh")) {
					glm::vec3 offset = position; // -mesh_center;
					rot_scale = SetAffineOffset(rot_scale, offset);
					int instId = AddMesh(mesh, rot_scale, mi, RTC_BUILD_QUALITY_MEDIUM);
				}
			} //loop over instances
		} // if instanced
		else if (d[m].HasMember("Random")) {
			if (d[m]["Random"].HasMember("Number")) {
				int num_rand = d[m]["Random"]["Number"].GetInt();
				if (d[m]["Random"].HasMember("Polygon")) {
					glm::vec3 offset(0.0f, 0.0f, 0.0f);
					if (d[m]["Random"].HasMember("Offset")) {
						offset.x = d[m]["Random"]["Offset"][0].GetFloat();
						offset.y = d[m]["Random"]["Offset"][1].GetFloat();
						offset.z = d[m]["Random"]["Offset"][2].GetFloat();
					}
					std::vector<glm::vec2> nodes;
					for (unsigned int p = 0; p<d[m]["Random"]["Polygon"].Capacity(); p++) {
						glm::vec2 node;
						node.x = d[m]["Random"]["Polygon"][p][0].GetFloat();
						node.y = d[m]["Random"]["Polygon"][p][1].GetFloat();
						nodes.push_back(node);
					}
					math::Polygon area(nodes);
					for (int mr = 0; mr<num_rand; mr++) {
						glm::vec3 position;
						glm::vec2 location = area.GetRandomInside();
						position.x = location.x;
						position.y = location.y;
						if (surface_loaded_) {
							position.z = GetSurfaceHeight(position.x, position.y);
						}
						else {
							position.z = 0.0f;
						}
						glm::vec3 euler_angles;
						euler_angles.x = 0.0f;
						euler_angles.y = math::rand_in_range(0.0f, (float)k2Pi);
						euler_angles.z = 0.0f;
						glm::mat3x4 rot_scale = GetRotFromEuler(euler_angles);
						float x_scale = 1; float y_scale = 1; float z_scale = 1;
						if (d[m]["Random"].HasMember("Scale")) {
							if (d[m]["Random"]["Scale"].Capacity() == 2) {
								float lo = d[m]["Random"]["Scale"][0].
									GetFloat();
								float hi = d[m]["Random"]["Scale"][1].
									GetFloat();
								float scale = math::rand_in_range(lo, hi);
								x_scale = scale; y_scale = scale; z_scale = scale;
							}
							else if (d[m]["Random"]["Scale"].Capacity() == 3) {
								if (d[m]["Random"]["Scale"][0].Capacity() == 2 &&
									d[m]["Random"]["Scale"][1].Capacity() == 2 &&
									d[m]["Random"]["Scale"][2].Capacity() == 2) {
									float xlo = d[m]["Random"]["Scale"][0][0].
										GetFloat();
									float xhi = d[m]["Random"]["Scale"][0][1].
										GetFloat();
									float ylo = d[m]["Random"]["Scale"][1][0].
										GetFloat();
									float yhi = d[m]["Random"]["Scale"][1][1].
										GetFloat();
									float zlo = d[m]["Random"]["Scale"][2][0].
										GetFloat();
									float zhi = d[m]["Random"]["Scale"][2][1].
										GetFloat();
									x_scale = math::rand_in_range(xlo, xhi);
									y_scale = math::rand_in_range(ylo, yhi);
									z_scale = math::rand_in_range(zlo, zhi);
								}

							}
							rot_scale = ScaleAffine(rot_scale, x_scale, y_scale, z_scale);
						}
						//glm::vec3 final_pos = position - mesh_center + offset;
						glm::vec3 final_pos = position + offset;
						rot_scale = SetAffineOffset(rot_scale, final_pos);
						int instId = AddMesh(mesh, rot_scale, mr, RTC_BUILD_QUALITY_MEDIUM);
					} // for all random placements, mr
				}
			}
		}
		else {
			std::cerr << "ERROR: No instances or random specs listed for object "
				<< meshfile << std::endl;
			exit(1);
		}
	}
}

void EmbreeTracer::LoadKeyframeAnimation(std::string anim_file) {
	if (!mavs::utils::file_exists(anim_file)) {
		std::cerr << "ERROR: Couldn't find animation file " << anim_file << std::endl;
	}

	FILE* fp = fopen(anim_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d1;
	d1.ParseStream(is);
	fclose(fp);

	if (d1.HasMember("Animations")) {
		const rapidjson::Value& d = d1["Animations"];
		std::vector<Animation> anims;
		for (int i = 0; i < (int)d.Capacity(); i++) {
			Animation anim;
			if (d[i].HasMember("Animation Folder")) {
				std::string folder = d[i]["Animation Folder"].GetString();
				anim.SetPathToMeshes(file_path_ + folder);
			}
			else {
				continue;
			}
			if (d[i].HasMember("Frame List")) {
				std::string frame_list = d[i]["Frame List"].GetString();
				anim.LoadFrameList(file_path_ + frame_list);
			}
			else {
				continue;
			}
			std::string behavior = "";
			if (d[i].HasMember("Path")) {
				std::string path_file = d[i]["Path"].GetString();
				anim.LoadPathFile(file_path_ + "../../" + path_file);
			}
			else if (d[i].HasMember("Behavior")) {
				behavior = d[i]["Behavior"].GetString();
				anim.SetBehavior(behavior);
			}
			else {
				continue;
			}
			float frame_rate = 30.0f;
			if (d[i].HasMember("Frame Rate")) {
				frame_rate = d[i]["Frame Rate"].GetFloat();
				anim.SetFrameRate(frame_rate);
			}
			float mesh_scale = 1.0f;
			if (d[i].HasMember("Mesh Scale")) {
				mesh_scale = d[i]["Mesh Scale"].GetFloat();
				anim.SetMeshScale(mesh_scale);
			}
			if (d[i].HasMember("Speed")) {
				float speed = d[i]["Speed"].GetFloat();
				anim.SetSpeed(speed);
			}
			if (d[i].HasMember("Initial Position")) {
				if (d[i]["Initial Position"].Capacity() == 3) {
					float x = d[i]["Initial Position"][0].GetFloat();
					float y = d[i]["Initial Position"][1].GetFloat();
					float z = d[i]["Initial Position"][2].GetFloat();
					anim.SetPosition(glm::vec3(x, y, z));
				}
			}
			if (d[i].HasMember("Initial Heading")) {
				float theta = d[i]["Initial Heading"].GetFloat();
				anim.SetHeading(theta);
			}

			if (d[i].HasMember("Rotate Y to X")) {
				bool ryx = d[i]["Rotate Y to X"].GetBool();
				anim.SetRotateYToX(ryx);
			}
			if (d[i].HasMember("Rotate Y to Z")) {
				bool ryz = d[i]["Rotate Y to Z"].GetBool();
				anim.SetRotateYToZ(ryz);
			}
			bool smooth_normals = false;
			if (d[i].HasMember("Smooth Normals")) {
				smooth_normals = d[i]["Smooth Normals"].GetBool();
			}
			if (behavior == "crowd") {
				int num_to_add = 0;
				glm::vec2 ll(-100, -100), ur(100, 100);
				if (d[i].HasMember("Bounding Box")) {
					ll.x = d[i]["Bounding Box"][0][0].GetFloat();
					ll.y = d[i]["Bounding Box"][0][1].GetFloat();
					ur.x = d[i]["Bounding Box"][1][0].GetFloat();
					ur.y = d[i]["Bounding Box"][1][1].GetFloat();
				}
				if (d[i].HasMember("Number")) {
					num_to_add = d[i]["Number"].GetInt();
				}
				for (int j = 0; j < num_to_add; j++) {
					float theta = (float)mavs::math::rand_in_range(0.0, mavs::k2Pi);
					float x = mavs::math::rand_in_range(ll.x, ur.x);
					float y = mavs::math::rand_in_range(ll.y, ur.y);
					anim.SetPosition(x, y);
					anim.SetHeading(theta);
					anims.push_back(anim);
				}
			}
			else {
				anims.push_back(anim);
			}
		} //loop over "Animations" Capacity
		for (int p = 0; p < anims.size(); p++) {
			AddAnimation(anims[p]);
		}
	} // If has member "Animations
} // LoadKeyframeAnimations

int EmbreeTracer::AddAnimation(Animation &anim) {
	animations_.push_back(anim);
	Mesh mesh = anim.GetMesh(0);
	mesh.Scale(anim.GetMeshScale(), anim.GetMeshScale(), anim.GetMeshScale());
	int animId = AddMeshToScene(mesh, animation_scene_, RTC_BUILD_QUALITY_REFIT);
	meshes_.push_back(mesh);
	inst_to_meshnum_.push_back((int)(meshes_.size() - 1));
	animation_ids_.push_back(animId);
	anim_map_[animId] = (int)animations_.size() - 1;
	return animId;
}

bool EmbreeTracer::Load(std::string input_file, unsigned int seed) {
	srand(seed);
	bool loaded = Load(input_file);
	return loaded;
}

bool EmbreeTracer::Load(std::string input_file) {

	if (scene_loaded_) {
		return scene_loaded_;
	}

	if (!mavs::utils::file_exists(input_file)) {
		std::cerr << "ERROR: Scene file " << input_file << " doesn't exist." << std::endl;
		return false;
	}

	std::clock_t start_time = std::clock();
	
  FILE* fp = fopen(input_file.c_str(),"rb");
  char readBuffer[65536];
  rapidjson::FileReadStream is(fp,readBuffer, sizeof(readBuffer));
  rapidjson::Document d;
  d.ParseStream(is);
  fclose(fp);

  mavs::MavsDataPath mavs_data_path;
  
  if (d.HasMember("PathToMeshes")){
    file_path_ = d["PathToMeshes"].GetString();
  }
  else {
    file_path_ = mavs_data_path.GetPath();
    file_path_.append("/scenes/meshes/");
  }
	//bool use_full_paths = false;
	if (d.HasMember("UseFullPaths")) {
		use_full_paths_ = d["UseFullPaths"].GetBool();
	}

	if (d.HasMember("Object Labels")) {
		std::string full_labels = file_path_;
		std::string label_file = d["Object Labels"].GetString();
		full_labels.append(label_file);
		LoadSemanticLabels(full_labels);
	}
	else {
		//
	}

	// Add layered surface mesh
	if (d.HasMember("Layered Surface")) {
		LoadLayeredSurface(d["Layered Surface"]);
	} // If HasMember "Layered Surface"

	// Add surface mesh
	if (d.HasMember("Surface Mesh")) {
		LoadSurfaceMesh(d["Surface Mesh"]);
	} // If HasMember "Surface Mesh"

	// Add objects
  if (d.HasMember("Objects")){
		LoadObjects(d["Objects"]);
  } //if document has member objects

	// Add veg spatial distribution
	if (d.HasMember("Vegetation Distribution")) {
		LoadVegDistribution(d["Vegetation Distribution"]);
	}

	if (d.HasMember("Animations")) {
		std::string anim_file = mavs_data_path.GetPath();
		anim_file.append("/");
		std::string anim_file_t = d["Animations"].GetString();
		anim_file.append(anim_file_t);
		//LoadKeyframeAnimation(d["Animations"]);
		LoadKeyframeAnimation(anim_file);
	}

	CommitScene();
	load_time_ = (std::clock() - start_time) / (double)CLOCKS_PER_SEC;

	if (num_facets_in_scene_>0)scene_loaded_ = true;

	return scene_loaded_;
}

void EmbreeTracer::CommitScene() {
	rtcCommitScene(rtc_scene_);
	rtcCommitScene(animation_scene_);
	LoadTextures();
}

void EmbreeTracer::CommitSceneWithoutTextureReload() {
	rtcCommitScene(rtc_scene_);
}

void EmbreeTracer::CommitAnimationScene() {
	rtcCommitScene(animation_scene_);
}

void EmbreeTracer::WriteSceneStats(std::string output_directory) {
	std::ofstream fout;
	std::string output_file = output_directory;
	output_file.append("/scene_stats.txt");
	fout.open(output_file.c_str());
	fout << "Num objects = " << num_meshes_ << std::endl;
	fout << "Num facets = " << num_facets_in_scene_ << std::endl;
	fout << "Load time = " << load_time_ << std::endl;
	fout.close();
}

void EmbreeTracer::WriteSceneStats() {
	WriteSceneStats("./");
}


std::vector<int> EmbreeTracer::AddActor(std::string meshfile, bool y_to_z, bool x_to_y, bool y_to_x, glm::vec3 offset, glm::vec3 scale) {
	Mesh mesh;
	std::string to_load = (file_path_ + meshfile);
	if (!semantic_labels_.count(meshfile))std::cerr << "WARNING: NO LABEL FOUND FOR MESH " << meshfile << std::endl;
	mesh.Load(file_path_, to_load);
	if (y_to_z)mesh.RotateYToZ();
	if (x_to_y)mesh.RotateXToY();
	if (y_to_x)mesh.RotateYToX();
	
	glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::mat3x4 rot_scale = GetAffineIdentity();
	rot_scale[0][0] = rot_scale[0][0] * scale.x;
	rot_scale[1][1] = rot_scale[1][1] * scale.y;
	rot_scale[2][2] = rot_scale[2][2] * scale.z;

	//glm::vec3 mesh_center = mesh.GetCenter();
	//glm::vec3 offset = position - mesh_center;
	rot_scale = SetAffineOffset(rot_scale, offset);
	int instId = AddMesh(mesh, rot_scale, 0, RTC_SCENE_FLAG_DYNAMIC | RTC_BUILD_QUALITY_LOW);
	actor_nums_.push_back(instId);

	LoadTextures();
	return actor_nums_;
}

std::vector<int> EmbreeTracer::AddActors(std::string actorfile) {
	if (!mavs::utils::file_exists(actorfile)) {
		std::cerr << "ERROR: file " << actorfile << " does not exist " << std::endl;
		return actor_nums_;
	}
	FILE* fp = fopen(actorfile.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);
	if (d.HasMember("Actors")) {
		for (unsigned int m = 0; m<d["Actors"].Capacity(); m++) {
			bool y_to_z = false;
			if (d["Actors"][m].HasMember("Rotate Y to Z")) {
				y_to_z = d["Actors"][m]["Rotate Y to Z"].GetBool();
			}
			bool x_to_y = false;
			if (d["Actors"][m].HasMember("Rotate X to Y")) {
				x_to_y = d["Actors"][m]["Rotate X to Y"].GetBool();
			}
			bool y_to_x = false;
			if (d["Actors"][m].HasMember("Rotate Y to X")) {
				y_to_x = d["Actors"][m]["Rotate Y to X"].GetBool();
			}
			std::string meshfile;
			glm::vec3 mesh_center;
			Mesh mesh;
			if (d["Actors"][m].HasMember("Mesh")) {
				meshfile = d["Actors"][m]["Mesh"].GetString();
				std::string to_load = (file_path_ + meshfile);
				if (!semantic_labels_.count(meshfile))std::cerr << "WARNING: NO LABEL FOUND FOR MESH " << meshfile << std::endl;
				mesh.Load(file_path_, to_load);
				if (y_to_z)mesh.RotateYToZ();
				if (x_to_y)mesh.RotateXToY();
				if (y_to_x)mesh.RotateYToX();
				mesh_center = mesh.GetCenter();
			}
			if (d["Actors"][m].HasMember("Instances")) {
				int num_inst = d["Actors"][m]["Instances"].Capacity();
				for (int mi = 0; mi<num_inst; mi++) {
					glm::mat3x4 rot_scale = GetAffineIdentity();
					glm::vec3 position;
					if (d["Actors"][m]["Instances"][mi].HasMember("Position")) {
						if (d["Actors"][m]["Instances"][mi]["Position"].Capacity() == 3) {
							position.x = d["Actors"][m]["Instances"][mi]["Initial Position"][0].GetFloat();
							position.y = d["Actors"][m]["Instances"][mi]["Initial Position"][1].GetFloat();
							position.z = d["Actors"][m]["Instances"][mi]["Initial Position"][2].GetFloat();
						}
					}
					else {
						position = glm::vec3(0.0f, 0.0f, 0.0f);
					}
					if (d["Actors"][m]["Instances"][mi].HasMember("Scale")) {
						if (d["Actors"][m]["Instances"][mi]["Scale"].Capacity() != 3) {
							std::cerr << "ERROR: Scale must have 3 elements." << std::endl;
							exit(1);
						}
						rot_scale[0][0] = rot_scale[0][0] *
							d["Actors"][m]["Instances"][mi]["Scale"][0].GetFloat();
						rot_scale[1][1] = rot_scale[1][1] *
							d["Actors"][m]["Instances"][mi]["Scale"][1].GetFloat();
						rot_scale[2][2] = rot_scale[2][2] *
							d["Actors"][m]["Instances"][mi]["Scale"][2].GetFloat();
					}
					if (d["Actors"][m].HasMember("Mesh")) {
						glm::vec3 offset = position - mesh_center;
						rot_scale = SetAffineOffset(rot_scale, offset);
						int instId = AddMesh(mesh, rot_scale, mi, RTC_SCENE_FLAG_DYNAMIC | RTC_BUILD_QUALITY_LOW);
						actor_nums_.push_back(instId);
					}
				} //loop over instances
			} // if instanced
			else {
				std::cerr << "ERROR: No instances listed for Actor "
					<< meshfile << std::endl;
				exit(1);
			}
		}
	}
	LoadTextures();
	return actor_nums_;
}

unsigned int EmbreeTracer::AddMesh(Mesh &mesh, glm::mat3x4 aff_rot, int inst_num, int state) {
	num_facets_in_scene_ += (unsigned long int)(mesh.GetNumFaces());
	if (inst_num == 0) {
		RTCScene scene = rtcNewScene(rtc_device_);
		instanced_scenes_.push_back(scene); // this does the scene commit
		int mesh_id = AddMeshToScene(mesh, instanced_scenes_.back(), (RTCBuildQuality)state);
		meshes_.push_back(mesh);
	}

	RTCGeometry geom = rtcNewGeometry(rtc_device_, RTC_GEOMETRY_TYPE_INSTANCE);
	rtcSetGeometryInstancedScene(geom, instanced_scenes_.back());
	rtcSetGeometryTimeStepCount(geom, 1);
	if (state == RTC_SCENE_FLAG_DYNAMIC) {
		rtcAttachGeometry(animation_scene_, geom);
		num_dynamic_++;
	}
	else {
		rtcAttachGeometry(rtc_scene_, geom);
	}
	rtcSetGeometryTransform(geom, 0, RTC_FORMAT_FLOAT3X4_ROW_MAJOR, (float*)&aff_rot);
	rtcReleaseGeometry(geom);
	
	// add the bounding box of the mesh to the list
	BoundingBox bb = mesh.GetBoundingBox();
	bb.Transform(aff_rot); //might have to fix this
	bounding_boxes_.push_back(bb);
	if (bb.GetLowerLeft().x < lower_left_corner_.x)lower_left_corner_.x = bb.GetLowerLeft().x;
	if (bb.GetLowerLeft().y < lower_left_corner_.y)lower_left_corner_.y = bb.GetLowerLeft().y;
	if (bb.GetLowerLeft().z < lower_left_corner_.z)lower_left_corner_.z = bb.GetLowerLeft().z;
	if (bb.GetUpperRight().x > upper_right_corner_.x)upper_right_corner_.x = bb.GetUpperRight().x;
	if (bb.GetUpperRight().y > upper_right_corner_.y)upper_right_corner_.y = bb.GetUpperRight().y;
	if (bb.GetUpperRight().z > upper_right_corner_.z)upper_right_corner_.z = bb.GetUpperRight().z;
	glm::mat3 R(aff_rot);
	glm::quat orientation = glm::toQuat(R);
	orientations_.push_back(orientation);

	// set transfroms and id maps
	if (inst_id_>=inst_to_meshnum_.size()){
		int to_push = (int)(inst_id_-inst_to_meshnum_.size()+1);
		for (int tp =0; tp<to_push;tp++){
			inst_to_meshnum_.push_back(0);
		}
	}

	inst_to_meshnum_[inst_id_] = (int)(meshes_.size() - 1);
	if (state == RTC_SCENE_FLAG_DYNAMIC) {
		rtcCommitGeometry(rtcGetGeometry(animation_scene_, inst_id_));
	}
	else {
		rtcCommitGeometry(rtcGetGeometry(rtc_scene_, inst_id_));
	}
	// This is done after all meshes are loaded
	// Doing it after every mesh slows the load down
	//rtcCommitScene(rtc_scene_);

	num_meshes_++;
	
	inst_id_++;
	return inst_id_-1;
}
 
unsigned int EmbreeTracer::SetLayeredSurfaceMesh(Mesh &mesh, glm::mat3x4 aff_rot) {	
	unsigned int ls_instId = 0;
	
	num_facets_in_scene_ += (unsigned long int)(mesh.GetNumFaces());

	RTCScene scene = rtcNewScene(rtc_device_);
	mesh.CalculateNormals();
	layered_surface_meshes_.push_back(mesh);
	int mesh_id = AddMeshToScene(mesh, scene, RTC_BUILD_QUALITY_MEDIUM); // this does the commit to the local scene
	BoundingBox bb = mesh.GetBoundingBox();
	bb.Transform(aff_rot); //might have to fix this
	bounding_boxes_.push_back(bb);
	if (bb.GetLowerLeft().x < lower_left_corner_.x)lower_left_corner_.x = bb.GetLowerLeft().x;
	if (bb.GetLowerLeft().y < lower_left_corner_.y)lower_left_corner_.y = bb.GetLowerLeft().y;
	if (bb.GetLowerLeft().z < lower_left_corner_.z)lower_left_corner_.z = bb.GetLowerLeft().z;
	if (bb.GetUpperRight().x > upper_right_corner_.x)upper_right_corner_.x = bb.GetUpperRight().x;
	if (bb.GetUpperRight().y > upper_right_corner_.y)upper_right_corner_.y = bb.GetUpperRight().y;
	if (bb.GetUpperRight().z > upper_right_corner_.z)upper_right_corner_.z = bb.GetUpperRight().z;
	
	RTCGeometry geom = rtcNewGeometry(rtc_device_, RTC_GEOMETRY_TYPE_INSTANCE);
	rtcSetGeometryInstancedScene(geom, scene);
	rtcAttachGeometry(layered_surface_, geom);
	rtcSetGeometryTimeStepCount(geom, 1);
	rtcSetGeometryTransform(geom, 0, RTC_FORMAT_FLOAT3X4_ROW_MAJOR, (float*)&aff_rot);
	rtcReleaseGeometry(geom);

	rtcCommitGeometry(rtcGetGeometry(layered_surface_, ls_instId));
	rtcCommitScene(layered_surface_);

	return ls_instId;
}

unsigned int EmbreeTracer::SetSurfaceMesh(Mesh &mesh, glm::mat3x4 aff_rot) {
	
	unsigned int instId = 0;
	
	num_facets_in_scene_ += (unsigned long int)(mesh.GetNumFaces());
	mesh.ApplyAffineTransformation(aff_rot);
	instId = AddMeshToScene(mesh, surface_, RTC_BUILD_QUALITY_MEDIUM);
	
	
	surface_meshes_.push_back(mesh);
	BoundingBox bb = mesh.GetBoundingBox();
	//bb.Transform(aff_rot); //might have to fix this
	bounding_boxes_.push_back(bb);
	if (bb.GetLowerLeft().x < lower_left_corner_.x)lower_left_corner_.x = bb.GetLowerLeft().x;
	if (bb.GetLowerLeft().y < lower_left_corner_.y)lower_left_corner_.y = bb.GetLowerLeft().y;
	if (bb.GetLowerLeft().z < lower_left_corner_.z)lower_left_corner_.z = bb.GetLowerLeft().z;
	if (bb.GetUpperRight().x > upper_right_corner_.x)upper_right_corner_.x = bb.GetUpperRight().x;
	if (bb.GetUpperRight().y > upper_right_corner_.y)upper_right_corner_.y = bb.GetUpperRight().y;
	if (bb.GetUpperRight().z > upper_right_corner_.z)upper_right_corner_.z = bb.GetUpperRight().z;

	mesh.ClearMemory();

	surface_loaded_ = true;
	
	return instId;
}


unsigned int EmbreeTracer::AddMeshToScene(Mesh &mesh, RTCScene &scene, RTCBuildQuality flags) {
	// create new mesh
  int numTriangles = (int)mesh.GetNumFaces();
  int numVertices = (int)mesh.GetNumVertices();

	RTCGeometry geom = rtcNewGeometry(rtc_device_, RTC_GEOMETRY_TYPE_TRIANGLE);
	
	//fill vertices from mesh
	Vertex* vertices = (Vertex*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), numVertices);
  for (int i=0; i<numVertices; i++){
    vertices[i].x = mesh.GetVertex(i).x;
    vertices[i].y = mesh.GetVertex(i).y;
    vertices[i].z = mesh.GetVertex(i).z;
  }

	//fill triangles from mesh
	Triangle* triangles = (Triangle*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), numTriangles);
  for (int i=0; i<numTriangles; i++){
    triangles[i].v0 = mesh.GetFace(i).v1;
    triangles[i].v1 = mesh.GetFace(i).v2;
    triangles[i].v2 = mesh.GetFace(i).v3;
  }

	unsigned int geomID = rtcAttachGeometry(scene, geom);
	rtcReleaseGeometry(geom);

	rtcCommitGeometry(rtcGetGeometry(scene, geomID));
	rtcCommitScene(scene);

  std::vector<std::string> tex_names = mesh.GetAllTextureNames();
  for (int i=0;i<(int)tex_names.size();i++){
		std::string extension = mavs::utils::GetFileExtension(tex_names[i]);
		if (extension == "spec") {
			//load a mavs spectrum
			std::string to_erase = mavs::utils::GetPathFromFile(tex_names[i]);
			to_erase.append("/");
			mavs::utils::EraseSubString(tex_names[i], to_erase);
			spectrum_names_.push_back(tex_names[i]);
		}
		else {
			texture_names_.push_back(tex_names[i]);
		}
  }

  return geomID;
}

// Methods to access mesh information
Mesh * EmbreeTracer::GetMesh(int meshnum){
	return &meshes_[meshnum];
}

Mesh * EmbreeTracer::GetMeshByInstID(int instID){
	return &meshes_[inst_to_meshnum_[instID]];
}

glm::vec3 EmbreeTracer::GetAnimationPosition(int anim_num) {
	glm::vec3 p(0.0f, 0.0f, 0.0f);
	if (anim_num >= 0 && anim_num < animations_.size()) {
		p = animations_[anim_num].GetPosition();
	}
	return p;
}

void EmbreeTracer::UpdateAnimations(float dt) {

	for (int i = 0; i < (int)animation_ids_.size(); i++) {
		int anim_idx = anim_map_[animation_ids_[i]];
		if (animations_[anim_idx].GetBehavior() != "crowd") {
			animations_[anim_idx].Update(dt);
			glm::vec3 p = animations_[anim_idx].GetPosition();
			float z = GetSurfaceHeight(p.x, p.y);
			animations_[anim_idx].SetHeight(z);
		}
	}
	MoveCrowd(dt, animations_);

	for (int i = 0; i < (int)animation_ids_.size(); i++) {
		MoveAnimationVerts(animation_ids_[i]);
	}
	//rtcCommitScene(rtc_scene_);
	rtcCommitScene(animation_scene_);
}

void EmbreeTracer::SetAnimationPosition(int anim_id, float x, float y, float heading) {
	animations_[anim_map_[anim_id]].SetPosition(x, y);
	float z = GetSurfaceHeight(x, y);
	animations_[anim_map_[anim_id]].SetHeight(z);
	animations_[anim_map_[anim_id]].SetHeading(heading);
}

void EmbreeTracer::MoveAnimationVerts(int anim_id) {
	int nv = animations_[anim_map_[anim_id]].GetNumVerts();
	//Vertex *verts = (Vertex*)rtcGetGeometryBufferData(rtcGetGeometry(rtc_scene_, anim_id), RTC_BUFFER_TYPE_VERTEX, 0);
	Vertex *verts = (Vertex*)rtcGetGeometryBufferData(rtcGetGeometry(animation_scene_, anim_id), RTC_BUFFER_TYPE_VERTEX, 0);
	for (int i = 0; i < nv; i++) {
		Vertex* v = &verts[i];
		v->x = animations_[anim_map_[anim_id]].GetVertex(i).x;
		v->y = animations_[anim_map_[anim_id]].GetVertex(i).y;
		v->z = animations_[anim_map_[anim_id]].GetVertex(i).z;
	}
	//rtcCommitGeometry(rtcGetGeometry(rtc_scene_, anim_id));
	rtcCommitGeometry(rtcGetGeometry(animation_scene_, anim_id));
}

void EmbreeTracer::SetActorVelocity(int actor_id, glm::vec3 velocity) {
	int meshnum = inst_to_meshnum_[actor_id];
	meshes_[meshnum].SetVelocity(velocity);
}

void EmbreeTracer::UpdateActor(glm::vec3 position, glm::quat orientation, glm::vec3 scale, int actor_id) {
	UpdateActor(position, orientation, scale, actor_id, true);
}

void EmbreeTracer::UpdateActor(glm::vec3 position, glm::quat orientation, glm::vec3 scale, int actor_id, bool commit_scene) {

	int meshnum = inst_to_meshnum_[actor_id];

	glm::vec3 offset = position; // -mesh_center;

	glm::mat3 rot_o = glm::mat3_cast(orientation);
	glm::mat3 rot = glm::inverse(rot_o);
	glm::mat3x4 rot_scale = ScaleAffine(rot, scale[0], scale[1], scale[2]);
	
	rot_scale[0][3] = offset.x; rot_scale[1][3] = offset.y; rot_scale[2][3] = offset.z;

	orientation = glm::normalize(orientation);
	meshes_[meshnum].SetOrientation(orientation);

	//rtcSetGeometryTransform(rtcGetGeometry(rtc_scene_, actor_id), 0, RTC_FORMAT_FLOAT3X4_ROW_MAJOR, (float*)&rot_scale);
	//rtcCommitGeometry(rtcGetGeometry(rtc_scene_, actor_id));
	//if (commit_scene)rtcCommitScene(rtc_scene_);

	rtcSetGeometryTransform(rtcGetGeometry(animation_scene_, actor_id), 0, RTC_FORMAT_FLOAT3X4_ROW_MAJOR, (float*)&rot_scale);
	rtcCommitGeometry(rtcGetGeometry(animation_scene_, actor_id));
	if (commit_scene)rtcCommitScene(animation_scene_);
}

float EmbreeTracer::GetSurfaceSlope(glm::vec2 point, glm::vec2 direction, float h) {
	glm::vec2 p1 = point - 0.5f*h*direction;
	glm::vec2 p2 = point + 0.5f*h*direction;
	float z1 = GetSurfaceHeight(p1.x, p1.y);
	float z2 = GetSurfaceHeight(p2.x, p2.y);
	float slope = (z2 - z1) / h;
	return slope;
}

float EmbreeTracer::GetSurfaceHeight(float x, float y) {
	glm::vec4 nret = GetSurfaceHeightAndNormal(x, y);
	return nret.w;
}

Intersection EmbreeTracer::GetClosestTerrainIntersection(glm::vec3 pos, glm::vec3 dir) {
	RTCRayHit query;
	query.ray.org_x = pos.x;
	query.ray.org_y = pos.y;
	query.ray.org_z = pos.z;
	query.ray.dir_x = dir.x;
	query.ray.dir_y = dir.y;
	query.ray.dir_z = dir.z;
	query.ray.tnear = 0.0f;
	query.ray.tfar = std::numeric_limits<float>::infinity();
	query.ray.id = RTC_INVALID_GEOMETRY_ID;
	query.ray.mask = 0xFFFFFFFF;
	query.ray.time = 0.0f;

	if (surface_loaded_) {
		rtcIntersect(surface_, query);
	}
	else {
		rtcIntersect(rtc_scene_, query);
	}
	glm::vec3 normal(0.0f, 0.0f, 1.0f);
	float elev = std::numeric_limits<float>::lowest();
	Intersection inter;
	if (query.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
		elev = query.ray.org_z + query.ray.tfar*query.ray.dir_z;
		normal.x = query.hit.Ng_x;
		normal.y = query.hit.Ng_y;
		normal.z = query.hit.Ng_z;
		normal = normal / glm::length(normal);
		inter.dist = query.ray.tfar;
		inter.normal = normal;
	}
	
	return inter;
}

glm::vec4 EmbreeTracer::GetSurfaceHeightAndNormal(float x, float y, float z) {
	RTCRayHit query;
	query.ray.org_x = x;
	query.ray.org_y = y;
	query.ray.org_z = z;
	query.ray.dir_x = 0.0f;
	query.ray.dir_y = 0.0f;
	query.ray.dir_z = -1.0f;
	query.ray.tnear = 0.0f;
	query.ray.tfar = std::numeric_limits<float>::infinity();
	query.ray.mask = 0xFFFFFFFF;
	query.ray.time = 0.0f;

	if (surface_loaded_) {
		rtcIntersect(surface_, query);
	}
	else {
		rtcIntersect(rtc_scene_, query);
	}

	glm::vec3 normal(0.0f, 0.0f, 1.0f);
	float elev = std::numeric_limits<float>::lowest();
	if (query.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
		elev = query.ray.org_z + query.ray.tfar*query.ray.dir_z;
		normal.x = query.hit.Ng_x;
		normal.y = query.hit.Ng_y;
		normal.z = query.hit.Ng_z;
		normal = normal / glm::length(normal);
	}
	glm::vec4 nret;
	nret.x = normal.x;
	nret.y = normal.y;
	nret.z = normal.z;
	nret.w = elev;
	return nret;
}

glm::vec4 EmbreeTracer::GetSurfaceHeightAndNormal(float x, float y){
	return GetSurfaceHeightAndNormal(x, y, upper_right_corner_.z + 500.0f);
}

glm::vec3 EmbreeTracer::GetInterpolatedNormalFromLayeredSurface(int instID, int primID, float u, float v, glm::vec3 vd) {
	float w = 1.0f - u - v;
	int v0 = layered_surface_meshes_[instID].GetFace(primID).v1;
	int v1 = layered_surface_meshes_[instID].GetFace(primID).v2;
	int v2 = layered_surface_meshes_[instID].GetFace(primID).v3;
	glm::vec3 n0 = layered_surface_meshes_[instID].GetNormal(v0);
	glm::vec3 n1 = layered_surface_meshes_[instID].GetNormal(v1);
	glm::vec3 n2 = layered_surface_meshes_[instID].GetNormal(v2);
	glm::vec3 normal = w * n0 + u * n1 + v * n2;
	normal = glm::normalize(normal);
	return normal;
}

glm::vec3 EmbreeTracer::GetInterpolatedNormal(int instID, int primID, float u, float v, glm::vec3 vd) {
	float w = 1.0f - u - v;
	int v0 = meshes_[instID].GetFace(primID).v1;
	int v1 = meshes_[instID].GetFace(primID).v2;
	int v2 = meshes_[instID].GetFace(primID).v3;
	glm::vec3 n0 = meshes_[instID].GetNormal(v0);
	glm::vec3 n1 = meshes_[instID].GetNormal(v1);
	glm::vec3 n2 = meshes_[instID].GetNormal(v2);
	glm::vec3 normal = w * n0 + u * n1 + v * n2;
	normal = glm::normalize(normal);
	return normal;
}

void EmbreeTracer::GetInterpolatedTextures(int instID, int primID, int texwidth, int texheight, float u_in, float v_in, int &u_out, int &v_out){
  float w = 1.0f-u_in-v_in;
  int t0 = meshes_[instID].GetFace(primID).t1;
  int t1 = meshes_[instID].GetFace(primID).t2;
  int t2 = meshes_[instID].GetFace(primID).t3;
  float u0 = meshes_[instID].GetTexCoord(t0).x;
  float v0 = meshes_[instID].GetTexCoord(t0).y;
  float u1 = meshes_[instID].GetTexCoord(t1).x;
  float v1 = meshes_[instID].GetTexCoord(t1).y;
  float u2 = meshes_[instID].GetTexCoord(t2).x;
  float v2 = meshes_[instID].GetTexCoord(t2).y;
  float upf = w*u0 + u_in*u1 + v_in*u2;
  float vpf = w*v0 + u_in*v1 + v_in*v2;
  v_out = (int)((1.0f-vpf)*texheight);
  u_out = (int)(upf*texwidth);
  while(u_out>=texwidth)u_out -= texwidth;
  while(u_out<0)u_out += texwidth;
  while(v_out>=texheight)v_out -= texheight;
  while(v_out<0)v_out += texheight;
}

//https://math.stackexchange.com/questions/1956699/getting-a-transformation-matrix-from-a-normal-vector
glm::mat3 EmbreeTracer::MatrixFromNormal(float nx, float ny, float nz) {
	float nxy2 = nx * nx + ny * ny;
	float n = sqrt(nxy2 + nz * nz); 
	float nxy = sqrt(nxy2);
	nx = nx / n;
	ny = ny / n;
	nz = nz / n;
	float nxy_inv = 1.0f/nxy;
	float nz_nxy = nz*nxy_inv;
	glm::mat3 R = GetIdentity(); // kidentity_matrix_;
	if (nxy > 0.0f) {
		R[0][0] = ny*nxy_inv;
		R[0][1] = -nx*nxy_inv;
		//R[0][2] = 0.0f;
		R[1][0] = nx * nz_nxy;
		R[1][1] = ny * nz_nxy;
		R[1][2] = -nxy;
		R[2][0] = nx;
		R[2][1] = ny;
		R[2][2] = nz;
	}
	return R;
}

float EmbreeTracer::GetReflectance(std::string spec_name, float wavelength) {
	float rho = 0.0;
	if (spectra_.count(spec_name) > 0) {
		rho = spectra_[spec_name].GetReflectanceAtWavelength(wavelength);
	}
	return rho;
}

bool EmbreeTracer::IsInPothole(float x, float y) {
	for (int i = 0; i < potholes_.size(); i++) {
		float dx = potholes_[i][2] - x;
		float dy = potholes_[i][3] - y;
		float r = (float)sqrt(dx*dx + dy * dy);
		if (r < potholes_[i][1]) return true;
	}
	return false;
}

Intersection EmbreeTracer::GetClosestIntersection(glm::vec3 origin, glm::vec3 direction){
	Intersection inter;
	GetClosestIntersection(origin,direction, inter);
	return inter;
}

void EmbreeTracer::GetClosestIntersection(glm::vec3 origin, glm::vec3 direction, Intersection &inter){
	RTCRayHit query;
	glm::vec3 original_origin = origin;
	float dist_accumulator = 0.0;
	inter.dist = -1.0f;
	unsigned short int counter = 0;
	while (true) {
		query.ray.org_x = origin.x;
		query.ray.org_y = origin.y;
		query.ray.org_z = origin.z;
		query.ray.dir_x = direction.x;
		query.ray.dir_y = direction.y;
		query.ray.dir_z = direction.z;
		query.ray.tnear = 0.0f;
		query.ray.tfar = std::numeric_limits<float>::infinity();
		query.ray.mask = 0xFFFFFFFF;
		query.ray.time = 0.0f;
		RTCRayHit anim_query = query;

		rtcIntersect(rtc_scene_, query);
		rtcIntersect(animation_scene_, anim_query);

		//if (query.hit.geomID != RTC_INVALID_GEOMETRY_ID ) {
		if (query.hit.geomID != RTC_INVALID_GEOMETRY_ID || anim_query.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

			// next few lines are added to split animation scene from static scene
			if (query.hit.geomID != RTC_INVALID_GEOMETRY_ID && anim_query.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
				// they both, chooose the closes one
				if (anim_query.ray.tfar < query.ray.tfar) query = anim_query;
			}
			else if (query.hit.geomID == RTC_INVALID_GEOMETRY_ID && anim_query.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
				// only the animation hit
				query = anim_query;
			}

			if (query.hit.instID[0] == RTC_INVALID_GEOMETRY_ID)query.hit.instID[0] = query.hit.geomID;
			int instID = inst_to_meshnum_[(int)query.hit.instID[0]];
			// store id of object instance at intersection
			inter.object_id = (int)query.hit.instID[0];
			inter.velocity = meshes_[instID].GetVelocity();
			if (perform_labeling_) {
				if (meshes_[instID].GetLabelByGroup()) {
					inter.object_name = meshes_[instID].GetGroupOf(query.hit.primID);
				}
				else {
					inter.object_name = meshes_[instID].GetName();
				}
				inter.label = semantic_labels_[inter.object_name];
			}
			int mnum = meshes_[instID].GetMatNum(query.hit.primID);
			Material * return_mat = meshes_[instID].GetMaterial(mnum);
			
			//check transparency
			float trans = 1.0;
			if (return_mat->map_d.length() > 0) {
				int up = 0;
				int vp = 0;
				GetInterpolatedTextures(instID, query.hit.primID,
					textures_[return_mat->map_d].width(),
					textures_[return_mat->map_d].height(),
					query.hit.u, query.hit.v, up, vp);
				trans = textures_[return_mat->map_d](up, vp, 0, 0);
			}
			if (trans > 0.5) { //not transparent
				inter.color = return_mat->kd;
				if (return_mat->map_kd.length() > 0) {
					int up = 0;
					int vp = 0;
					
					GetInterpolatedTextures(instID, query.hit.primID,
						textures_[return_mat->map_kd].width(),
						textures_[return_mat->map_kd].height(),
						query.hit.u, query.hit.v, up, vp);
					inter.color.x =
						return_mat->kd.x*textures_[return_mat->map_kd](up, vp, 0, 0);
					inter.color.y =
						return_mat->kd.y*textures_[return_mat->map_kd](up, vp, 1, 0);
					inter.color.z =
						return_mat->kd.z*textures_[return_mat->map_kd](up, vp, 2, 0);
					
				}

				if (use_spectral_) {
					if (return_mat->refl.length() > 0) {
						if (spectra_.count(return_mat->refl) > 0) {
							inter.spectrum_name = return_mat->refl;
						}
						else {
							std::cerr << "WARNING: No map entry for spectrum " << return_mat->refl << std::endl;
							inter.spectrum_name = "";
						}
					}
				}
				//if (meshes_[inst_to_meshnum_[(int)query.hit.instID[0]]].HasNormals()) {
				//	glm::vec3 normal = GetInterpolatedNormal(inst_to_meshnum_[(int)query.hit.instID[0]], query.hit.primID, query.hit.u, query.hit.v, direction);
				//	glm::vec4 new_n = glm::rotate(meshes_[(int)query.hit.instID[0]].GetOrientation(), glm::vec4(normal.x, normal.y, normal.z, 0.0f));
				//	inter.normal = glm::vec3(new_n.x, new_n.y, new_n.z);
				//}
				//else {
					glm::vec3 normal(query.hit.Ng_x, query.hit.Ng_y, query.hit.Ng_z);
					glm::quat rotquat = meshes_[instID].GetOrientation();
					glm::mat3x3 rotmat(rotquat);
					normal = rotmat * normal;
					float mag = glm::length(normal);
					if (mag > 1.0E-4f) {
						inter.normal = normal / mag;
					}
					else {
						inter.normal = glm::vec3(query.hit.Ng_x, query.hit.Ng_y, query.hit.Ng_z);
						inter.normal = inter.normal / glm::length(inter.normal);
					}
				//}
				inter.dist = dist_accumulator + query.ray.tfar;
				inter.material = *return_mat;
				break;
			} // if not transparent
			else { //it is transparent
				// embree requires the offset to be quite high, apparently due to the
				// construction of the BVH
				//float dist_to_add = ray.tfar + 0.5f;
				float dist_to_add = query.ray.tfar + 0.1f;
				origin = origin + dist_to_add * direction;
				dist_accumulator = dist_accumulator + dist_to_add;
			}
		} //if intersected object
		else { //not intersected object
			break;
		}
		//if (counter>10)break;
		if (counter > 30)break;
		counter++;
		
	} //while(true) over distance accumulator
	
	origin = original_origin;
	if (layered_surfaces_.size() > 0) {
		query.ray.org_x = origin.x;
		query.ray.org_y = origin.y;
		query.ray.org_z = origin.z;
		query.ray.dir_x = direction.x;
		query.ray.dir_y = direction.y;
		query.ray.dir_z = direction.z;
		query.ray.tnear = 0.0f;
		query.ray.tfar = std::numeric_limits<float>::infinity();
		query.ray.mask = 0xFFFFFFFF;
		query.ray.time = 0.0f;
		rtcIntersect(layered_surface_, query);
		if (query.hit.geomID != RTC_INVALID_GEOMETRY_ID && (query.ray.tfar < inter.dist || inter.dist < 0.0f)) {
			inter.object_id = -99;		// store id of object instance at intersection
			glm::vec3 ret_point = origin + query.ray.tfar*direction;
			glm::vec2 surf_point(ret_point.x, ret_point.y);
			if (perform_labeling_) {

				if (IsInPothole(surf_point.x, surf_point.y)) {
					inter.object_name = "pothole";
				}
				else {
					bool on_trail = layered_surfaces_[query.hit.geomID].IsPointOnTrail(surf_point.x, surf_point.y);
					if (on_trail) {
						inter.object_name = "textured_trail";
					}
					else {
						inter.object_name = "textured_surface";
					}
				}
				inter.label = semantic_labels_[inter.object_name];
			}

			Material * return_mat; 
			if (use_surface_textures_) {
				glm::vec3 normal;
				return_mat = layered_surfaces_[query.hit.geomID].GetColorAndNormalAtPoint(surf_point, inter.color, normal);
			  
				// all layered surfaces will be interplated now
				normal = GetInterpolatedNormalFromLayeredSurface((int)query.hit.instID[0], query.hit.primID, query.hit.u, query.hit.v, direction);
				glm::vec4 new_n = glm::rotate(layered_surface_meshes_[(int)query.hit.instID[0]].GetOrientation(), glm::vec4(normal.x, normal.y, normal.z, 0.0f));
				inter.normal = glm::vec3(new_n.x, new_n.y, new_n.z);
				if (inter.normal.z < 0.0f)inter.normal = -1.0f*inter.normal;

			}
			else {
				return_mat = layered_surfaces_[query.hit.geomID].GetMaterial(2);
				inter.color = return_mat->kd;
				
				if (meshes_[inst_to_meshnum_[(int)query.hit.instID[0]]].HasNormals()) {
					glm::vec3 normal = GetInterpolatedNormal(inst_to_meshnum_[(int)query.hit.instID[0]], query.hit.primID, query.hit.u, query.hit.v, direction);
					glm::vec4 new_n = glm::rotate(meshes_[(int)query.hit.instID[0]].GetOrientation(), glm::vec4(normal.x, normal.y, normal.z, 0.0f));
					inter.normal = glm::vec3(new_n.x, new_n.y, new_n.z);
				}
				else {
					inter.normal = glm::vec3(query.hit.Ng_x, query.hit.Ng_y, query.hit.Ng_z);
					inter.normal = inter.normal / glm::length(inter.normal);
				}
			}
			inter.dist = query.ray.tfar;
			//need to make a copy constructor at some point
			inter.material.ka = inter.color;
			inter.material.kd = inter.color;
			inter.material.ks = zero_vec3;
			inter.material.ns = return_mat->ns;
			inter.material.ni = return_mat->ni;
			if (use_spectral_) {
				if (return_mat->refl.length() > 0) {
					if (spectra_.count(return_mat->refl) > 0) {
						inter.spectrum_name = return_mat->refl;
					}
					else {
						std::cerr << "WARNING: No map entry for spectrum " << return_mat->refl << std::endl;
						inter.spectrum_name = "";
					}
				}
			}
		}
	}
}

bool EmbreeTracer::GetAnyIntersection(glm::vec3 org, glm::vec3 light_dir){
	RTCRayHit shadow;
	shadow.ray.org_x = org.x;
	shadow.ray.org_y = org.y;
	shadow.ray.org_z = org.z;
	shadow.ray.dir_x = light_dir.x;
	shadow.ray.dir_y = light_dir.y;
	shadow.ray.dir_z = light_dir.z;
  shadow.ray.tnear = 0.001f;
  shadow.ray.tfar = std::numeric_limits<float>::infinity();
	shadow.hit.geomID = 1;
	shadow.hit.primID = 0;
	shadow.ray.mask = -1;
	shadow.ray.time = 0;

	RTCRayHit shadow_anim = shadow;

	RTCIntersectContext context;
	rtcInitIntersectContext(&context);
	rtcOccluded1(rtc_scene_, &context, &shadow.ray);

	rtcOccluded1(animation_scene_, &context, &shadow_anim.ray);

	// ray is occluded when ray.tfar < 0.0f
	bool shadowed = false;
	if (shadow.ray.tfar < 0.0f || shadow_anim.ray.tfar < 0.0f)shadowed = true;

	return shadowed;
}

void EmbreeTracer::LoadTextures(){
	textures_.clear();
	for (int i=0; i<(int)texture_names_.size(); i++){
		if (mavs::utils::file_exists(texture_names_[i])) {
			cimg_library::CImg<float> image(texture_names_[i].c_str());
			//change scale from [0,255] to [0,1]
			image = 0.00392156862745*image;
			textures_[texture_names_[i]] = image;
		}
		else {
			std::cerr << "WARNING: Attempted to load texture file " <<
				texture_names_[i] << ", but it doesn't exist." << std::endl;
		}
	}

	mavs::MavsDataPath mavs_data_path;
	std::string data_path = mavs_data_path.GetPath();
	spectra_.clear();
	for (int i = 0; i < (int)spectrum_names_.size(); i++) {
		std::string spec_to_load;
		if (spectrum_names_[i].substr(0, 7) != "spectra") {
			spec_to_load = data_path + "/scenes/meshes/spectra/" + spectrum_names_[i];
		}
		else {
			spec_to_load = data_path + "/scenes/meshes/" + spectrum_names_[i];
		}
		ReflectanceSpectrum spectrum(spec_to_load);
		if (spectrum_names_[i].substr(0, 7) != "spectra") {
			std::string new_name = "spectra/";
			new_name.append(spectrum_names_[i]);
			spectrum_names_[i] = new_name;
		}
		spectra_[spectrum_names_[i]] = spectrum;
	}

	for (int i = 0; i < layered_surfaces_.size(); i++) {
		for (int ln = 0; ln < 3; ln++) {
			std::string fname = data_path;
			fname.append("/scenes/meshes/");
			fname.append(layered_surfaces_[i].GetMaterial(ln)->refl);
			ReflectanceSpectrum spectrum(fname);
			spectra_[layered_surfaces_[i].GetMaterial(ln)->refl] = spectrum;
		}
	}
}

glm::quat EmbreeTracer::GetObjectOrientation(int id) {
	glm::quat q(1.0f, 0.0f, 0.0f, 0.0f);
	if (id >= 0 && id < (int)orientations_.size()) {
		q = orientations_[id];
	}
	return q;
}

std::string EmbreeTracer::GetObjectName(int id) {
	std::string name = "";
	if (id >= 0 && id < (int)inst_to_meshnum_.size()) {
		name = meshes_[inst_to_meshnum_[id]].GetName();
	}
	else if (id == -99) {
		name = "textured_surface";
	}
	else  if (id == -1) {
		name = "sky";
	}
	return name;
}

int EmbreeTracer::GetNumberObjects() { 
	int nobj = num_meshes_;
	nobj += (int)(surface_meshes_.size() + layered_surfaces_.size());
	return nobj; 
}

} //namespace embree
} //namespace raytracer
} //namespace mavs
