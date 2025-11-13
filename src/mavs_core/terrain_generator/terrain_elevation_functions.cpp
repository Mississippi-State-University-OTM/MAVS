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
#include <limits>
#include <mavs_core/terrain_generator/terrain_elevation_functions.h>
#include <mavs_core/data_path.h>
#include <mavs_core/math/polygon.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif
#include <mavs_core/terrain_generator/heightmap.h>

namespace mavs {
namespace terraingen {

TerrainCreator::TerrainCreator() {
}

TerrainCreator::~TerrainCreator() {
	terrain_features_.clear();
}

TerrainCreator::TerrainCreator(const TerrainCreator& s) {
	this->scene_ = s.scene_;
	this->terrain_features_ = s.terrain_features_;
}

void TerrainCreator::AddTrapezoid(float bottom_width, float top_width, float depth, float x0) {
	TerrainElevationFunction terrain_feature;
	terrain_feature.TrapezoidalObstacle(bottom_width, top_width, depth, x0);
	terrain_features_.push_back(terrain_feature);
}

void TerrainCreator::AddRoughness(float rms) {
	TerrainElevationFunction terrain_feature;
	terrain_feature.RoughTerrain(rms);
	terrain_features_.push_back(terrain_feature);
}

void TerrainCreator::AddHole(float x, float y, float depth, float diameter, float steepness) {
	TerrainElevationFunction terrain_feature;
	terrain_feature.HoleTerrain(x, y, depth, diameter, steepness);
	terrain_features_.push_back(terrain_feature);
}

void TerrainCreator::AddParabolic(float square_coeff) {
	TerrainElevationFunction terrain_feature;
	terrain_feature.ParabolicTerrain(square_coeff);
	terrain_features_.push_back(terrain_feature);
}

void TerrainCreator::AddSlope(float fractional_slope) {
	TerrainElevationFunction terrain_feature;
	terrain_feature.SlopedTerrain(fractional_slope);
	terrain_features_.push_back(terrain_feature);
}

/**
* Creates a terrain surface with a user-defined size and shape.
* User provides the boundaries for the lower-left and upper-right corners,
* And a pointer to a terrain geometry class that inherits from the TerrainElevationFuction base class
* \param llx X (easting) coordinate of the lower-left (southwest) coordinate of the terrain in local ENU meters
* \param lly Y (northing) coordinate of the lower-left (southwest) coordinate of the terrain in local ENU meters
* \param urx X (easting) coordinate of the upper-right (northeast) coordinate of the terrain in local ENU meters
* \param ury Y (northing) coordinate of the upper-right (northeast) coordinate of the terrain in local ENU meters
* \param elevation_function Pointer to a derived class of the TerrainElevationFunction base-classs
*/
 void TerrainCreator::CreateTerrain(float llx, float lly, float urx, float ury, float res) {
	mavs::MavsDataPath mdp;
	std::string mavs_data_path = mdp.GetPath();
	mavs::terraingen::HeightMap heightmap;
	int nx = (int)floor((urx - llx) / res) + 1;
	int ny = (int)floor((ury - lly) / res) + 1;
	heightmap.Resize(nx, ny);
	heightmap.SetCorners(llx, lly, urx, ury);
	heightmap.SetResolution(res);
	for (int i = 0; i < nx; i++) {
		float x = llx + i * res;
		for (int j = 0; j < ny; j++) {
			float y = lly + j * res;
			float z = 0.0f;
			for (int k = 0; k < (int)terrain_features_.size(); k++) {
				float znew = terrain_features_[k].GetElevation(x, y);
				z += znew;
			}
			heightmap.SetHeight(i, j, z);
		}
	}
	std::string file_path = mavs_data_path + "/scenes/meshes/";
	
	mavs::raytracer::Mesh surf_mesh = heightmap.GetAsMesh();
	glm::mat3x4 rot_scale = math::GetAffineIdentity();
	scene_.SetLayeredSurfaceMesh(surf_mesh, rot_scale);
	scene_.SetSurfaceMesh(surf_mesh, rot_scale);
	std::string layer_file = file_path + "surface_textures/road_surfaces.json";
	mavs::raytracer::LayeredSurface layers;
	layers.LoadSurfaceTextures(file_path, layer_file);
	scene_.AddLayeredSurface(layers);

	for (int i = 0; i < veg_polys_.size(); i++) {
		std::string mesh_path = mavs_data_path; 
		mesh_path.append("/scenes/meshes/");
		mavs::raytracer::Mesh mesh;
		mesh.Load(mesh_path, veg_polys_[i].meshfile);
		math::Polygon area(veg_polys_[i].polygon);
		for (unsigned j = 0; j < veg_polys_[i].number; j++) {
			glm::vec2 location = area.GetRandomInside();
			float z = scene_.GetSurfaceHeight(location.x, location.y);
			glm::vec3 position(location.x, location.y, z);
			glm::vec3 euler_angles(0.0f, math::rand_in_range(0.0f, (float)k2Pi), 0.0f);
			glm::mat3x4 rot_scale = math::GetRotFromEuler(euler_angles);
			float scale = math::rand_in_range(veg_polys_[i].scale_low, veg_polys_[i].scale_high);
			rot_scale = math::ScaleAffine(rot_scale, scale, scale, scale);
			rot_scale = math::SetAffineOffset(rot_scale, position);
			int mesh_id = scene_.AddMesh(mesh, rot_scale, j, 1);
		}
	}

	scene_.LoadSemanticLabels(file_path + "labels.json");
	scene_.SetLabelsLoaded(true);
	scene_.CommitScene();
	scene_.SetLoaded(true);
	scene_.SetFilePath(file_path);
} // function CreateScene

 TerrainElevationFunction::TerrainElevationFunction() {
	 terrain_type_ = "NONE";
	 terrain_params_[0] = 0.0f;
	 terrain_params_[1] = 0.0f;
	 terrain_params_[2] = 0.0f;
	 terrain_params_[3] = 0.0f;
	 terrain_params_[4] = 0.0f;
	 terrain_params_[5] = 0.0f;
 }

 TerrainElevationFunction::TerrainElevationFunction(const TerrainElevationFunction& s) {
	 this->distribution_ = s.distribution_;
	 this->generator_ = s.generator_;
	 for (int i = 0; i < 6; i++) {
		 this->terrain_params_[i] = s.terrain_params_[i];
	 }
	 this->terrain_type_ = s.terrain_type_;
 }

 TerrainElevationFunction::~TerrainElevationFunction() {
	 //terrain_params_.clear();
 }

float TerrainElevationFunction::GetElevation(float x, float y) {
	float z = 0.0f;
	if (terrain_type_ == "rough") {
		z = distribution_(generator_);
	}
	else if (terrain_type_ == "sloped") {
		z = terrain_params_[0] * x;
	}
	else if (terrain_type_ == "parabolic") {
		z = terrain_params_[0] * x * x;
	}
	else if (terrain_type_ == "hole") {
		float dx = x - terrain_params_[0];
		float dy = y - terrain_params_[1];
		float r = sqrtf(dx * dx + dy * dy);
		z = 0.0f;
		if (r < terrain_params_[3]) {
			z = terrain_params_[4] * powf(1.0f + cosf(terrain_params_[5] * r), terrain_params_[2]);
		}
	}
	else if (terrain_type_ == "trapezoid") {
		float a = terrain_params_[1];
		float b = terrain_params_[0];
		float h = terrain_params_[2];
		float x0 = terrain_params_[3];
		float halfTop = a / 2.0f;
		float halfBottom = b / 2.0f;

		if (x < x0 - halfTop || x > x0 + halfTop) {
			return 0.0f; // Outside the ditch
		}
		else if (x >= x0 - halfBottom && x <= x0 + halfBottom) {
			return -h; // Flat bottom
		}
		else {
			// Sloped sides
			float slope = h / ((a - b) / 2.0f);
			if (x < x0) {
				return -slope * (x - (x0 - halfTop));
			}
			else {
				return -slope * ((x0 + halfTop) - x);
			}
		}
	}
	return z;
}
void TerrainElevationFunction::RoughTerrain(float rms) {
	distribution_ = std::normal_distribution<float>(0.0f, rms);
	std::random_device rd;
	generator_.seed(rd());
	terrain_type_ = "rough";
}

void TerrainElevationFunction::SlopedTerrain(float fractional_slope) {
	terrain_type_ = "sloped";
	terrain_params_[0] = fractional_slope;
}

void TerrainElevationFunction::ParabolicTerrain(float square_coeff) {
	terrain_type_ = "parabolic";
	terrain_params_[0] = square_coeff;
}

void TerrainElevationFunction::HoleTerrain(float x, float y, float depth, float diameter, float steepness) {
	terrain_type_ = "hole";
	terrain_params_[0] = x; // 0
	terrain_params_[1] = y; // 1
	float n = 0.0f;
	if (steepness == 0.0f) {
		n = std::numeric_limits<float>::max();
	}
	else {
		n = 1.0f / fabsf(steepness);
	}
	terrain_params_[2] = n; // 2
	float radius = 0.5f * fabsf(diameter);
	terrain_params_[3] = radius; // 3
	float h = -depth;
	float c = h / powf(2.0f, n);
	terrain_params_[4] = c; // 4
	float b = acosf(-1.0f)/radius;
	terrain_params_[5] = b; // 5
}

void TerrainElevationFunction::TrapezoidalObstacle(float bottom_width, float top_width, float depth, float x_coord) {
	terrain_type_ = "trapezoid";
	float b = bottom_width;
	terrain_params_[0] = b; // 0
	float a = top_width;
	terrain_params_[1] = a; // 1
	float h = depth;
	terrain_params_[2] =h; // 2
	float x0 = x_coord;
	terrain_params_[3] =x0; // 3
}

} // namespace terraingen
} // namespace mavs