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
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif
#include <mavs_core/terrain_generator/heightmap.h>

namespace mavs {
namespace terraingen {

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
 void TerrainElevationFunction::CreateTerrain(float llx, float lly, float urx, float ury, float res) {
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
			float z = GetElevation(x, y);
			heightmap.SetHeight(i, j, z);
		}
	}
	std::string file_path = mavs_data_path + "/scenes/meshes/";
	mavs::raytracer::Mesh surf_mesh = heightmap.GetAsMesh();
	//mavs::raytracer::embree::EmbreeTracer scene;
	glm::mat3x4 rot_scale = scene_.GetAffineIdentity();

	scene_.SetLayeredSurfaceMesh(surf_mesh, rot_scale);
	scene_.SetSurfaceMesh(surf_mesh, rot_scale);
	std::string layer_file = file_path + "surface_textures/road_surfaces.json";
	mavs::raytracer::LayeredSurface layers;
	layers.LoadSurfaceTextures(file_path, layer_file);
	scene_.AddLayeredSurface(layers);
	scene_.LoadSemanticLabels(file_path + "labels.json");
	scene_.SetLabelsLoaded(true);
	scene_.CommitScene();
	scene_.SetLoaded(true);
	scene_.SetFilePath(file_path);
} // function CreateScene

RoughTerrain::RoughTerrain(float rms) : distribution_(0.0f, rms) {
	std::random_device rd;
	generator_.seed(rd());
}

float RoughTerrain::GetElevation(float x, float y) {
	float z = distribution_(generator_);
	return z;
}

SlopedTerrain::SlopedTerrain(float fractional_slope) {
	m_ = fractional_slope;
}

float SlopedTerrain::GetElevation(float x, float y) {
	float z = m_ * x;
	return z;
}

ParabolicTerrain::ParabolicTerrain(float square_coeff) {
	a_ = square_coeff;
}
float ParabolicTerrain::GetElevation(float x, float y) {
	float z = a_ * x * x;
	return z;
}

HoleTerrain::HoleTerrain(float x, float y, float depth, float diameter, float steepness) {
	x0_ = x;
	y0_ = y; 
	if (steepness == 0.0f) {
		n_ = std::numeric_limits<float>::max();
	}
	else {
		n_ = 1.0f / fabsf(steepness);
	}
	radius_ = 0.5f * fabsf(diameter);
	h_ = -depth;
	c_ = h_ / powf(2.0f, n_);
	b_ = acosf(-1.0f)/radius_;
}
float HoleTerrain::GetElevation(float x, float y) {
	float dx = x - x0_;
	float dy = y - y0_;
	float r = sqrtf(dx * dx + dy * dy);
	float z = 0.0f;
	if (r < radius_) {
		z = c_ * powf(1.0f + cosf(b_ * r), n_);
	}
	return z;
}

TrapezoidalObstacle::TrapezoidalObstacle(float bottom_width, float top_width, float depth, float x_coord) {
	b_ = bottom_width;
	a_ = top_width;
	h_ = depth;
	x0_ = x_coord;
}

// Function to define the trapezoidal ditch profile
float TrapezoidalObstacle::GetElevation(float x, float y) {
	float halfTop = a_ / 2.0f;
	float halfBottom = b_ / 2.0f;

	if (x < x0_ - halfTop || x > x0_ + halfTop) {
		return 0.0f; // Outside the ditch
	}
	else if (x >= x0_ - halfBottom && x <= x0_ + halfBottom) {
		return -h_; // Flat bottom
	}
	else {
		// Sloped sides
		float slope = h_ / ((a_ - b_) / 2.0f);
		if (x < x0_) {
			return -slope * (x - (x0_ - halfTop));
		}
		else {
			return -slope * ((x0_ + halfTop) - x);
		}
	}
}

} // namespace terraingen
} // namespace mavs