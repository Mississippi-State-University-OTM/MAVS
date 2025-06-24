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
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <iostream>
#include <algorithm>
#include <limits>
#include <cstdio>
#include <mavs_core/math/utils.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include <mavs_core/terrain_generator/heightmap.h>

namespace mavs {
namespace terraingen {

HeightMap::HeightMap() {
	ll_corner_ = glm::vec2(-25.0f, -25.0f);
	ur_corner_ = glm::vec2(25.0f, 25.0f);
	resolution_ = 0.25f;
	max_height_ = std::numeric_limits<float>::lowest();
	min_height_ = std::numeric_limits<float>::max();
}

void HeightMap::Resize(int nx, int ny) {
	Resize(nx, ny, 0.0f);
}

void HeightMap::Resize(int nx, int ny, float NODATA) {
	heights_.clear();
	heights_ = mavs::utils::Allocate2DVector(nx, ny, NODATA);
	nx_ = nx;
	ny_ = ny;
}

void HeightMap::CreateFromImage(cimg_library::CImg<float> heightmap, float resolution, float zstep) {
	int nx = heightmap.width();
	int ny = heightmap.height();
	float zmin = -128.0f*zstep;
	Resize(nx, ny);
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			heights_[i][j] = zmin + zstep * heightmap(i, j, 0);
		}
	}
	SetResolution(resolution);
	SetCorners(-0.5f*nx*resolution, -0.5f*ny*resolution, 0.5f*nx*resolution, 0.5f*ny*resolution);
}

void HeightMap::WriteImage(std::string image_name) {
	float height_range = max_height_ - min_height_;
	cimg_library::CImg<float> image;
	image.assign((const unsigned int)GetHorizontalDim(), (const unsigned int)GetVerticalDim(), 1, 1);
	for (int i = 0; i < (int)GetHorizontalDim(); i++) {
		for (int j = 0; j < (int)GetVerticalDim(); j++) {
			image(i, j) = 255.0f*(GetCellHeight(i, j) - min_height_) / height_range;
		}
	}
	image.save(image_name.c_str());
}

void HeightMap::Display(bool trim) {
	float height_range = max_height_ - min_height_;
	cimg_library::CImg<float> image;
	image.assign((const unsigned int)GetHorizontalDim(), (const unsigned int)GetVerticalDim(), 1, 1);
	for (int i = 0; i < (int)GetHorizontalDim(); i++) {
		for (int j = 0; j < (int)GetVerticalDim(); j++) {
			image(i, j) = 255.0f*(GetCellHeight(i, j) - min_height_) / height_range;
		}
	}
	image.normalize(0, 255);
	cimg_library::CImgDisplay display(image);
	if (trim) {
		std::cout << "Select an area from the displayed image" << std::endl;
		int xsize = 10;
		int ysize = 10;
		while (xsize > 0 && ysize > 0) {
			cimg_library::CImg <int> SelectedImageCords = image.get_select(display, 2, 0, 0);
			xsize = SelectedImageCords(3) - SelectedImageCords(0);
			ysize = SelectedImageCords(4) - SelectedImageCords(1);
			if (xsize > 0 && ysize > 0) {
				CropSelf(xsize, ysize, SelectedImageCords(0), SelectedImageCords(1));
				image = image.get_crop(SelectedImageCords(0), SelectedImageCords(1), SelectedImageCords(3), SelectedImageCords(4));
			}
			display.assign(image);
			std::cout << "New size of terrain is " << GetHorizontalDim() << " by " << GetVerticalDim() << " cells." << std::endl;
		}
	}
	else {
		while (!display.is_closed()) {
			display.wait();
		}
	}
}

void HeightMap::CropSelf(int xsize, int ysize, int LLx, int LLy) {
	HeightMap new_heightmap = *this;
	new_heightmap.Resize(xsize, ysize);
	float res = GetResolution();
	glm::vec2 ll_old = GetLLCorner();
	glm::vec2 ll_new(ll_old.x + LLx * res, ll_old.y + LLy * res);
	glm::vec2 ur_new(ll_new.x + xsize * res, ll_new.y + ysize * res);
	new_heightmap.SetCorners(ll_new.x, ll_new.y, ur_new.x, ur_new.y);
	new_heightmap.SetResolution(res);
	int ii = 0;
	for (int i = LLx; i<(LLx + xsize); i++) {
		int jj = 0;
		for (int j = LLy; j<(LLy + ysize); j++) {
			new_heightmap.SetHeight(ii, jj, GetCellHeight(i, j));
			jj++;
		}
		ii++;
	}
	*this = new_heightmap;
}

/// Display the slope of the heightmap
void HeightMap::DisplaySlopes() {
	cimg_library::CImg<float> image;
	image.assign((const unsigned int)GetHorizontalDim(), (const unsigned int)GetVerticalDim(), 1, 1);
	for (int i = 0; i < (int)GetHorizontalDim(); i++) {
		for (int j = 0; j < (int)GetVerticalDim(); j++) {
			glm::vec2 slope = GetSlopeAtCoordinate(i, j);
			image(i, j) = glm::length(slope);
		}
	}
	image.normalize(0, 255);
	cimg_library::CImgDisplay display(image);
	while (!display.is_closed()) {
		display.wait();
	}

}

void HeightMap::Recenter() {
	//First translate the LL corner so that 
	// the heightmap is centered at 0,0
	glm::vec2 current_center = 0.5f*(ur_corner_ + ll_corner_);
	ll_corner_ = ll_corner_ - current_center;
	ur_corner_ = ur_corner_ - current_center;

	int i_center = (int)(0.5f*GetHorizontalDim());
	int j_center = (int)(0.5f*GetHorizontalDim());
	float z_center = heights_[i_center][j_center];
	
	for (int i = 0; i<nx_; i++) {
		for (int j = 0; j<ny_; j++) {
			heights_[i][j] = heights_[i][j]-z_center;
		}
	}
	glm::vec2 new_center = 0.5f*(ur_corner_ - ll_corner_);
}

glm::vec2 HeightMap::GetSlopeAtPoint(glm::vec2 p) {
	int i = (int)((p.x - ll_corner_.x) / resolution_);
	int j = (int)((p.y - ll_corner_.y) / resolution_);
	glm::vec2 slope = GetSlopeAtCoordinate(i, j);
	return slope;
}

glm::vec2 HeightMap::GetSlopeAtCoordinate(int i, int j){
	glm::vec2 slope(0.0f, 0.0f);
	if (i < 0 || i >= nx_ || j < 0 || j >= ny_) {
		return slope;
	}
	int ilo = std::max(i - 1, 0);
	int ihi = std::min(nx_ - 1, i + 1);
	int jlo = std::max(j - 1, 0);
	int jhi = std::min(ny_ - 1, j + 1);
	float dx = (float)(ihi - ilo)*resolution_;
	float dy = (float)(jhi - jlo)*resolution_;
	slope.x = (heights_[ihi][j] - heights_[ilo][j]) / dx;
	slope.y = (heights_[i][jhi] - heights_[i][jlo]) / dy;
	return slope;
}

std::vector< std::vector<float> > HeightMap::GetSlopeMap() {
	//glm::vec2 zero(0.0f, 0.0f);
	std::vector< std::vector<float> > slope = mavs::utils::Allocate2DVector(nx_, ny_, 0.0f);
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			slope[i][j] = glm::length(GetSlopeAtCoordinate(i, j));
		}
	}
	return slope;
}

void HeightMap::CleanHeights(float thresh) {
	float nodata = -9999;
	int num_over = 1;
	float window = 10.0f*resolution_;
	std::vector <std::vector< float> > tmp = heights_;
	int num_loops = 0;
	while (num_over > 0 && num_loops<20) {
		num_over = 0;
		for (int i = 0; i < nx_; i++) {
			for (int j = 0; j < ny_; j++) {
				glm::vec2 slope = GetSlopeAtCoordinate(i, j);
				float smag = glm::length(slope);
				if (smag > thresh) {
					tmp[i][j] = GetLocalAverage(i, j, nodata, window);
					num_over++;
				}
			}
		}
		heights_ = tmp;
		num_loops++;
		std::cout << num_over << std::endl;
	}
}

void HeightMap::Downsample(int ds_factor, float nodata_val) {
	int nx_new = (int)ceil(nx_ / (ds_factor*1.0f));
	int ny_new = (int)ceil(ny_ / (ds_factor*1.0f));
	std::vector <std::vector< float> > new_heights = mavs::utils::Allocate2DVector(nx_new, ny_new, nodata_val);
	int ns = (int)ceil(0.5f*ds_factor);
	for (int i = 0; i < nx_new; i++) {
		for (int j = 0; j < ny_new; j++) {
			int ic = i*ds_factor + ns;
			int jc = j*ds_factor + ns;
			new_heights[i][j] = GetLocalAverage(ic, jc, nodata_val);
		}
	}
	resolution_ = resolution_ * ds_factor;
	nx_ = nx_new;
	ny_ = ny_new;
	heights_ = new_heights;
}

float HeightMap::GetLocalAverage(int i, int j, float nodata_value, float window_size) {
	float weight = 0.0f;
	int n = (int)(window_size / resolution_);
	float avg = 0.0f;
	for (int ii = -n; ii <= n; ii++) {
		for (int jj = -n; jj <= n; jj++) {
			int ix = std::max(0, std::min(nx_ - 1, i + ii));
			int jx = std::max(0, std::min(ny_ - 1, j + jj));
			if (heights_[ix][jx] != nodata_value && ii!=0 && jj!=0) {
				float w = (float)(1.0 / sqrt(float(ii*ii) + float(jj*jj)));
				avg += w * heights_[ix][jx];
				weight += w;
			}
		}
	}
	float h = avg / weight;
	return h;
}

float HeightMap::GetLocalAverage(int i, int j, float nodata_value) {
	float weight = 0.0f;
	float avg = 0.0f;
	int n = 1;
	int nmax = (int)(0.5*std::min(nx_, ny_));
	while (weight == 0.0f && n<nmax) {
		for (int ii = -n; ii <= n; ii++) {
			for (int jj = -n; jj <= n; jj++) {
				int ix = std::max(0, std::min(nx_ - 1, i + ii));
				int jx = std::max(0, std::min(ny_ - 1, j + jj));
				if (heights_[ix][jx] != nodata_value && ii != 0 && jj != 0) {
					float w = (float)(1.0 / sqrt(float(ii*ii) + float(jj*jj)));
					avg += w * heights_[ix][jx];
					weight += w;
				}
			}
		}
		n++;
	}
	float h = avg / weight;
	return h;
}

void HeightMap::ReplaceCellsByValue(float nodata_value) {
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			if (heights_[i][j] == nodata_value) {
				float h = GetLocalAverage(i, j, nodata_value);
				if (std::isnan(h))std::cout << "Found a nan at " << i << " " << j << std::endl;
				heights_[i][j] = h;  
			}
		}
	}
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			if (heights_[i][j] == nodata_value) {
				std::cerr << "Heightmap cell " << i << " " << j << " was not fixed" << std::endl;
			}
		}
	}
}

glm::vec2  HeightMap::IndexToCoordinate(int i, int j) {
	glm::vec2 p;
	p.x = i * resolution_ + ll_corner_.x;
	p.y = j * resolution_ + ll_corner_.y;
	return p;
}

glm::ivec2  HeightMap::CoordinateToIndex(float x, float y) {
	glm::ivec2 idx;
	idx.x = (int)((x - ll_corner_.x) / resolution_);
	idx.y = (int)((y - ll_corner_.y) / resolution_);
	return idx;
}

glm::ivec2  HeightMap::CoordinateToIndex(glm::vec2 v) {
	glm::ivec2 idx;
	idx.x = (int)((v.x - ll_corner_.x) / resolution_);
	idx.y = (int)((v.y - ll_corner_.y) / resolution_);
	return idx;
}

void HeightMap::AddPoint(float x, float y, float z) {
	glm::ivec2 idx = CoordinateToIndex(x, y);
	if (idx.x<nx_ && idx.x >= 0 && idx.y<ny_ && idx.y >= 0) {
		SetHeight(idx.x, idx.y, z);
	}
}

float HeightMap::GetHeightAtPoint(float x, float y) {
	glm::ivec2 idx = CoordinateToIndex(x, y);
	float h = 0.0f;
	if (idx.x<nx_ && idx.x >= 0 && idx.y<ny_ && idx.y >= 0) {
		h = heights_[idx.x][idx.y];
	}
	return h;
}

void HeightMap::SetHeight(int i, int j, float z) {
	if (i < (int)GetHorizontalDim() && i >= 0 && j < (int)GetVerticalDim() && j >= 0) {
		heights_[i][j] = z;
		if (z > max_height_)max_height_ = z;
		if (z < min_height_)min_height_ = z;
	}
}

float HeightMap::GetHeightAtPoint(glm::vec2 p) {
	int ilo = (int)floor((p.x - ll_corner_.x) / resolution_);
	int jlo = (int)floor((p.y - ll_corner_.y) / resolution_);
	int ihi = (int)ceil((p.x - ll_corner_.x) / resolution_);
	int jhi = (int)ceil((p.y - ll_corner_.y) / resolution_);
	ilo = std::min(nx_ - 1, std::max(ilo, 0));
	jlo = std::min(ny_ - 1, std::max(jlo, 0));
	ihi = std::min(nx_ - 1, std::max(ihi, 0));
	jhi = std::min(ny_ - 1, std::max(jhi, 0));
	glm::vec2 xlyl(ll_corner_.x + resolution_ * ilo, ll_corner_.y + resolution_ * jlo );
	glm::vec2 xlyu(ll_corner_.x + resolution_ * ilo, ll_corner_.y + resolution_ * jhi );
	glm::vec2 xuyl(ll_corner_.x + resolution_ * ihi, ll_corner_.y + resolution_ * jlo );
	glm::vec2 xuyu(ll_corner_.x + resolution_ * ihi, ll_corner_.y + resolution_ * jhi );
	float w1 = 1.0f / glm::length(p - xlyl);
	float w2 = 1.0f / glm::length(p - xlyu);
	float w3 = 1.0f / glm::length(p - xuyl);
	float w4 = 1.0f / glm::length(p - xuyu);
	float z = (w1*heights_[ilo][jlo] + w2 * heights_[ilo][jhi] + w3 * heights_[ihi][jlo] + w4 * heights_[ihi][jhi]) / (w1 + w2 + w3 + w4);
	return z;
}

void HeightMap::GetPoseAtPosition(glm::vec2 pos, glm::vec2 lt, glm::vec3 &position, glm::quat &orientation) {
	glm::vec2 slope = GetSlopeAtPoint(pos);
	position = glm::vec3(pos.x, pos.y, (GetHeightAtPoint(pos) + 0.5f));
	glm::vec3 surface_normal(-slope.x, -slope.y, 1.0f);
	surface_normal = glm::normalize(surface_normal);
	glm::vec3 look_up = surface_normal;
	float ltz = (-lt.x*look_up.x - lt.y*look_up.y) / look_up.z;
	glm::vec3 look_to(lt.x, lt.y, ltz);
	look_to = glm::normalize(look_to);
	glm::vec3 look_side = glm::cross(look_up, look_to);
	glm::mat3 R;
	R[0] = look_to;
	R[1] = look_side;
	R[2] = look_up;
	orientation = glm::quat_cast(R);
}

void HeightMap::CreateFromMesh(std::string surface_file) {
#ifdef USE_EMBREE
	mavs::raytracer::Mesh surface_mesh;
	std::string path_to_mesh = mavs::utils::GetPathFromFile(surface_file);
	path_to_mesh.append("/");
	surface_mesh.Load(path_to_mesh, surface_file);
	mavs::raytracer::embree::EmbreeTracer scene;
	glm::mat3x4 unit_rot_scale;
	unit_rot_scale[0][0] = 1.0f; unit_rot_scale[0][1] = 0.0f; unit_rot_scale[0][2] = 0.0f;
	unit_rot_scale[1][0] = 0.0f; unit_rot_scale[1][1] = 1.0f; unit_rot_scale[1][2] = 0.0f;
	unit_rot_scale[2][0] = 0.0f; unit_rot_scale[2][1] = 0.0f; unit_rot_scale[2][2] = 1.0f;
	unit_rot_scale[0][3] = 0.0f; unit_rot_scale[1][3] = 0.0f; unit_rot_scale[2][3] = 0.0f;
	int mesh_id;
	scene.AddMesh(surface_mesh, unit_rot_scale, 0, 0,mesh_id);
	scene.CommitScene();
	glm::vec3 ll = scene.GetLowerLeftCorner();
	glm::vec3 ur = scene.GetUpperRightCorner();
	SetCorners(ll.x, ll.y, ur.x, ur.y);
	glm::vec3 v = ur - ll;
	int nx = (int)std::ceil(v.x / resolution_);
	int ny = (int)std::ceil(v.y / resolution_);
	Resize(nx, ny, ll.z);
	for (int i = 0; i < nx; i++) {
		float x = ll.x + i * resolution_;
		for (int j = 0; j < ny; j++) {
			float y = ll.y + j * resolution_;
			float z = scene.GetSurfaceHeight(x, y);
			if (z < ll.z)z = ll.z;
			SetHeight(i, j, z);
		}
	}
#endif
}

raytracer::Mesh HeightMap::GetAsMesh() {
	raytracer::Mesh mesh;
	std::string meshname = "tmp";
	WriteObj(meshname, true, false);
	mesh.Load("./", "tmp.obj");
	remove("tmp.obj");
	remove("tmp.mtl");
	return mesh;
}

void HeightMap::WriteObj(std::string surfname, bool textured, bool floor) {
	float zmin = std::numeric_limits<float>::max();
	std::vector<glm::vec3> vertices;

	glm::vec2 ll_corner = GetLLCorner();
	glm::vec2 ur_corner = GetURCorner();
	int nx = (int)GetHorizontalDim();
	int ny = (int)GetVerticalDim();
	float resolution = GetResolution();
	float x = ll_corner.x;
	for (int i = 0; i < nx; i++) {
		float y = ll_corner.y;
		for (int j = 0; j < ny; j++) {
			glm::vec3 v(x, y, GetCellHeight(i, j));
			vertices.push_back(v);
			if (GetCellHeight(i, j) < zmin)zmin = GetCellHeight(i, j);
			y = y + resolution;
		}
		x = x + resolution;
	}

	int nv = (int)vertices.size();
	int nf = 2 * (nx - 1)*(ny - 1);
	std::vector<glm::i32vec3> faces;
	for (int i = 0; i < nx - 1; i++) {
		for (int j = 0; j < ny - 1; j++) {
			//add two triangles to each cell
			glm::i32vec3 f0, f1;
			int v1 = i * ny + j + 1;
			int v2 = (i + 1)*ny + j + 1;
			int v3 = (i + 1)*ny + j + 2;
			int v4 = i * ny + j + 2;

			f0.x = v1;
			f0.y = v2;
			f0.z = v4;
			f1.x = v4;
			f1.y = v2;
			f1.z = v3;

			faces.push_back(f0);
			faces.push_back(f1);
		}
	}

	std::string fname = surfname; // ("surface.obj");
	fname.append(".obj");
	std::ofstream fout(fname);
	fout << "# surface generated with two decades of perlin noise" << std::endl;
	fout << "# units are meters" << std::endl;
	fout << "# scene corners are [" << ll_corner.x << "," << ll_corner.y << "], [" << ur_corner.x << "," << ur_corner.y << "]"
		<< std::endl;
	fout << "# resolution = " << resolution << std::endl << std::endl;
	fout << "mtllib " << surfname << ".mtl" << std::endl << std::endl;
	for (int i = 0; i < (int)vertices.size(); i++) {
		fout << "v " << vertices[i].x << " " << vertices[i].y << " " <<
			vertices[i].z << std::endl;
	}

	glm::vec2 surf_size = ur_corner - ll_corner;
	if (textured) {
		for (int i = 0; i < (int)vertices.size(); i++) {
			float dx = (vertices[i].x - ll_corner.x) / surf_size.x;
			float dy = (vertices[i].y - ll_corner.y) / surf_size.y;
			fout << "vt " << dx << " " << dy << std::endl;
		}
	}
	fout << std::endl << "usemtl " << surfname << std::endl << std::endl;

	for (int i = 0; i < (int)faces.size(); i++) {
		if (textured) {
			fout << "f " << faces[i].x << "/" << faces[i].x << " " << faces[i].y << "/" << faces[i].y << " " <<
				faces[i].z << "/" << faces[i].z << std::endl;
		}
		else {
			fout << "f " << faces[i].x << " " << faces[i].y << " " <<
				faces[i].z << std::endl;
		}
	}

	if (floor) {
		//add a floor that extends to the horizon
		float floor_size = 100000.0;
		fout << "v " << -floor_size << " " << -floor_size << " " << zmin << std::endl;
		fout << "v " << -10000.0 << " " << floor_size << " " << zmin << std::endl;
		fout << "v " << floor_size << " " << floor_size << " " << zmin << std::endl;
		fout << "v " << floor_size << " " << -floor_size << " " << zmin << std::endl;
		if (textured) {
			fout << "vt " << ((-floor_size - ll_corner.x) / surf_size.x) << " " << ((-floor_size - ll_corner.y) / surf_size.y) << std::endl;
			fout << "vt " << ((-floor_size - ll_corner.x) / surf_size.x) << " " << ((floor_size - ll_corner.y) / surf_size.y) << std::endl;
			fout << "vt " << ((floor_size - ll_corner.x) / surf_size.x) << " " << ((floor_size - ll_corner.y) / surf_size.y) << std::endl;
			fout << "vt " << ((floor_size - ll_corner.x) / surf_size.x) << " " << ((-floor_size - ll_corner.y) / surf_size.y) << std::endl;
		}
		if (textured) {
			fout << "f " << vertices.size() + 1 << "/" << vertices.size() + 1 << " " << vertices.size() + 2 << "/" << vertices.size() + 2 << " " << vertices.size() + 3 << "/" << vertices.size() + 3 << std::endl;
			fout << "f " << vertices.size() + 3 << "/" << vertices.size() + 3 << " " << vertices.size() + 4 << "/" << vertices.size() + 4 << " " << vertices.size() + 1 << "/" << vertices.size() + 1 << std::endl;
		}
		else {
			fout << "f " << vertices.size() + 1 << " " << vertices.size() + 2 << " " << vertices.size() + 3 << std::endl;
			fout << "f " << vertices.size() + 3 << " " << vertices.size() + 4 << " " << vertices.size() + 1 << std::endl;
		}
	}

	fout.close();

	std::string matname = surfname;
	matname.append(".mtl");
	std::ofstream mout(matname);
	mout << "newmtl " << surfname << std::endl;
	mout << "	 Ns 1.0000" << std::endl;
	mout << "	 Ni -1.5000" << std::endl;
	mout << "	 d 1.0000" << std::endl;
	mout << "	 Tr 0.0000" << std::endl;
	mout << "	 Tf 1.0000 1.0000 1.0000" << std::endl;
	mout << "	 illum 2" << std::endl;
	mout << "	 Ka 0.5 0.5 0.5" << std::endl;
	mout << "	 Kd 0.5 0.5 0.5" << std::endl;
	mout << "	 Kd spectral spectra/gravelly_loam.spec" << std::endl;
	mout << "	 Ks 0.0000 0.0000 0.0000" << std::endl;
	mout << "	 Ke 0.0000 0.0000 0.0000" << std::endl;
	mout.close();
}

} //nampespace terraingen
} //namespace heightmap