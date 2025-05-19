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
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
//#include <glm/gtx/intersect.hpp>
#include <mavs_core/math/utils.h>
#include <raytracers/mesh.h>
#include <iostream>
#include <limits>
#include <algorithm>
#include <mavs_core/data_path.h>
#include <glm/gtx/intersect.hpp>

namespace mavs {
namespace raytracer {

Mesh::Mesh() {
	float max = std::numeric_limits<float>::max();
	float min = std::numeric_limits<float>::lowest();
	max_corner_.x = min;
	min_corner_.x = max;
	max_corner_.y = min;
	min_corner_.y = max;
	max_corner_.z = min;
	min_corner_.z = max;
	size_calculated_ = false;
	label_by_group_ = false;
	orientation_ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
	velocity_ = glm::vec3(0.0f, 0.0f, 0.0f);
}

Mesh::~Mesh() {

}

void Mesh::WriteBinary(std::string inpath, std::string binary_out) {
	// see: 
	// https://stackoverflow.com/questions/37038909/c-read-write-class-from-to-binary-file
	std::ofstream fout(binary_out.c_str(), std::ios::out | std::ios::binary);
	
	size_t size;
	// 1. Write out the mesh name
	size = mesh_name_.size();
	fout.write((char*)&size, sizeof(size_t));
	fout.write((char*)mesh_name_.c_str(), size);

	// 2. Write out the mesh center, min_corner, max_corner, and orientation
	/*if (!size_calculated_)glm::vec3 meshdims = GetSize();
	fout.write((char*)&center_.x, sizeof(center_.x));
	fout.write((char*)&center_.y, sizeof(center_.y));
	fout.write((char*)&center_.z, sizeof(center_.z));
	fout.write((char*)&min_corner_.x, sizeof(min_corner_.x));
	fout.write((char*)&min_corner_.y, sizeof(min_corner_.y));
	fout.write((char*)&min_corner_.z, sizeof(min_corner_.z));
	fout.write((char*)&max_corner_.x, sizeof(max_corner_.x));
	fout.write((char*)&max_corner_.y, sizeof(max_corner_.y));
	fout.write((char*)&max_corner_.z, sizeof(max_corner_.z));
	fout.write((char*)&orientation_.w, sizeof(orientation_.w));
	fout.write((char*)&orientation_.x, sizeof(orientation_.x));
	fout.write((char*)&orientation_.y, sizeof(orientation_.y));
	fout.write((char*)&orientation_.z, sizeof(orientation_.z));*/

	// 3. Write out the boolean flags
	fout.write((char*)&size_calculated_, sizeof(size_calculated_));
	fout.write((char*)&label_by_group_, sizeof(label_by_group_));
	
	// 4. Write out the material numbers
	size_t num_mat_ind = mat_nums_.size();
	fout.write((char*)&num_mat_ind, sizeof(size_t));
	if (num_mat_ind > 0) {
		fout.write((char*)&mat_nums_[0], num_mat_ind * sizeof(mat_nums_[0]));
	}
	
	// 5. Write out the vertices
	size_t num_verts = vertices_.size();
	fout.write((char*)&num_verts, sizeof(size_t));
	if (num_verts > 0) {
		fout.write((char*)&vertices_[0], num_verts * sizeof(glm::vec3));
	}
	
	// 6. Write out the normals
	if (normals_.size() <= 0)CalculateNormals();
	size_t num_norms = normals_.size();
	fout.write((char*)&num_norms, sizeof(size_t));
	if (num_norms > 0) {
		fout.write((char*)&normals_[0], num_norms*sizeof(glm::vec3));
	}
	
	// 7. Write out the texture coordinates
	size_t num_texc = texcoords_.size();
	fout.write((char*)&num_texc, sizeof(size_t));
	if (num_texc > 0) {
		fout.write((char*)&texcoords_[0], num_texc * sizeof(glm::vec2));
	}
	
	// 8. Write out the faces
	size_t num_faces = faces_.size();
	fout.write((char*)&num_faces, sizeof(size_t));
	if (num_faces > 0) {
		fout.write((char*)&faces_[0], num_faces*sizeof(Face));
	}

	// 9. Write out the materials
	size_t num_mats = materials_.size();
	fout.write((char*)&num_mats, sizeof(size_t));
	if (num_mats > 0) {
		for (int i = 0; i < num_mats; i++) {
			fout.write((char*)&materials_[i].ns, sizeof(materials_[i].ns));
			fout.write((char*)&materials_[i].ni, sizeof(materials_[i].ni));
			fout.write((char*)&materials_[i].dissolve, sizeof(materials_[i].dissolve));
			fout.write((char*)&materials_[i].illum, sizeof(materials_[i].illum));
			fout.write((char*)&materials_[i].ka.x, sizeof(materials_[i].ka.x));
			fout.write((char*)&materials_[i].ka.y, sizeof(materials_[i].ka.y));
			fout.write((char*)&materials_[i].ka.z, sizeof(materials_[i].ka.z));
			fout.write((char*)&materials_[i].kd.x, sizeof(materials_[i].kd.x));
			fout.write((char*)&materials_[i].kd.y, sizeof(materials_[i].kd.y));
			fout.write((char*)&materials_[i].kd.z, sizeof(materials_[i].kd.z));
			fout.write((char*)&materials_[i].ks.x, sizeof(materials_[i].ka.x));
			fout.write((char*)&materials_[i].ks.y, sizeof(materials_[i].ks.y));
			fout.write((char*)&materials_[i].ks.z, sizeof(materials_[i].ks.z));
			fout.write((char*)&materials_[i].tr.x, sizeof(materials_[i].tr.x));
			fout.write((char*)&materials_[i].tr.y, sizeof(materials_[i].tr.y));
			fout.write((char*)&materials_[i].tr.z, sizeof(materials_[i].tr.z));
			fout.write((char*)&materials_[i].ke.x, sizeof(materials_[i].ke.x));
			fout.write((char*)&materials_[i].ke.y, sizeof(materials_[i].ke.y));
			fout.write((char*)&materials_[i].ke.z, sizeof(materials_[i].ke.z));
			size_t char_len = materials_[i].name.size();
			fout.write((char*)&char_len, sizeof(size_t));
			fout.write((char*)materials_[i].name.c_str(), char_len);
			materials_[i].map_kd.erase(0, inpath.size());
			char_len = materials_[i].map_kd.size();
			fout.write((char*)&char_len, sizeof(size_t));
			fout.write((char*)materials_[i].map_kd.c_str(), char_len);
			materials_[i].map_ka.erase(0, inpath.size());
			char_len = materials_[i].map_ka.size();
			fout.write((char*)&char_len, sizeof(size_t));
			fout.write((char*)materials_[i].map_ka.c_str(), char_len);
			materials_[i].map_ks.erase(0, inpath.size());
			char_len = materials_[i].map_ks.size();
			fout.write((char*)&char_len, sizeof(size_t));
			fout.write((char*)materials_[i].map_ks.c_str(), char_len);
			materials_[i].map_ns.erase(0, inpath.size());
			char_len = materials_[i].map_ns.size();
			fout.write((char*)&char_len, sizeof(size_t));
			fout.write((char*)materials_[i].map_ns.c_str(), char_len);
			materials_[i].map_bump.erase(0, inpath.size());
			char_len = materials_[i].map_bump.size();
			fout.write((char*)&char_len, sizeof(size_t));
			fout.write((char*)materials_[i].map_bump.c_str(), char_len);
			materials_[i].map_d.erase(0, inpath.size());
			char_len = materials_[i].map_d.size();
			fout.write((char*)&char_len, sizeof(size_t));
			fout.write((char*)materials_[i].map_d.c_str(), char_len);
			materials_[i].disp.erase(0, inpath.size());
			char_len = materials_[i].disp.size();
			fout.write((char*)&char_len, sizeof(size_t));
			fout.write((char*)materials_[i].disp.c_str(), char_len);
			materials_[i].refl.erase(0, inpath.size());
			char_len = materials_[i].refl.size();
			fout.write((char*)&char_len, sizeof(size_t));
			fout.write((char*)materials_[i].refl.c_str(), char_len);
		}
	}
	
	fout.close();
}

void Mesh::ReadBinary(std::string inpath, std::string binary_in) {
	std::ifstream fin(binary_in.c_str(), std::ios::in | std::ios::binary);
	
	size_t size;
	char *data;

	// 1. Read in the mesh name
	fin.read((char*)&size, sizeof(size));
	data = new char[size + 1];
	fin.read(data, size);
	data[size] = '\0';
	mesh_name_ = data;
	delete data;

	// 2. Read in the mesh center and corners
	/*fin.read((char*)&center_.x, sizeof(center_.x));
	fin.read((char*)&center_.y, sizeof(center_.y));
	fin.read((char*)&center_.z, sizeof(center_.z));
	fin.read((char*)&min_corner_.x, sizeof(min_corner_.x));
	fin.read((char*)&min_corner_.y, sizeof(min_corner_.y));
	fin.read((char*)&min_corner_.z, sizeof(min_corner_.z));
	fin.read((char*)&max_corner_.x, sizeof(max_corner_.x));
	fin.read((char*)&max_corner_.y, sizeof(max_corner_.y));
	fin.read((char*)&max_corner_.z, sizeof(max_corner_.z));
	fin.read((char*)&orientation_.w, sizeof(orientation_.w));
	fin.read((char*)&orientation_.x, sizeof(orientation_.x));
	fin.read((char*)&orientation_.y, sizeof(orientation_.y));
	fin.read((char*)&orientation_.z, sizeof(orientation_.z));*/

	// 3. Read in the boolean flags
	fin.read((char*)&size_calculated_, sizeof(size_calculated_));
	fin.read((char*)&label_by_group_, sizeof(label_by_group_));
	
	// 4. Read in the material number index
	size_t num_mat_ind;
	fin.read((char*)&num_mat_ind, sizeof(size_t));
	if (num_mat_ind > 0) {
		mat_nums_.resize(num_mat_ind, 0);
		fin.read((char*)&mat_nums_[0], num_mat_ind * sizeof(mat_nums_[0]));
	}

	// 5. Read in the vertices
	size_t num_verts;
	fin.read((char*)&num_verts, sizeof(size_t));
	if (num_verts > 0) {
		vertices_.resize(num_verts, glm::vec3(0.0f, 0.0f, 0.0f));
		fin.read((char*)&vertices_[0], num_verts*sizeof(glm::vec3));
	}
	
	// 6. Read in the normals
	size_t num_norms;
	fin.read((char*)&num_norms, sizeof(size_t));
	if (num_norms > 0) {
		normals_.resize(num_norms, glm::vec3(0.0f, 0.0f, 1.0f));
		fin.read((char*)&normals_[0], num_norms * sizeof(glm::vec3));
	}

	// 7.Read in the texture coordinates
	size_t num_texc;
	fin.read((char*)&num_texc, sizeof(size_t));
	if (num_texc > 0) {
		texcoords_.resize(num_texc, glm::vec2(0.0f, 0.0f));
		fin.read((char*)&texcoords_[0], num_texc * sizeof(glm::vec2));
	}
	// 8. Read in the faces
	size_t num_faces;
	fin.read((char*)&num_faces, sizeof(size_t));
	if (num_faces > 0) {
		faces_.resize(num_faces);
		fin.read((char*)&faces_[0], num_faces * sizeof(Face));
	}
	
	// 9. Write out the materials
	size_t num_mats;
	fin.read((char*)&num_mats, sizeof(size_t));
	if (num_mats > 0) {
		materials_.resize(num_mats);
		for (int i = 0; i < num_mats; i++) {
			fin.read((char*)&materials_[i].ns, sizeof(materials_[i].ns));
			fin.read((char*)&materials_[i].ni, sizeof(materials_[i].ni));
			fin.read((char*)&materials_[i].dissolve, sizeof(materials_[i].dissolve));
			fin.read((char*)&materials_[i].illum, sizeof(materials_[i].illum));
			fin.read((char*)&materials_[i].ka.x, sizeof(materials_[i].ka.x));
			fin.read((char*)&materials_[i].ka.y, sizeof(materials_[i].ka.y));
			fin.read((char*)&materials_[i].ka.z, sizeof(materials_[i].ka.z));
			fin.read((char*)&materials_[i].kd.x, sizeof(materials_[i].kd.x));
			fin.read((char*)&materials_[i].kd.y, sizeof(materials_[i].kd.y));
			fin.read((char*)&materials_[i].kd.z, sizeof(materials_[i].kd.z));
			fin.read((char*)&materials_[i].ks.x, sizeof(materials_[i].ka.x));
			fin.read((char*)&materials_[i].ks.y, sizeof(materials_[i].ks.y));
			fin.read((char*)&materials_[i].ks.z, sizeof(materials_[i].ks.z));
			fin.read((char*)&materials_[i].tr.x, sizeof(materials_[i].tr.x));
			fin.read((char*)&materials_[i].tr.y, sizeof(materials_[i].tr.y));
			fin.read((char*)&materials_[i].tr.z, sizeof(materials_[i].tr.z));
			fin.read((char*)&materials_[i].ke.x, sizeof(materials_[i].ke.x));
			fin.read((char*)&materials_[i].ke.y, sizeof(materials_[i].ke.y));
			fin.read((char*)&materials_[i].ke.z, sizeof(materials_[i].ke.z));
			fin.read((char*)&size, sizeof(size));
			data = new char[size + 1];
			fin.read(data, size);
			data[size] = '\0';
			materials_[i].name = data;
			delete data;

			fin.read((char*)&size, sizeof(size));
			if (size > 0) {
				data = new char[size + 1];
				fin.read(data, size);
				data[size] = '\0';
				materials_[i].map_kd = inpath;
				materials_[i].map_kd.append(data);
				delete data;
			}
			fin.read((char*)&size, sizeof(size));
			if (size > 0) {
				data = new char[size + 1];
				fin.read(data, size);
				data[size] = '\0';
				materials_[i].map_ka = inpath;
				materials_[i].map_ka.append(data);
				delete data;
			}
			fin.read((char*)&size, sizeof(size));
			if (size > 0) {
				data = new char[size + 1];
				fin.read(data, size);
				data[size] = '\0';
				materials_[i].map_ks = inpath;
				materials_[i].map_ks.append(data);
				delete data;
			}
			fin.read((char*)&size, sizeof(size));
			if (size > 0) {
				data = new char[size + 1];
				fin.read(data, size);
				data[size] = '\0';
				materials_[i].map_ns = inpath;
				materials_[i].map_ns.append(data);
				delete data;
			}
			fin.read((char*)&size, sizeof(size));
			if (size > 0) {
				data = new char[size + 1];
				fin.read(data, size);
				data[size] = '\0';
				materials_[i].map_bump = inpath;
				materials_[i].map_bump.append(data);
				delete data;
			}
			fin.read((char*)&size, sizeof(size));
			if (size > 0) {
				data = new char[size + 1];
				fin.read(data, size);
				data[size] = '\0';
				materials_[i].map_d = inpath;
				materials_[i].map_d.append(data);
				delete data;
			}
			fin.read((char*)&size, sizeof(size));
			if (size > 0) {
				data = new char[size + 1];
				fin.read(data, size);
				data[size] = '\0';
				materials_[i].disp = inpath;
				materials_[i].disp.append(data);
				delete data;
			}
			fin.read((char*)&size, sizeof(size));
			if (size > 0) {
				data = new char[size + 1];
				fin.read(data, size);
				data[size] = '\0';
				materials_[i].refl = inpath;
				materials_[i].refl.append(data);
				delete data;
			}
		}
	}
	fin.close();
	size_calculated_ = false;
}

void Mesh::LoadTransformAndSave(std::string infile_name, std::string outfile_name, glm::mat3x4 aff_rot) {
	std::ifstream infile(infile_name.c_str());
	std::ofstream outfile(outfile_name.c_str());
	std::string line;
	glm::vec3 trans(aff_rot[0][3], aff_rot[1][3], aff_rot[2][3]);
	glm::mat3 R;
	for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) { R[i][j] = aff_rot[i][j]; } }
	for (int line_num = 1; getline(infile, line); line_num++){
		std::stringstream ss(line);
		std::string word;
		ss >> word;
		if (word == "v") {
			glm::vec3 vertex;
			for (int word_num = 2; ss >> word; word_num++) {
				vertex[word_num - 2] = (float)mavs::utils::StringToDouble(word);
			}
			glm::vec3 v_prime = R * vertex + trans;
			outfile << "v " << v_prime.x << " " << v_prime.y << " " << v_prime.z << std::endl;
		}
		else {
			outfile << line << std::endl;
		}
	}
	infile.close();
	outfile.close();
}

void Mesh::Write(std::string ofname) {
	std::ofstream fout;
	std::string fname = ofname;
	fname.append(".obj");
	fout.open(fname.c_str());
	fout << "#" << ofname << std::endl;
	fout << "mtllib " << ofname << ".mtl" << std::endl;
	fout << std::endl;
	for (int i = 0; i < (int)vertices_.size(); i++) {
		fout << "v " << vertices_[i].x << " " << vertices_[i].y << " " << vertices_[i].z
			<< std::endl;
	}

	for (int i = 0; i < (int)normals_.size(); i++) {
		fout << "vn " << normals_[i].x << " " << normals_[i].y << " " << normals_[i].z <<
			std::endl;
	}

	for (int i = 0; i < (int)texcoords_.size(); i++) {
		fout << "vt " << texcoords_[i].x << " " << texcoords_[i].y << std::endl;
	}

	fout << "usemtl " << ofname << std::endl;
	for (int i = 0; i < (int)faces_.size(); i++) {
		fout << "f " <<
			faces_[i].v1 + 1 << "/" << faces_[i].t1 + 1 << "/" << faces_[i].n1 + 1 << " " <<
			faces_[i].v2 + 1 << "/" << faces_[i].t2 + 1 << "/" << faces_[i].n2 + 1 << " " <<
			faces_[i].v3 + 1 << "/" << faces_[i].t3 + 1 << "/" << faces_[i].n3 + 1 << std::endl;
	}
	fout.close();
}

glm::vec3 Mesh::GetSize() {
	if (!size_calculated_) {
		float max = std::numeric_limits<float>::max();
		float min = std::numeric_limits<float>::lowest();
		max_corner_.x = min;
		min_corner_.x = max;
		max_corner_.y = min;
		min_corner_.y = max;
		max_corner_.z = min;
		min_corner_.z = max;
		for (int i = 0; i < (int)vertices_.size(); i++) {
			if (vertices_[i].x > max_corner_.x) max_corner_.x = vertices_[i].x;
			if (vertices_[i].x < min_corner_.x) min_corner_.x = vertices_[i].x;
			if (vertices_[i].y > max_corner_.y) max_corner_.y = vertices_[i].y;
			if (vertices_[i].y < min_corner_.y) min_corner_.y = vertices_[i].y;
			if (vertices_[i].z > max_corner_.z) max_corner_.z = vertices_[i].z;
			if (vertices_[i].z < min_corner_.z) min_corner_.z = vertices_[i].z;
		}
		center_ = 0.5f*(max_corner_ + min_corner_);
		size_calculated_ = true;
	}
	glm::vec3 size = max_corner_ - min_corner_;
	return size;
}

void Mesh::Translate(glm::vec3 offset) {
	for (int i = 0; i < (int)vertices_.size(); i++) {
		vertices_[i] = vertices_[i] + offset;
	}
	center_ += offset;
	min_corner_ += offset;
	max_corner_ += offset;
}

void Mesh::Scale(float sx, float sy, float sz) {
	for (int i = 0; i < (int)vertices_.size(); i++) {
		glm::vec3 v = vertices_[i] - center_;
		vertices_[i].x = sx * v.x + center_.x;
		vertices_[i].y = sy * v.y + center_.y;
		vertices_[i].z = sz * v.z + center_.z;
	}
	size_calculated_ = false;
	glm::vec3 size = GetSize();
}

void Mesh::MirrorX() {
	float x_center = 0.0f;
	float nxv = 0.0f;
	for (int i = 0; i < (int)vertices_.size(); i++) {
		x_center += vertices_[i].x;
		nxv += 1.0f;
	}
	x_center = x_center / nxv;
	for (int i = 0; i < (int)vertices_.size(); i++) {
		float dx = vertices_[i].x - x_center;
		vertices_[i].x = x_center - dx;
	}
	size_calculated_ = false;
	glm::vec3 size = GetSize();
}

void Mesh::MirrorY() {
	float y_center = 0.0f;
	float nyv = 0.0f;
	for (int i = 0; i < (int)vertices_.size(); i++) {
		y_center += vertices_[i].y;
		nyv += 1.0f;
	}
	y_center = y_center / nyv;
	for (int i = 0; i < (int)vertices_.size(); i++) {
		float dy = vertices_[i].y - y_center;
		vertices_[i].y = y_center - dy;
	}
	size_calculated_ = false;
	glm::vec3 size = GetSize();
}

void Mesh::RotateYToZ() {
	for (int i = 0; i < (int)vertices_.size(); i++) {
		float zold = vertices_[i].z;
		vertices_[i].z = vertices_[i].y;
		vertices_[i].y = -zold;
	}
	size_calculated_ = false;
	glm::vec3 size = GetSize();
}

void Mesh::RotateZToY() {
	for (int i = 0; i < (int)vertices_.size(); i++) {
		float zold = vertices_[i].z;
		vertices_[i].z = -vertices_[i].y;
		vertices_[i].y = zold;
	}
	size_calculated_ = false;
	glm::vec3 size = GetSize();
}

void Mesh::RotateXToY() {
	for (int i = 0; i < (int)vertices_.size(); i++) {
		float yold = vertices_[i].y;
		vertices_[i].y = vertices_[i].x;
		vertices_[i].x = -yold;
	}
	size_calculated_ = false;
	glm::vec3 size = GetSize();
}

void Mesh::RotateYToX() {
	for (int i = 0; i < (int)vertices_.size(); i++) {
		float xold = vertices_[i].x;
		vertices_[i].x = vertices_[i].y;
		vertices_[i].y = -xold;
	}
	size_calculated_ = false;
	glm::vec3 size = GetSize();
}

glm::vec3 Mesh::GetCenter() {
	GetSize();
	return center_;
}

void Mesh::Load(std::string inpath, std::string fname) {
	std::string mtllib_filename = "";
	Load(inpath, fname, mtllib_filename);
}

void Mesh::Load(std::string inpath, std::string fname, std::string mtllib_filename) {
	mesh_name_ = fname;
	mesh_name_.erase(0, inpath.size());

	if (mesh_name_[0] == '/' || mesh_name_[0] == '\\'){
		mesh_name_.erase(0, 1);
	}
	std::string bin_name = fname;
	for (int i = 0; i<3; i++)bin_name.pop_back();
	bin_name.append("bin");
	if (utils::file_exists(bin_name)) {
		ReadBinary(inpath,bin_name);
	}
	else {
		const char* filename = fname.c_str();
		inpath = mavs::utils::GetPathFromFile(fname);
		inpath.append("/");
		const char* basepath = inpath.c_str();
		bool triangulate = true;

		tinyobj::attrib_t attrib;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> tobj_materials;

		std::string err;

		bool ret = tinyobj::LoadObj(&attrib, &shapes, &tobj_materials, &err,
			filename, basepath, triangulate, mtllib_filename);
		//shapes are the same as groups.

		if (ret == false) {
			std::cerr << "ERROR: mesh.cpp could not load mesh file" << filename << std::endl;
		}

		for (size_t i = 0; i < shapes.size(); i++) {
			if (shapes[i].name == "")shapes[i].name = mesh_name_;
			assert((shapes[i].mesh.indices.size() % 3) == 0);
			for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++) {
				tinyobj::index_t i0 = shapes[i].mesh.indices[3 * f + 0];
				tinyobj::index_t i1 = shapes[i].mesh.indices[3 * f + 1];
				tinyobj::index_t i2 = shapes[i].mesh.indices[3 * f + 2];
				Face this_face;
				this_face.v1 = i0.vertex_index;
				this_face.v2 = i1.vertex_index;
				this_face.v3 = i2.vertex_index;
				this_face.n1 = i0.normal_index;
				this_face.n2 = i1.normal_index;
				this_face.n3 = i2.normal_index;
				this_face.t1 = i0.texcoord_index;
				this_face.t2 = i1.texcoord_index;
				this_face.t3 = i2.texcoord_index;
				//std::copy(shapes[i].name.begin(), shapes[i].name.end(), this_face.group);
				this_face.group = shapes[i].name;
				faces_.push_back(this_face);
				if (tobj_materials.size() > 0) {
					mat_nums_.push_back((unsigned short int)shapes[i].mesh.material_ids[f]);
				}
				else {
					mat_nums_.push_back(0);
				}
			}
		}

		for (size_t v = 0; v < attrib.vertices.size() / 3; v++) {
			glm::vec3 this_vertex;
			this_vertex.x = static_cast<const float>(attrib.vertices[3 * v + 0]);
			this_vertex.y = static_cast<const float>(attrib.vertices[3 * v + 1]);
			this_vertex.z = static_cast<const float>(attrib.vertices[3 * v + 2]);
			if (this_vertex.x > max_corner_.x)max_corner_.x = this_vertex.x;
			if (this_vertex.y > max_corner_.y)max_corner_.y = this_vertex.y;
			if (this_vertex.z > max_corner_.z)max_corner_.z = this_vertex.z;
			if (this_vertex.x < min_corner_.x)min_corner_.x = this_vertex.x;
			if (this_vertex.y < min_corner_.y)min_corner_.y = this_vertex.y;
			if (this_vertex.z < min_corner_.z)min_corner_.z = this_vertex.z;
			vertices_.push_back(this_vertex);
		}

		for (size_t v = 0; v < attrib.normals.size() / 3; v++) {
			glm::vec3 this_normal;
			this_normal.x = static_cast<const float>(attrib.normals[3 * v + 0]);
			this_normal.y = static_cast<const float>(attrib.normals[3 * v + 1]);
			this_normal.z = static_cast<const float>(attrib.normals[3 * v + 2]);
			normals_.push_back(this_normal);
		}

		for (size_t v = 0; v < attrib.texcoords.size() / 2; v++) {
			glm::vec2 this_tex;
			this_tex.x = static_cast<const float>(attrib.texcoords[2 * v + 0]);
			this_tex.y = static_cast<const float>(attrib.texcoords[2 * v + 1]);
			texcoords_.push_back(this_tex);
		}

		if (tobj_materials.size() > 0) {
			for (size_t i = 0; i < tobj_materials.size(); i++) {
				Material this_material;
				this_material.name = tobj_materials[i].name;
				this_material.ka.x = static_cast<const float>(tobj_materials[i].ambient[0]);
				this_material.ka.y = static_cast<const float>(tobj_materials[i].ambient[1]);
				this_material.ka.z = static_cast<const float>(tobj_materials[i].ambient[2]);
				this_material.kd.x = static_cast<const float>(tobj_materials[i].diffuse[0]);
				this_material.kd.y = static_cast<const float>(tobj_materials[i].diffuse[1]);
				this_material.kd.z = static_cast<const float>(tobj_materials[i].diffuse[2]);
				this_material.ks.x = static_cast<const float>(tobj_materials[i].specular[0]);
				this_material.ks.y = static_cast<const float>(tobj_materials[i].specular[1]);
				this_material.ks.z = static_cast<const float>(tobj_materials[i].specular[2]);
				this_material.tr.x = static_cast<const float>(tobj_materials[i].transmittance[0]);
				this_material.tr.y = static_cast<const float>(tobj_materials[i].transmittance[1]);
				this_material.tr.z = static_cast<const float>(tobj_materials[i].transmittance[2]);
				this_material.ke.x = static_cast<const float>(tobj_materials[i].emission[0]);
				this_material.ke.y = static_cast<const float>(tobj_materials[i].emission[1]);
				this_material.ke.z = static_cast<const float>(tobj_materials[i].emission[2]);
				this_material.ns = static_cast<const float>(tobj_materials[i].shininess);
				this_material.ni = static_cast<const float>(tobj_materials[i].ior);
				this_material.dissolve = static_cast<const float>(tobj_materials[i].dissolve);
				this_material.illum = static_cast<const float>(tobj_materials[i].illum);
				if (tobj_materials[i].ambient_texname.length() > 0) {
					this_material.map_ka = (basepath + tobj_materials[i].ambient_texname);
				}
				if (tobj_materials[i].diffuse_texname.length() > 0) {
					this_material.map_kd = (basepath + tobj_materials[i].diffuse_texname);
				}
				if (tobj_materials[i].specular_texname.length() > 0) {
					this_material.map_ks = (basepath + tobj_materials[i].specular_texname);
				}
				if (tobj_materials[i].specular_highlight_texname.length() > 0) {
					this_material.map_ns =
						(basepath + tobj_materials[i].specular_highlight_texname);
				}
				if (tobj_materials[i].bump_texname.length() > 0) {
					this_material.map_bump = (basepath + tobj_materials[i].bump_texname);
				}
				if (tobj_materials[i].alpha_texname.length() > 0) {
					this_material.map_d = (basepath + tobj_materials[i].alpha_texname);
				}
				if (tobj_materials[i].displacement_texname.length() > 0) {
					this_material.disp = (basepath + tobj_materials[i].displacement_texname);
				}
				if (tobj_materials[i].diffuse_specname.length() > 0) {
					this_material.refl = (tobj_materials[i].diffuse_specname);
				}
				materials_.push_back(this_material);
			}
		}
		else {
			Material this_material;
			this_material.kd *= 0.2;
			materials_.push_back(this_material);
		}
	}
} // Load()

glm::vec3 Mesh::CalculateTriangleNormal(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3) {
	glm::vec3 v = p2 - p1;
	glm::vec3 w = p3 - p1;
	glm::vec3 n = glm::cross(v, w);
	n = glm::normalize(n);
	return n;
}

void Mesh::CalculateNormals() {
	//if (normals_.size() != vertices_.size()) {
	normals_.clear();
	normals_.resize(vertices_.size());
	for (int i = 0; i < (int)faces_.size(); i++) {
		glm::vec3 v1 = vertices_[faces_[i].v1];
		glm::vec3 v2 = vertices_[faces_[i].v2];
		glm::vec3 v3 = vertices_[faces_[i].v3];
		glm::vec3 n = CalculateTriangleNormal(v1, v2, v3);
		normals_[faces_[i].v1] = normals_[faces_[i].v1] + n;
		normals_[faces_[i].v2] = normals_[faces_[i].v2] + n;
		normals_[faces_[i].v3] = normals_[faces_[i].v3] + n;
	}
	//}
	for (int i = 0; i < (int)normals_.size(); i++) {
		normals_[i] = glm::normalize(normals_[i]);
	}
}

float Mesh::GetMeshHeight(glm::vec2 position) {
	float hmax = 1000.0f;
	glm::vec3 orig(position.x, position.y, hmax);
	glm::vec3 dir(0.0f, 0.0f, -1.0f);
	float highest = std::numeric_limits<float>::lowest();
	for (int i = 0; i < (int)faces_.size(); i++) {
		glm::vec3 v1 = vertices_[faces_[i].v1];
		glm::vec3 v2 = vertices_[faces_[i].v2];
		glm::vec3 v3 = vertices_[faces_[i].v3];
		//bool intersected = math::RayTriangleIntersection( orig,dir,
		//	v1,v2,v3,point);  
		float dist = -1.0f;
		glm::vec2 barypos;
		bool intersected = glm::intersectRayTriangle(orig, dir,v1, v2, v3, barypos, dist);
		glm::vec3 point = orig + dir*dist;
		if (intersected) {
			float h = hmax - point.z;
			if (h > highest)highest = h;
		}
	}
	return highest;
}

void Mesh::ApplyAffineTransformation(glm::mat3x4 aff_rot) {
	float max = std::numeric_limits<float>::max();
	float min = std::numeric_limits<float>::lowest();
	max_corner_.x = min;
	min_corner_.x = max;
	max_corner_.y = min;
	min_corner_.y = max;
	max_corner_.z = min;
	min_corner_.z = max;
	glm::mat3x3 R;
	for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) { R[i][j] = aff_rot[i][j]; } }
	orientation_ = glm::quat_cast(R);
	orientation_ = orientation_ / glm::length(orientation_);
	glm::vec3 T(aff_rot[0][3], aff_rot[1][3], aff_rot[2][3]);
	for (int i = 0; i < (int)vertices_.size(); i++) {
		glm::vec3 v = vertices_[i];
		glm::vec3 vprime = R * v;
		vprime = vprime + T;
		vertices_[i] = vprime;
		if (vertices_[i].x > max_corner_.x)max_corner_.x = vertices_[i].x;
		if (vertices_[i].y > max_corner_.y)max_corner_.y = vertices_[i].y;
		if (vertices_[i].z > max_corner_.z)max_corner_.z = vertices_[i].z;
		if (vertices_[i].x < min_corner_.x)min_corner_.x = vertices_[i].x;
		if (vertices_[i].y < min_corner_.y)min_corner_.y = vertices_[i].y;
		if (vertices_[i].z < min_corner_.z)min_corner_.z = vertices_[i].z;
	}
}

std::vector<std::string> Mesh::GetAllTextureNames() {
	std::vector<std::string> names;
	for (int i = 0; i < (int)materials_.size(); i++) {
		if (materials_[i].map_ka.length() > 0) {
			names.push_back(materials_[i].map_ka);
		}
		if (materials_[i].map_kd.length() > 0) {
			names.push_back(materials_[i].map_kd);
		}
		if (materials_[i].map_ks.length() > 0) {
			names.push_back(materials_[i].map_ks);
		}
		if (materials_[i].map_ns.length() > 0) {
			names.push_back(materials_[i].map_ns);
		}
		if (materials_[i].map_bump.length() > 0) {
			names.push_back(materials_[i].map_bump);
		}
		if (materials_[i].map_d.length() > 0) {
			names.push_back(materials_[i].map_d);
		}
		if (materials_[i].map_ka.length() > 0) {
			names.push_back(materials_[i].map_ka);
		}
		if (materials_[i].disp.length() > 0) {
			names.push_back(materials_[i].disp);
		}
		if (materials_[i].refl.length() > 0) {
			names.push_back(materials_[i].refl);
		}
	}
	return names;
}

void Mesh::ClearAll() {
	faces_.clear();
	mat_nums_.clear();
	vertices_.clear();
	normals_.clear();
	texcoords_.clear();
	materials_.clear();
	mesh_name_="";
	float max = std::numeric_limits<float>::max();
	float min = std::numeric_limits<float>::lowest();
	max_corner_.x = min;
	min_corner_.x = max;
	max_corner_.y = min;
	min_corner_.y = max;
	max_corner_.z = min;
	min_corner_.z = max;
	size_calculated_ = false;
	label_by_group_ = false;
}

} //namespace raytracer
} //namespace mavs
