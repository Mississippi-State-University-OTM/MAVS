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
#include <mavs_core/terrain_generator/surface.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <limits>

#include <mavs_core/math/utils.h>
#include <FastNoise.h>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>

namespace mavs {
namespace terraingen {

Surface::Surface() {
	color_ = glm::vec3(0.38f, 0.25f, 0.06f);
	float size = 100.0f;
	trail_set_ = false;
	//ll_corner_ = glm::vec2(-size, -size);
	//ur_corner_ = glm::vec2(size, size);
	//resolution_ = 0.25;
	//scene_area_ = (ur_corner_.x - ll_corner_.x)*(ur_corner_.y - ll_corner_.y);
	//nx_ = 0;
	//ny_ = 0;
}

Surface::~Surface() {

}

void Surface::SetTrail(Trail &trail) {
	trail_ = trail;
	trail_set_ = true;
	layered_surface_.SetTrail(trail);
}

void Surface::GetPoseAtPosition(glm::vec2 pos, glm::vec2 lt, glm::vec3 &position, glm::quat &orientation){
	heightmap_.GetPoseAtPosition(pos, lt, position, orientation);
	/*
	glm::vec2 slope = heightmap_.GetSlopeAtPoint(pos);
	position = glm::vec3 (pos.x, pos.y, (heightmap_.GetHeightAtPoint(pos) + 0.5f));
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
	orientation = glm::quat_cast(R);*/
}

Trail Surface::GetTrail(std::string path_type) {
	if (trail_set_) {
		return trail_;
	}
	glm::vec3 zhat(0.0f, 0.0f, 1.0f);
	Trail trail;
	Waypoints path;
	if (path_type == "Loop") {
		float dt = (float)(0.005*mavs::k2Pi); //parameteric spacing of points
		glm::vec2 ll = heightmap_.GetLLCorner();
		glm::vec2 ur = heightmap_.GetURCorner();
		float a = 0.4f*(ur.x - ll.x);
		float b = 0.4f*(ur.y - ll.y);
		float t = 0.0;
		while (t <= (float)mavs::k2Pi) {
			glm::vec2 pos(a*cos(t), b*sin(t));
			glm::vec2 next_pos(a*cos(t + dt), b*sin(t + dt));
			glm::vec2 lt = next_pos - pos;
			lt = lt / glm::length(lt);
			path.AddPoint(pos);
			glm::vec3 position;
			glm::quat orientation;
			GetPoseAtPosition(pos, lt, position, orientation);
			trail.AddPose(position, orientation);
			t += dt;
		}
	}// path type == Loop
	else if (path_type == "Square") {
		glm::vec2 ll = heightmap_.GetLLCorner();
		glm::vec2 pos = 0.4f*ll;
		glm::vec2 ur = heightmap_.GetURCorner();
		float dx = 1.0;
		while (pos.x <= 0.4f*ur.x) {
			path.AddPoint(pos);
			pos.x += dx;
		}
		while (pos.y <= 0.4f*ur.y) {
			path.AddPoint(pos);
			pos.y += dx;
		}
		while (pos.x >= 0.4f*ll.x) {
			path.AddPoint(pos);
			pos.x -= dx;
		}
		while (pos.y >= 0.4f*ll.y) {
			path.AddPoint(pos);
			pos.y -= dx;
		}
	}
	else {
		glm::vec2 pos = heightmap_.GetLLCorner(); // ll_corner_;
		float offset = 0.02f*glm::length(heightmap_.GetSize());
		offset = std::max(3.0f, offset);
		pos.x += offset;
		pos.y += offset;
		path.AddPoint(pos);
		float dist = glm::length(pos - heightmap_.GetURCorner());
		float mult = -1.0f;
		float dt = 0.5;
		if (path_type == "Ridges") mult = 1.0f;
		int nsteps = 0;
		while (dist > offset && nsteps < 5000) {
			// get next point in path
			glm::vec2 to_goal = heightmap_.GetURCorner() - pos;
			dist = glm::length(to_goal);
			to_goal = to_goal / dist;
			glm::vec2 slope = heightmap_.GetSlopeAtPoint(pos);
			glm::vec2 force = mult * slope + to_goal;
			glm::vec2 newpos = pos + dt * force;
			glm::vec2 lt = newpos - pos;
			pos = newpos;
			path.AddPoint(pos);
			//estimate vehicle pose
			glm::vec3 position(pos.x, pos.y, (heightmap_.GetHeightAtPoint(pos) + 0.5f));
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
			glm::quat quaternion = glm::quat_cast(R);
			trail.AddPose(position, quaternion);
			nsteps++;
		}
	}
	trail.SetPath(path);
	trail_set_ = true;
	trail_ = trail;
	return trail;
}

void Surface::SetDimensions(float llx, float lly, float urx, float ury, float res) {
	heightmap_.SetCorners(llx, lly, urx, ury);
	heightmap_.SetResolution(res);
	int nx = (int)(ceil(urx - llx)/res)+1;
	int ny = (int)(ceil(ury - lly)/res)+1;
	heightmap_.Resize(nx, ny);
}

void Surface::WriteObj() {
	std::string surfname("surface");
	WriteObj(surfname);
}

void Surface::WriteObj(std::string surfname) {
	heightmap_.WriteObj(surfname, textured_);
	/*
	float zmin = std::numeric_limits<float>::max();
	std::vector<glm::vec3> vertices;

	glm::vec2 ll_corner = heightmap_.GetLLCorner();
	glm::vec2 ur_corner = heightmap_.GetURCorner();
	int nx = (int)heightmap_.GetHorizontalDim();
	int ny = (int)heightmap_.GetVerticalDim();
	float resolution = heightmap_.GetResolution();
	float x = ll_corner.x;
	for (int i = 0; i < nx; i++) {
		float y = ll_corner.y;
		for (int j = 0; j < ny; j++) {
			glm::vec3 v(x, y, heightmap_.GetCellHeight(i,j) );
			vertices.push_back(v);
			if (heightmap_.GetCellHeight(i,j) < zmin)zmin = heightmap_.GetCellHeight(i,j);
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
	fout << "mtllib "<<surfname<<".mtl" << std::endl << std::endl;
	for (int i = 0; i < (int)vertices.size(); i++) {
		fout << "v " << vertices[i].x << " " << vertices[i].y << " " <<
			vertices[i].z << std::endl;
	}

	glm::vec2 surf_size = ur_corner - ll_corner;
	if (textured_) {
		for (int i = 0; i < (int)vertices.size(); i++) {
			float dx = (vertices[i].x - ll_corner.x) / surf_size.x;
			float dy = (vertices[i].y - ll_corner.y) / surf_size.y;
			fout << "vt " << dx << " " << dy << std::endl;
		}
	}
	fout << std::endl << "usemtl " << surfname<<std::endl << std::endl;

	for (int i = 0; i < (int)faces.size(); i++) {
		if (textured_) {
			fout << "f " << faces[i].x << "/" << faces[i].x << " " << faces[i].y << "/" << faces[i].y << " " <<
				faces[i].z << "/" << faces[i].z << std::endl;
		}
		else {
			fout << "f " << faces[i].x << " " << faces[i].y << " " <<
				faces[i].z << std::endl;
		}
	}

	//add a floor that extends to the horizon
	float floor_size = 100000.0;
	fout << "v " << -floor_size << " " << -floor_size << " " << zmin << std::endl;
	fout << "v " << -10000.0 << " " << floor_size << " " << zmin << std::endl;
	fout << "v " << floor_size << " " << floor_size << " " << zmin << std::endl;
	fout << "v " << floor_size << " " << -floor_size << " " << zmin << std::endl;
	if (textured_) {
		fout << "vt " << ((-floor_size - ll_corner.x) / surf_size.x) << " " << ((-floor_size - ll_corner.y) / surf_size.y) << std::endl;
		fout << "vt " << ((-floor_size - ll_corner.x) / surf_size.x) << " " << ((floor_size - ll_corner.y) / surf_size.y) << std::endl;
		fout << "vt " << ((floor_size - ll_corner.x) / surf_size.x) << " " << ((floor_size - ll_corner.y) / surf_size.y) << std::endl;
		fout << "vt " << ((floor_size - ll_corner.x) / surf_size.x) << " " << ((-floor_size - ll_corner.y) / surf_size.y) << std::endl;
	}
	if (textured_) {
		fout << "f " << vertices.size() + 1 << "/" << vertices.size() + 1<<" "<<vertices.size() + 2 << "/" << vertices.size() + 2<<" "<<vertices.size() + 3 << "/"<< vertices.size() + 3<<std::endl;
		fout << "f " << vertices.size() + 3 << "/" << vertices.size() + 3 << " " << vertices.size() + 4 << "/" << vertices.size() + 4 << " " << vertices.size() + 1 << "/" << vertices.size() + 1 << std::endl;
	}
	else {
		fout << "f " << vertices.size() + 1 << " " << vertices.size() + 2 << " " << vertices.size() + 3 << std::endl;
		fout << "f " << vertices.size() + 3 << " " << vertices.size() + 4 << " " << vertices.size() + 1 << std::endl;
	}
	fout.close();

	std::string matname = surfname; 
	matname.append(".mtl");
	std::ofstream mout(matname);
	mout << "newmtl "<<surfname<< std::endl;
	mout << "	 Ns 10.0000" << std::endl;
	mout << "	 Ni 1.5000" << std::endl;
	mout << "	 d 1.0000" << std::endl;
	mout << "	 Tr 0.0000" << std::endl;
	mout << "	 Tf 1.0000 1.0000 1.0000" << std::endl;
	mout << "	 illum 2" << std::endl;
	mout << "	 Ka 0.5 0.5 0.5" << std::endl;
	mout << "	 Kd 0.5 0.5 0.5" << std::endl;
	mout << "	 Kd spectral spectra/gravelly_loam.spec" << std::endl;
	mout << "	 Ks 0.0000 0.0000 0.0000" << std::endl;
	mout << "	 Ke 0.0000 0.0000 0.0000" << std::endl;
	mout.close();*/
}

} //namespace terraingen
} //namespace mavs