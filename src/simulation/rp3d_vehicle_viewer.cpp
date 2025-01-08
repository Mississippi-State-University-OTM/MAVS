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
#include <simulation/rp3d_vehicle_viewer.h>
#include <algorithm>
#ifdef USE_OMP
#include <omp.h>
#endif
#include <raytracers/mesh.h>
#include <mavs_core/data_path.h>
#include <mavs_core/math/utils.h>

namespace mavs {
Rp3dVehicleViewer::Rp3dVehicleViewer() {
	npix_ = 512;
	np2_ = npix_ / 2;
	front_image_.assign(npix_, npix_, 1, 3);
	side_image_.assign(npix_, npix_, 1, 3);
	disp_filled_ = false;
}

void Rp3dVehicleViewer::LoadVehicle(std::string fname) {
	mavs::MavsDataPath mavs_data_path;
	std::string veh_path = mavs_data_path.GetPath();
	veh_path.append("/vehicles/rp3d/");
	std::string scene_file = mavs_data_path.GetPath();
	scene_file.append("/scenes/cube_scene.json");
	scene_.Load(scene_file);
	env_.SetRaytracer(&scene_);
	vehicle_.Load(fname);
	std::vector<float> args;
	args.push_back(0.0f);
	vehicle_.SetTerrainProperties("paved", 1000.0f, "flat", args);
	float elapsed_time = 0.0f;
	while (elapsed_time < 2.0f) {
		vehicle_.Update(&env_, 0.0f, 0.0f, 1.0f, 0.01f);
		env_.SetActorPosition(0, vehicle_.GetPosition(), vehicle_.GetOrientation());
		elapsed_time += 0.01f;
	}
}

void Rp3dVehicleViewer::FillDisplay(bool show_debug) {
	glm::vec3 position = vehicle_.GetPosition();
	float cgz = vehicle_.GetCgOffset();

	glm::vec3 size = 3.0f*vehicle_.GetChassisDimensions();

	float front_dist = std::max(0.5f*size.x, std::max(size.y, size.z));
	front_camera_.Initialize(npix_, npix_, front_dist, front_dist, 0.0035f);
	front_camera_.SetBackgroundColor(0, 0, 0);
	float front_pixdim = front_dist / ((float)npix_);
	glm::vec3 front_camera_pos(position.x + size.x, position.y, position.z);
	glm::quat front_camera_orient(0.0f, 0.0f, 0.0f, -1.0f);
	front_camera_.SetPose(front_camera_pos, front_camera_orient);
	front_camera_.SetName("Front");

	float side_dist = std::max(0.5f*size.y, std::max(size.x, size.z));
	side_camera_.Initialize(npix_, npix_, side_dist, side_dist, 0.0035f);
	side_camera_.SetBackgroundColor(0, 0, 0);
	float side_pixdim = side_dist / ((float)npix_);
	glm::vec3 side_camera_pos(position.x, position.y - size.y, position.z);
	glm::quat side_camera_orient(0.7071f, 0.0f, 0.0f, 0.7071f);
	side_camera_.SetPose(side_camera_pos, side_camera_orient);
	side_camera_.SetName("Side");

	front_camera_.Update(&env_, 0.1f);
	front_image_ = front_camera_.GetCurrentImage();

	side_camera_.Update(&env_, 0.1f);
	side_image_ = side_camera_.GetCurrentImage();
	
	std::string xdim_str = mavs::utils::ToString(size.x);
	std::string ydim_str = mavs::utils::ToString(size.y);
	std::string zdim_str = mavs::utils::ToString(size.z);

	float green[] = { 0.0f, 255.0f, 0.0f };
	float yellow[] = { 255.0f, 255.0f, 0.0f };
	float orange[] = { 255.0f, 165.0f, 0.0f };
	float red[] = { 255.0f, 0.0f, 0.0f };
	float blue[] = { 0.0f, 0.0f, 255.0f };

	glm::vec3 center = position;

	float ylow = center.y - np2_ * front_pixdim;
	float zlow = center.z - np2_ * front_pixdim;
	int y_size = (int)(size.y / front_pixdim);
	int y_start = (int)(0.5f*(npix_ - y_size));
	int z_size = (int)(size.z / front_pixdim);
	int z_start = (int)(0.5f*(npix_ - z_size));
	int yp = (int)((position.y - ylow) / front_pixdim);
	int zp = npix_ - (int)((position.z - zlow) / front_pixdim);
	int z_0 = npix_ - (int)((0.0 - zlow) / front_pixdim);
	front_image_.draw_arrow(10, z_start + z_size, 10, z_start, yellow, 1.0f, 30, 10);
	front_image_.draw_arrow(y_start, 10, y_start + y_size, 10, yellow, 1.0f, 30, 10);
	front_image_.draw_text(np2_, 15, ydim_str.c_str(), yellow);
	front_image_.draw_text(15, np2_, zdim_str.c_str(), yellow);
	front_image_.draw_text(y_start + y_size - 10, 15, "Y", yellow);
	front_image_.draw_text(15, z_start, "Z", yellow);
	if (show_debug) {
		front_image_.draw_rectangle(
			(int)(((position.y - 0.5*size.y / 3.0) - ylow) / front_pixdim),
			npix_ - (int)(((position.z + cgz - 0.5*size.z / 3.0) - zlow) / front_pixdim) - 1,
			(int)(((position.y + 0.5*size.y / 3.0) - ylow) / front_pixdim),
			npix_ - (int)(((position.z + cgz + 0.5*size.z / 3.0) - zlow) / front_pixdim) - 1,
			blue, 0.5f);
		front_image_.draw_circle(yp, zp, 5, green, 1.0f);
		for (int tn = 0; tn < 2; tn++) {
			glm::vec3 tp0 = vehicle_.GetTirePosition(tn);
			int tp0y = (int)((tp0.y - ylow) / front_pixdim);
			int tp0z = front_image_.height() - (int)((tp0.z - zlow) / front_pixdim) - 1;
			int tp0r = (int)(vehicle_.GetTire(tn)->GetRadius() / front_pixdim);
			int tp0w = (int)(vehicle_.GetTire(tn)->GetWidth() / front_pixdim);
			front_image_.draw_rectangle(tp0y - 0.5*tp0w, tp0z - tp0r, tp0y + 0.5*tp0w, tp0z + tp0r, green, 0.5f);
		}
		front_image_.draw_line(0, z_0, front_image_.width(), z_0, red, 1.0f);
	}

	float xlow = center.x - np2_ * side_pixdim;
	zlow = center.z - np2_ * side_pixdim;
	int x_size = (int)(size.x / side_pixdim);
	int x_start = (int)(0.5f*(npix_ - x_size));
	//int x_start = (int)(npix_+(center.x-0.5*size.x)/side_pixdim);
	z_size = (int)(size.z / side_pixdim);
	z_start = (int)(0.5f*(npix_ - z_size));
	int x_0 = npix_ - (int)((0.0f - xlow) / side_pixdim);
	z_0 = npix_ - (int)((0.0f - zlow) / side_pixdim);
	zp = npix_ - (int)((position.z - zlow) / side_pixdim);
	int xp = npix_ - (int)((position.x - xlow) / side_pixdim);
	//-----
	glm::vec3 tf = vehicle_.GetTirePosition(0);
	glm::vec3 tr = vehicle_.GetTirePosition(2);
	//------
	side_image_.draw_arrow(10, z_start + z_size, 10, z_start, yellow, 1.0f, 30, 10);
	side_image_.draw_arrow(x_start, 10, x_start + x_size, 10, yellow, 1.0f, 30, 10);
	side_image_.draw_text(np2_, 15, xdim_str.c_str(), yellow);
	side_image_.draw_text(15, np2_, zdim_str.c_str(), yellow);
	side_image_.draw_text(x_start + x_size - 10, 15, "X", yellow);
	side_image_.draw_text(15, z_start, "Z", yellow);
	if (show_debug) {
		side_image_.draw_rectangle(
			(int)(((position.x - 0.5*size.x / 3.0) - xlow) / side_pixdim),
			npix_ - (int)(((position.z + cgz - 0.5*size.z / 3.0) - zlow) / side_pixdim) - 1,
			(int)(((position.x + 0.5*size.x / 3.0) - xlow) / side_pixdim),
			npix_ - (int)(((position.z + cgz + 0.5*size.z / 3.0) - zlow) / side_pixdim) - 1,
			blue, 0.5f);
		side_image_.draw_circle(xp, zp, 5, green, 1.0f);
		for (int tn = 0; tn < vehicle_.GetNumTires(); tn++) {
			if (tn % 2 == 0) {
				glm::vec3 tp0 = vehicle_.GetTirePosition(tn);
				int tp0x = (int)((tp0.x - xlow) / side_pixdim);
				int tp0z = side_image_.height() - (int)((tp0.z - zlow) / side_pixdim) - 1;
				int tp0r = (int)(vehicle_.GetTire(0)->GetRadius() / side_pixdim);
				float tcol[3];
				if (tn == 0) {
					side_image_.draw_circle(tp0x, tp0z, tp0r, green, 1.0f, 2);
				}
				else {
					side_image_.draw_circle(tp0x, tp0z, tp0r, orange, 1.0f, 2);
				}
			}
		}
	}
	disp_filled_ = true;
}

void Rp3dVehicleViewer::Update(bool show_debug) {
	if (!disp_filled_) {
		FillDisplay(show_debug);
	}
}

void Rp3dVehicleViewer::Display(bool show_debug) {
	if (!disp_filled_) {
		FillDisplay(show_debug);
	}
	front_disp_ = front_image_;
	side_disp_ = side_image_;
}

} // namespace mavs
