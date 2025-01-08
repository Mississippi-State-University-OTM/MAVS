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
/**
* \file rp3d_vehicle_viewer.cpp
*
* Quickly, easily view an MAVS vehicle to check the alignment of the mesh and physics
*
* Usage: >./rp3d_vehicle_viewer
*
* Will prompt you to open an obj file.
*
* \author Chris Goodin
*
* \date 4/2/2020
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
#ifdef USE_OMP
#include <omp.h>
#endif
#include <simulation/rp3d_vehicle_viewer.h>
#include <sensors/io/user_io.h>

int main(int argc, char *argv[]) {

	mavs::Rp3dVehicleViewer viewer;

	std::string fname;
	if (argc >= 2) {
		fname = std::string(argv[1]);
	}
	else {
		fname = mavs::io::GetInputFileName("Select the .rp3d file", "*.rp3d");
	}
	if (fname == "") {
		std::cerr << "ERROR, no mesh file found " << std::endl;
		return 1;
	}

	viewer.LoadVehicle(fname);

	while (true) {
		viewer.Display(true);
	}

	return 0;
}

/*
#include <algorithm>
#ifdef USE_OMP
#include <omp.h>
#endif
#include <raytracers/mesh.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include <mavs_core/environment/environment.h>
#include <mavs_core/data_path.h>
#include <mavs_core/math/utils.h>
#include <sensors/camera/ortho_camera.h>
#include <sensors/io/user_io.h>
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>

int main(int argc, char * argv[]) {
	const int npix = 512;
	const int np2 = npix / 2;
	cimg_library::CImg<float> front_image(npix, npix);
	cimg_library::CImg<float> side_image(npix, npix);
	cimg_library::CImgDisplay front_disp(npix, npix, "Front");
	cimg_library::CImgDisplay side_disp(npix, npix, "Side");

	std::string fname;
	if (argc >= 2) {
		fname = std::string(argv[1]);
	}
	else {
		fname = mavs::io::GetInputFileName("Select the .rp3d file", "*.rp3d");
	}
	if (fname == "") {
		std::cerr << "ERROR, no mesh file found " << std::endl;
		return 1;
	}

	mavs::MavsDataPath mavs_data_path;
	std::string veh_path = mavs_data_path.GetPath();
	veh_path.append("/vehicles/rp3d/");
	std::string scene_file = mavs_data_path.GetPath();
	scene_file.append("/scenes/cube_scene.json");
	mavs::environment::Environment env;
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	env.SetRaytracer(&scene);
	mavs::vehicle::Rp3dVehicle vehicle;
	vehicle.Load(fname);
	std::vector<float> args;
	args.push_back(0.0f);
	vehicle.SetTerrainProperties("paved", 1000.0f, "flat", args);
	float elapsed_time = 0.0f;
	while (elapsed_time < 2.0f) {
		vehicle.Update(&env, 0.0f, 0.0f, 1.0f, 0.01f);
		env.SetActorPosition(0, vehicle.GetPosition(), vehicle.GetOrientation());
		elapsed_time += 0.01f;
	}
	glm::vec3 position = vehicle.GetPosition();
	float cgz = vehicle.GetCgOffset();

	glm::vec3 size = 3.0f*vehicle.GetChassisDimensions();
	mavs::sensor::camera::OrthoCamera front_camera;
	float front_dist = std::max(0.5f*size.x, std::max(size.y, size.z));
	front_camera.Initialize(npix, npix, front_dist, front_dist, 0.0035f);
	front_camera.SetBackgroundColor(0, 0, 0);
	float front_pixdim = front_dist / ((float)npix);
	glm::vec3 front_camera_pos(position.x + size.x, position.y, position.z);
	glm::quat front_camera_orient(0.0f, 0.0f, 0.0f, -1.0f);
	front_camera.SetPose(front_camera_pos, front_camera_orient);
	front_camera.SetName("Front");

	mavs::sensor::camera::OrthoCamera side_camera;
	float side_dist = std::max(0.5f*size.y, std::max(size.x, size.z));
	side_camera.Initialize(npix, npix, side_dist, side_dist, 0.0035f);
	side_camera.SetBackgroundColor(0, 0, 0);
	float side_pixdim = side_dist / ((float)npix);
	glm::vec3 side_camera_pos(position.x, position.y + size.y, position.z);
	glm::quat side_camera_orient(0.7071f, 0.0f, 0.0f, -0.7071f);
	side_camera.SetPose(side_camera_pos, side_camera_orient);
	side_camera.SetName("Side");

	front_camera.Update(&env, 0.1f);
	front_image = front_camera.GetCurrentImage();

	side_camera.Update(&env, 0.1f);
	side_image = side_camera.GetCurrentImage();

	std::string xdim_str = mavs::utils::ToString(size.x);
	std::string ydim_str = mavs::utils::ToString(size.y);
	std::string zdim_str = mavs::utils::ToString(size.z);

	float green[] = { 0.0f, 255.0f, 0.0f };
	float yellow[] = { 255.0f, 255.0f, 0.0f };
	float orange[] = { 255.0f, 165.0f, 0.0f };
	float red[] = { 255.0f, 0.0f, 0.0f };
	float blue[] = { 0.0f, 0.0f, 255.0f };

	glm::vec3 center = position;

	float ylow = center.y - np2 * front_pixdim;
	float zlow = center.z - np2 * front_pixdim;
	int y_size = (int)(size.y / front_pixdim);
	int y_start = (int)(0.5f*(npix - y_size));
	int z_size = (int)(size.z / front_pixdim);
	int z_start = (int)(0.5f*(npix - z_size));
	int yp = (int)((position.y - ylow) / front_pixdim);
	int zp = npix - (int)((position.z - zlow) / front_pixdim);
	int z_0 = npix - (int)((0.0 - zlow) / front_pixdim);
	front_image.draw_arrow(10, z_start + z_size, 10, z_start, yellow, 1.0f, 30, 10);
	front_image.draw_arrow(y_start, 10, y_start + y_size, 10, yellow, 1.0f, 30, 10);
	front_image.draw_text(np2, 15, ydim_str.c_str(), yellow);
	front_image.draw_text(15, np2, zdim_str.c_str(), yellow);
	front_image.draw_text(y_start + y_size - 10, 15, "Y", yellow);
	front_image.draw_text(15, z_start, "Z", yellow);
	front_image.draw_rectangle(
		(int)(((position.y - 0.5*size.y / 3.0) - ylow) / front_pixdim),
		npix - (int)(((position.z + cgz - 0.5*size.z / 3.0) - zlow) / front_pixdim) - 1,
		(int)(((position.y + 0.5*size.y / 3.0) - ylow) / front_pixdim),
		npix - (int)(((position.z + cgz + 0.5*size.z / 3.0) - zlow) / front_pixdim) - 1,
		blue,0.5f);
	front_image.draw_circle(yp, zp, 5, green, 1.0f);
	for (int tn = 0; tn < 2; tn++) {
		glm::vec3 tp0 = vehicle.GetTirePosition(tn);
		int tp0y = (int)((tp0.y - ylow) / front_pixdim);
		int tp0z = front_image.height() - (int)((tp0.z - zlow) / front_pixdim) - 1;
		int tp0r = (int)(vehicle.GetTire(tn)->GetRadius() / front_pixdim);
		int tp0w = (int)(vehicle.GetTire(tn)->GetWidth() / front_pixdim);
		front_image.draw_rectangle(tp0y-0.5*tp0w,tp0z-tp0r,tp0y+0.5*tp0w,tp0z+tp0r,green, 0.5f);
	}
	front_image.draw_line(0, z_0, front_image.width(), z_0, red, 1.0f);

	float xlow = center.x - np2 * side_pixdim;
	zlow = center.z - np2 * side_pixdim;
	int x_size = (int)(size.x / side_pixdim);
	int x_start = (int)(0.5f*(npix - x_size));
	z_size = (int)(size.z / side_pixdim);
	z_start = (int)(0.5f*(npix - z_size));
	int x_0 = npix - (int)((0.0f - xlow) / side_pixdim);
	z_0 = npix - (int)((0.0f - zlow) / side_pixdim);
	zp = npix - (int)((position.z - zlow) / side_pixdim);
	int xp = npix - (int)((position.x - xlow) / side_pixdim);
	side_image.draw_arrow(10, z_start + z_size, 10, z_start, yellow, 1.0f, 30, 10);
	side_image.draw_arrow(x_start + x_size, 10, x_start, 10, yellow, 1.0f, 30, 10);
	side_image.draw_text(np2, 15, xdim_str.c_str(), yellow);
	side_image.draw_text(15, np2, zdim_str.c_str(), yellow);
	side_image.draw_text(x_start + x_size - 10, 15, "X", yellow);
	side_image.draw_text(15, z_start, "Z", yellow);
	side_image.draw_rectangle(
		(int)(((position.x - 0.5*size.x / 3.0) - xlow) / side_pixdim),
		npix - (int)(((position.z + cgz - 0.5*size.z / 3.0) - zlow) / side_pixdim)-1,
		(int)(((position.x + 0.5*size.x / 3.0) - xlow) / side_pixdim),
		npix - (int)(((position.z + cgz + 0.5*size.z / 3.0) - zlow) / side_pixdim)-1, 
		blue, 0.5f);
	side_image.draw_circle(xp, zp, 5, green, 1.0f);
	for (int tn = 0; tn < vehicle.GetNumTires(); tn++) {
		if (tn % 2 == 0) {
			glm::vec3 tp0 = vehicle.GetTirePosition(tn);
			int tp0x = (int)((tp0.x - xlow) / side_pixdim);
			int tp0z = side_image.height() - (int)((tp0.z - zlow) / side_pixdim) - 1;
			int tp0r = (int)(vehicle.GetTire(0)->GetRadius() / side_pixdim);
			side_image.draw_circle(tp0x, tp0z, tp0r, green, 1.0f, 2);
		}
	}

	front_disp.display(front_image);
	side_disp.display(side_image);
	while (!front_disp.is_closed() && !side_disp.is_closed()) {
		front_disp.display(front_image);
		side_disp.display(side_image);
	}

	return 0;
}
*/

