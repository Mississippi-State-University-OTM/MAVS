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
* \file obj_viewer.cpp
*
* Quickly, easily view an obj as it will be rendered in MAVS.
* Will only work for objects in the MAVS data directory.
*
* Usage: >./obj_viewer
*
* Will prompt you to open an obj file.
*
* \author Chris Goodin
*
* \date 11/5/2018
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
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

float tx_, ty_, tz_, scale_;
bool increment_rotation_ = false;
//float pitch_, yaw_, roll_;
bool save_, exit_, x_to_y_, y_to_x_, z_to_y_, y_to_z_;
bool mirror_x_, mirror_y_;
float front_pixdim, side_pixdim, top_pixdim;
glm::mat3x3 rot_keep;
glm::vec3 trans_keep;
float scale_keep;
glm::vec3 center;
const int npix = 512;
cimg_library::CImg<float> top_image(npix, npix);
cimg_library::CImg<float> front_image(npix, npix);
cimg_library::CImg<float> side_image(npix, npix);

cimg_library::CImgDisplay front_disp(npix, npix, "Front");
cimg_library::CImgDisplay side_disp(npix, npix, "Side");
cimg_library::CImgDisplay top_disp(npix, npix, "Top");

mavs::sensor::camera::OrthoCamera front_camera, side_camera, top_camera;

glm::mat3x3 Rz(float theta) {
	glm::mat3 R(0.0);
	float c = cos(theta);
	float s = sin(theta);
	R[0][0] = c;
	R[0][1] = -s;
	R[1][0] = s;
	R[1][1] = c;
	R[2][2] = 1.0f;
	return R;
}

glm::mat3x3 Rx(float theta) {
	glm::mat3 R(0.0);
	float c = cos(theta);
	float s = sin(theta);
	R[1][1] = c;
	R[1][2] = -s;
	R[2][1] = s;
	R[2][2] = c;
	R[0][0] = 1.0f;
	return R;
}

glm::mat3x3 Ry(float theta) {
	glm::mat3 R(0.0);
	float c = cos(theta);
	float s = sin(theta);
	R[0][0] = c;
	R[0][2] = s;
	R[1][1] = 1.0f;
	R[2][0] = -s;
	R[2][2] = c;
	return R;
}

bool GetKeyboardInput() {
	bool pressed = false;
	if (front_disp.is_keyU() || side_disp.is_keyU() || top_disp.is_keyU()) {
		float rx = mavs::io::GetUserNumericInput("Rotation", "Rotation about X (deg)");
		rot_keep = Rx((float)(rx*mavs::kPi/180.0f)) * rot_keep;
		increment_rotation_ = true;
		pressed = true;
	}
	if (front_disp.is_keyV() || side_disp.is_keyV() || top_disp.is_keyV()) {
		float ry = mavs::io::GetUserNumericInput("Rotation", "Rotation about Y (deg)");
		rot_keep = Ry((float)(ry * mavs::kPi / 180.0f)) * rot_keep;
		increment_rotation_ = true;
		pressed = true;
	}
	if (front_disp.is_keyW() || side_disp.is_keyW() || top_disp.is_keyW()) {
		float rz = mavs::io::GetUserNumericInput("Rotation", "Rotation about Z (deg)");
		rot_keep = Rz((float)(rz * mavs::kPi / 180.0f)) * rot_keep;
		increment_rotation_ = true;
		pressed = true;
	}

	if (front_disp.is_keyI() || side_disp.is_keyI() || top_disp.is_keyI()) {
		mirror_x_ = true;
		pressed = true;
	}
	if (front_disp.is_keyJ() || side_disp.is_keyJ() || top_disp.is_keyJ()) {
		mirror_y_ = true;
		pressed = true;
	}

	if (front_disp.is_keyX() || side_disp.is_keyX() || top_disp.is_keyX()) {
		tx_ = mavs::io::GetUserNumericInput("Translation", "Translation in X");
		trans_keep.x += tx_;
		pressed = true;
	}
	if (front_disp.is_keyY() || side_disp.is_keyY() || top_disp.is_keyY()) {
		ty_ = mavs::io::GetUserNumericInput("Translation", "Translation in Y");
		trans_keep.y += ty_;
		pressed = true;
	}
	if (front_disp.is_keyZ() || side_disp.is_keyZ() || top_disp.is_keyZ()) {
		tz_ = mavs::io::GetUserNumericInput("Translation", "Translation in Z");
		trans_keep.z += tz_;
		pressed = true;
	}
	if (front_disp.is_keyS() || side_disp.is_keyS() || top_disp.is_keyS()) {
		scale_ = mavs::io::GetUserNumericInput("Scale", "Scale factor");
		scale_keep = scale_ * scale_keep;
		pressed = true;
	}

	// Select point coordinates
	if (front_disp.is_keyP()) {
		cimg_library::CImg <int> SelectedImageCords = front_image.get_select(front_disp, 0, 0, 0);
		float x = -(center.y - front_pixdim * (SelectedImageCords(0) - 0.5f*npix));
		float y = center.z - front_pixdim * (SelectedImageCords(1) - 0.5f*npix);
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
	if (side_disp.is_keyP() ) {
		cimg_library::CImg <int> SelectedImageCords = side_image.get_select(side_disp, 0, 0, 0);
		float x = center.x - side_pixdim * (SelectedImageCords(0) - 0.5f*npix);
		float y = center.z - side_pixdim * (SelectedImageCords(1) - 0.5f*npix);
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
	if (top_disp.is_keyP()) {
		cimg_library::CImg <int> SelectedImageCords = top_image.get_select(top_disp, 0, 0, 0);
		float x = center.x - top_pixdim * (SelectedImageCords(0) - 0.5f*npix);
		float y = center.y - top_pixdim * (SelectedImageCords(1) - 0.5f*npix);
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
	//measure distance between two points
	if (front_disp.is_keyM()) {
		cimg_library::CImg <int> SelectedImageCords = front_image.get_select(front_disp, 1, 0, 0);
		float x0 = -(center.y - front_pixdim * (SelectedImageCords(0) - 0.5f*npix));
		float y0 = center.z - front_pixdim * (SelectedImageCords(1) - 0.5f*npix);
		float x1 = -(center.y - front_pixdim * (SelectedImageCords(3) - 0.5f*npix));
		float y1 = center.z - front_pixdim * (SelectedImageCords(4) - 0.5f*npix);
		float dx = x0 - x1;
		float dy = y0 - y1;
		float d = (float)sqrt(dx*dx + dy * dy);
		std::cout << "|" << dx << ", " << dy << "| = " << d << std::endl;
	}
	if (side_disp.is_keyM()) {
		cimg_library::CImg <int> SelectedImageCords = side_image.get_select(side_disp, 1, 0, 0);
		float x0 = center.x - side_pixdim * (SelectedImageCords(0) - 0.5f*npix);
		float y0 = center.z - side_pixdim * (SelectedImageCords(1) - 0.5f*npix);
		float x1 = center.x - side_pixdim * (SelectedImageCords(3) - 0.5f*npix);
		float y1 = center.z - side_pixdim * (SelectedImageCords(4) - 0.5f*npix);
		float dx = x0 - x1;
		float dy = y0 - y1;
		float d = (float)sqrt(dx*dx + dy * dy);
		std::cout << "|" << dx << ", " << dy << "| = " << d << std::endl;
	}
	if (top_disp.is_keyM()) {
		cimg_library::CImg <int> SelectedImageCords = top_image.get_select(top_disp, 1, 0, 0);
		float x0 = center.x - top_pixdim * (SelectedImageCords(0) - 0.5f*npix);
		float y0 = center.y - top_pixdim * (SelectedImageCords(1) - 0.5f*npix);
		float x1 = center.x - top_pixdim * (SelectedImageCords(3) - 0.5f*npix);
		float y1 = center.y - top_pixdim * (SelectedImageCords(4) - 0.5f*npix);
		float dx = x0 - x1;
		float dy = y0 - y1;
		float d = (float)sqrt(dx*dx + dy * dy);
		std::cout << "|" << dx << ", " << dy << "| = " << d << std::endl;
	}

	//if (front_disp.is_keyP() || side_disp.is_keyP() || top_disp.is_keyP()) {
	//	pitch_ = GetUserInput("Pitch", "Rotation about Y-axis");
	//	pressed = true;
	//}
	//if (front_disp.is_keyW() || side_disp.is_keyW() || top_disp.is_keyW()) {
	//	yaw_ = GetUserInput("Yaw", "Rotation about Z-axis");
	//	pressed = true;
	//}
	//if (front_disp.is_keyR() || side_disp.is_keyR() || top_disp.is_keyR()) {
	//	roll_ = GetUserInput("Pitch", "Rotation about X-axis");
	//	pressed = true;
	//}
	if (front_disp.is_keyARROWUP() || side_disp.is_keyARROWUP() || top_disp.is_keyARROWUP()) {
		y_to_z_ = true;
		rot_keep = Rx((float)-mavs::kPi_2)*rot_keep;
		pressed = true;
	}
	if (front_disp.is_keyARROWDOWN() || side_disp.is_keyARROWDOWN() || top_disp.is_keyARROWDOWN()) {
		z_to_y_ = true;
		rot_keep = Rx((float)mavs::kPi_2)*rot_keep;
		pressed = true;
	}
	if (front_disp.is_keyARROWLEFT() || side_disp.is_keyARROWLEFT() || top_disp.is_keyARROWLEFT()) {
		y_to_x_ = true;
		rot_keep = Rz((float)mavs::kPi_2)*rot_keep;
		pressed = true;
	}
	if (front_disp.is_keyARROWRIGHT() || side_disp.is_keyARROWRIGHT() || top_disp.is_keyARROWRIGHT()) {
		x_to_y_ = true;
		rot_keep = Rz((float)-mavs::kPi_2)*rot_keep;
		pressed = true;
	}
	if (front_disp.is_keyD() || side_disp.is_keyD() || top_disp.is_keyD()) {
		save_ = true;
		pressed = true;
	}
	if (front_disp.is_keyR()){
		cimg_library::CImg<float> tmp_img = front_camera.GetCurrentImage();
		std::string fname = mavs::io::GetSaveFileName("Save the image", "out.bmp", "*.bmp");
		tmp_img.save(fname.c_str());
	}
	if (side_disp.is_keyR()) {
		cimg_library::CImg<float> tmp_img = side_camera.GetCurrentImage();
		std::string fname = mavs::io::GetSaveFileName("Save the image", "out.bmp", "*.bmp");
		tmp_img.save(fname.c_str());
	}
	if (top_disp.is_keyR()) {
		cimg_library::CImg<float> tmp_img = top_camera.GetCurrentImage();
		std::string fname = mavs::io::GetSaveFileName("Save the image", "out.bmp", "*.bmp");
		tmp_img.save(fname.c_str());
	}
	if (front_disp.is_keyQ() || side_disp.is_keyQ() || top_disp.is_keyQ()) {
		exit_ = true;
		pressed = true;
	}
	return pressed;
}

int main(int argc, char * argv[]) {
	trans_keep.x = 0.0f; trans_keep.y = 0.0f; trans_keep.z = 0.0f;
	scale_keep = 1.0f;
	rot_keep[0][0] = 1.0f; rot_keep[0][1] = 0.0f; rot_keep[0][2] = 0.0f;
	rot_keep[1][0] = 0.0f; rot_keep[1][1] = 1.0f; rot_keep[1][2] = 0.0f;
	rot_keep[2][0] = 0.0f; rot_keep[2][1] = 0.0f; rot_keep[2][2] = 1.0f;

	const int np2 = npix / 2;
	
	mavs::environment::Environment env;
	std::string fname;
	if (argc >= 2) {
		fname = std::string(argv[1]);
	}
	else {
		fname = mavs::io::GetInputFileName("Select the .obj file", "*.obj,*.bin");
	}
	if (fname == "") {
		std::cerr << "ERROR, no mesh file found " << std::endl;
		return 1;
	}
	//std::string path = mavs::utils::GetPathFromFile(fname);
	//path.append("/");

	mavs::MavsDataPath mavs_data_path;
	std::string path = mavs_data_path.GetPath();
	path.append("/scenes/meshes/");

	mavs::raytracer::Mesh mesh;

	mesh.Load(path, fname);
	save_ = false;
	exit_ = false;
	
	while (true) {
		tx_ = 0.0f;
		ty_ = 0.0f;
		tz_ = 0.0f;
		scale_ = 1.0f;
		//pitch_ = 0.0f;
		//yaw_ = 0.0f;
		//roll_ = 0.0f;
		x_to_y_ = false;
		y_to_x_ = false;
		z_to_y_ = false;
		y_to_z_ = false;
		mirror_x_ = false;
		mirror_y_ = false;
		glm::vec3 size; // , center;
		size = mesh.GetSize();
		center = mesh.GetCenter();
		std::cout << "The mesh size is " << size.x << " x " << size.y << " x " << size.z << std::endl;
		std::cout << "The mesh center is " << center.x << ", " << center.y << ", " << center.z << std::endl;
		std::cout << "The number of triangles is " << mesh.GetNumFaces() << std::endl;
		
		mavs::raytracer::embree::EmbreeTracer scene;
		
		glm::mat3x4 rot_scale;
		rot_scale[0][0] = 1.0f; rot_scale[0][1] = 0.0f; rot_scale[0][2] = 0.0f;
		rot_scale[1][0] = 0.0f; rot_scale[1][1] = 1.0f; rot_scale[1][2] = 0.0f;
		rot_scale[2][0] = 0.0f; rot_scale[2][1] = 0.0f; rot_scale[2][2] = 1.0f;
		rot_scale[0][3] = 0.0f; rot_scale[1][3] = 0.0f; rot_scale[2][3] = 0.0f;

		int mesh_id;
		scene.AddMesh(mesh, rot_scale, 0, 0,mesh_id);

		scene.CommitScene();
		env.SetRaytracer(&scene);
		env.SetDateTime(2023, 2, 13, 3, 28, 45, 6);
		
		float front_dist = std::max(0.5f*size.x,std::max(size.y, size.z));
		front_camera.Initialize(npix, npix, front_dist, front_dist, 0.0035f);
		front_camera.SetBackgroundColor(119, 136, 153);
		front_pixdim = front_dist / ((float)npix);
		glm::vec3 front_camera_pos(center.x + front_dist, center.y, center.z);
		glm::quat front_camera_orient(0.0f, 0.0f, 0.0f, -1.0f);
		front_camera.SetPose(front_camera_pos, front_camera_orient);
		front_camera.SetName("Front");

		//mavs::sensor::camera::OrthoCamera side_camera;
		float side_dist = std::max(0.5f*size.y,std::max(size.x, size.z));
		side_camera.Initialize(npix, npix, side_dist, side_dist, 0.0035f);
		side_camera.SetBackgroundColor(119, 136, 153);
		side_pixdim = side_dist / ((float)npix);
		glm::vec3 side_camera_pos(center.x, center.y + side_dist, center.z);
		glm::quat side_camera_orient(0.7071f, 0.0f, 0.0f, -0.7071f);
		side_camera.SetPose(side_camera_pos, side_camera_orient);
		side_camera.SetName("Side");

		//mavs::sensor::camera::OrthoCamera top_camera;
		float top_dist = std::max(0.5f*size.z,std::max(size.x, size.y));
		top_camera.Initialize(npix, npix, top_dist, top_dist, 0.0035f);
		top_camera.SetBackgroundColor(119, 136, 153);
		top_pixdim = top_dist / ((float)npix);
		glm::vec3 top_camera_pos(center.x, center.y, center.z + top_dist);
		glm::quat top_camera_orient(0.7071f, 0.0f, 0.7071f, 0.0f);
		top_camera.SetPose(top_camera_pos, top_camera_orient);
		top_camera.SetName("Top");
		
		front_camera.Update(&env, 0.1f);
		front_image = front_camera.GetCurrentImage();
		
		side_camera.Update(&env, 0.1f);
		side_image = side_camera.GetCurrentImage();

		top_camera.Update(&env, 0.1f);
		top_image = top_camera.GetCurrentImage();

		std::string xdim_str = mavs::utils::ToString(size.x);
		std::string ydim_str = mavs::utils::ToString(size.y);
		std::string zdim_str = mavs::utils::ToString(size.z);
		std::string trans_str = "Press X, Y, or Z to translate the mesh.";
		std::string rot_str = "Press the arrow keys to rotate the mesh";
		std::string scale_str = "Press S to scale the mesh";
		std::string save_str = "Press D to save, Q to exit";
		std::string point_str = "Press P to get coordinate of a point";
		std::string meas_str = "Press M to measure the distance between two points";
		std::string rend_str = "Press R to render current view to file";
		std::string mirr_str = "Press I,J to mirror around X,Y";

		float green[] = { 0.0f,255.0f,0.0f };
		float yellow[] = { 255.0f,255.0f,0.0f };
		float red[] = { 255.0f,0.0f,0.0f };

		float ylow = center.y - np2 * front_pixdim;
		float zlow = center.z - np2 * front_pixdim;
		int y_size = (int)(size.y / front_pixdim);
		int y_start = (int)(0.5f*(npix - y_size));
		int z_size = (int)(size.z / front_pixdim);
		int z_start = (int)(0.5f*(npix - z_size));
		int y_0 = (int)((0.0f-ylow) / front_pixdim);
		int z_0 = npix - (int)((0.0f-zlow) / front_pixdim);
		front_image.draw_arrow(10, z_start + z_size, 10, z_start, yellow, 1.0f, 30, 10);
		front_image.draw_arrow( y_start, 10, y_start + y_size, 10, yellow,1.0f, 30, 10);
		front_image.draw_text(np2, 15, ydim_str.c_str(), yellow);
		front_image.draw_text(15, np2, zdim_str.c_str(), yellow);
		front_image.draw_text(y_start+y_size-10, 15, "Y", yellow);
		front_image.draw_text(15, z_start, "Z", yellow);
		if (y_0 >= 0 && z_0 >= 0 && y_0 < npix && z_0 < npix) {
			front_image.draw_circle(y_0, z_0, 5, green, 1.0f);
		}
		front_image.draw_text(15, npix - 120, mirr_str.c_str(), yellow);
		front_image.draw_text(15, npix - 105, rend_str.c_str(), yellow);
		front_image.draw_text(15, npix - 90, point_str.c_str(), yellow);
		front_image.draw_text(15, npix - 75, meas_str.c_str(), yellow);
		front_image.draw_text(15, npix - 60, save_str.c_str(), yellow);
		front_image.draw_text(15, npix - 45, trans_str.c_str(), yellow);
		front_image.draw_text(15, npix - 30, rot_str.c_str(), yellow);
		front_image.draw_text(15, npix - 15, scale_str.c_str(), yellow);

		float xlow = center.x - np2 * side_pixdim;
		zlow = center.z - np2 * side_pixdim;
		int x_size = (int)(size.x / side_pixdim);
		int x_start = (int)(0.5f*(npix - x_size));
		z_size = (int)(size.z / side_pixdim);
		z_start = (int)(0.5f*(npix - z_size));
		int x_0 = npix - (int)((0.0f-xlow) / side_pixdim);
		z_0 = npix - (int)((0.0f-zlow) / side_pixdim);
		side_image.draw_arrow(10, z_start + z_size, 10, z_start,  yellow, 1.0f, 30, 10);
		side_image.draw_arrow(x_start + x_size, 10, x_start, 10, yellow, 1.0f, 30, 10);
		side_image.draw_text(np2, 15, xdim_str.c_str(), yellow);
		side_image.draw_text(15, np2, zdim_str.c_str(), yellow);
		side_image.draw_text(x_start+x_size-10, 15, "X", yellow);
		side_image.draw_text(15, z_start, "Z", yellow);
		if (x_0 >= 0 && z_0 >= 0 && x_0 < npix && z_0 < npix) {
			side_image.draw_circle(x_0, z_0, 5, green, 1.0f);
		}
		side_image.draw_text(15, npix - 120, mirr_str.c_str(), yellow);
		side_image.draw_text(15, npix - 105, rend_str.c_str(), yellow);
		side_image.draw_text(15, npix - 90, point_str.c_str(), yellow);
		side_image.draw_text(15, npix - 75, meas_str.c_str(), yellow);
		side_image.draw_text(15, npix - 60, save_str.c_str(), yellow);
		side_image.draw_text(15, npix - 45, trans_str.c_str(), yellow);
		side_image.draw_text(15, npix - 30, rot_str.c_str(), yellow);
		side_image.draw_text(15, npix - 15, scale_str.c_str(), yellow);

		xlow = center.x - np2 * top_pixdim;
		ylow = center.y - np2 * top_pixdim;
		x_size = (int)(size.x / top_pixdim);
		x_start = (int)(0.5f*(npix - x_size));
		y_size = (int)(size.y / top_pixdim);
		y_start = (int)(0.5f*(npix - y_size));
		x_0 = npix - (int)((0.0f-xlow) / top_pixdim);
		y_0 = npix - (int)((0.0f-ylow) / top_pixdim);
		top_image.draw_arrow(10, x_start + x_size, 10, x_start,  yellow, 1.0f, 30, 10);
		top_image.draw_arrow(y_start + y_size, 10, y_start, 10,  yellow, 1.0f, 30, 10);
		top_image.draw_text(np2, 15, ydim_str.c_str(), yellow);
		top_image.draw_text(15, np2, xdim_str.c_str(), yellow);
		//top_image.draw_text(y_start+y_size-10, 15, "Y", yellow);
		//top_image.draw_text(15, x_start, "X", yellow);
		top_image.draw_text(y_start + y_size - 10, 15, "X", yellow);
		top_image.draw_text(15, x_start, "Y", yellow);
		if (x_0 >= 0 && y_0 >= 0 && x_0 < npix && y_0 < npix) {
			top_image.draw_circle(y_0, x_0, 5, green, 1.0f);
		}
		top_image.draw_text(15, npix - 120, mirr_str.c_str(), yellow);
		top_image.draw_text(15, npix - 105, rend_str.c_str(), yellow);
		top_image.draw_text(15, npix - 90, point_str.c_str(), yellow);
		top_image.draw_text(15, npix - 75, meas_str.c_str(), yellow);
		top_image.draw_text(15, npix - 60, save_str.c_str(), yellow);
		top_image.draw_text(15, npix - 45, trans_str.c_str(), yellow);
		top_image.draw_text(15, npix - 30, rot_str.c_str(), yellow);
		top_image.draw_text(15, npix - 15, scale_str.c_str(), yellow);

		front_disp.display(front_image);
		side_disp.display(side_image);
		top_disp.display(top_image);
		bool pressed = false;
		double dt = 0.1;
		while (!pressed){
#ifdef USE_OMP
			double t1 = omp_get_wtime();
#endif
			pressed = GetKeyboardInput();
			front_disp.display(front_image);
			side_disp.display(side_image);
			top_disp.display(top_image);
#ifdef USE_OMP
			double dwall = omp_get_wtime() - t1;
			if (dwall < (double)dt) {
				int msleep = (int)(1000 * (dt - dwall));
				mavs::utils::sleep_milliseconds(msleep);
			}
#endif
		}

		mesh.Scale(scale_, scale_, scale_);
		glm::vec3 t(tx_, ty_, tz_);
		mesh.Translate(t);

		if (x_to_y_) {
			mesh.RotateXToY();
		}
		if (y_to_x_) {
			mesh.RotateYToX();
		}
		if (z_to_y_) {
			mesh.RotateZToY();
		}
		if (y_to_z_) {
			mesh.RotateYToZ();
		}
		if (mirror_x_) {
			mesh.MirrorX();
		}
		if (mirror_y_) {
			mesh.MirrorY();
		}
		if (increment_rotation_) {
			glm::mat3x4 aff_rot;
			for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) { aff_rot[i][j] = scale_keep * rot_keep[i][j]; } }
			mesh.ApplyAffineTransformation(aff_rot);
			increment_rotation_ = false;
		}
		
		std::cout << std::endl;
		if (save_) {
			std::string ofname = mavs::io::GetSaveFileName("Select the save file name", "file.obj", "*.obj");
			//glm::mat3x4 aff_rot;
			//for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++){aff_rot[i][j] = scale_keep * rot_keep[i][j]; } }
			//aff_rot[0][3] = trans_keep.x; aff_rot[1][3] = trans_keep.y; aff_rot[2][3] = trans_keep.z;
			//mesh.LoadTransformAndSave(fname, ofname, aff_rot);
			mesh.Write(ofname);
			break;
		}
		if (exit_) {
			break;
		}
	
	} // editing loop

	return 0;
}

