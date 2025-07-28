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
#include <mavs_core/plotting/mavs_plotting.h>
#include <iostream>
#include <limits>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace utils {

Mplot::Mplot() {
	display_assigned_ = false;
	image_assigned_ = false;
	traj_np_ = 512;
	title_ = "mavs_plot";
	max_trajectory_width_ = std::numeric_limits<float>::max();
	traj_xmin_ = 0.0f;
	traj_ymin_ = 0.0f;
	traj_dp_x_ = 1.0f;
	traj_dp_y_ = 1.0f;
	plot_axis_ = false;
}

Mplot::~Mplot() {

}

Mplot::Mplot(const Mplot &plot) {
	display_assigned_ = plot.display_assigned_;
	image_assigned_ = plot.image_assigned_;
	traj_np_ = plot.traj_np_;
	title_ = plot.title_;
	max_trajectory_width_ = plot.max_trajectory_width_;
	traj_xmin_ = plot.traj_xmin_;
	traj_ymin_ = plot.traj_ymin_;
	traj_dp_x_ = plot.traj_dp_x_;
	traj_dp_y_ = plot.traj_dp_y_;
	image_ = plot.image_;
	plot_axis_ = plot.plot_axis_;
}

Color Mplot::MorelandColormap(float x, float xmin, float xmax){
  // Diverging Color Maps for Scientific Visualization
  // Kenneth Moreland
	x = (x - xmin)/(xmax-xmin);
	if (x < 0.0f) x = 0.0f;
	if (x > 1.0f) x = 1.0f;
  float x2 = x*x;
  float x3 = x2*x;
  float x4 = x3*x;
  float x5 = x4*x;
  float x6 = x5*x;
  Color color;
  color.b = -971.97f*x6 + 1841.3f*x5 - 225.53f*x4 - 804.89f*x3 - 387.13f*x2 + 393.62f*x + 192.0f;
  color.r = 226.07f*x6 + 100.66f*x5 - 883.07f*x4 + 242.79f*x3 + 146.89f*x2 + 288.16f*x + 59.0f;
  color.g = -6584.7f*x6 + 18406.0f*x5 - 18432.0f*x4 + 7588.7f*x3 - 1589.2f*x2 + 541.05f*x + 76.0f;
  return color;
}

std::vector<std::vector<std::vector<float> > > Mplot::UnFlatten3D(int width, int height, int depth, float *data){
	std::vector<std::vector<std::vector<float> > > vec = Allocate3DVector(width,height,depth,0.0f);
	int n = 0;
	for (int i=0;i<width;i++){
		for (int j=0;j<height;j++){
			for (int k=0;k<depth;k++){
				vec[i][j][k] = data[n];
				n++;
			}
		}
	}
	return vec;
}

std::vector<std::vector<float> > Mplot::UnFlatten2D(int width, int height, float *data){
	std::vector<std::vector<float> > vec = Allocate2DVector(width,height,0.0f);
		int n = 0;;
	for (int i=0;i<width;i++){
		for (int j=0;j<height;j++){
			vec[i][j] = data[n];
			n++;
		}
	}
	return vec;
}

void Mplot::PlotColorMap(std::vector<std::vector<std::vector<float> > > &data){
	int width  = (int)data.size();
	int height = 0;
	if (width>0){
		height = (int)data[0].size();
	}
	else {
		std::cerr<<"ERROR: Could not plot matrix with width = 0 "<<std::endl;
		return;
	}
	int depth = 0;
	if (height>0){
		depth = (int)data[0][0].size();
	}
	else {
		std::cerr<<"ERROR: Could not plot matrix with height = 0"<<std::endl;
		return;
	}
	if (depth!=3){
		std::cerr<<"ERROR: Attempted to plot color matrix with "<<depth<<" channels"<<std::endl;
		return;
	}
	if (!image_assigned_){
		image_.assign(width,height,1,depth,0.0f);
		image_assigned_ = true;
	}
	if (image_.width()!=width || image_.height()!=height) {
		image_.assign(width,height,1,depth,0.0f);
		disp_.assign(width,height,title_.c_str());
	}
	for (int i=0;i<width;i++){
		for (int j= 0;j<height;j++){
			float color[3];
			color[0] = data[i][j][0];
			color[1] = data[i][j][1];
			color[2] = data[i][j][2];
			image_.draw_point(i,j,(float *)&color);
		}
	}
	if (!display_assigned_){
		disp_.assign(width,height,title_.c_str());
		display_assigned_ = true;
	}
	image_.mirror('y');
	disp_ = image_;
}

void Mplot::PlotScalarColorMap(std::vector<std::vector<float> > &data){
	int width  = (int)data.size();
	int height = 0;
	if (width>0){
		height = (int)data[0].size();
	}
	else {
		std::cerr<<"ERROR: Could not plot matrix with width = 0 "<<std::endl;
		return;
	}
	if (height<=0){
		std::cerr<<"ERROR: Could not plot matrix with height = 0"<<std::endl;
		return;
	}
	if (!image_assigned_){
		image_.assign(width,height,1,3,0.0f);
		image_assigned_ = true;
	}
	if (image_.width()!=width || image_.height()!=height) {
		image_.assign(width,height,1,3,0.0f);
		disp_.assign(width,height,title_.c_str());
	}
	float cmax = std::numeric_limits<float>::lowest();
	float cmin = std::numeric_limits<float>::max();
	for (int i=0;i<width;i++){
		for (int j= 0;j<height;j++){
			if (data[i][j]<cmin)cmin=data[i][j];
			if (data[i][j]>cmax)cmax=data[i][j];
		}
	}
	for (int i=0;i<width;i++){
		for (int j= 0;j<height;j++){
			Color color = MorelandColormap(data[i][j],cmin,cmax);
			image_.draw_point(i,j,(float *)&color);
		}
	}
	if (!display_assigned_){
		disp_.assign(width,height,title_.c_str());
		display_assigned_ = true;
	}
	image_.mirror('y');
	disp_ = image_;
}

void Mplot::PlotGrayMap(std::vector<std::vector<float> > &data){
	int width  = (int)data.size();
	int height = 0;
	if (width>0){
		height = (int)data[0].size();
	}
	else {
		std::cerr<<"ERROR: Could not plot matrix with width = 0 "<<std::endl;
		return;
	}
	if (height<=0){
		std::cerr<<"ERROR: Could not plot matrix with height = 0"<<std::endl;
		return;
	}
	if (!image_assigned_){
		image_.assign(width,height,1,3,0.0f);
		image_assigned_ = true;
	}
	if (image_.width()!=width || image_.height()!=height) {
		image_.assign(width,height,1,3,0.0f);
		disp_.assign(width,height,title_.c_str());
	}
	for (int i=0;i<width;i++){
		for (int j= 0;j<height;j++){
			float color[3];
			color[0] = data[i][j];
			color[1] = data[i][j];
			color[2] = data[i][j];
			image_.draw_point(i,j,(float *)&color);
		}
	}
	if (!display_assigned_){
		disp_.assign(width,height,title_.c_str());
		display_assigned_ = true;
	}
	image_.mirror('y');
	disp_ = image_;
}

void Mplot::AddToExistingTrajectory(float xp, float yp) {
	std::vector<float> x, y;
	x.push_back(xp);
	y.push_back(yp);
	AddToExistingTrajectory(x, y);
}

void Mplot::AddToExistingTrajectory(std::vector<float> x, std::vector<float> y) {
	if (x.size() != y.size() || x.size() <= 1 || y.size() <= 1 || traj_xmin_ == std::numeric_limits<float>::max())return;
	float color[3];
	color[0] = 255.0f;
	color[1] = 0.0f;
	color[2] = 0.0;
	for (int i = 0; i < (int)(x.size() - 1); i++) {
		int u0 = (int)((x[i] - traj_xmin_) / traj_dp_x_);
		int v0 = (int)((y[i] - traj_ymin_) / traj_dp_y_);
		int u1 = (int)((x[i + 1] - traj_xmin_) / traj_dp_x_);
		int v1 = (int)((y[i + 1] - traj_ymin_) / traj_dp_y_);
		if (u0 >= 0 && u0 < traj_np_ && v0 >= 0 && v0 < traj_np_ &&
			u1 >= 0 && u1 < traj_np_ && v1 >= 0 && v1 < traj_np_) {
			image_.draw_line(u0, traj_np_-v0-1, u1, traj_np_-v1-1, (float *)&color);
		}
	}
	//image_.mirror('y');
	disp_ = image_;
}

void Mplot::PlotCurveFit(std::vector<float> x, std::vector<float> y, std::vector<float> y_fit) {
	if (!image_assigned_) {
		image_.assign(traj_np_, traj_np_, 1, 3, 0.0f);
		image_assigned_ = true;
		disp_.assign(traj_np_, traj_np_, title_.c_str());
		display_assigned_ = true;
	}
	image_ = 0.0f;
	if (x.size() != y.size() || x.size() <= 1 || y.size() <= 1)return;
	traj_xmin_ = std::numeric_limits<float>::max();
	float xmax = std::numeric_limits<float>::lowest();
	traj_ymin_ = std::numeric_limits<float>::max();
	float ymax = std::numeric_limits<float>::lowest();
	for (int i = 0; i < (int)(x.size()); i++) {
		if (x[i] < traj_xmin_)traj_xmin_ = x[i];
		if (x[i] > xmax)xmax = x[i];
		if (y[i] < traj_ymin_)traj_ymin_ = y[i];
		if (y[i] > ymax)ymax = y[i];
	}
	float xr = xmax - traj_xmin_;
	float yr = ymax - traj_ymin_;
	traj_dp_x_ = std::max(xr, yr) / (float)traj_np_;
	traj_dp_y_ = traj_dp_x_;
	float white[3];
	white[0] = 255.0f;
	white[1] = 255.0f;
	white[2] = 255.0f;
	float red[3];
	red[0] = 255.0f;
	red[1] = 255.0f;
	red[2] = 255.0f;
	for (int i = 0; i < (int)(x.size() - 1); i++) {
		int u0 = (int)((x[i] - traj_xmin_) / traj_dp_x_);
		int v0 = (int)((y_fit[i] - traj_ymin_) / traj_dp_y_);
		int u1 = (int)((x[i + 1] - traj_xmin_) / traj_dp_x_);
		int v1 = (int)((y_fit[i + 1] - traj_ymin_) / traj_dp_y_);
		int vd = (int)((y[i] - traj_ymin_) / traj_dp_y_);
		image_.draw_line(u0, v0, u1, v1, (float*)&red);
		image_.draw_circle(u0, vd, 3, (float*)&white);
	}
	image_.mirror('y');
	disp_ = image_;
}

void Mplot::SetTrajectoryXSize(float x_max_size) {
	max_trajectory_width_ = x_max_size;
}

void Mplot::PlotTrajectory(std::vector<float> x, std::vector<float> y) {
	if (!image_assigned_) {
		image_.assign(traj_np_, traj_np_, 1, 3, 0.0f);
		image_assigned_ = true;
		disp_.assign(traj_np_, traj_np_, title_.c_str());
		display_assigned_ = true;
	}
	image_ = 0.0f;
	if (x.size() != y.size() || x.size() <= 1 || y.size() <= 1)return;
	traj_xmin_ = std::numeric_limits<float>::max();
	float xmax = std::numeric_limits<float>::lowest();
	traj_ymin_ = std::numeric_limits<float>::max();
	float ymax = std::numeric_limits<float>::lowest();
	for (int i = 0; i < (int)(x.size()); i++) {
		if (x[i] < traj_xmin_)traj_xmin_ = x[i];
		if (x[i] > xmax)xmax = x[i];
		if (y[i] < traj_ymin_)traj_ymin_ = y[i];
		if (y[i] > ymax)ymax = y[i];
	}
	float xrange = xmax - traj_xmin_;
	if (xrange > max_trajectory_width_)traj_xmin_ = xmax - max_trajectory_width_;

	float xr = xmax - traj_xmin_;
	float yr = ymax - traj_ymin_;
	//traj_dp_ = std::max(xr, yr) / (float)traj_np_;
	traj_dp_x_ = xr / (float)traj_np_;
	traj_dp_y_ = yr / (float)traj_np_;

	float axis_color[3];
	axis_color[0] = 0.0f;
	axis_color[1] = 255.0f;
	axis_color[2] = 0.0f;
	for (int i = 0; i < (int)(x.size() - 1); i++) {
		int u0 = (int)((x[i] - traj_xmin_) / traj_dp_x_);
		int v0 = (int)((0.0f - traj_ymin_) / traj_dp_y_);
		int u1 = (int)((x[i + 1] - traj_xmin_) / traj_dp_x_);
		int v1 = (int)((0.0f - traj_ymin_) / traj_dp_y_);
		image_.draw_line(u0, v0, u1, v1, (float*)&axis_color);
	}

	float color[3];
	color[0] = 255.0f;
	color[1] = 255.0f;
	color[2] = 255.0f;
	for (int i = 0; i < (int)(x.size()-1); i++) {
		int u0 = (int)((x[i] - traj_xmin_) / traj_dp_x_);
		int v0 = (int)((y[i] - traj_ymin_) / traj_dp_y_);
		int u1 = (int)((x[i+1] - traj_xmin_) / traj_dp_x_);
		int v1 = (int)((y[i+1] - traj_ymin_) / traj_dp_y_);
		image_.draw_line(u0, v0, u1, v1, (float *)&color);
	}
	image_.mirror('y');
	disp_ = image_;
}

void Mplot::SaveCurrentPlot(std::string filename) {
	if (image_.width() > 0 && image_.height() > 0) {
		image_.save(filename.c_str());
	}
}

} //namespace utils
} //namespace mavs