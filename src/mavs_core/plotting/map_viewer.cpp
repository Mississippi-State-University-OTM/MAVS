// class definition
#include "mavs_core/plotting/map_viewer.h"
// c++ includes
#include <math.h>

namespace mavs {
namespace utils {

MapViewer::MapViewer() {
	glm::vec2 llc(-256.0f, -256.0f);
	glm::vec2 urc(256.0f, 256.0f);
	float pixdim = 1.0f;
	Init(llc, urc, pixdim);
}

MapViewer::MapViewer(glm::vec2 llc, glm::vec2 urc, float pixdim) {
	Init(llc, urc, pixdim);
}

MapViewer::~MapViewer() {
	ortho_cam_.CloseDisplay();
}

void MapViewer::Init(glm::vec2 llc, glm::vec2 urc, float pixdim) {
	float xdim = urc.x - llc.x;
	float ydim = urc.y - llc.y;
	int nx = (int)ceil(xdim / pixdim);
	int ny = (int)ceil(ydim / pixdim);
	ortho_cam_.Initialize(nx, ny, xdim, ydim, pixdim);
	ortho_cam_.SetName("Map Viewer");
	glm::vec2 center = 0.5f*(urc + llc);
	ortho_cam_.SetPose(glm::vec3(center.x, center.y, 10000.0f), glm::quat(0.7071f, 0.0f, 0.7071f, 0.0f));
	llc_ = llc;
	urc_ = urc;
	pixdim_ = pixdim;
	im_h_ = ortho_cam_.GetCImg()->height();
	im_w_ = ortho_cam_.GetCImg()->width();
	displayed_ = false;
}

bool MapViewer::IsOpen() {
	if (!displayed_) return true;
	return ortho_cam_.DisplayOpen();
}

void MapViewer::UpdateMap(mavs::environment::Environment *env) {
	ortho_cam_.Update(env, 1.0f);
	UpdateWaypoints();
	ortho_cam_.GetCImg()->mirror("y");
	ortho_cam_.Display();
	features_.clear();
	points_.clear();
	displayed_ = true;
}

glm::ivec2 MapViewer::EnuToPix(glm::vec2 p) {
	glm::ivec2 pix;
	pix.x = std::min(im_h_ - 1, std::max(0, (int)floor((p.x - llc_.x) / pixdim_)));
	pix.y = std::min(im_w_ - 1, std::max(0, (int)floor((p.y - llc_.y) / pixdim_)));
	return pix;
}

void MapViewer::AddLine(glm::vec2 p0, glm::vec2 p1, glm::ivec3 col) {
	PlotFeature feature;
	feature.points.push_back(p0);
	feature.points.push_back(p1);
	feature.color = col;
	feature.close_loop = false;
	features_.push_back(feature);
}

void MapViewer::AddWaypoints(std::vector<glm::vec2> wp, glm::ivec3 col) {
	AddWaypoints(wp, col, false);
}

void MapViewer::AddWaypoints(std::vector<glm::vec2> wp, glm::ivec3 col, bool close_loop) {
	PlotFeature feature;
	feature.points = wp;
	feature.color = col;
	feature.close_loop = close_loop;
	features_.push_back(feature);
}

void MapViewer::UpdateWaypoints() {
	ortho_cam_.GetCImg()->rotate(90);
	ortho_cam_.GetCImg()->mirror("y");
	for (int i = 0; i < (int)features_.size(); i++) {
		PlotWaypoints(features_[i].points, features_[i].color, features_[i].close_loop);
	}
	for (int i = 0; i < (int)points_.size(); i++) {
		glm::ivec2 idx = EnuToPix(points_[i].position);
		ortho_cam_.GetCImg()->draw_circle(idx.x, idx.y, 3, (int*)&points_[i].color);
	}
}

void MapViewer::AddPoint(glm::vec2 point, glm::ivec3 col) {
	PlotPoint p;
	p.position = point;
	p.color = col;
	points_.push_back(p);
}

void MapViewer::AddCircle(glm::vec2 center, float radius, glm::ivec3 col) {
	std::vector<glm::vec2> circle;
	float nstep = 100.0f;
	float theta = 0.0f;
	float two_pi = 2.0f*acosf(-1.0f);
	float tstep = two_pi / nstep;
	while (theta < two_pi) {
		glm::vec2 p = center + radius*glm::vec2(cosf(theta), sinf(theta));
		circle.push_back(p);
		theta += tstep;
	}
	AddWaypoints(circle, col, true);
}


static void DrawThickLine(cimg_library::CImg<float> *image, const int x1, const int y1, const int x2, const int y2, const int* const color, const int line_width, const float opacity = 1.0f){

	//taken from 2nd answer on this page:
	//https://stackoverflow.com/questions/5673448/can-the-cimg-library-draw-thick-lines

	if (x1 == x2 && y1 == y2) {
		return;
	}
	// Convert line (p1, p2) to polygon (pa, pb, pc, pd)
	const float x_diff = (float)(x1 - x2);
	const float y_diff = (float)(y1 - y2);
	const float w_diff = (float)line_width / 2.0f;

	// Triangle between pa and p1: x_adj^2 + y_adj^2 = w_diff^2
	// Triangle between p1 and p2: x_diff^2 + y_diff^2 = length^2 
	// Similar triangles: y_adj / x_diff = x_adj / y_diff = w_diff / length
	// -> y_adj / x_diff = w_diff / sqrt(x_diff^2 + y_diff^2) 
	const int x_adj = (int)(y_diff * w_diff / sqrtf(powf(x_diff, 2) + powf(y_diff, 2)));
	const int y_adj = (int)(x_diff * w_diff / sqrtf(powf(x_diff, 2) + powf(y_diff, 2)));

	// Points are listed in clockwise order, starting from top-left
	cimg_library::CImg<int> points(4, 2);
	points(0, 0) = x1 - x_adj;
	points(0, 1) = y1 + y_adj;
	points(1, 0) = x1 + x_adj;
	points(1, 1) = y1 - y_adj;
	points(2, 0) = x2 + x_adj;
	points(2, 1) = y2 - y_adj;
	points(3, 0) = x2 - x_adj;
	points(3, 1) = y2 + y_adj;

	image->draw_polygon(points, color, opacity);
}


void MapViewer::PlotWaypoints(std::vector<glm::vec2> wp, glm::ivec3 col, bool close_loop) {
	for (int i = 0; i < (int)wp.size(); i++) {
		glm::ivec2 idx0 = EnuToPix(wp[i]);
		if (i < (int)wp.size() - 1) {
			glm::ivec2 idx1 = EnuToPix(wp[i + 1]);
			DrawThickLine(ortho_cam_.GetCImg(), idx0.x, idx0.y, idx1.x, idx1.y, (int *)&col, 3);
		}
		else if (close_loop){
			glm::ivec2 idx1 = EnuToPix(wp[0]);
			DrawThickLine(ortho_cam_.GetCImg(), idx0.x, idx0.y, idx1.x, idx1.y, (int *)&col, 3);
		}
	}
}

} //namespace visualization
} //namespace mavs_uav
