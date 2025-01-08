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
#include <sensors/io/applanix_io.h>

#include <iostream>
#include <fstream>
#include <limits>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace io {

ApplanixHeightMap::ApplanixHeightMap() {

}


ApplanixHeightMap::ApplanixHeightMap(mavs::PointCloud &point_cloud) {
	float xmin = std::numeric_limits<float>::max();
	float xmax = std::numeric_limits<float>::lowest();
	float ymin = std::numeric_limits<float>::max();
	float ymax = std::numeric_limits<float>::lowest();
	for (int i = 0; i < (int)point_cloud.points.size(); i++) {
		if (point_cloud.points[i].x < xmin) xmin = point_cloud.points[i].x;
		if (point_cloud.points[i].x > xmax) xmax = point_cloud.points[i].x;
		if (point_cloud.points[i].y < ymin) ymin = point_cloud.points[i].y;
		if (point_cloud.points[i].y > ymax) ymax = point_cloud.points[i].y;
	}
	float dx = (xmax - xmin);
	float dy = (ymax - ymin);
	float area = dx * dy;
	float npoints = (float)point_cloud.points.size();
	float res = sqrt(area / npoints);
	int nx = (int)ceil(dx / res);
	int ny = (int)ceil(dy / res);

	mavs::terraingen::HeightMap hm_lo, hm_hi;
	float NODATA = -9999;
	hm_lo.SetCorners(xmin, ymin, xmax, ymax);
	hm_lo.SetResolution(res);
	hm_lo.Resize(nx, ny, NODATA);
	hm_hi.SetCorners(xmin, ymin, xmax, ymax);
	hm_hi.SetResolution(res);
	hm_hi.Resize(nx, ny, NODATA);
	for (int i = 0; i < (int)point_cloud.points.size(); i++) {
		float x = point_cloud.points[i].x;
		float y = point_cloud.points[i].y;
		float h_lo = hm_lo.GetHeightAtPoint(x, y);
		float h_hi = hm_hi.GetHeightAtPoint(x, y);
		float z = point_cloud.points[i].z;
		if (h_lo == NODATA || (z != NODATA && z < h_lo)) hm_lo.AddPoint(x, y, z);
		if (h_hi == NODATA || (z != NODATA && z > h_hi)) hm_hi.AddPoint(x, y, z);
	}
	hm_hi.ReplaceCellsByValue(NODATA);
	hm_lo.ReplaceCellsByValue(NODATA);
	cells_.clear();
	for (int i = 0; i < (int)hm_hi.GetHorizontalDim(); i++) {
		for (int j = 0; j < (int)hm_hi.GetVerticalDim(); j++) {
			applanix::vlr::PerceptionRayTracingCell cell;
			cell.x = xmin + res * i;
			cell.y = ymin + res * j;
			cell.type = 0;
			cell.height = hm_hi.GetCellHeight(i, j);
			cell.z_min = hm_lo.GetCellHeight(i, j);
			cells_.push_back(cell);
		}
	}
}

void ApplanixHeightMap::LoadHmf(std::string filename) {

	if (!mavs::utils::file_exists(filename)) {
		std::cerr << "Attempting to open .hmf file " << filename << " that does not exist" << std::endl;
	}

	std::ifstream infile;
	infile.open(filename.c_str(), std::ios::binary | std::ios::in);
	applanix::vlr::PerceptionRayTracedMap ray_traced_map;

	infile.read((char*)&ray_traced_map, sizeof(applanix::vlr::PerceptionRayTracedMap));

	cells_.resize(ray_traced_map.num_cells);

	for (int i = 0; i < ray_traced_map.num_cells; i++) {
		applanix::vlr::PerceptionRayTracingCell ray_cell;
		infile.read((char*)&ray_cell, sizeof(applanix::vlr::PerceptionRayTracingCell));
		cells_[i] = ray_cell;
	}

	applanix::dgc::ApplanixPose pose;
	infile.read((char *)&pose, sizeof(applanix::dgc::ApplanixPose));
	infile.close();

	pose_ = pose;
}

void ApplanixHeightMap::WriteHmf(std::string filename) {
	std::ofstream outfile;
	outfile.open(filename.c_str(), std::ios::binary | std::ios::out);
	applanix::vlr::PerceptionRayTracedMap ray_traced_map;
	ray_traced_map.num_cells = (int)cells_.size();
	outfile.write((char*)&ray_traced_map, sizeof(applanix::vlr::PerceptionRayTracedMap));

	for (int i = 0; i < (int)cells_.size(); i++) {
		outfile.write((char*)&cells_[i], sizeof(applanix::vlr::PerceptionRayTracingCell));
	}
	outfile.write((char *)&pose_, sizeof(applanix::dgc::ApplanixPose));
	outfile.close();
}

void ApplanixHeightMap::PrintStats() {
	std::cout << "Heightmap has " << cells_.size() << " cells" << std::endl;
	std::cout << "Pose = (" << pose_.latitude << "," << pose_.longitude << ")" << std::endl << std::endl;
}

mavs::terraingen::GridSurface ApplanixHeightMap::ConvertToSurface() {
	mavs::terraingen::HeightMap hm = ConvertToMavsHeightMap();
	mavs::terraingen::GridSurface surface(hm);
	return surface;
}

mavs::terraingen::HeightMap ApplanixHeightMap::ConvertToMavsHeightMap() {
	mavs::terraingen::HeightMap hm;

	//determine the limits and resolution of the hmf
	glm::vec2 ll(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	glm::vec2 ur(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
	float res = std::numeric_limits<float>::max();
	for (int i = 0; i < (int)cells_.size(); i++) {
		if (cells_[i].x > ur.x) ur.x = cells_[i].x;
		if (cells_[i].x < ll.x) ll.x = cells_[i].x;
		if (cells_[i].y > ur.y) ur.y = cells_[i].y;
		if (cells_[i].y < ll.y) ll.y = cells_[i].y;
		if (i > 0) {
			float dx = cells_[i + 1].x - cells_[i].x;
			float dy = cells_[i + 1].y - cells_[i].y;
			float dist = sqrt(dx*dx + dy * dy);
			if (dist < res)res = dist;
		}
	}

	// resize the heightmap
	float NODATA = -9999;
	hm.SetCorners(ll.x, ll.y, ur.x, ur.y);
	hm.SetResolution(res);
	glm::vec2 size = ur - ll;
	int nx = (int)(ceil(size.x / res));
	int ny = (int)(ceil(size.y / res));
	hm.Resize(nx, ny, NODATA);

	//fill the heightmap with data from the hmf
	for (int i = 0; i < (int)cells_.size(); i++) {
		hm.AddPoint(cells_[i].x, cells_[i].y, cells_[i].z_min);
	}

	//Fill any missing cells
	hm.ReplaceCellsByValue(NODATA);

	// Center at 0,0,0
	hm.Recenter();

	return hm;
}

} //namespace io
} //namespace mavs