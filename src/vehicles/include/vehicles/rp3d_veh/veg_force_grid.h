#ifndef VEG_FORCE_GRID_H
#define VEG_FORCE_GRID_H
// MAVS includes
#include <glm/glm.hpp>
#include "mavs_core/math/utils.h"

namespace mavs {
namespace vehicle {

class VegForceGrid {
public:
	VegForceGrid() {
		nx_ = 0;
		ny_ = 0;
		res_ = 1.0f;
		llc_ = glm::vec2(0.0f, 0.0f);
	}

	void InitGrid(glm::vec2 llc_in, float res_in, glm::vec2 dimensions) {
		llc_ = llc_in;
		res_ = res_in;
		nx_ = (int)ceil(dimensions.x / res_);
		ny_ = (int)ceil(dimensions.y / res_);
		data_ = mavs::utils::Allocate2DVector(nx_, ny_, 0.0f);
	}

	glm::ivec2 PointToGrid(float x, float y) {
		glm::ivec2 c;
		c.x = (int)floor((x - llc_.x) / res_);
		c.y = (int)floor((y - llc_.y) / res_);
		return c;
	}

	void AddForceAtPoint(float x, float y, float force) {
		glm::ivec2 c = PointToGrid(x, y);
		int i = c.x; int j = c.y;
		if (i >= 0 && i < nx_ && j >= 0 && j < ny_) {
			data_[i][j] += force;
		}
	}

	/*float GetForceAtPoint(float x, float y) {
		glm::ivec2 c = PointToGrid(x, y);
		int i = c.x; int j = c.y;
		if (i >= 0 && i < nx_ && j >= 0 && j < ny_) {
			return data_[i][j];
		}
		else {
			return 0.0f;
		}
	}*/

	float GetForceAtPoint(float x, float y) {
		glm::ivec2 c = PointToGrid(x, y);
		int i = c.x; int j = c.y;
		if (!(i >= 0 && i < nx_ && j >= 0 && j < ny_)) {
			return 0.0f;
		}
		// Compute continuous grid coordinates relative to cell centers
		float gx = (x - llc_.x) / res_ - 0.5f;
		float gy = (y - llc_.y) / res_ - 0.5f;

		int i0 = (int)floor(gx);
		int j0 = (int)floor(gy);
		int i1 = i0 + 1;
		int j1 = j0 + 1;

		// Fractional parts
		float tx = gx - i0;
		float ty = gy - j0;

		// Clamp indices to valid grid range
		i0 = std::clamp(i0, 0, nx_ - 1);
		i1 = std::clamp(i1, 0, nx_ - 1);
		j0 = std::clamp(j0, 0, ny_ - 1);
		j1 = std::clamp(j1, 0, ny_ - 1);

		// Bilinear interpolation across the four neighboring cells
		float v00 = data_[i0][j0];
		float v10 = data_[i1][j0];
		float v01 = data_[i0][j1];
		float v11 = data_[i1][j1];

		float v0 = v00 + tx * (v10 - v00);  // interpolate along x, bottom
		float v1 = v01 + tx * (v11 - v01);  // interpolate along x, top
		return v0 + ty * (v1 - v0);          // interpolate along y
	}

private:
	int nx_;
	int ny_;
	float res_;
	glm::vec2 llc_;
	std::vector< std::vector<float> > data_;
}; // class VegForceGrid

class PlantObstacle {
public:
	PlantObstacle(){
		radius_ = 0.001f;
		r2_ = 0.00001f;
		force_ = 0.0f;
		//k_ = 0.0f;
		//intersection_depth_ = 0.0f;
		position_ = glm::vec2(0.0f, 0.0f);
	}
	
	void SetRadius(float r) {
		radius_ = r;
		r2_ = r * r;
		//k_ = 2.0f * force_ / radius_;
	}

	void SetForce(float f) {
		force_ = f;
		//k_ = 2.0f * force_ / radius_;
	}

	void SetPosition(float x, float y) {
		position_ = glm::vec2(x, y);
	}

	float GetForce() {
		return force_; 
	}

	bool GetVehicleIntersection(glm::mat2 veh_R, glm::vec2 veh_pos, float veh_len, float veh_width) {
		// first rotate the circle into the rectangle frame
		float xp = position_.x - veh_pos.x;
		float yp = position_.y - veh_pos.y;
		float cx = veh_R[0][0] * xp + veh_R[0][1] * yp;
		float cy = veh_R[1][0] * xp + veh_R[1][1] * yp;
		bool intersected = GetRectangleIntersection(cx, cy, -0.5f * veh_len, -0.5f * veh_width, 0.5f * veh_len, 0.5f * veh_width);
		return intersected;
	}

private:
	bool GetRectangleIntersection(float circle_x, float circle_y, float rect_x1, float rect_y1, float rect_x2, float rect_y2) {
		// rect_x1: x - coordinate of the rectangle's lower-left corner.
		// rect_y1 : y - coordinate of the rectangle's lower-left corner.
		// rect_x2 : x - coordinate of the rectangle's upper-right corner.
		// rect_y2 : y - coordinate of the rectangle's upper-right corner.
		// Find the closest point on the rectangle to the circle's center
		float closest_x = std::max(rect_x1, std::min(circle_x, rect_x2));
		float closest_y = std::max(rect_y1, std::min(circle_y, rect_y2));

		// Calculate the distance between the closest point and the circle's center
		float distance_x = circle_x - closest_x;
		float distance_y = circle_y - closest_y;
		float distance_squared = distance_x * distance_x + distance_y * distance_y;

		// Check if the distance is less than or equal to the circle's radius squared
		bool intersected = false;
		//float old_intersection_depth = intersection_depth_;
		//intersection_depth_ = 0.0f;
		if (distance_squared <= r2_) {
			intersected = true;
			//intersection_depth_ = radius_ - sqrtf(distance_squared);
		}
		return intersected;
	}

	//float intersection_depth_;
	float radius_;
	float r2_;
	float force_;
	//float k_;
	glm::vec2 position_;
};

} // namespace vehicle

} // namespace mavs

#endif