/**
* \class MapViewer
*
* Class that creates a map view and updates trajectories and positions
*
* \author Chris Goodin
*
* \date 9/6/2024
*/

#ifndef MAVS_MAP_VIEWER_H_
#define MAVS_MAP_VIEWER_H_
// mavs includes
#include <sensors/camera/ortho_camera.h>

namespace mavs {

namespace utils {

const glm::ivec3 magenta(255, 0, 255);
const glm::ivec3 cyan(0, 255, 255);
const glm::ivec3 yellow(255, 255, 0);
const glm::ivec3 orange(255, 165, 0);
const glm::ivec3 red(255, 0, 0);
const glm::ivec3 green(0, 255, 0);
const glm::ivec3 blue(0, 0, 255);
const glm::ivec3 white(255, 255, 255);
const glm::ivec3 grey(128, 128, 128);

struct PlotFeature {
	std::vector<glm::vec2> points;
	glm::ivec3 color;
	bool close_loop;
};

struct PlotPoint {
	glm::vec2 position;
	glm::ivec3 color;
};

class MapViewer {
public:
	MapViewer();

	MapViewer(glm::vec2 llc, glm::vec2 urc, float pixdim);

	~MapViewer();

	void Init(glm::vec2 llc, glm::vec2 urc, float pixdim);

	void UpdateMap(mavs::environment::Environment *env);

	void AddWaypoints(std::vector<glm::vec2> wp, glm::ivec3 col);

	void AddWaypoints(std::vector<glm::vec2> wp, glm::ivec3 col, bool close_loop);

	void AddPoint(glm::vec2 point, glm::ivec3 col);

	void AddCircle(glm::vec2 center, float radius, glm::ivec3 col);

	void AddLine(glm::vec2 p0, glm::vec2 p1, glm::ivec3 col);

	bool IsOpen();

private:
	glm::ivec2 EnuToPix(glm::vec2 p);
	void UpdateWaypoints();
	void PlotWaypoints(std::vector<glm::vec2> wp, glm::ivec3 col, bool close_loop);
	mavs::sensor::camera::OrthoCamera ortho_cam_;
	glm::vec2 llc_;
	glm::vec2 urc_;
	float pixdim_;
	int im_h_;
	int im_w_;
	std::vector<PlotFeature> features_;
	std::vector<PlotPoint> points_;
	bool displayed_;
}; // class MapViewer
} // namespace visualization
} //namespace mavs_uav

#endif // include guard