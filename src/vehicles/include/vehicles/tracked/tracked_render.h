#ifndef TRACKED_RENDER_H
#define TRACKED_RENDER_H
// debug rendering for the tracked vehicle class

// project includes
#include "tracked_heightmap_terrain.h"
#include "tracked_helpers.h"
#include "tracked_sim_options.h"
#include "tracked_vehicle_params.h"
#include <CImg.h>

namespace mavs {
namespace vehicle {
namespace tracked {

class TrackedRender {
public:

    TrackedRender() {}
    
    void Init(HeightMapTerrain* tracked_terrain_in);

    bool DisplayOpen() const { return !debug_display_.is_closed(); }

    // ---- 3D visualization (software z-buffer renderer drawn into a CImg window)
    // Opens the window; after this, Step() redraws it at ~30 Hz of simulated time.
    void Enable3DDisplay(int width = 960, int height = 540);
    bool Display3DOpen() const { return !display3d_.is_closed(); }
    // Camera pose in world coordinates. yaw about world z (0 = looking down +x),
    // pitch positive looking up. Default: (-10, 0, 2), yaw = pitch = 0.
    void SetCameraPose(const glm::dvec3& position, double yaw_radians, double pitch_radians) {
        cam_pos_ = position; cam_yaw_ = yaw_radians; cam_pitch_ = pitch_radians;
    }
    const glm::dvec3& GetCameraPosition() const { return cam_pos_; }

    void UpdateDebugDisplay(glm::dvec3 pos, glm::dmat3 R, TrackedVehicleParams vp, std::array<glm::dvec3, 2> track_center, double sinkage);

    void Update3DDisplay(glm::dvec3 pos, glm::dmat3 R, TrackedVehicleParams vp, std::array<glm::dvec3, 2> track_center, double sinkage, std::array<std::vector<glm::dvec3>, 2> r_el, double du, double dw);

    TrackSpeeds GetKeyboardDrivingCommand(TrackSpeeds cmd);

    void Update(glm::dvec3 pos, glm::dmat3 R, TrackedVehicleParams vp, std::array<glm::dvec3, 2> track_center, double sinkage, std::array<std::vector<glm::dvec3>, 2> r_el, double du, double dw);

private:

    HeightMapTerrain* tracked_terrain_;

    // visualization windows
    cimg_library::CImgDisplay debug_display_;
    cimg_library::CImg<float> debug_image_;
    cimg_library::CImg<float> debug_terrain_base_;   // cached grayscale heightmap (terrain is static)
    
    glm::dvec2 DebugPixelToWorld(int px, int py) const;
    glm::ivec2 DebugWorldToPixel(const glm::dvec3& w) const;

    // 3D view
    cimg_library::CImgDisplay display3d_;
    cimg_library::CImg<float> image3d_;
    std::vector<float> zbuf3d_;                      // stores 1/z per pixel, 0 = empty
    glm::dvec3 cam_pos_ = glm::dvec3(-10.0, 0.0, 2.0);
    double cam_yaw_ = 0.0;
    double cam_pitch_ = 0.0;
    double cam_hfov_ = 1.0471975512;                 // 60 deg horizontal field of view

    // cached terrain mesh (decimated heightmap grid): world positions and grey level
    std::vector<glm::dvec3> terrain3d_pos_;
    std::vector<double> terrain3d_gray_;
    int terrain3d_nx_ = 0, terrain3d_ny_ = 0;
    void BuildTerrainMesh3D();
    
};

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif