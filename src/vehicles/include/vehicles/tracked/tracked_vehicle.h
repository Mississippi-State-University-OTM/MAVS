#ifndef TRACKED_VEHICLE_H
#define TRACKED_VEHICLE_H
// tracked_vehicle.h
// =====================
// Time-stepped 6-DOF tracked-vehicle dynamics on deformable, non-flat terrain.
// C++17 port of tracked_dynamics.py (same physics, same numerics), using GLM
// (https://github.com/g-truc/glm, header-only) for vector/matrix math.
//
// Physics summary (see the Python module docstring for details):
//   * each track is a rigid plate of ny x nx contact elements;
//   * normal pressure: Bekker p = (kc/b + kphi) z^n on loading, linear elastic
//     unloading from the maximum sinkage stored in a world-fixed rut map;
//   * shear: per-element shear-displacement vectors advected with the belt,
//     rotated with the hull and grown by the sliding velocity (transient form
//     of Al-Milli et al. 2010, Eqs. 14-18); Janosi-Hanamoto stress (Eq. 22);
//   * compaction resistance: Bekker Rc at the leading edge, measured from the
//     plastic sinkage already present ahead of the track (multi-pass aware);
//   * bulldozing (Wong) on the leading face and on the side walls, per row.
//
// Frames: world z up. Body frame at the CG: x forward, y left, z up.
// Track 0 = LEFT (y = +B/2), track 1 = RIGHT (y = -B/2). SI units.
//
// Integration point for a host simulator: subclass td::Terrain and implement
// height() and extent(); the rut layer is managed by the base class.

// c++ includes
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
// mavs includes
#include <vehicles/vehicle.h>
#include "glm/glm.hpp"
#include "vehicles/tracked/tracked_soil.h"
#include "vehicles/tracked/tracked_heightmap_terrain.h"
#include "vehicles/tracked/tracked_helpers.h"
#include "vehicles/tracked/tracked_vehicle_params.h"
#include "vehicles/tracked/tracked_sim_options.h"
#include "vehicles/tracked/tracked_track_path.h"
#include "vehicles/tracked/tracked_rendering_asset.h"

namespace mavs {
namespace vehicle {
namespace tracked {

class TrackedRender;

class TrackedVehicle : public Vehicle {
public:

    TrackedVehicle(std::string input_file);

    void SetPose(double x, double y, double yaw_radians);

    void Update(environment::Environment* env, float throttle, float steer, float brake, float dt);

    void SetInitialPose(double init_x, double init_y, double init_yaw) { initial_position_x_ = init_x; initial_position_y_ = init_y; initial_yaw_ = init_yaw; }

    const TrackSpeeds GetSprocketSpeeds() const { return sprocket_; }

    double GetElapsedTime() const { return elapsed_time_; }

    double GetSinkage() const { return z_static_estimate_; }
    SimOptions& GetSimOptions() { return sim_options_; }

    double GetGravity() const { return sim_options_.gravity; }
    void SetGravity(double grav_in) { sim_options_.gravity = grav_in; }

    HeightMapTerrain& GetTerrain() { return terrain_; }

    void SetTerrain(HeightMapTerrain terrain_in) { terrain_ = terrain_in; }

    void SetSimulationMaxDt(double dt) { sim_options_.max_dt = dt; }
    double GetSimulationMaxDt() const { return sim_options_.max_dt; }

    double GetDu()const { return du_; }
    double GetDw()const { return dw_; }

    TrackedVehicleParams& GetVehicle() { return vehicle_params_; }

    TrackedSoil& GetSoil() { return soil_; }

    const std::array<glm::dvec3, 2>& GetTrackCenter() const { return track_center_; }

    const std::array<std::vector<glm::dvec3>, 2>& GetTrackElements() const { return r_el_; }

    // ---- moving terrain window
    // Called after the terrain origin moves; must refill the heights for the new window
    // (terrain.SetHeights). Ruts are already shifted when it runs.
    using TerrainRefresh = std::function<void(HeightMapTerrain& terrain)>;
    // At the start of each Step(), if the vehicle is more than recenter_distance [m] from the
    // window centre (in x or y), the window is recentred on it. Keep recenter_distance well
    // under half the window size minus the vehicle's footprint, so the tracks never leave it.
    void EnableMovingTerrain(double recenter_distance, TerrainRefresh refresh);
    void DisableMovingTerrain() { moving_terrain_ = false; }
    // Recentre now. The shift is a whole number of height cells, so existing heights stay
    // grid-aligned with the new window. Calls the refresh function if one is set.
    void RecenterTerrain();

    // ---- track shoe animation (visual only; see tracked_track_path.h for frames)
    // Replace the belt layout. Can be called at any time; the belt phase is kept.
    void SetTrackLayout(const TrackLayout& layout);
    const TrackLayout& GetTrackLayout() const { return track_layout_; }
    int GetNumShoesPerTrack() const { return num_shoes_; }
    double GetShoePitch() const { return shoe_pitch_; }
    double GetTrackPathLength() const { return track_path_.Length(); }
    // Belt travel [m] along the path since start, wrapped to [0, path length).
    double GetTrackPhase(int k) const { return track_phase_[k]; }
    // Spin angle [rad] about body +y of a non-slipping wheel of the given radius on track k
    // (use it to spin sprocket/idler/road-wheel meshes in sync with the shoes).
    double GetWheelSpinAngle(int k, double radius) const { return track_phase_[k] / radius; }
    // Poses of all shoes: left track first (indices 0..n-1), then right (n..2n-1).
    // world_frame = false returns them in the body frame (relative to GetPosition()/GetRotationMatrix()).
    void GetTrackShoePoses(std::vector<TrackShoePose>& out, bool world_frame = true) const;
    std::vector<TrackShoePose> GetTrackShoePoses(bool world_frame = true) const;

    glm::dmat3 GetRotationMatrix() const { return R_; }

private:
    // cmd = (left, right): sprocket speeds [rad/s] (Speed) or torques [N m] (Torque).
    void Step(double dt, TrackSpeeds cmd);

    void UpdateSim(double dt, TrackSpeeds cmd);

    void Settle(double duration = 1.5, double dt = 1e-3);

    TrackDiag TrackForces(int k, double dt, glm::dvec3& F, glm::dvec3& M);

    void Advect(std::vector<std::array<double, 2>>& j, double shift);

    double BulldozePerWidth(double z) const;

    size_t Idx(int i, int c) const { return static_cast<size_t>(i) * sim_options_.nx + c; }

    void Init();

    TrackedVehicleParams vehicle_params_; 
    TrackedSoil soil_; 
    HeightMapTerrain terrain_;
    SimOptions sim_options_; 

    double time_since_last_terrain_refresh_ = 0.0;
    double initial_position_x_ = 0.0;
    double initial_position_y_ = 0.0;
    double initial_yaw_ = 0.0;

    glm::dvec3 I_ = glm::dvec3(0.0);
    glm::dvec3 Iinv_ = glm::dvec3(0.0);
    double du_ = 0.0;
    double dw_ = 0.0;
    double A_ = 0.0;
    std::vector<double> u_;
    std::array<glm::dvec3, 2> track_center_{ glm::dvec3(0.0), glm::dvec3(0.0) };
    std::array<std::vector<glm::dvec3>, 2> r_el_;
    double kb_ = 0.0;
    double Kc_ = 0.0;
    double Kg_ = 0.0;
    double c_area_ = 0.0;
    double z_static_estimate_ = 0.0;

    // state
    glm::dvec3 p_ = glm::dvec3(0.0), vel_ = glm::dvec3(0.0), omega_ = glm::dvec3(0.0);
    glm::dmat3 R_ = glm::dmat3(1.0);

    TrackSpeeds sprocket_;
    std::array<std::vector<std::array<double, 2>>, 2> j_;
    double g_scale_ = 1.0;
    double elapsed_time_ = 0.0;

    // scratch buffers (avoid per-step allocation)
    std::vector<double> z_, zp_, pn_;
    std::vector<glm::dvec3> vb_;
    std::vector<HeightMapTerrain::RutIndex> ri_;
    std::vector<char> loading_, contact_;
    std::vector<std::array<double, 2>> jtmp_;

    double max_track_speed_ = 15.0;

    // moving terrain window
    bool moving_terrain_ = false;
    double recenter_distance_ = 0.0;
    TerrainRefresh terrain_refresh_;

    void SetMavsParams();
    RenderingAsset vehicle_asset_;
    RenderingAsset track_asset_;
    void UpdateMavsAnimations(environment::Environment* env);
    bool vehicle_loaded_ = false;
    std::vector<int> actor_ids_;
    std::vector<int> track_pad_ids_;

    void InitAnimation(environment::Environment* env);

    void UpdateTerrain(environment::Environment* env, float dt);
    void ResetTerrain(environment::Environment* env);

    TrackSpeeds GetSprocketSpeedsFromTsb(double throttle, double steer, double brake);

    // track shoe animation
    void BuildTrackPath();
    TrackLayout track_layout_;
    TrackPath track_path_;
    int num_shoes_ = 0;
    double shoe_pitch_ = 0.0;
    std::array<double, 2> track_phase_{ 0.0, 0.0 };
};

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif
