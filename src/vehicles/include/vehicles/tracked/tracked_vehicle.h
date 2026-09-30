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

namespace mavs {
namespace vehicle {
namespace tracked {

class TrackedVehicle : public Vehicle {
public:

    using Controller = std::function<TrackSpeeds(double t, const TrackedVehicle&)>;

    TrackedVehicle(std::string input_file);

    TrackedVehicle(const TrackedVehicleParams& vehicle, const TrackedSoil& soil, HeightMapTerrain& terrain, const SimOptions& options = SimOptions());

    void SetPose(double x, double y, double yaw_radians);

    // cmd = (left, right): sprocket speeds [rad/s] (Speed) or torques [N m] (Torque).
    void Step(double dt, TrackSpeeds cmd);

    void Update(environment::Environment* env, float throttle, float steer, float brake, float dt);

    void Settle(double duration = 1.5, double dt = 1e-3);
    
    std::vector<LogRow> Run();

    VehicleState GetCurrentVehicleState() const;

    const SimulationState& GetCurrentSimulationState() const { return current_simulation_state_; }

    // raw state access (for coupling to a host simulator)
    const glm::dvec3& GetPosition() const { return p_; }
    const glm::dmat3& GetRotationMatrix() const { return R_; }
    const glm::dvec3& GetVelocityWorld() const { return vel_; }
    const glm::dvec3& GetOmegaBody() const { return omega_; }

    const TrackSpeeds SprocketSpeeds() const { return sprocket_; }

    double GetTime() const { return elapsed_time_; }
    double GetSinkage() const { return z_static_estimate_; }
    SimOptions& GetSimOptions() { return sim_options_; }

    double GetGravity() const { return sim_options_.gravity; }
    void SetGravity(double grav_in) { sim_options_.gravity = grav_in; }

    HeightMapTerrain& GetTerrain() { return terrain_; }

    void SetTerrain(HeightMapTerrain terrain_in) { terrain_ = terrain_in; }

    void SetSimulationDuration(double duration) { sim_options_.simulation_duration = duration; }
    double GetSimulationDuration() const { return sim_options_.simulation_duration; }

    void SetSimulationDt(double dt) { sim_options_.dt = dt; }
    double GetSimulationDt() const { return sim_options_.dt; }

    void SetController(Controller controller_in) { controller_ = controller_in; }

    void SetLogStepFrequency(int log_every_in) { sim_options_.log_every = log_every_in; }
    int GetLogStopFrequency() const { return sim_options_.log_every; }

    double GetDu()const { return du_; }
    double GetDw()const { return dw_; }

    TrackedVehicleParams& GetVehicle() { return vehicle_params_; }

    TrackedSoil& GetSoil() { return soil_; }

    double GetElapsedTime()const { return elapsed_time_; }

    const std::array<glm::dvec3, 2>& GetTrackCenter() const { return track_center_; }

    const std::array<std::vector<glm::dvec3>, 2>& GetTrackElements() const { return r_el_; }

private:
    TrackDiag TrackForces(int k, double dt, glm::dvec3& F, glm::dvec3& M);

    void Advect(std::vector<std::array<double, 2>>& j, double shift);

    double BulldozePerWidth(double z) const;

    size_t Idx(int i, int c) const { return static_cast<size_t>(i) * sim_options_.nx + c; }

    void Init();

    Controller controller_;

    TrackedVehicleParams vehicle_params_; 
    TrackedSoil soil_; 
    HeightMapTerrain terrain_;
    SimOptions sim_options_; 

    //double m_;
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
    SimulationState current_simulation_state_;

    // scratch buffers (avoid per-step allocation)
    std::vector<double> z_, zp_, pn_;
    std::vector<glm::dvec3> vb_;
    std::vector<HeightMapTerrain::RutIndex> ri_;
    std::vector<char> loading_, contact_;
    std::vector<std::array<double, 2>> jtmp_;

    double max_track_speed_ = 15.0;

    void SetMavsParams();
};

namespace controller {

inline TrackedVehicle::Controller Ramp(double wl, double wr, double tr = 2.0) {
    return [=](double t, const TrackedVehicle&) {
        const double a = std::min(t / tr, 1.0);
        TrackSpeeds cmd_track_speed;
        cmd_track_speed.left = wl * a;
        cmd_track_speed.right = wr * a;
        return cmd_track_speed;
        };
}

inline TrackedVehicle::Controller RampThenTurn(
    double w,
    double t_turn = 9.0,
    double left_scale = 0.65,
    double right_scale = 1.1,
    double tr = 2.0)
{
    return [=](double t, const TrackedVehicle&) {
        TrackSpeeds cmd;
        if (t < t_turn) {
            const double a = std::min(t / tr, 1.0);
            cmd.left = w * a;
            cmd.right = w * a;
        }
        else {
            cmd.left = left_scale * w;
            cmd.right = right_scale * w;
        }
        return cmd;
        };
}

inline TrackedVehicle::Controller ConstantTorque(double wl, double wr) {
	return [=](double, const TrackedVehicle&) {
		return TrackSpeeds{ wl, wr };
		};
}

} // namespace controller

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif