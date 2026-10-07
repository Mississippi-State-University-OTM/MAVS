#ifndef TRACKED_VEHICLE_PARAMS_H
#define TRACKED_VEHICLE_PARAMS_H
// tracked_vehicle_params.h
// =====================
//
// Frames: world z up. Body frame at the CG: x forward, y left, z up.
// Track 0 = LEFT (y = +B/2), track 1 = RIGHT (y = -B/2). SI units.

// C++ includes 
#include <string>
// project includes
#include "glm/glm.hpp"
#include "rapidjson/document.h"

namespace mavs {
namespace vehicle {
namespace tracked {

// vehicle inputs
struct TrackedVehicleParams {
    double mass = 0.0; // kg
    double cg_height = 0.0; // CG height above track bottom [m]
    double tread_width = 0.0; // tread [m]
    double track_contact_length = 0.0; // track contact length [m]
    double track_width = 0.0; // track width [m]
    double sprocket_radius = 0.0; // sprocket radius [m]
    double cg_lateral_offset = 0.0; // CG lateral offset (+ = left) [m]
    double cg_longitudinal_offset = 0.0; // CG longitudinal offset (+ = forward) from track centre [m]
    double i_xx = -1.0, i_yy = -1.0, i_zz = -1.0; // <0 -> box approximation
    double sprocket_inertia = 0.0; // per side, for torque drive [kg m^2]
    double internal_friction = 0.0; // Coulomb drivetrain torque per side [N m]
    double internal_viscous = 0.0; // viscous drivetrain loss per side [N m s/rad]
    double max_sprocket_speed = 25.0; // max rotational speed of the sprocket in rad/s
    double track_static_defl = 0.03; // deflection of the track under static normal load, meters
    double track_max_travel = 0.15; // max vertical displacement of a track element, meters
    double lateral_force_scale = 8.0; // lateral force empirical scaling constant, unitless, 1.0 = isotropic shear; >1 = more lateral grip

    glm::dvec3 InertiaDiag() const;

    void Load(std::string input_file);

    void ParseJsonObject(const rapidjson::Value& vehicle);
};

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif