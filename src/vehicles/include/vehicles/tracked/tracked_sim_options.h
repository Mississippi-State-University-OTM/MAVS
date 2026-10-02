#ifndef TRACKED_SIM_OPTIONS_H
#define TRACKED_SIM_OPTIONS_H

// project includes
#include "rapidjson/document.h"
// c++ includes
#include <string>

namespace mavs {
namespace vehicle {
namespace tracked {

enum class ShearDirection { Displacement, Velocity };
enum class DriveMode { Speed, Torque };

struct SimOptions {
    int nx = 5;
    int ny = 40;
    bool compaction = true;
    bool bulldozing = true;
    double extra_rolling_coeff = 0.0;
    ShearDirection shear_direction = ShearDirection::Displacement;
    DriveMode drive = DriveMode::Speed;
    double v_eps = 0.05;
    double normal_damping_ratio = 0.2;
    double rut_dx = 0.0;   // <= 0 -> 1.2 * max(element spacing)
    double gravity = 9.806;
    double max_dt = 2.0E-3;

    void Load(std::string input_file);

    void ParseJsonObject(const rapidjson::Value& vehicle);
};

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif