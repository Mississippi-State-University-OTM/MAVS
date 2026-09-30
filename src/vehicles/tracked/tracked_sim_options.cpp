// project includes
#include "vehicles/tracked/tracked_sim_options.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
// c++ includes
#include <iostream>
#include <fstream>

namespace mavs {
namespace vehicle {
namespace tracked {

void SimOptions::Load(std::string input_file) {
    // Open safely using standard C++ streams (ios::binary matches "rb")
    std::ifstream ifs(input_file, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file: " << input_file << std::endl;
        return;
    }

    // Wrap the C++ stream and parse
    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document d;
    d.ParseStream(isw);

    // Get the data members
    if (d.HasMember("Sim Options") && d["Sim Options"].IsObject()) {
        const rapidjson::Value& sim_options = d["Sim Options"];

        ParseJsonObject(sim_options);
    }
    else {
        std::cerr << "WARNING: No field for \"Sim Options\" in simulation input file." << std::endl;
    }
}

void SimOptions::ParseJsonObject(const rapidjson::Value& sim_options) {
    if (sim_options.HasMember("nx") && sim_options["nx"].IsNumber()) {
        nx = sim_options["nx"].GetInt();
    }
    if (sim_options.HasMember("ny") && sim_options["ny"].IsNumber()) {
        nx = sim_options["ny"].GetInt();
    }
    if (sim_options.HasMember("log_every") && sim_options["log_every"].IsNumber()) {
        log_every = sim_options["log_every"].GetInt();
    }
    if (sim_options.HasMember("bulldozing") && sim_options["bulldozing"].IsBool()) {
        bulldozing = sim_options["bulldozing"].GetBool();
    }
    if (sim_options.HasMember("compaction") && sim_options["compaction"].IsBool()) {
        compaction = sim_options["compaction"].GetBool();
    }
    if (sim_options.HasMember("display_debug") && sim_options["display_debug"].IsBool()) {
        display_debug = sim_options["display_debug"].GetBool();
    }
    if (sim_options.HasMember("render_3d") && sim_options["render_3d"].IsBool()) {
        render_3d = sim_options["render_3d"].GetBool();
    }
    if (sim_options.HasMember("v_eps") && sim_options["v_eps"].IsNumber()) {
        v_eps = sim_options["v_eps"].GetDouble();
    }
    if (sim_options.HasMember("gravity") && sim_options["gravity"].IsNumber()) {
        gravity = sim_options["gravity"].GetDouble();
    }
    if (sim_options.HasMember("initial_pose_x") && sim_options["initial_pose_x"].IsNumber()) {
        initial_position_x = sim_options["initial_pose_x"].GetDouble();
    }
    if (sim_options.HasMember("initial_pose_y") && sim_options["initial_pose_y"].IsNumber()) {
        initial_position_y = sim_options["initial_pose_y"].GetDouble();
    }
    if (sim_options.HasMember("initial_pose_yaw") && sim_options["initial_pose_yaw"].IsNumber()) {
        initial_yaw= sim_options["initial_pose_yaw"].GetDouble();
    }
    if (sim_options.HasMember("simulation_duration") && sim_options["simulation_duration"].IsNumber()) {
        simulation_duration = sim_options["simulation_duration"].GetDouble();
    }
    if (sim_options.HasMember("dt") && sim_options["dt"].IsNumber()) {
        dt = sim_options["dt"].GetDouble();
    }
    if (sim_options.HasMember("rut_dx") && sim_options["rut_dx"].IsNumber()) {
        rut_dx = sim_options["rut_dx"].GetDouble();
    }
    if (sim_options.HasMember("normal_damping_ratio") && sim_options["normal_damping_ratio"].IsNumber()) {
        normal_damping_ratio = sim_options["normal_damping_ratio"].GetDouble();
    }
    if (sim_options.HasMember("extra_rolling_coeff") && sim_options["extra_rolling_coeff"].IsNumber()) {
        extra_rolling_coeff = sim_options["extra_rolling_coeff"].GetDouble();
    }
    if (sim_options.HasMember("shear_direction") && sim_options["shear_direction"].IsString()) {
        std::string shear_direction_in = sim_options["shear_direction"].GetString();
        if (shear_direction_in == "Displacement") {
            shear_direction = ShearDirection::Displacement;
        }
        else if (shear_direction_in=="Velocity") {
            shear_direction = ShearDirection::Velocity;
        }
        else {
            std::cerr << "shear_direction " << shear_direction_in << " not recognized, must be \"Displacement\" or \"Velocity\"." << std::endl;
        }
    }
    if (sim_options.HasMember("drive_mode") && sim_options["drive_mode"].IsString()) {
        std::string drive_mode_in = sim_options["drive_mode"].GetString();
        if (drive_mode_in == "Speed") {
            drive = DriveMode::Speed;
        }
        else if (drive_mode_in == "Torque") {
            drive = DriveMode::Torque;
        }
        else {
            std::cerr << "drive_mode " << drive_mode_in << " not recognized, must be \"Speed\" or \"Torque\"." << std::endl;
        }
    }
}

}  // namespace tracked
} // namespace vehicle
} // namespace mavs
