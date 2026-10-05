// c++ includes
#include <iostream>
#include <fstream>
// project includes
#include "vehicles/tracked/tracked_vehicle_params.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

namespace mavs {
namespace vehicle {
namespace tracked {

void TrackedVehicleParams::ParseJsonObject(const rapidjson::Value& vehicle) {
    if (vehicle.HasMember("Mass") && vehicle["Mass"].IsNumber()) {
        mass = vehicle["Mass"].GetDouble();
    }
    if (vehicle.HasMember("CG Height") && vehicle["CG Height"].IsNumber()) {
        cg_height = vehicle["CG Height"].GetDouble();
    }
    if (vehicle.HasMember("Tread") && vehicle["Tread"].IsNumber()) {
        tread_width = vehicle["Tread"].GetDouble();
    }
    if (vehicle.HasMember("Track Width") && vehicle["Track Width"].IsNumber()) {
        track_width = vehicle["Track Width"].GetDouble();
    }
    if (vehicle.HasMember("Track Contact Length") && vehicle["Track Contact Length"].IsNumber()) {
        track_contact_length = vehicle["Track Contact Length"].GetDouble();
    }
    if (vehicle.HasMember("Sprocket Radius") && vehicle["Sprocket Radius"].IsNumber()) {
        sprocket_radius = vehicle["Sprocket Radius"].GetDouble();
    }
    if (vehicle.HasMember("CG Lateral Offset") && vehicle["CG Lateral Offset"].IsNumber()) {
        cg_lateral_offset = vehicle["CG Lateral Offset"].GetDouble();
    }
    if (vehicle.HasMember("CG Longitudinal Offset") && vehicle["CG Longitudinal Offset"].IsNumber()) {
        cg_longitudinal_offset = vehicle["CG Longitudinal Offset"].GetDouble();
    }
    if (vehicle.HasMember("Sprocket Inertia") && vehicle["Sprocket Inertia"].IsNumber()) {
        sprocket_inertia = vehicle["Sprocket Inertia"].GetDouble();
    }
    if (vehicle.HasMember("Internal Friction") && vehicle["Internal Friction"].IsNumber()) {
        internal_friction = vehicle["Internal Friction"].GetDouble();
    }
    if (vehicle.HasMember("Internal Viscous") && vehicle["Internal Viscous"].IsNumber()) {
        internal_viscous = vehicle["Internal Viscous"].GetDouble();
    }
    if (vehicle.HasMember("Max Sprocket Speed") && vehicle["Max Sprocket Speed"].IsNumber()) {
        max_sprocket_speed = vehicle["Max Sprocket Speed"].GetDouble();
    }
}

void TrackedVehicleParams::Load(std::string input_file) {

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
    if (d.HasMember("Vehicle") && d["Vehicle"].IsObject()) {
        const rapidjson::Value& vehicle = d["Vehicle"];

        ParseJsonObject(vehicle);
    }
    else {
        std::cerr << "WARNING: No entry for \"Vehicle\" in soil input file." << std::endl;
    }
}

glm::dvec3 TrackedVehicleParams::InertiaDiag() const {
    const double H = 2 * cg_height;
    double B = tread_width;
    double l = track_contact_length;

    glm::dvec3 inertia(0.0, 0.0, 0.0);
    inertia.x = (i_xx >= 0.0) ? i_xx : mass * (B * B + H * H) / 12.0;
    inertia.y = (i_yy >= 0.0) ? i_yy : mass * (l * l + H * H) / 12.0;
    inertia.z = (i_zz >= 0.0) ? i_zz : mass * (l * l + B * B) / 12.0;
    return inertia;
}

}  // namespace tracked
} // namespace vehicle
} // namespace mavs
