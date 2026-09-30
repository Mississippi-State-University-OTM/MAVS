#define _USE_MATH_DEFINES // Must be defined BEFORE including cmath
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <fstream>
#include <iostream>
// project includes
#include "vehicles/tracked/tracked_soil.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

namespace mavs {
namespace vehicle {
namespace tracked {


double TrackedSoil::TanPhi() const { 
    return std::tan(phi_deg * M_PI / 180.0); 
}

void TrackedSoil::BulldozingFactors(double& Kc, double& Kg) const {
    const double phi = phi_deg * M_PI / 180.0;
    const double t = std::tan(phi);
    const double Nq = std::exp(M_PI * t) * std::pow(std::tan(M_PI / 4 + phi / 2), 2);
    const double Nc = t > 0 ? (Nq - 1) / t : 5.14;
    const double Ng = 2 * (Nq + 1) * t;
    const double c2 = std::cos(phi) * std::cos(phi);
    Kc = (Nc - t) * c2;
    Kg = t > 0 ? (2 * Ng / t + 1) * c2 : 0.0;
}

void TrackedSoil::ParseJsonObject(const rapidjson::Value& soil) {
    if (soil.HasMember("c") && soil["c"].IsNumber()) {
        c = soil["c"].GetDouble();
    }
    if (soil.HasMember("phi_deg") && soil["phi_deg"].IsNumber()) {
        phi_deg = soil["phi_deg"].GetDouble();
    }
    if (soil.HasMember("K") && soil["K"].IsNumber()) {
        K = soil["K"].GetDouble();
    }
    if (soil.HasMember("n") && soil["n"].IsNumber()) {
        n = soil["n"].GetDouble();
    }
    if (soil.HasMember("kc") && soil["kc"].IsNumber()) {
        kc = soil["kc"].GetDouble();
    }
    if (soil.HasMember("kphi") && soil["kphi"].IsNumber()) {
        kphi = soil["kphi"].GetDouble();
    }
    if (soil.HasMember("gamma") && soil["gamma"].IsNumber()) {
        gamma = soil["gamma"].GetDouble();
    }
    if (soil.HasMember("rebound") && soil["rebound"].IsNumber()) {
        rebound = soil["rebound"].GetDouble();
    }
}

void TrackedSoil::Load(std::string input_file) {

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

    if (d.HasMember("Soil") && d["Soil"].IsObject()) {
        const rapidjson::Value& soil = d["Soil"];

        ParseJsonObject(soil);
    }
    else {
        std::cerr << "WARNING: No entry for \"Soil\" in soil input file." << std::endl;
    }
}

}  // namespace tracked
} // namespace vehicle
} // namespace mavs
