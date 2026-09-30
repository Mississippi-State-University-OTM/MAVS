#ifndef TRACKED_SOIL_H
#define TRACKED_SOIL_H
//project includes 
#include "rapidjson/document.h"
// c++ includes
#include <string>

namespace mavs {
namespace vehicle {
namespace tracked {

struct TrackedSoil {
    double c = 0.0;          // cohesion [Pa]
    double phi_deg = 0.0;    // internal friction angle [deg]
    double K = 0.025;      // shear deformation modulus [m]
    double n = 1.0;          // Bekker sinkage exponent
    double kc = 0.0;         // [N/m^(n+1)]
    double kphi = 0.0;       // [N/m^(n+2)]
    double gamma = 15e3;   // unit weight [N/m^3] (bulldozing)
    double rebound = 0.1;  // elastic rebound as a fraction of plastic sinkage

    double TanPhi() const; 

    void BulldozingFactors(double& Kc, double& Kg) const; 

    void Load(std::string input_file);

    void ParseJsonObject(const rapidjson::Value& soil);

};

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif