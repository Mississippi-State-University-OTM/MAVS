// project includes
#include "vehicles/tracked/tracked_rendering_asset.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

namespace mavs {
namespace vehicle {
namespace tracked {

void RenderingAsset::ParseJsonObject(const rapidjson::Value& asset){
    
    mesh_file = asset["File"].GetString();
    rotate_y_to_z = asset["Rotate Y to Z"].GetBool();
    rotate_y_to_x = asset["Rotate Y to X"].GetBool();
    rotate_x_to_y = asset["Rotate X to Y"].GetBool();
    for (int ij = 0; ij < 3; ij++) {
        offset[ij] = asset["Offset"][ij].GetFloat();
        scale[ij] = asset["Scale"][ij].GetFloat();
    }
    
}

}  // namespace tracked
} // namespace vehicle
} // namespace mavs