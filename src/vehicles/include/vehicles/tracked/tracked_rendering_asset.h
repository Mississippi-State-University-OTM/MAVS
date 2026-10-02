#ifndef TRACKED_RENDERING_ASSET_H
#define TRACKED_RENDERING_ASSET_H
// debug rendering for the tracked vehicle class

// c++ includes
#include <string>
// project includes
#include "glm/glm.hpp"
#include "rapidjson/document.h"

namespace mavs {
namespace vehicle {
namespace tracked {

struct RenderingAsset {
    std::string mesh_file = "NULL";
    glm::vec3 offset = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 scale = glm::vec3(1.0f, 1.0f, 1.0f);
    bool rotate_y_to_x = false;
    bool rotate_x_to_y = false;;
    bool rotate_y_to_z = false;

    void RenderingAsset::ParseJsonObject(const rapidjson::Value& asset);
};

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif