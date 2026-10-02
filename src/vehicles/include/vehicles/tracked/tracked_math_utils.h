#ifndef TRACKED_MATH_UTILS_H
#define TRACKED_MATH_UTILS_H

// c++ includes
#include <vector>
// project includes
#include "glm/glm.hpp"

namespace mavs {
namespace vehicle {
namespace tracked {

glm::dmat3 RotExp(const glm::dvec3& w);            // exp([w]x), Rodrigues

glm::dmat3 Orthonormalise(const glm::dmat3& R);     // polar factor (== U V^T of the SVD)

glm::dmat3 RFromRpy(double roll, double pitch, double yaw);   // ZYX, body -> world

void RpyFromR(const glm::dmat3& R, double& roll, double& pitch, double& yaw);

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif