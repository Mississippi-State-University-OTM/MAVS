#ifndef TRACKED_MATH_UTILS_H
#define TRACKED_MATH_UTILS_H

// c++ includes
#include <string>
#include <fstream>
#include <vector>
// project includes
#include "glm/glm.hpp"
#include "tracked_helpers.h"

namespace mavs {
namespace vehicle {
namespace tracked {

glm::dmat3 RotExp(const glm::dvec3& w);            // exp([w]x), Rodrigues

glm::dmat3 Orthonormalise(const glm::dmat3& R);     // polar factor (== U V^T of the SVD)

glm::dmat3 RFromRpy(double roll, double pitch, double yaw);   // ZYX, body -> world

void RpyFromR(const glm::dmat3& R, double& roll, double& pitch, double& yaw);

template <class F>
inline double MeanWhere(const std::vector<LogRow>& log, F cond, double LogRow::* field) {
    double s = 0; int n = 0;
    for (const LogRow& r : log) if (cond(r)) { s += r.*field; ++n; }
    return n ? s / n : 0.0;
}

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif