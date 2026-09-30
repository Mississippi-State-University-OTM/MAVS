// tracked_dynamics.cpp -- see tracked_dynamics.h
#include "vehicles/tracked/tracked_math_utils.h"
#include <algorithm>

namespace mavs {
namespace vehicle {
namespace tracked {

// ============================================================ math
// GLM is column-major: R[c][r] is row r, column c.  For a body->world
// rotation, column c is body axis c expressed in world coordinates.

glm::dmat3 RotExp(const glm::dvec3& w) {
    const double th = glm::length(w);
    if (th < 1e-12) return glm::dmat3(1.0);
    const glm::dvec3 k = w / th;
    const glm::dmat3 Kx(glm::dvec3(0.0, k.z, -k.y),      // column 0
                  glm::dvec3(-k.z, 0.0, k.x),      // column 1
                  glm::dvec3(k.y, -k.x, 0.0));     // column 2
    return glm::dmat3(1.0) + std::sin(th) * Kx + (1.0 - std::cos(th)) * (Kx * Kx);
}

glm::dmat3 Orthonormalise(const glm::dmat3& R) {
    // Newton iteration for the polar factor X <- (X + X^-T)/2; identical to
    // U V^T from the SVD, converges quadratically from a near-rotation.
    glm::dmat3 X = R;
    for (int it = 0; it < 6; ++it) {
        const glm::dmat3 Xn = 0.5 * (X + glm::transpose(glm::inverse(X)));
        double d = 0.0;
        for (int col = 0; col < 3; ++col)
            for (int row = 0; row < 3; ++row) d = std::max(d, std::abs(Xn[col][row] - X[col][row]));
        X = Xn;
        if (d < 1e-15) break;
    }
    return X;
}

glm::dmat3 RFromRpy(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch),
                 sp = std::sin(pitch), cy = std::cos(yaw), sy = std::sin(yaw);
    return glm::dmat3(glm::dvec3(cy * cp, sy * cp, -sp),                                     // column 0
                glm::dvec3(cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),   // column 1
                glm::dvec3(cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr));  // column 2
}

void RpyFromR(const glm::dmat3& R, double& roll, double& pitch, double& yaw) {
    // element (row r, col c) is R[c][r]
    yaw = std::atan2(R[0][1], R[0][0]);
    pitch = std::asin(-std::max(-1.0, std::min(1.0, R[0][2])));
    roll = std::atan2(R[1][2], R[2][2]);
}

}  // namespace tracked
} // namespace vehicle
} // namespace mavs