#ifndef TRACKED_HELPERS_H
#define TRACKED_HELPERS_H

namespace mavs {
namespace vehicle {
namespace tracked {

struct TrackSpeeds {
    double left = 0.0;
    double right = 0.0;
    TrackSpeeds() { left = 0.0; right = 0.0; }
    TrackSpeeds(double init_l, double init_r) { left = init_l; right = init_r; }
    double& operator[](std::size_t i) {
        assert(i < 2);
        return i == 0 ? left : right;
    }

    const double& operator[](std::size_t i) const {
        assert(i < 2);
        return i == 0 ? left : right;
    }
};

// ---------------------------------------------------------------- outputs
struct TrackDiag {
    double thrust = 0;          // soil force on belt along body x [N]
    double N = 0;               // normal load [N]
    double sinkage = 0;         // mean sinkage over the element grid [m]
    double contact_frac = 0;
    double F_compaction = 0;    // magnitude [N]
    double F_bulldoze_x = 0;
    double F_bulldoze_y = 0;
    double belt_speed = 0;      // r * omega [m/s]
    double vx_track = 0;        // track-centre ground speed along body x [m/s]
    int outside_map = 0;        // elements outside the rut map
};

}  // namespace tracked
} // namespace vehicle
} // namespace mavs

#endif