#ifndef TRACKED_TRACK_PATH_H
#define TRACKED_TRACK_PATH_H
// tracked_track_path.h
// =====================
// Kinematic belt path for animating track shoes. Purely visual: it does not
// feed back into the dynamics in tracked_vehicle.cpp.
//
// TRACK FRAME (one per track): origin at the track's contact-patch centre
// (TrackedVehicle::GetTrackCenter()[k]), axes parallel to the body frame,
// x forward, z up. The wheels live in the x-z plane of this frame, so z = 0
// is the soil contact plane used by the physics.
//
// The belt pitch line (pin centres) wraps the outside of the wheels: outer
// tangent lines between consecutive wheels plus arcs on each wheel. Arc length
// s increases in the direction the belt moves when driving forward: forward
// along the top run, down around the front, rearward along the ground run, up
// around the rear. List the wheels in order around the loop, starting anywhere and
// going either way round; the direction is fixed automatically. Every wheel must touch the outside of the loop (convex
// layout): sprocket, idler, road wheels on the ground line, return rollers on
// the top line. Track sag is not modelled.

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

namespace mavs {
namespace vehicle {
namespace tracked {

struct TrackWheel {
    double x = 0.0;       // [m] track frame, forward
    double z = 0.0;       // [m] track frame, up (z = radius puts the wheel on the ground plane)
    double radius = 0.0;  // [m] radius of the belt pitch line around this wheel
};

struct TrackLayout {
    // Empty -> default: sprocket-radius wheels at x = +/- track_contact_length/2, resting on z = 0.
    std::vector<TrackWheel> wheels;
    // Shoe count per track. If <= 0 it is computed from shoe_pitch (rounded so the loop closes).
    int num_shoes = 0;
    double shoe_pitch = 0.15;  // [m] nominal pin-to-pin spacing, used only when num_shoes <= 0
    // Shoe frame: origin at the midpoint between the shoe's two pins; on the ground run it is
    // aligned with the body (x forward, y left, z up = toward the wheels). shoe_offset moves the
    // mesh origin from that midpoint, in shoe-frame coordinates (e.g. z = +half thickness).
    glm::dvec3 shoe_offset = glm::dvec3(0.0);
    // Applied in the shoe frame, after shoe_offset, to align your mesh's axes with the shoe frame.
    glm::dquat shoe_mesh_rotation = glm::dquat(1.0, 0.0, 0.0, 0.0);
};

struct TrackShoePose {
    glm::dvec3 position = glm::dvec3(0.0);
    glm::dquat orientation = glm::dquat(1.0, 0.0, 0.0, 0.0);
    int track = 0;  // 0 = left, 1 = right
    int index = 0;  // shoe index along the belt
};

class TrackPath {
public:
    void Build(std::vector<TrackWheel> w) {
        seg_.clear();
        length_ = 0.0;
        const size_t n = w.size();
        if (n < 2) throw std::invalid_argument("TrackPath: need at least two wheels");
        for (size_t i = 0; i < n; ++i)
            if (!(w[i].radius > 0.0))
                throw std::invalid_argument("TrackPath: wheel " + std::to_string(i) + " needs radius > 0");

        // The construction below produces a loop that is clockwise in the (x right, z up) plane,
        // which is the forward-driving belt direction. That needs the centres in clockwise order.
        double area2 = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;
            area2 += w[i].x * w[j].z - w[j].x * w[i].z;
        }
        if (area2 > 0.0) std::reverse(w.begin(), w.end());

        // m[i]: outward unit normal of the outer tangent line from wheel i to wheel i+1
        std::vector<glm::dvec2> m(n);
        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;
            const glm::dvec2 d(w[j].x - w[i].x, w[j].z - w[i].z);
            const double D = glm::length(d);
            if (D <= std::abs(w[i].radius - w[j].radius) + 1e-9)
                throw std::invalid_argument("TrackPath: wheels " + std::to_string(i) + " and " +
                                            std::to_string(j) + " overlap; no outer tangent exists");
            const glm::dvec2 u = d / D;
            const glm::dvec2 left(-u.y, u.x);
            const double a = (w[i].radius - w[j].radius) / D;
            m[i] = a * u + std::sqrt(1.0 - a * a) * left;
        }

        // segments: line (wheel i -> i+1), then the arc wrapping wheel i+1
        const double two_pi = 2.0 * 3.14159265358979323846;
        double total_sweep = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;
            const glm::dvec2 ci(w[i].x, w[i].z), cj(w[j].x, w[j].z);

            Segment line;
            line.arc = false;
            line.a = ci + w[i].radius * m[i];
            line.b = cj + w[j].radius * m[i];
            line.len = glm::length(line.b - line.a);
            seg_.push_back(line);

            Segment arc;
            arc.arc = true;
            arc.c = cj;
            arc.r = w[j].radius;
            arc.phi0 = std::atan2(m[i].y, m[i].x);
            double sweep = std::fmod(arc.phi0 - std::atan2(m[j].y, m[j].x), two_pi);
            if (sweep < 0.0) sweep += two_pi;
            if (sweep > two_pi - 1e-6) sweep = 0.0;  // collinear wheels (e.g. road wheels)
            arc.len = arc.r * sweep;
            total_sweep += sweep;
            seg_.push_back(arc);
        }
        if (std::abs(total_sweep - two_pi) > 1e-3)
            throw std::invalid_argument(
                "TrackPath: wheels must be listed in order around the loop, and every wheel must touch the outside of the belt (convex layout)");

        for (Segment& s : seg_) {
            s.s0 = length_;
            length_ += s.len;
        }
    }

    double Length() const { return length_; }
    bool Empty() const { return seg_.empty(); }

    double Wrap(double s) const {
        if (length_ <= 0.0) return s;
        return s - length_ * std::floor(s / length_);
    }

    // Pitch-line point (x, z) in the track frame at arc length s (any value; wrapped).
    glm::dvec2 Point(double s) const {
        if (seg_.empty()) return glm::dvec2(0.0);
        s = Wrap(s);
        auto it = std::upper_bound(seg_.begin(), seg_.end(), s,
                                   [](double v, const Segment& g) { return v < g.s0; });
        const Segment& g = (it == seg_.begin()) ? seg_.front() : *(it - 1);
        const double t = std::min(std::max(s - g.s0, 0.0), g.len);
        if (!g.arc) return g.len > 0.0 ? g.a + (g.b - g.a) * (t / g.len) : g.a;
        const double phi = g.phi0 - t / g.r;  // clockwise
        return g.c + g.r * glm::dvec2(std::cos(phi), std::sin(phi));
    }

private:
    struct Segment {
        bool arc = false;
        double s0 = 0.0, len = 0.0;
        glm::dvec2 a = glm::dvec2(0.0), b = glm::dvec2(0.0);  // line end points
        glm::dvec2 c = glm::dvec2(0.0);                       // arc centre
        double r = 0.0, phi0 = 0.0;                           // arc radius, start angle
    };
    std::vector<Segment> seg_;
    double length_ = 0.0;
};

}  // namespace tracked
}  // namespace vehicle
}  // namespace mavs

#endif
