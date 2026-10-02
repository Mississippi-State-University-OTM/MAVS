// project includes
#include "vehicles/tracked/tracked_heightmap_terrain.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

#include "vehicles/tracked/tracked_math_utils.h"
// c++ includes
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace mavs {
namespace vehicle {
namespace tracked {

static double pi = 3.1415926535897932384626433832795;

void HeightMapTerrain::Load(std::string input_file) {
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
    if (d.HasMember("Terrain") && d["Terrain"].IsObject()) {
        const rapidjson::Value& terrain = d["Terrain"];

        ParseJsonObject(terrain);
    }
    else {
        std::cerr << "WARNING: No field for \"Terrain\" in simulation input file." << std::endl;
    }
}

void HeightMapTerrain::ParseJsonObject(const rapidjson::Value& terrain) {
    double xmin = -40.0;
    double xmax = 40.0;
    double ymin = -40.0;
    double ymax = 40.0;
    double res = 1.0;
    if (terrain.HasMember("resolution") && terrain["resolution"].IsNumber()) {
        res = terrain["resolution"].GetDouble();
    }
    if (terrain.HasMember("xmin") && terrain["xmin"].IsNumber()) {
        xmin = terrain["xmin"].GetDouble();
    }
    if (terrain.HasMember("xmax") && terrain["xmax"].IsNumber()) {
        xmax = terrain["xmax"].GetDouble();
    }
    if (terrain.HasMember("ymin") && terrain["ymin"].IsNumber()) {
        ymin = terrain["ymin"].GetDouble();
    }
    if (terrain.HasMember("ymax") && terrain["ymax"].IsNumber()) {
        ymax = terrain["ymax"].GetDouble();
    }

    *this = Flat(xmin, xmax, ymin, ymax, res, 0.0);
}


// ============================================================ construction
HeightMapTerrain::HeightMapTerrain()
    : x0_(-30.0), y0_(-30.0), dx_(60.0), nx_(2), ny_(2), h_(4, 0.0) {}

HeightMapTerrain::HeightMapTerrain(double x0, double y0, double dx, int nx, int ny, std::vector<double> heights) {
    x0_ = x0;
    y0_ = y0;
    dx_ = dx;
    nx_ = nx;
    ny_ = ny;
    h_ = heights;
    CheckDims();
    ++revision_;
}

void HeightMapTerrain::CheckDims() {
    if (nx_ < 2 || ny_ < 2) throw std::invalid_argument("height map needs >= 2x2 nodes");
    if (h_.size() != static_cast<size_t>(nx_) * ny_) throw std::invalid_argument("height map size");
}

HeightMapTerrain HeightMapTerrain::FromFunction(const std::function<double(double, double)>& f,
                                                 double xmin, double xmax, double ymin,
                                                 double ymax, double dx) {
    // same node layout as numpy.arange(min, max + dx/2, dx)
    const int nx = static_cast<int>(std::ceil(((xmax + 0.5 * dx) - xmin) / dx));
    const int ny = static_cast<int>(std::ceil(((ymax + 0.5 * dx) - ymin) / dx));
    std::vector<double> h(static_cast<size_t>(nx) * ny);
    for (int iy = 0; iy < ny; ++iy)
        for (int ix = 0; ix < nx; ++ix)
            h[static_cast<size_t>(iy) * nx + ix] = f(xmin + ix * dx, ymin + iy * dx);
    return HeightMapTerrain(xmin, ymin, dx, nx, ny, std::move(h));
}

HeightMapTerrain HeightMapTerrain::Flat(double xmin, double xmax, double ymin, double ymax,
                                        double dx, double z) {
    return FromFunction([z](double, double) { return z; }, xmin, xmax, ymin, ymax, dx);
}

// ============================================================ height map
double HeightMapTerrain::Height(double x, double y) const {
    const double fx = std::clamp((x - x0_) / dx_, 0.0, nx_ - 1 - 1e-9);
    const double fy = std::clamp((y - y0_) / dx_, 0.0, ny_ - 1 - 1e-9);
    const int ix = static_cast<int>(fx), iy = static_cast<int>(fy);
    const double tx = fx - ix, ty = fy - iy;
    auto H = [&](int j, int i) { return h_[static_cast<size_t>(j) * nx_ + i]; };
    return (1 - tx) * (1 - ty) * H(iy, ix) + tx * (1 - ty) * H(iy, ix + 1) +
           (1 - tx) * ty * H(iy + 1, ix) + tx * ty * H(iy + 1, ix + 1);
}

void HeightMapTerrain::Extent(double& xmin, double& xmax, double& ymin, double& ymax) const {
    xmin = x0_;
    xmax = x0_ + (nx_ - 1) * dx_;
    ymin = y0_;
    ymax = y0_ + (ny_ - 1) * dx_;
}

// ============================================================ rut map
void HeightMapTerrain::InitRut(double rdx) {
    if (!rut_.empty() && std::abs(rdx_ - rdx) < 1e-12) return;
    double xmin, xmax, ymin, ymax;
    Extent(xmin, xmax, ymin, ymax);
    rx0_ = xmin;
    ry0_ = ymin;
    rdx_ = rdx;
    rut_nx_ = static_cast<int>(std::ceil((xmax - xmin) / rdx)) + 1;
    rut_ny_ = static_cast<int>(std::ceil((ymax - ymin) / rdx)) + 1;
    rut_.assign(static_cast<size_t>(rut_nx_) * rut_ny_, 0.0);
    rut_base_x_ = rx0_;
    rut_base_y_ = ry0_;
    rut_off_x_ = 0;
    rut_off_y_ = 0;
}

void HeightMapTerrain::ResetRuts() { std::fill(rut_.begin(), rut_.end(), 0.0); }

// ============================================================ moving window
void HeightMapTerrain::SetOrigin(double x0, double y0) {
    x0_ = x0;
    y0_ = y0;
    ++revision_;
    if (rut_.empty()) return;

    // Snap the rut grid to the whole rut cell nearest the new origin, so cells stay world-fixed.
    const long long ox = std::llround((x0 - rut_base_x_) / rdx_);
    const long long oy = std::llround((y0 - rut_base_y_) / rdx_);
    const long long kx = ox - rut_off_x_;   // grid moves +kx cells: new(ix) = old(ix + kx)
    const long long ky = oy - rut_off_y_;
    if (kx == 0 && ky == 0) return;
    rut_off_x_ = ox;
    rut_off_y_ = oy;
    rx0_ = rut_base_x_ + static_cast<double>(ox) * rdx_;
    ry0_ = rut_base_y_ + static_cast<double>(oy) * rdx_;

    if (std::llabs(kx) >= rut_nx_ || std::llabs(ky) >= rut_ny_) {   // no overlap left
        ResetRuts();
        return;
    }
    rut_tmp_.assign(rut_.size(), 0.0);   // cells entering the window start at zero sinkage
    const int sx = static_cast<int>(kx), sy = static_cast<int>(ky);
    const int ix_lo = std::max(0, -sx), ix_hi = std::min(rut_nx_, rut_nx_ - sx);
    for (int iy = 0; iy < rut_ny_; ++iy) {
        const int src_y = iy + sy;
        if (src_y < 0 || src_y >= rut_ny_) continue;
        const double* src = &rut_[static_cast<size_t>(src_y) * rut_nx_];
        double* dst = &rut_tmp_[static_cast<size_t>(iy) * rut_nx_];
        for (int ix = ix_lo; ix < ix_hi; ++ix) dst[ix] = src[ix + sx];
    }
    rut_.swap(rut_tmp_);
}

void HeightMapTerrain::SetHeights(const std::vector<double>& heights) {
    if (heights.size() != h_.size())
        throw std::invalid_argument("SetHeights: size must be Nx * Ny");
    h_ = heights;
    ++revision_;
}

HeightMapTerrain::RutIndex HeightMapTerrain::GetRutIndex(double x, double y) const {
    // nearbyint = round-half-to-even, matching numpy.rint
    const int ix = static_cast<int>(std::nearbyint((x - rx0_) / rdx_));
    const int iy = static_cast<int>(std::nearbyint((y - ry0_) / rdx_));
    const bool inside = ix >= 0 && ix < rut_nx_ && iy >= 0 && iy < rut_ny_;
    return {std::clamp(iy, 0, rut_ny_ - 1), std::clamp(ix, 0, rut_nx_ - 1), inside};
}

}  // namespace tracked
}  // namespace vehicle
}  // namespace mavs
