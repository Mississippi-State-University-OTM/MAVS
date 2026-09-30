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
    if (terrain.HasMember("Type") && terrain["Type"].IsString()) {
        std::string terrain_type = terrain["Type"].GetString();
        if (terrain_type == "Sloped") {
            const double th = 10.0 * pi / 180.0;
            *this = FromFunction([th](double x, double) { return std::max(x - 5, 0.0) * std::tan(th); }, xmin, xmax, ymin, ymax, res);
        }
        else if (terrain_type == "Flat") {
            *this = Flat(xmin, xmax, ymin, ymax, res, 0.0);
        }
        else if (terrain_type == "Bumpy") {
            *this = FromFunction([](double X, double Y) { return 0.04 * X + 0.25 * std::exp(-((X - 12) * (X - 12) + (Y + 1.2) * (Y + 1.2)) / 2.0) + 0.08 * std::sin(0.6 * X) * std::cos(0.5 * Y); },xmin, xmax, ymin, ymax, res);
        }
        else {
            std::cerr << "Type " << terrain_type << " not recognized, must be \"Flat\", \"Sloped\" or \"Bumpy\"." << std::endl;
        }
    }
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
}

void HeightMapTerrain::Init(double x0, double y0, double dx, int nx, int ny,std::vector<double> heights) {
    x0_ = x0;
    y0_ = y0;
    dx_ = dx;
    nx_ = nx;
    ny_ = ny;
    h_ = heights;
    CheckDims();
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
}

void HeightMapTerrain::ResetRuts() { std::fill(rut_.begin(), rut_.end(), 0.0); }

HeightMapTerrain::RutIndex HeightMapTerrain::GetRutIndex(double x, double y) const {
    // nearbyint = round-half-to-even, matching numpy.rint
    const int ix = static_cast<int>(std::nearbyint((x - rx0_) / rdx_));
    const int iy = static_cast<int>(std::nearbyint((y - ry0_) / rdx_));
    const bool inside = ix >= 0 && ix < rut_nx_ && iy >= 0 && iy < rut_ny_;
    return {std::clamp(iy, 0, rut_ny_ - 1), std::clamp(ix, 0, rut_nx_ - 1), inside};
}

/*void HeightMapTerrain::PlotHeightMap() {
    std::vector<std::vector<float> > heights = Allocate2DVector(nx_, ny_, 0.0f);
    int nc = 0;
    for (int i = 0; i < nx_; i++) {
        for (int j = 0; j < ny_; j++) {
            heights[i][j] = (float)h_[nc];
            nc++;
        }
    }
    plotter_.PlotScalarColorMap(heights);
}

void HeightMapTerrain::SaveHeightMap(std::string fname) {
    std::vector<std::vector<float> > heights = Allocate2DVector(nx_, ny_, 0.0f);
    int nc = 0;
    for (int i = 0; i < nx_; i++) {
        for (int j = 0; j < ny_; j++) {
            heights[i][j] = (float)h_[nc];
            nc++;
        }
    }
    plotter_.PlotScalarColorMap(heights);
    plotter_.SaveCurrentPlot(fname);
}*/

}  // namespace tracked
}  // namespace vehicle
}  // namespace mavs
