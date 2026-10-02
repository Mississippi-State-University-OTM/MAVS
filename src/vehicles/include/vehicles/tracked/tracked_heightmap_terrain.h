#ifndef TRACKED_HEIGHTMAP_TERRAIN_H
#define TRACKED_HEIGHTMAP_TERRAIN_H
// tracked_heightmap_terrain.h
// ===========================
// Terrain for the tracked-vehicle dynamics: a regular height map (undisturbed
// surface) plus a world-fixed rut map (maximum plastic sinkage per cell).
//
// Height map: heights[iy * nx + ix] at (x0 + ix*dx, y0 + iy*dx), bilinear
// interpolation, clamped at the edges.
//
// Rut map: allocated over the height-map extent with its own spacing rdx,
// nearest-cell lookup (round-half-to-even, matching numpy.rint).
//
// Frames: world z up. SI units.

// project includes
#include "rapidjson/document.h"
// c++ includes
#include <functional>
#include <vector>
#include <string>

namespace mavs {
namespace vehicle {
namespace tracked {

class HeightMapTerrain {
public:
    // ------------------------------------------------ construction
    HeightMapTerrain();

    HeightMapTerrain(double x0, double y0, double dx, int nx, int ny, std::vector<double> heights);

    //void Init(double x0, double y0, double dx, int nx, int ny, std::vector<double> heights);

    // Undisturbed surface height at (x, y).
    double Height(double x, double y) const;
    // Bounds of the height map (also used to allocate the rut map).
    void Extent(double& xmin, double& xmax, double& ymin, double& ymax) const;

    double X0() const { return x0_; }
    double Y0() const { return y0_; }
    double Dx() const { return dx_; }
    int Nx() const { return nx_; }
    int Ny() const { return ny_; }
    const std::vector<double>& heights() const { return h_; }

    // ------------------------------------------------ moving window
    // Move the height-map origin (lower-left node) to (x0, y0). Size and spacing are unchanged.
    // Heights are NOT touched: refill them afterwards with SetHeights().
    // The rut map is shifted by whole rut cells so every rut keeps its world position;
    // ruts that leave the window are discarded, cells entering it start undisturbed (0).
    void SetOrigin(double x0, double y0);
    // Replace all heights; size must be Nx() * Ny(), same layout as the constructor.
    void SetHeights(const std::vector<double>& heights);
    // Bumped whenever heights or the origin change, so caches (e.g. the renderer) can rebuild.
    unsigned long long Revision() const { return revision_; }

    // ------------------------------------------------ rut map
    struct RutIndex { int iy, ix; bool inside; };

    void InitRut(double rdx);   // no-op if already allocated with this rdx
    void ResetRuts();
    RutIndex GetRutIndex(double x, double y) const;
    double Rut(int iy, int ix) const { return rut_[static_cast<size_t>(iy) * rut_nx_ + ix]; }
    double& Rut(int iy, int ix) { return rut_[static_cast<size_t>(iy) * rut_nx_ + ix]; }

    bool HasRut() const { return !rut_.empty(); }
    double RutDx() const { return rdx_; }
    double RutX0() const { return rx0_; }
    double RutY0() const { return ry0_; }
    int RutNx() const { return rut_nx_; }
    int RutNy() const { return rut_ny_; }
    const std::vector<double>& rut_data() const { return rut_; }

    void Load(std::string input_file);

    void ParseJsonObject(const rapidjson::Value& vehicle);

    double XDim() const { return nx_ * dx_; }

    double YDim() const { return ny_ * dx_; }

    std::vector<double> GetHeights() { return h_; }

private:
    static HeightMapTerrain FromFunction(const std::function<double(double, double)>& f,
        double xmin, double xmax, double ymin, double ymax,
        double dx);

    static HeightMapTerrain Flat(double xmin, double xmax, double ymin, double ymax,
        double dx = 0.5, double z = 0.0);

    // height map
    double x0_, y0_, dx_;
    int nx_, ny_;
    std::vector<double> h_;

    // rut map
    std::vector<double> rut_;
    double rx0_ = 0, ry0_ = 0, rdx_ = 0;
    int rut_nx_ = 0, rut_ny_ = 0;
    // rut origin = base + whole-cell offset (kept as integers so repeated shifts don't drift)
    double rut_base_x_ = 0, rut_base_y_ = 0;
    long long rut_off_x_ = 0, rut_off_y_ = 0;
    std::vector<double> rut_tmp_;   // scratch for shifting

    unsigned long long revision_ = 0;

    void CheckDims();
};

}  // namespace tracked
}  // namespace vehicle
}  // namespace mavs

#endif
