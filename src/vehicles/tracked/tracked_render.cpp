// project includes
#include "vehicles/tracked/tracked_render.h"
#include "vehicles/tracked/tracked_math_utils.h"
// c++ includes
#include <algorithm>

namespace mavs {
namespace vehicle {
namespace tracked {

TrackedRender::TrackedRender(TrackedVehicle* tracked_vehicle_in) {
    tracked_vehicle_ = tracked_vehicle_in;
    if (tracked_vehicle_->GetSimOptions().display_debug) {
        debug_image_.assign(tracked_vehicle_->GetTerrain().Nx(), tracked_vehicle_->GetTerrain().Ny(), 1, 3, 0.0f);
        debug_display_.assign(tracked_vehicle_->GetTerrain().Nx(), tracked_vehicle_->GetTerrain().Ny(), "Tracked Vehicle Simulation", 0);  // 0 = no auto-normalisation, pixels are 0..255
    }

    if (tracked_vehicle_->GetSimOptions().render_3d) {
        Enable3DDisplay();
    }
}

void TrackedRender::Update() {
    if (tracked_vehicle_->GetSimOptions().display_debug) {
        UpdateDebugDisplay();
    }
    if (!display3d_.is_closed()) {
        Update3DDisplay();
    }
}

// ---- shared terrain colouring (2D and 3D views use the same rule)
// grey by height, blended toward brown by plastic sinkage; full tint at rut_scale
const double kRutColour[3] = { 200.0, 90.0, 20.0 };

static glm::dvec3 TerrainColour(double gray, double sink, double rut_scale) {
    const double a = std::min(std::max(sink / rut_scale, 0.0), 1.0);
    return glm::dvec3((1.0 - a) * gray + a * kRutColour[0],
                      (1.0 - a) * gray + a * kRutColour[1],
                      (1.0 - a) * gray + a * kRutColour[2]);
}

// ---- minimal software renderer: pinhole camera, near-plane clipping, z-buffer
const double kNearPlane = 0.05;   // [m]

struct Camera3D {
    glm::dvec3 pos, right, up, fwd;
    double f;        // focal length [px]
    double cx, cy;   // principal point [px]
    glm::dvec3 ToCam(const glm::dvec3& w) const {   // camera frame: x right, y up, z forward
        const glm::dvec3 d = w - pos;
        return glm::dvec3(glm::dot(d, right), glm::dot(d, up), glm::dot(d, fwd));
    }
};

struct CamVert {
    glm::dvec3 p;     // camera-frame position
    glm::dvec3 col;   // colour 0..255
};

struct RenderTarget {
    cimg_library::CImg<float>& img;
    std::vector<float>& zbuf;
    const Camera3D& cam;
};

// rasterise a triangle that lies entirely in front of the near plane
static void RasterTriangle(RenderTarget& rt, const CamVert& a, const CamVert& b, const CamVert& c) {
    const int W = rt.img.width(), H = rt.img.height();
    const CamVert* v[3] = { &a, &b, &c };
    double sx[3], sy[3], iz[3];
    glm::dvec3 ciz[3];
    for (int i = 0; i < 3; ++i) {
        iz[i] = 1.0 / v[i]->p.z;
        sx[i] = rt.cam.cx + rt.cam.f * v[i]->p.x * iz[i];
        sy[i] = rt.cam.cy - rt.cam.f * v[i]->p.y * iz[i];
        ciz[i] = v[i]->col * iz[i];   // perspective-correct colour interpolation
    }
    const double area = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sy[1] - sy[0]) * (sx[2] - sx[0]);
    if (std::abs(area) < 1e-12) return;
    const double inv_area = 1.0 / area;

    const int x0 = std::max(0, static_cast<int>(std::floor(std::min({ sx[0], sx[1], sx[2] }))));
    const int x1 = std::min(W - 1, static_cast<int>(std::ceil(std::max({ sx[0], sx[1], sx[2] }))));
    const int y0 = std::max(0, static_cast<int>(std::floor(std::min({ sy[0], sy[1], sy[2] }))));
    const int y1 = std::min(H - 1, static_cast<int>(std::ceil(std::max({ sy[0], sy[1], sy[2] }))));
    if (x0 > x1 || y0 > y1) return;

    for (int y = y0; y <= y1; ++y) {
        const double py = y + 0.5;
        for (int x = x0; x <= x1; ++x) {
            const double px = x + 0.5;
            const double w0 = ((sx[2] - sx[1]) * (py - sy[1]) - (sy[2] - sy[1]) * (px - sx[1])) * inv_area;
            const double w1 = ((sx[0] - sx[2]) * (py - sy[2]) - (sy[0] - sy[2]) * (px - sx[2])) * inv_area;
            const double w2 = 1.0 - w0 - w1;
            if (w0 < 0.0 || w1 < 0.0 || w2 < 0.0) continue;
            const double z_inv = w0 * iz[0] + w1 * iz[1] + w2 * iz[2];
            float& zb = rt.zbuf[static_cast<size_t>(y) * W + x];
            if (static_cast<float>(z_inv) <= zb) continue;
            zb = static_cast<float>(z_inv);
            const glm::dvec3 col = (w0 * ciz[0] + w1 * ciz[1] + w2 * ciz[2]) / z_inv;
            rt.img(x, y, 0, 0) = static_cast<float>(col.r);
            rt.img(x, y, 0, 1) = static_cast<float>(col.g);
            rt.img(x, y, 0, 2) = static_cast<float>(col.b);
        }
    }
}

// clip against the near plane (gives 0, 1 or 2 triangles), then rasterise
static void DrawTriangle(RenderTarget& rt, const CamVert& a, const CamVert& b, const CamVert& c) {
    const CamVert in[3] = { a, b, c };
    CamVert out[4];
    int n = 0;
    for (int i = 0; i < 3; ++i) {
        const CamVert& p = in[i];
        const CamVert& q = in[(i + 1) % 3];
        const bool p_in = p.p.z >= kNearPlane, q_in = q.p.z >= kNearPlane;
        if (p_in) out[n++] = p;
        if (p_in != q_in) {
            const double t = (kNearPlane - p.p.z) / (q.p.z - p.p.z);
            out[n++] = { p.p + t * (q.p - p.p), p.col + t * (q.col - p.col) };
        }
    }
    if (n < 3) return;
    RasterTriangle(rt, out[0], out[1], out[2]);
    if (n == 4) RasterTriangle(rt, out[0], out[2], out[3]);
}

// flat-shaded cuboid: centre c and half-extents h in the body frame, placed by (pos, R)
static void DrawBox(RenderTarget& rt, const glm::dvec3& pos, const glm::dmat3& R,
             const glm::dvec3& c, const glm::dvec3& h, const glm::dvec3& colour,
             const glm::dvec3& light_dir) {
    glm::dvec3 world[8];
    CamVert cv[8];
    for (int k = 0; k < 8; ++k) {
        const glm::dvec3 s((k & 1) ? 1.0 : -1.0, (k & 2) ? 1.0 : -1.0, (k & 4) ? 1.0 : -1.0);
        world[k] = pos + R * (c + s * h);
        cv[k].p = rt.cam.ToCam(world[k]);
    }
    // corner index bits: 1 = +x, 2 = +y, 4 = +z
    static const int faces[6][4] = {
        { 1, 3, 7, 5 }, { 0, 4, 6, 2 },   // +x, -x
        { 2, 6, 7, 3 }, { 0, 1, 5, 4 },   // +y, -y
        { 4, 5, 7, 6 }, { 0, 2, 3, 1 } }; // +z, -z
    static const double normals[6][3] = {
        { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    for (int f = 0; f < 6; ++f) {
        const glm::dvec3 n = R * glm::dvec3(normals[f][0], normals[f][1], normals[f][2]);
        const glm::dvec3 centre = 0.25 * (world[faces[f][0]] + world[faces[f][1]] +
                                          world[faces[f][2]] + world[faces[f][3]]);
        if (glm::dot(n, rt.cam.pos - centre) <= 0.0) continue;   // back face
        const double shade = 0.35 + 0.65 * std::max(glm::dot(n, light_dir), 0.0);
        CamVert q[4];
        for (int i = 0; i < 4; ++i) { q[i] = cv[faces[f][i]]; q[i].col = colour * shade; }
        DrawTriangle(rt, q[0], q[1], q[2]);
        DrawTriangle(rt, q[0], q[2], q[3]);
    }
}

// ---- debug display: pixel <-> world mapping
// Pixel (px, py) is heightmap cell (px, Ny-1-py), so world +y points up the screen.
// ASSUMPTION: HeightMapTerrain exposes its cell size and lower-left corner as
// Resolution(), OriginX(), OriginY(). Rename these three calls to match your class.
glm::dvec2 TrackedRender::DebugPixelToWorld(int px, int py) const {
    HeightMapTerrain& terrain = tracked_vehicle_->GetTerrain();
    const double res = terrain.Dx();
    const int H = debug_image_.height();
    return glm::dvec2(terrain.X0() + (px + 0.5) * res,
                      terrain.Y0() + (H - 1 - py + 0.5) * res);
}

glm::ivec2 TrackedRender::DebugWorldToPixel(const glm::dvec3& w) const {
    HeightMapTerrain& terrain = tracked_vehicle_->GetTerrain();
    const double res = terrain.Dx();
    const int H = debug_image_.height();
    const int px = static_cast<int>(std::floor((w.x - terrain.X0()) / res));
    const int iy = static_cast<int>(std::floor((w.y - terrain.Y0()) / res));
    return glm::ivec2(px, H - 1 - iy);
}

TrackSpeeds TrackedRender::GetKeyboardDrivingCommand() {
    double speed_step = 1.0e-3;
    double twice_speed_step = 4.0e-3;
    TrackSpeeds cmd = tracked_vehicle_->SprocketSpeeds(); // sprocket_; // set the commanded speed to the current speed
    if (debug_display_.is_keyARROWUP()) {
        double new_speed = std::max(cmd.left, cmd.right);
        cmd.left = new_speed + speed_step;
        cmd.right = new_speed + speed_step;
    }
    else if (debug_display_.is_keyARROWDOWN()) {
        double new_speed = std::max(cmd.left, cmd.right);
        cmd.left = new_speed - speed_step;
        cmd.right = new_speed - speed_step;
    }

    else if (debug_display_.is_keyARROWRIGHT()) {
        cmd.left += twice_speed_step;
        cmd.right -= twice_speed_step;
    }

    else if (debug_display_.is_keyARROWLEFT()) {
        cmd.left -= twice_speed_step;
        cmd.right += twice_speed_step;
    }
    else if (debug_display_.is_keyARROWLEFT() && debug_display_.is_keyARROWUP()) {
        cmd.right += speed_step;
    }
    else if (debug_display_.is_keyARROWRIGHT() && debug_display_.is_keyARROWUP()) {
        cmd.left += speed_step;
    }
    else if (debug_display_.is_keyARROWLEFT() && debug_display_.is_keyARROWDOWN()) {
        cmd.right -= speed_step;
    }
    else if (debug_display_.is_keyARROWRIGHT() && debug_display_.is_keyARROWDOWN()) {
        cmd.left -= speed_step;
    }
    else {
        cmd.left *= 0.95;
        cmd.right *= 0.95;
    }

    return cmd;
}

void TrackedRender::UpdateDebugDisplay() {
    if (debug_image_.is_empty() || debug_display_.is_closed()) return;

    // Redraw at ~30 Hz of simulated time instead of every physics step.
    // Settle() resets the clock to 0, so also redraw whenever time goes backwards.
    const double frame_dt = 1.0 / 30.0;
    if (tracked_vehicle_->GetElapsedTime() >= debug_last_draw_time_ &&
        tracked_vehicle_->GetElapsedTime() - debug_last_draw_time_ < frame_dt) return;
    debug_last_draw_time_ = tracked_vehicle_->GetElapsedTime();

    const int W = debug_image_.width(), H = debug_image_.height();

    // Fetch everything from the vehicle once per frame, not once per pixel.
    HeightMapTerrain& terrain = tracked_vehicle_->GetTerrain();   // GetTerrain() must return a reference
    const double res = terrain.Dx(), x0 = terrain.X0(), y0 = terrain.Y0();
    const glm::dvec3 pos = tracked_vehicle_->GetPosition();
    const glm::dmat3 R = tracked_vehicle_->GetRotationMatrix();
    const auto& vp = tracked_vehicle_->GetVehicle();
    const auto track_centre = tracked_vehicle_->GetTrackCenter();   // two dvec3, cheap to copy

    // ---- 1. grayscale heightmap, computed once and cached
    //if (debug_terrain_base_.is_empty()) {
    std::vector<double> h(static_cast<size_t>(W) * H);
    double hmin = 1e300, hmax = -1e300;
    for (int py = 0; py < H; ++py)
        for (int px = 0; px < W; ++px) {
            const double z = terrain.Height(x0 + (px + 0.5) * res, y0 + (H - 1 - py + 0.5) * res);
            h[static_cast<size_t>(py) * W + px] = z;
            hmin = std::min(hmin, z);
            hmax = std::max(hmax, z);
        }
    const double range = std::max(hmax - hmin, 1e-9);
    debug_terrain_base_.assign(W, H, 1, 1);
    for (int py = 0; py < H; ++py)
        for (int px = 0; px < W; ++px)   // map to 40..220 so ruts and the vehicle stay visible
            debug_terrain_base_(px, py) =
                static_cast<float>(40.0 + 180.0 * (h[static_cast<size_t>(py) * W + px] - hmin) / range);
    //}

    // ---- 2. terrain + sinkage overlay: blend toward orange/brown by plastic sinkage
    const double rut_scale = std::max(2.0 * tracked_vehicle_->GetSinkage(), 1e-3);   // full tint at 2x static sinkage
    for (int py = 0; py < H; ++py) {
        const double wy = y0 + (H - 1 - py + 0.5) * res;
        for (int px = 0; px < W; ++px) {
            const double wx = x0 + (px + 0.5) * res;
            const HeightMapTerrain::RutIndex ri = terrain.GetRutIndex(wx, wy);
            const double sink = ri.inside ? terrain.Rut(ri.iy, ri.ix) : 0.0;
            const glm::dvec3 col = TerrainColour(debug_terrain_base_(px, py), sink, rut_scale);
            for (int ch = 0; ch < 3; ++ch)
                debug_image_(px, py, 0, ch) = static_cast<float>(col[ch]);
        }
    }

    // ---- 3. vehicle: filled green tracks, hull outline, heading line
    const float green[3]  = { 0.0f, 255.0f, 0.0f };
    const float white[3]  = { 255.0f, 255.0f, 255.0f };
    const float yellow[3] = { 255.0f, 255.0f, 0.0f };

    // rectangle of size L x Wd centred at body-frame point c, aligned with the hull
    auto draw_body_rect = [&](const glm::dvec3& c, double L, double Wd,
                              const float* color, float opacity, bool outline) {
        cimg_library::CImg<int> pts(4, 2);
        const double hx = 0.5 * L, hy = 0.5 * Wd;
        const double cx[4] = { +hx, +hx, -hx, -hx };
        const double cy[4] = { +hy, -hy, -hy, +hy };
        for (int i = 0; i < 4; ++i) {
            const glm::dvec3 w = pos + R * glm::dvec3(c.x + cx[i], c.y + cy[i], c.z);
            const glm::ivec2 pp = DebugWorldToPixel(w);
            pts(i, 0) = pp.x;
            pts(i, 1) = pp.y;
        }
        if (outline) debug_image_.draw_polygon(pts, color, opacity, ~0U);
        else         debug_image_.draw_polygon(pts, color, opacity);
    };

    const double L = vp.track_contact_length;
    for (int k = 0; k < 2; ++k)
        draw_body_rect(track_centre[k], L, vp.track_width, green, 0.85f, false);

    const glm::dvec3 hull_c = 0.5 * (track_centre[0] + track_centre[1]);
    draw_body_rect(hull_c, L, vp.tread_width + vp.track_width, white, 1.0f, true);

    const glm::ivec2 cg = DebugWorldToPixel(pos);
    const glm::ivec2 nose = DebugWorldToPixel(pos + R * glm::dvec3(hull_c.x + 0.5 * L, hull_c.y, hull_c.z));
    debug_image_.draw_line(cg.x, cg.y, nose.x, nose.y, yellow);
    debug_image_.draw_circle(cg.x, cg.y, 2, yellow);

    debug_display_.set_title("Tracked Vehicle Simulation  t = %.2f s", tracked_vehicle_->GetElapsedTime());
    debug_display_.display(debug_image_);
}

// ---- 3D visualization ----------------------------------------------------

void TrackedRender::Enable3DDisplay(int width, int height) {
    image3d_.assign(width, height, 1, 3, 0.0f);
    zbuf3d_.assign(static_cast<size_t>(width) * height, 0.0f);
    display3d_.assign(width, height, "Tracked Vehicle 3D", 0);   // 0 = no auto-normalisation
    last_draw_time_3d_ = -1.0e9;
}

void TrackedRender::BuildTerrainMesh3D() {
    HeightMapTerrain& terrain = tracked_vehicle_->GetTerrain();
    const int Nx = terrain.Nx(), Ny = terrain.Ny();
    const double dx = terrain.Dx(), x0 = terrain.X0(), y0 = terrain.Y0();
    auto cell_world = [&](int i, int j) {   // heightmap cell centre, same as the 2D view
        return glm::dvec2(x0 + (i + 0.5) * dx, y0 + (j + 0.5) * dx);
    };

    // grey mapping over the full grid, identical to the 2D view (40..220)
    double hmin = 1e300, hmax = -1e300;
    for (int j = 0; j < Ny; ++j)
        for (int i = 0; i < Nx; ++i) {
            const glm::dvec2 w = cell_world(i, j);
            const double z = terrain.Height(w.x, w.y);
            hmin = std::min(hmin, z);
            hmax = std::max(hmax, z);
        }
    const double range = std::max(hmax - hmin, 1e-9);

    // decimate large maps so the mesh stays at <= ~256 x 256 vertices
    const int kMaxVerts = 256;
    const int stride = std::max(1, (std::max(Nx, Ny) + kMaxVerts - 1) / kMaxVerts);
    auto samples = [&](int n) {
        std::vector<int> s;
        for (int i = 0; i < n; i += stride) s.push_back(i);
        if (s.back() != n - 1) s.push_back(n - 1);
        return s;
    };
    const std::vector<int> xs = samples(Nx), ys = samples(Ny);
    terrain3d_nx_ = static_cast<int>(xs.size());
    terrain3d_ny_ = static_cast<int>(ys.size());
    terrain3d_pos_.resize(xs.size() * ys.size());
    terrain3d_gray_.resize(xs.size() * ys.size());
    for (int b = 0; b < terrain3d_ny_; ++b)
        for (int a = 0; a < terrain3d_nx_; ++a) {
            const glm::dvec2 w = cell_world(xs[a], ys[b]);
            const double z = terrain.Height(w.x, w.y);
            const size_t idx = static_cast<size_t>(b) * terrain3d_nx_ + a;
            terrain3d_pos_[idx] = glm::dvec3(w.x, w.y, z);
            terrain3d_gray_[idx] = 40.0 + 180.0 * (z - hmin) / range;
        }
}

void TrackedRender::Update3DDisplay() {
    if (image3d_.is_empty() || display3d_.is_closed()) return;

    if (fabs(cam_pos_.x - tracked_vehicle_->GetTerrain().X0()) > 0.01 || fabs(cam_pos_.y - tracked_vehicle_->GetTerrain().Y0()) > 0.01) {
        cam_pos_ = glm::vec3(tracked_vehicle_->GetTerrain().X0(), tracked_vehicle_->GetTerrain().Y0(), tracked_vehicle_->GetPosition().z + 2.0);
        BuildTerrainMesh3D();
    }
    

    // ~30 Hz of simulated time; redraw if the clock went backwards (Settle)
    const double frame_dt = 1.0 / 30.0;
    if (tracked_vehicle_->GetElapsedTime() >= last_draw_time_3d_ &&
        tracked_vehicle_->GetElapsedTime() - last_draw_time_3d_ < frame_dt) return;
    last_draw_time_3d_ = tracked_vehicle_->GetElapsedTime();

    if (terrain3d_pos_.empty()) BuildTerrainMesh3D();

    const int W = image3d_.width(), H = image3d_.height();

    // Fetch everything from the vehicle once per frame, not once per vertex/element.
    HeightMapTerrain& terrain = tracked_vehicle_->GetTerrain();   // GetTerrain() must return a reference
    const glm::dvec3 pos = tracked_vehicle_->GetPosition();
    const glm::dmat3 R = tracked_vehicle_->GetRotationMatrix();
    const auto& vp = tracked_vehicle_->GetVehicle();
    const auto track_centre = tracked_vehicle_->GetTrackCenter();
    const auto& r_el = tracked_vehicle_->GetTrackElements();       // no per-frame copy if it returns a reference

    // ---- camera
    double ltx = pos.x - cam_pos_.x;
    double lty = pos.y - cam_pos_.y;
    cam_yaw_ = atan2(lty, ltx);

    Camera3D cam;
    cam.pos = cam_pos_;
    const double cy = std::cos(cam_yaw_), sy = std::sin(cam_yaw_);
    const double cp = std::cos(cam_pitch_), sp = std::sin(cam_pitch_);
    cam.fwd = glm::dvec3(cp * cy, cp * sy, sp);
    cam.right = glm::dvec3(sy, -cy, 0.0);                // world z up, body/world y left
    cam.up = glm::cross(cam.right, cam.fwd);
    cam.f = 0.5 * W / std::tan(0.5 * cam_hfov_);
    cam.cx = 0.5 * W;
    cam.cy = 0.5 * H;

    // ---- clear: sky colour, empty depth
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            image3d_(x, y, 0, 0) = 150.0f;
            image3d_(x, y, 0, 1) = 185.0f;
            image3d_(x, y, 0, 2) = 225.0f;
        }
    std::fill(zbuf3d_.begin(), zbuf3d_.end(), 0.0f);
    RenderTarget rt{ image3d_, zbuf3d_, cam };

    // ---- terrain: 2D colouring; rutted vertices are lowered by the plastic sinkage
    //      so the tracks sit visibly in their ruts instead of below the surface
    const double rut_scale = std::max(2.0 * tracked_vehicle_->GetSinkage(), 1e-3);
    std::vector<CamVert> tv(terrain3d_pos_.size());
    for (size_t i = 0; i < terrain3d_pos_.size(); ++i) {
        glm::dvec3 w = terrain3d_pos_[i];
        const HeightMapTerrain::RutIndex ri = terrain.GetRutIndex(w.x, w.y);
        const double sink = ri.inside ? terrain.Rut(ri.iy, ri.ix) : 0.0;
        w.z -= sink;
        tv[i].p = cam.ToCam(w);
        tv[i].col = TerrainColour(terrain3d_gray_[i], sink, rut_scale);
    }
    for (int b = 0; b + 1 < terrain3d_ny_; ++b)
        for (int a = 0; a + 1 < terrain3d_nx_; ++a) {
            const size_t i00 = static_cast<size_t>(b) * terrain3d_nx_ + a;
            const size_t i10 = i00 + 1, i01 = i00 + terrain3d_nx_, i11 = i01 + 1;
            DrawTriangle(rt, tv[i00], tv[i10], tv[i11]);
            DrawTriangle(rt, tv[i00], tv[i11], tv[i01]);
        }

    // ---- vehicle
    const glm::dvec3 light = glm::normalize(glm::dvec3(0.3, 0.2, 0.9));
    const glm::dvec3 green(0.0, 200.0, 0.0), yellow(255.0, 220.0, 0.0);

    // track elements: one yellow cuboid per contact element, bottom face on the contact plane
    double du = tracked_vehicle_->GetDu();
    double dw = tracked_vehicle_->GetDw();
    const double t_el = 2.0*std::min(du, dw);                 // element thickness (visual only)
    const glm::dvec3 h_el(0.45 * du, 0.45 * dw, 0.5 * t_el);    // 10% gap between elements
    for (int k = 0; k < 2; ++k)
        for (const glm::dvec3& r : r_el[k])
            DrawBox(rt, pos, R, r + glm::dvec3(0.0, 0.0, 0.5 * t_el), h_el, yellow, light);

    // hull: green cuboid between the tracks (visual proportions; tweak to taste)
    const double L = vp.track_contact_length;
    const double rs = vp.sprocket_radius;
    const double z_track = track_centre[0].z;                     // contact plane, body frame
    const double z_bot = z_track + rs;
    const double z_top = z_bot + 2.0 * std::max(vp.cg_height - rs, rs);
    const double inner_w = std::max(vp.tread_width - vp.track_width, 0.25 * vp.tread_width);
    const glm::dvec3 hull_c = 0.5 * (track_centre[0] + track_centre[1]);
    DrawBox(rt, pos, R,
            glm::dvec3(hull_c.x, hull_c.y, 0.5 * (z_bot + z_top)),
            glm::dvec3(0.5 * L, 0.5 * 0.95 * inner_w, 0.5 * (z_top - z_bot)),
            green, light);

    display3d_.set_title("Tracked Vehicle 3D  t = %.2f s  cam (%.1f, %.1f, %.1f)",
                         tracked_vehicle_->GetElapsedTime(), cam_pos_.x, cam_pos_.y, cam_pos_.z);
    display3d_.display(image3d_);
}

}  // namespace tracked
} // namespace vehicle
} // namespace mavs