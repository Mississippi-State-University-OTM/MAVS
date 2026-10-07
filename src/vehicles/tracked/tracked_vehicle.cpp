// project includes
#include "vehicles/tracked/tracked_vehicle.h"
#include "vehicles/tracked/tracked_math_utils.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
// c++ includes
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <limits>

namespace mavs {
namespace vehicle {
namespace tracked {

TrackedVehicle::TrackedVehicle() {
    initialized_ = false;
}

void TrackedVehicle::Load(std::string input_file) {
    local_sim_time_ = 0.0;
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

    // Get the vehicle params and initialize
    if (d.HasMember("Vehicle") && d["Vehicle"].IsObject()) {
        const auto& vehicle = d["Vehicle"];
        vehicle_params_.ParseJsonObject(vehicle);
    }
    else {
        std::cerr << "WARNING: No field for \"Vehicle\" in simulation input file." << std::endl;
    }

    // get the soil params and initialize
    if (d.HasMember("Soil") && d["Soil"].IsObject()) {
        const rapidjson::Value& soil = d["Soil"];
        soil_.ParseJsonObject(soil);
    }
    else {
        std::cerr << "WARNING: No field for \"Soil\" in simulation input file." << std::endl;
    }

    // Get the simulation options
    if (d.HasMember("Sim Options") && d["Sim Options"].IsObject()) {
        const rapidjson::Value& sim_options = d["Sim Options"];
        sim_options_.ParseJsonObject(sim_options);
    }
    else {
        std::cerr << "WARNING: No field for \"Sim Options\" in simulation input file." << std::endl;
    }

    // Get the terrain inputs members
    if (d.HasMember("Terrain") && d["Terrain"].IsObject()) {
        const rapidjson::Value& terrain = d["Terrain"];
        terrain_.ParseJsonObject(terrain);
    }
    else {
        std::cerr << "WARNING: No field for \"Terrain\" in simulation input file." << std::endl;
    }

    if (d.HasMember("Vehicle Mesh") && d["Vehicle Mesh"].IsObject()) {
        const rapidjson::Value& mesh = d["Vehicle Mesh"];
        vehicle_asset_.ParseJsonObject(mesh);
    }

    if (d.HasMember("Track Pad Mesh") && d["Track Pad Mesh"].IsObject()) {
        const rapidjson::Value& pad_mesh = d["Track Pad Mesh"];
        track_asset_.ParseJsonObject(pad_mesh);
    }

    // Optional belt layout for track-shoe animation
    if (d.HasMember("Track Layout") && d["Track Layout"].IsObject()) {
        const rapidjson::Value& tl = d["Track Layout"];
        if (tl.HasMember("Wheels") && tl["Wheels"].IsArray()) {
            for (const auto& wv : tl["Wheels"].GetArray()) {
                if (!wv.IsArray() || wv.Size() != 3) {
                    std::cerr << "WARNING: Track Layout wheels must be [x, z, radius]; skipping entry." << std::endl;
                    continue;
                }
                TrackWheel wheel;
                wheel.x = wv[0].GetDouble();
                wheel.z = wv[1].GetDouble();
                wheel.radius = wv[2].GetDouble();
                track_layout_.wheels.push_back(wheel);
            }
        }
        if (tl.HasMember("Number of Shoes") && tl["Number of Shoes"].IsInt())
            track_layout_.num_shoes = tl["Number of Shoes"].GetInt();
        if (tl.HasMember("Shoe Pitch") && tl["Shoe Pitch"].IsNumber())
            track_layout_.shoe_pitch = tl["Shoe Pitch"].GetDouble();
        if (tl.HasMember("Shoe Offset") && tl["Shoe Offset"].IsArray() && tl["Shoe Offset"].Size() == 3)
            for (int ij = 0; ij < 3; ij++) track_layout_.shoe_offset[ij] = tl["Shoe Offset"][ij].GetDouble();
        // Euler angles in degrees about the shoe x, y, z axes
        if (tl.HasMember("Shoe Mesh Rotation") && tl["Shoe Mesh Rotation"].IsArray() && tl["Shoe Mesh Rotation"].Size() == 3) {
            glm::dvec3 eul;
            for (int ij = 0; ij < 3; ij++) eul[ij] = glm::radians(tl["Shoe Mesh Rotation"][ij].GetDouble());
            track_layout_.shoe_mesh_rotation = glm::dquat(eul);
        }
    }

    Init();
    if (sim_options_.render_debug) tracked_debug_render_.Init(&terrain_);
    initialized_ = true;
}

void TrackedVehicle::SetPose(double x, double y, double yaw_radians) {
    // Start from rest with a clean contact history
    velocity_ = glm::dvec3(0.0);
    angular_velocity_ = glm::dvec3(0.0);
    sprocket_speeds_ = { 0.0, 0.0 };
    for (int k = 0; k < 2; ++k)
        std::fill(j_[k].begin(), j_[k].end(), std::array<double, 2>{ 0.0, 0.0 });

    // 1) Yaw-only pose: find where every track element sits in x/y
    const glm::dmat3 R_yaw = RFromRpy(0, 0, yaw_radians);
    const glm::dvec3 p_xy(x, y, 0.0);
    std::vector<glm::dvec3> pts;
    for (int k = 0; k < 2; ++k)
        for (const glm::dvec3& rb : r_el_[k]) {
            const glm::dvec3 rw = p_xy + R_yaw * rb;
            pts.emplace_back(rw.x, rw.y, terrain_.Height(rw.x, rw.y));
        }

    // 2) Least-squares plane h = h0 + gx*x + gy*y through the terrain under the tracks
    glm::dvec3 mean(0.0);
    for (const auto& q : pts) mean += q;
    mean /= static_cast<double>(pts.size());
    double sxx = 0, sxy = 0, syy = 0, sxh = 0, syh = 0;
    for (const auto& q : pts) {
        const double dx = q.x - mean.x, dy = q.y - mean.y, dh = q.z - mean.z;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
        sxh += dx * dh; syh += dy * dh;
    }
    double gx = 0.0, gy = 0.0;
    const double det = sxx * syy - sxy * sxy;
    if (det > 1e-12) {
        gx = (syy * sxh - sxy * syh) / det;
        gy = (sxx * syh - sxy * sxh) / det;
    }

    // 3) Hull orientation: body z = terrain normal, body x = heading projected onto the plane
    const glm::dvec3 nrm = glm::normalize(glm::dvec3(-gx, -gy, 1.0));
    const glm::dvec3 fwd = R_yaw[0];
    const glm::dvec3 xb = glm::normalize(fwd - glm::dot(fwd, nrm) * nrm);
    const glm::dvec3 yb = glm::cross(nrm, xb);
    R_ = glm::dmat3(xb, yb, nrm);

    // 4) Height: lowest CG height at which no track element is below the terrain,
    //    plus a small clearance so the vehicle starts just above contact
    const double clearance = 0.05;
    double z_init = std::numeric_limits<double>::lowest();
    for (int k = 0; k < 2; ++k)
        for (const glm::dvec3& rb : r_el_[k]) {
            const glm::dvec3 off = R_ * rb;
            const double h = terrain_.Height(x + off.x, y + off.y);
            z_init = std::max(z_init, h - off.z);
        }
    z_init += clearance;
    position_ = glm::dvec3(x, y, z_init);
}

void TrackedVehicle::UpdateTerrain(environment::Environment* env, float dt) {
    time_since_last_terrain_refresh_ += dt;
    if (time_since_last_terrain_refresh_ > 1.0f) {
        ResetTerrain(env);
        time_since_last_terrain_refresh_ = 0.0;
    }
}

void TrackedVehicle::ResetTerrain(environment::Environment* env) {
    float zmin = std::numeric_limits<float>::lowest();
    zmin *= 0.5;
    std::vector<double> old_heights = terrain_.GetHeights();
    glm::dvec2 new_origin(position_.x - 0.5 * terrain_.XDim(), position_.y - 0.5 * terrain_.YDim());
    terrain_.SetOrigin(new_origin.x, new_origin.y);
    std::vector<double> new_heights;
    int nx = terrain_.Nx();
    int ny = terrain_.Ny();
    int ntot = nx * ny;
    new_heights.resize(ntot);
    double dx = terrain_.Dx();
    int n = 0;
    for (int j = 0; j < ny; j++) {
        double y = new_origin.y + (j + 0.5) * dx;
        for (int i = 0; i < nx; i++) {
            double x = new_origin.x + (i + 0.5) * dx;
            float z = env->GetGroundHeight((float)x, (float)y);
            if (z <= zmin && n>0) z = (float)new_heights[n-1];
            new_heights[n] = (double)z;
            n++;
        }
    }
    terrain_.SetHeights(new_heights);
}

TrackSpeeds TrackedVehicle::GetSprocketSpeedsFromTsb(double throttle, double steer, double brake, double dt) const {
    TrackSpeeds cmd;
    //double steering_gain = 1.0 - 0.7 * throttle;
    //double left_speed = throttle - steer*steering_gain;
    //double right_speed = throttle + steer*steering_gain;
    double left_speed = throttle - steer;
    double right_speed = throttle + steer;
    double max_mag = std::max(std::abs(left_speed), std::abs(right_speed));
    if (max_mag > 1.0) {
        left_speed /= max_mag;
        right_speed /= max_mag;
    }
    cmd.left = left_speed * max_mag * vehicle_params_.max_sprocket_speed;
    cmd.right = right_speed * max_mag* vehicle_params_.max_sprocket_speed;

    cmd.right = std::max(-vehicle_params_.max_sprocket_speed, std::min(cmd.right, vehicle_params_.max_sprocket_speed));
    cmd.left = std::max(-vehicle_params_.max_sprocket_speed, std::min(cmd.left, vehicle_params_.max_sprocket_speed));
    
    return cmd;
}

void TrackedVehicle::InitAnimation(environment::Environment* env) {
    actor_ids_ = env->AddActor(vehicle_asset_.mesh_file, vehicle_asset_.rotate_y_to_z, vehicle_asset_.rotate_x_to_y, vehicle_asset_.rotate_y_to_x, vehicle_asset_.offset, vehicle_asset_.scale);
    vehicle_loaded_ = true;
    std::vector<TrackShoePose> shoe_poses;
    GetTrackShoePoses(shoe_poses, true);
    int nanim = 1;
    for (size_t sp = 0; sp < shoe_poses.size(); sp++) {
        std::vector<int> shoe_id = env->AddActor(track_asset_.mesh_file, track_asset_.rotate_y_to_z, track_asset_.rotate_x_to_y, track_asset_.rotate_y_to_x, track_asset_.offset, track_asset_.scale);
        track_pad_ids_.push_back(nanim);
        nanim++;
    }
    // Center the local height map on the start position before sampling it,
    // otherwise SetPose reads heights from wherever position_ happened to be
    position_ = glm::dvec3(initial_position_x_, initial_position_y_, 0.0);
    ResetTerrain(env);

    // Settle the vehicle into the inital position
    SetPose(initial_position_x_, initial_position_y_, initial_yaw_);
    Settle(5.0, 0.25e-3);
}

void TrackedVehicle::UpdateSim(double dt, TrackSpeeds cmd) {
    // adjust the number of steps based on the requested time step
    int nsteps = 1;
    double dt_step = sim_options_.max_dt;
    if (dt > sim_options_.max_dt) {
        nsteps = (int)ceil(dt / sim_options_.max_dt);
        dt_step = dt / nsteps;
    }

    // step the simulation 
    for (int ti = 0; ti < nsteps; ti++) {
        Step(dt_step, cmd);
    }
}

void TrackedVehicle::Update(environment::Environment* env, float throttle, float steer, float brake, float dt) {
    if (!initialized_) {
        std::cerr << "ERROR: MAVS TRACKED VEHICLE: NO VEHICLE FILE LOADED" << std::endl;
        exit(47);
    }

    // Initialize the animations
    if (!vehicle_loaded_) InitAnimation(env);

    // get the commanded sprocket speeds
    TrackSpeeds cmd = GetSprocketSpeedsFromTsb(throttle, steer, brake, (double)dt);

    // Update the track simulation
    UpdateSim((double)dt, cmd);

    // Set the MAVS vehicle output params
    SetMavsParams((double)dt);

    // Update the animation positions
    UpdateMavsAnimations(env);

    // update the terrain
    UpdateTerrain(env, dt);

    // Update the debug render
    UpdateDebugRender(dt);
}

void TrackedVehicle::UpdateDebugRender(double dt) {
    if (sim_options_.render_debug) {
        time_since_last_debug_render_ += dt;
        if (time_since_last_debug_render_ > 0.1) {
            tracked_debug_render_.Update(current_state_.pose.position, glm::dmat3(current_state_.pose.quaternion), vehicle_params_, track_center_, z_static_estimate_, r_el_, du_, dw_);
            time_since_last_debug_render_ = 0.0;
        }
    }
}

void TrackedVehicle::UpdateMavsAnimations(environment::Environment* env) {
    for (size_t actor_idx = 0; actor_idx < actor_ids_.size(); actor_idx++) {
        env->SetActorPosition(actor_ids_[actor_idx], current_state_.pose.position, current_state_.pose.quaternion);
        std::vector<TrackShoePose> shoe_poses;
        GetTrackShoePoses(shoe_poses, true);
        for (size_t sp = 0; sp < shoe_poses.size(); sp++) {
            env->SetActorPosition(track_pad_ids_[sp], shoe_poses[sp].position, shoe_poses[sp].orientation);
        }
    }
}

void TrackedVehicle::SetMavsParams(double dt) {
    current_state_.accel.linear = (1.0 / dt) * (velocity_ - current_state_.twist.linear);
    current_state_.accel.angular = (1.0 / dt) * (angular_velocity_ - current_state_.twist.angular);
    current_state_.pose.position = position_;
    current_state_.pose.quaternion = glm::dquat(R_);
    current_state_.twist.linear = velocity_;
    current_state_.twist.angular = angular_velocity_;
}

void TrackedVehicle::EnableMovingTerrain(double recenter_distance, TerrainRefresh refresh) {
    recenter_distance_ = recenter_distance;
    terrain_refresh_ = std::move(refresh);
    moving_terrain_ = true;
}

void TrackedVehicle::RecenterTerrain() {
    double xmin, xmax, ymin, ymax;
    terrain_.Extent(xmin, xmax, ymin, ymax);
    const double dx = terrain_.Dx();
    const double sx = std::round((position_.x - 0.5 * (xmin + xmax)) / dx) * dx;
    const double sy = std::round((position_.y - 0.5 * (ymin + ymax)) / dx) * dx;
    if (sx == 0.0 && sy == 0.0) return;
    terrain_.SetOrigin(terrain_.X0() + sx, terrain_.Y0() + sy);
    if (terrain_refresh_) terrain_refresh_(terrain_);
}

void TrackedVehicle::Settle(double duration, double dt) {
    const DriveMode drive = sim_options_.drive;
    sim_options_.drive = DriveMode::Speed;
    const int n = static_cast<int>(duration / dt);
    for (int i = 0; i < n; ++i) {
        g_scale_ = std::min(1.0, 1.5 * (i + 1) / n);
        Step(dt, { 0.0, 0.0 });
    }
    g_scale_ = 1.0;
    sim_options_.drive = drive;
    local_sim_time_ = 0.0;
}

void TrackedVehicle::SetTrackLayout(const TrackLayout& layout) {
    track_layout_ = layout;
    BuildTrackPath();
}

void TrackedVehicle::BuildTrackPath() {
    std::vector<TrackWheel> wheels = track_layout_.wheels;
    if (wheels.empty()) {
        // default: sprocket-sized wheels at the ends of the contact patch, resting on the ground plane
        const double r = vehicle_params_.sprocket_radius;
        const double h = 0.5 * vehicle_params_.track_contact_length;
        wheels = { TrackWheel{ -h, r, r }, TrackWheel{ h, r, r } };
    }
    track_path_.Build(wheels);

    const double L = track_path_.Length();
    if (track_layout_.num_shoes > 0) {
        num_shoes_ = track_layout_.num_shoes;
    }
    else {
        if (!(track_layout_.shoe_pitch > 0.0))
            throw std::invalid_argument("TrackLayout: set num_shoes > 0 or shoe_pitch > 0");
        num_shoes_ = std::max(3, static_cast<int>(std::lround(L / track_layout_.shoe_pitch)));
    }
    shoe_pitch_ = L / num_shoes_;  // exact spacing so the loop closes
    for (int k = 0; k < 2; ++k) track_phase_[k] = track_path_.Wrap(track_phase_[k]);
}

void TrackedVehicle::GetTrackShoePoses(std::vector<TrackShoePose>& out, bool world_frame) const {
    const int n = num_shoes_;
    out.resize(static_cast<size_t>(2 * n));
    const glm::dquat q_hull = glm::quat_cast(R_);
    const glm::dvec3 Y(0.0, 1.0, 0.0);
    for (int k = 0; k < 2; ++k) {
        for (int j = 0; j < n; ++j) {
            // a shoe is a rigid link between two pins on the pitch line, so on the wheels it
            // lies along the chord (no gaps or overlap between neighbouring shoes)
            const double s_a = track_phase_[k] + j * shoe_pitch_;
            const glm::dvec2 a2 = track_path_.Point(s_a);
            const glm::dvec2 b2 = track_path_.Point(s_a + shoe_pitch_);
            const glm::dvec3 a(a2.x, 0.0, a2.y), b(b2.x, 0.0, b2.y);

            // shoe x points from the trailing pin to the leading one, i.e. body-forward on the ground run
            glm::dvec3 X = a - b;
            const double len = glm::length(X);
            X = len > 1e-12 ? X / len : glm::dvec3(1.0, 0.0, 0.0);
            const glm::dvec3 Z = glm::cross(X, Y);  // toward the inside of the loop
            const glm::dmat3 Rs(X, Y, Z);            // columns

            glm::dvec3 pos = track_center_[k] + 0.5 * (a + b) + Rs * track_layout_.shoe_offset;
            glm::dquat q = glm::quat_cast(Rs) * track_layout_.shoe_mesh_rotation;
            if (world_frame) {
                pos = position_ + R_ * pos;
                q = q_hull * q;
            }
            TrackShoePose& sp = out[static_cast<size_t>(k) * n + j];
            sp.position = pos;
            sp.orientation = glm::normalize(q);
            sp.track = k;
            sp.index = j;
        }
    }
}

// ------- All the real physics stuff is happening down here ------- //
//                             |                                     //
//                             |                                     //
//                             |                                     //
//                             |                                     //
//                             V                                     //
//-------------------------------------------------------------------//

void TrackedVehicle::Init(){

    I_ = vehicle_params_.InertiaDiag();
    for (int i = 0; i < 3; ++i) Iinv_[i] = 1.0 / I_[i];

    // element layout, body frame; rows i = rear -> front, columns c = right -> left
    du_ = vehicle_params_.track_contact_length / sim_options_.ny;
    dw_ = vehicle_params_.track_width / sim_options_.nx;
    A_ = du_ * dw_;
    u_.resize(sim_options_.ny);
    std::vector<double> w(sim_options_.nx);
    for (int i = 0; i < sim_options_.ny; ++i) u_[i] = -vehicle_params_.track_contact_length / 2 + (i + 0.5) * du_;
    for (int c = 0; c < sim_options_.nx; ++c) w[c] = -vehicle_params_.track_width / 2 + (c + 0.5) * dw_;
    const int sides[2] = {+1, -1};
    for (int k = 0; k < 2; ++k) {
        const glm::dvec3 cxy(-vehicle_params_.cg_longitudinal_offset, sides[k] * vehicle_params_.tread_width / 2 - vehicle_params_.cg_lateral_offset, -vehicle_params_.cg_height);
        track_center_[k] = cxy;
        r_el_[k].resize(static_cast<size_t>(sim_options_.ny) * sim_options_.nx);
        for (int i = 0; i < sim_options_.ny; ++i)
            for (int c = 0; c < sim_options_.nx; ++c)
                r_el_[k][Idx(i, c)] = glm::dvec3(cxy.x + u_[i], cxy.y + w[c], cxy.z);
        j_[k].assign(r_el_[k].size(), {0.0, 0.0});
    }

    kb_ = soil_.kc / vehicle_params_.track_width + soil_.kphi;
    soil_.BulldozingFactors(Kc_, Kg_);

    // contact damping per unit area from the static sinkage stiffness
    const double p0 = vehicle_params_.mass * sim_options_.gravity / (2 * vehicle_params_.track_width * vehicle_params_.track_contact_length);
    const double z0 = std::pow(p0 / kb_, 1.0 / soil_.n);
    const double k_area = soil_.n * p0 / z0;
    const double A_tot = 2 * vehicle_params_.track_width * vehicle_params_.track_contact_length;
    const double c_crit = 2 * std::sqrt(k_area * A_tot * vehicle_params_.mass);
    c_area_ = sim_options_.normal_damping_ratio * c_crit / A_tot;
    z_static_estimate_ = z0;

    const double rdx = sim_options_.rut_dx > 0 ? sim_options_.rut_dx : 1.2 * std::max(du_, dw_);
    terrain_.InitRut(rdx);

    const size_t ne = r_el_[0].size();
    z_.resize(ne); zp_.resize(ne); pn_.resize(ne); vb_.resize(ne); ri_.resize(ne);
    loading_.resize(ne); contact_.resize(ne); jtmp_.resize(ne);

    BuildTrackPath();

}

void TrackedVehicle::Advect(std::vector<std::array<double, 2>>& j, double shift) {
    // element now at u came from u + shift; soil entering at the leading edge has j = 0
    const double q = shift / du_;
    const int k0 = static_cast<int>(std::floor(q));
    const double f = q - k0;
    const int ny = sim_options_.ny, nx = sim_options_.nx;
    std::vector<std::array<double, 2>>& out = jtmp_;
    for (int i = 0; i < ny; ++i)
        for (int c = 0; c < nx; ++c) {
            std::array<double, 2> acc{0.0, 0.0};
            for (int di = 0; di < 2; ++di) {
                const int ii = i + k0 + di;
                const double wgt = di == 0 ? 1 - f : f;
                if (ii >= 0 && ii < ny) {
                    acc[0] += wgt * j[Idx(ii, c)][0];
                    acc[1] += wgt * j[Idx(ii, c)][1];
                }
            }
            out[Idx(i, c)] = acc;
        }
    j.swap(out);
}

double TrackedVehicle::BulldozePerWidth(double z) const {
    z = std::max(z, 0.0);
    return 0.67 * soil_.c * z * Kc_ + 0.5 * soil_.gamma * z * z * Kg_;
}

TrackDiag TrackedVehicle::TrackForces(int k, double dt, glm::dvec3& F, glm::dvec3& M) {
    const std::vector<glm::dvec3>& rb = r_el_[k];
    const int ny = sim_options_.ny, nx = sim_options_.nx;
    const size_t ne = rb.size();
    const double kb = kb_, n = soil_.n, tanp = soil_.TanPhi();
    const double cosn = std::max(R_[2][2], 1e-3);   // world-z component of body z
    const glm::dvec3 vel_b = glm::transpose(R_) * velocity_;
    TrackDiag d;
    F = glm::dvec3(0.0);
    M = glm::dvec3(0.0);

    // ---- track compliance: each element is backed by a spring (road wheels / suspension / belt)
    //      that lets it deflect up toward the hull so the track conforms to the terrain.
    //      track_static_defl <= 0 gives the old rigid-plate behaviour.
    //const double track_static_defl = 0.03;  // [m] element deflection under static load on flat ground
    //const double track_max_travel = 0.15;   // [m] bump stop: beyond this the element is rigid again
    const double p_static = vehicle_params_.mass * sim_options_.gravity /
        (2.0 * vehicle_params_.track_width * vehicle_params_.track_contact_length);
    const double track_k = vehicle_params_.track_static_defl > 0.0 ? p_static / vehicle_params_.track_static_defl : 0.0;  // [Pa/m]

    // ---- pass 1: sinkage and normal pressure (rut read before any update)
    for (size_t e = 0; e < ne; ++e) {
        const glm::dvec3 rw = position_ + R_ * rb[e];
        const double hs = terrain_.Height(rw.x, rw.y);
        ri_[e] = terrain_.GetRutIndex(rw.x, rw.y);
        const double zp = terrain_.Rut(ri_[e].iy, ri_[e].ix);
        const double z_rigid = (hs - rw.z) * cosn;   // sinkage if the element were rigidly attached
        const glm::dvec3 vb = vel_b + glm::cross(angular_velocity_, rb[e]);
        const double p_top = kb * std::pow(zp, n);
        const double ku = zp > 0 ? p_top / std::max(soil_.rebound * zp, 1e-12) : 0.0;
        // static soil pressure as a function of soil sinkage s (loading / unloading branches)
        auto soil_p = [&](double s) {
            return s > zp ? kb * std::pow(std::max(s, 0.0), n) : std::max(p_top - ku * (zp - s), 0.0);
        };

        // element deflection: solve soil_p(z_rigid - delta) = track_k * delta (monotone, bisection)
        double delta = 0.0;
        if (track_k > 0.0 && soil_p(z_rigid) > 0.0) {
            const double s_zero = zp > 0 ? zp * (1.0 - soil_.rebound) : 0.0;  // sinkage where soil_p hits 0
            double hi = std::min(vehicle_params_.track_max_travel, z_rigid - s_zero);
            if (soil_p(z_rigid - hi) - track_k * hi >= 0.0) {
                delta = hi;  // on the bump stop
            } else {
                double lo = 0.0;
                for (int it = 0; it < 20; ++it) {
                    const double mid = 0.5 * (lo + hi);
                    if (soil_p(z_rigid - mid) - track_k * mid > 0.0) lo = mid; else hi = mid;
                }
                delta = 0.5 * (lo + hi);
            }
        }

        const double z = z_rigid - delta;   // actual soil sinkage
        const bool loading = z > zp;
        const double p_st = soil_p(z);
        const bool contact = p_st > 0;
        z_[e] = z;
        zp_[e] = zp;
        vb_[e] = vb;
        loading_[e] = loading;
        contact_[e] = contact;
        pn_[e] = contact ? std::max(p_st - c_area_ * vb.z, 0.0) : 0.0;
        if (!ri_[e].inside) ++d.outside_map;
    }
    // rut update: plastic sinkage memory
    for (size_t e = 0; e < ne; ++e)
        if (loading_[e] && ri_[e].inside) {
            double& r = terrain_.Rut(ri_[e].iy, ri_[e].ix);
            r = std::max(r, z_[e]);
        }

    // ---- shear displacement field
    const double belt = vehicle_params_.sprocket_radius * sprocket_speeds_[k];
    std::vector<std::array<double, 2>>& j = j_[k];
    Advect(j, belt * dt);
    const double a = angular_velocity_.z * dt;
    const double ca = std::cos(a), sa = std::sin(a);
    const double eps_v = 0.01 * sim_options_.v_eps;
    double contact_count = 0, sink_sum = 0;
    for (size_t e = 0; e < ne; ++e) {
        const double jx = ca * j[e][0] + sa * j[e][1];
        const double jy = -sa * j[e][0] + ca * j[e][1];
        const double vsx = vb_[e].x - belt, vsy = vb_[e].y;
        double jnx = jx + vsx * dt, jny = jy + vsy * dt;
        if (!contact_[e]) { jnx = 0.0; jny = 0.0; }
        j[e] = {jnx, jny};

        const double jm = std::hypot(jnx, jny);
        double tau = (soil_.c + pn_[e] * tanp) * (1 - std::exp(-jm / soil_.K));
        if (!contact_[e]) tau = 0.0;
        double dx, dy;
        if (sim_options_.shear_direction == ShearDirection::Velocity) {
            const double vm = std::hypot(vsx, vsy);
            const double den = std::sqrt(vm * vm + eps_v * eps_v);
            dx = vsx / den;
            dy = vsy / den;
        } else {
            const double den = std::max(jm, 1e-12);
            dx = jnx / den;
            dy = jny / den;
        }
        const glm::dvec3 f(-tau * A_ * dx, -vehicle_params_.lateral_force_scale * tau * A_ * dy, pn_[e] * A_);
        F += f;
        M += glm::cross(rb[e], f);
        d.thrust += f.x;
        d.N += f.z;
        if (contact_[e]) { contact_count += 1; sink_sum += z_[e]; }
    }

    // ---- lumped forces at the track
    const glm::dvec3 rc = track_center_[k];
    const glm::dvec3 vc = vel_b + glm::cross(angular_velocity_, rc);
    const double vtx = vc.x, vty = vc.y;
    const double vt2 = vtx * vtx + vty * vty + sim_options_.v_eps * sim_options_.v_eps;
    auto add = [&](const glm::dvec3& pos, const glm::dvec3& fe) { F += fe; M += glm::cross(pos, fe); };

    if (sim_options_.extra_rolling_coeff > 0 && d.N > 0) {
        const double s = -sim_options_.extra_rolling_coeff * d.N / std::sqrt(vt2);
        add(rc, glm::dvec3(s * vtx, s * vty, 0.0));
    }

    // leading edge: front when moving forward, rear when reversing
    const double vx = vtx;
    const int row = vx >= 0 ? ny - 1 : 0;
    const double off = (row == ny - 1 ? 1.0 : -1.0) * (0.5 * du_ + terrain_.RutDx());
    double Rc = 0.0, z_fresh_sum = 0.0;
    for (int c = 0; c < nx; ++c) {
        glm::dvec3 ahead = rb[Idx(row, c)];
        ahead.x += off;   // one rut cell beyond the edge
        const glm::dvec3 aw = position_ + R_ * ahead;
        const HeightMapTerrain::RutIndex ra = terrain_.GetRutIndex(aw.x, aw.y);
        const double zp_ahead = terrain_.Rut(ra.iy, ra.ix);
        const double z_edge = std::max(z_[Idx(row, c)], zp_ahead);
        z_fresh_sum += z_edge - zp_ahead;
        Rc += kb / (n + 1) * (std::pow(z_edge, n + 1) - std::pow(zp_ahead, n + 1));
    }
    Rc *= dw_;

    if (sim_options_.compaction) {
        const double vmag = std::hypot(vtx, vty);
        double fcx = 0, fcy = 0;
        if (vmag > 1e-9) {
            const double s = -Rc * std::tanh(vmag / sim_options_.v_eps) / vmag;
            fcx = s * vtx;
            fcy = s * vty;
        }
        d.F_compaction = std::hypot(fcx, fcy);
        add(rc, glm::dvec3(fcx, fcy, 0.0));
    }

    if (sim_options_.bulldozing) {
        const double zl = z_fresh_sum / nx;
        d.F_bulldoze_x = -vehicle_params_.track_width * BulldozePerWidth(zl) * std::tanh(vx / sim_options_.v_eps);
        const glm::dvec3 pos = rc + glm::dvec3(row == ny - 1 ? vehicle_params_.track_contact_length / 2 : -vehicle_params_.track_contact_length / 2, 0.0, 0.0);
        add(pos, glm::dvec3(d.F_bulldoze_x, 0.0, 0.0));
        // side walls, row by row: the wall moving into the soil pushes it
        for (int i = 0; i < ny; ++i) {
            double vy_row = 0;
            for (int c = 0; c < nx; ++c) vy_row += vb_[Idx(i, c)].y;
            vy_row /= nx;
            const bool left = vy_row >= 0;
            const double zs = left ? z_[Idx(i, nx - 1)] : z_[Idx(i, 0)];
            const double fy = -BulldozePerWidth(zs) * du_ * std::tanh(vy_row / sim_options_.v_eps);
            const glm::dvec3 posr(rc.x + u_[i], left ? rc.y + vehicle_params_.track_width / 2 : rc.y - vehicle_params_.track_width / 2, rc.z);
            add(posr, glm::dvec3(0.0, fy, 0.0));
            d.F_bulldoze_y += fy;
        }
    }

    d.sinkage = contact_count > 0 ? sink_sum / static_cast<double>(ne) : 0.0;
    d.contact_frac = contact_count / static_cast<double>(ne);
    d.belt_speed = belt;
    d.vx_track = vtx;
    return d;
}

void TrackedVehicle::Step(double dt, TrackSpeeds cmd) {
    if (moving_terrain_) {
        double xmin, xmax, ymin, ymax;
        terrain_.Extent(xmin, xmax, ymin, ymax);
        if (std::abs(position_.x - 0.5 * (xmin + xmax)) > recenter_distance_ ||
            std::abs(position_.y - 0.5 * (ymin + ymax)) > recenter_distance_)
            RecenterTerrain();
    }
    if (sim_options_.drive == DriveMode::Speed) sprocket_speeds_ = cmd;

    glm::dvec3 F_b(0.0), M_b(0.0);
    std::array<TrackDiag, 2> diags;
    for (int k = 0; k < 2; ++k) {
        glm::dvec3 F(0.0), M(0.0);
        diags[k] = TrackForces(k, dt, F, M);
        F_b += F;
        M_b += M;
    }

    std::array<double, 2> torques{};
    for (int k = 0; k < 2; ++k) {
        const double w = sprocket_speeds_[k];
        const double T_int = vehicle_params_.internal_friction * std::tanh(w / 0.05) + vehicle_params_.internal_viscous * w;
        const double T_soil = vehicle_params_.sprocket_radius * diags[k].thrust;
        if (sim_options_.drive == DriveMode::Speed) {
            torques[k] = T_soil + T_int;
        } else {
            if (vehicle_params_.sprocket_inertia <= 0)
                throw std::invalid_argument("torque drive needs vehicle.sprocket_inertia > 0");
            torques[k] = cmd[k];
            sprocket_speeds_[k] += (cmd[k] - T_soil - T_int) / vehicle_params_.sprocket_inertia * dt;
        }
    }

    // advance the belt for animation: belt speed relative to the hull
    for (int k = 0; k < 2; ++k)
        track_phase_[k] = track_path_.Wrap(track_phase_[k] + vehicle_params_.sprocket_radius * sprocket_speeds_[k] * dt);

    // rigid body, semi-implicit Euler
    const glm::dvec3 F_w = R_ * F_b + glm::dvec3(0.0, 0.0, -vehicle_params_.mass * sim_options_.gravity * g_scale_);
    velocity_ += F_w * (dt / vehicle_params_.mass);
    const glm::dvec3 Iw(I_[0] * angular_velocity_.x, I_[1] * angular_velocity_.y, I_[2] * angular_velocity_.z);
    const glm::dvec3 rhs = M_b - glm::cross(angular_velocity_, Iw);
    angular_velocity_ += glm::dvec3(Iinv_[0] * rhs.x, Iinv_[1] * rhs.y, Iinv_[2] * rhs.z) * dt;
    position_ += velocity_ * dt;
    R_ = Orthonormalise(R_ * RotExp(angular_velocity_ * dt));
    local_sim_time_ += dt;

}

}  // namespace tracked
} // namespace vehicle
} // namespace mavs
