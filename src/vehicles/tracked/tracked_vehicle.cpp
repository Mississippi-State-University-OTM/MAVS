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

namespace mavs {
namespace vehicle {
namespace tracked {

TrackedVehicle::TrackedVehicle(std::string input_file) {

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

    // Get the terrain inputs members
    if (d.HasMember("Controller") && d["Controller"].IsObject()) {
        const rapidjson::Value& controller = d["Controller"];
        std::string controller_type = "Ramp"; // Can be "Ramp", "RampThenTurn" or "ConstantTorque"
        if (controller.HasMember("Type") && controller["Type"].IsString()) {
            controller_type = controller["Type"].GetString();
        }
        if (controller_type == "Ramp") {
            double wl = 7.87402;
            double wr = 7.87402;
            if (controller.HasMember("wl") && controller["wl"].IsNumber()) {
                wl = controller["wl"].GetDouble();
            }
            if (controller.HasMember("wr") && controller["wr"].IsNumber()) {
                wr = controller["wr"].GetDouble();
            }
            controller_ = controller::Ramp(wl, wr);
        }
        else if (controller_type == "RampThenTurn") {
            double w = 7.87402;
            if (controller.HasMember("w") && controller["w"].IsNumber()) {
                w = controller["w"].GetDouble();
            }
            controller_ = controller::RampThenTurn(w);
        }
        else if (controller_type == "ConstantTorque") {

        }
        else if (controller_type == "External") {

        }
        else {
            std::cerr << "ERROR: Controller Type " << controller_type << " not recognized, exiting." << std::endl;
            exit(91);
        }
    }
    else {
        std::cout << "WARNING: No field for \"Controller\" in simulation input file, using default." << std::endl;
    }

    Init();

    // Settle the vehicle into the inital position
    SetPose(sim_options_.initial_position_x, sim_options_.initial_position_y, sim_options_.initial_yaw);
    Settle(1.5, 1e-3);
    
}

TrackedVehicle::TrackedVehicle(const TrackedVehicleParams& vehicle, const TrackedSoil& soil, HeightMapTerrain& terrain, const SimOptions& options) {

    vehicle_params_ = vehicle;
    soil_ = soil;
    terrain_ = terrain;
    sim_options_ = options;

    Init();
}

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

    //SetPose(sim_options_.initial_position_x, sim_options_.initial_position_y, sim_options_.initial_yaw);
}

void TrackedVehicle::SetPose(double x, double y, double yaw_radians) {
    R_ = RFromRpy(0, 0, yaw_radians);
    p_ = glm::dvec3(x, y, terrain_.Height(x, y) + vehicle_params_.cg_height);
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
    const glm::dvec3 vel_b = glm::transpose(R_) * vel_;
    TrackDiag d;
    F = glm::dvec3(0.0);
    M = glm::dvec3(0.0);

    // ---- pass 1: sinkage and normal pressure (rut read before any update)
    for (size_t e = 0; e < ne; ++e) {
        const glm::dvec3 rw = p_ + R_ * rb[e];
        const double hs = terrain_.Height(rw.x, rw.y);
        ri_[e] = terrain_.GetRutIndex(rw.x, rw.y);
        const double zp = terrain_.Rut(ri_[e].iy, ri_[e].ix);
        const double z = (hs - rw.z) * cosn;
        const glm::dvec3 vb = vel_b + glm::cross(omega_, rb[e]);
        const bool loading = z > zp;
        const double p_load = kb * std::pow(std::max(z, 0.0), n);
        const double p_top = kb * std::pow(zp, n);
        const double ku = zp > 0 ? p_top / std::max(soil_.rebound * zp, 1e-12) : 0.0;
        const double p_unl = std::max(p_top - ku * (zp - z), 0.0);
        const double p_st = loading ? p_load : p_unl;
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
    const double belt = vehicle_params_.sprocket_radius * sprocket_[k];
    std::vector<std::array<double, 2>>& j = j_[k];
    Advect(j, belt * dt);
    const double a = omega_.z * dt;
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
        const glm::dvec3 f(-tau * A_ * dx, -tau * A_ * dy, pn_[e] * A_);
        F += f;
        M += glm::cross(rb[e], f);
        d.thrust += f.x;
        d.N += f.z;
        if (contact_[e]) { contact_count += 1; sink_sum += z_[e]; }
    }

    // ---- lumped forces at the track
    const glm::dvec3 rc = track_center_[k];
    const glm::dvec3 vc = vel_b + glm::cross(omega_, rc);
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
        const glm::dvec3 aw = p_ + R_ * ahead;
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
    if (sim_options_.drive == DriveMode::Speed) sprocket_ = cmd;

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
        const double w = sprocket_[k];
        const double T_int = vehicle_params_.internal_friction * std::tanh(w / 0.05) + vehicle_params_.internal_viscous * w;
        const double T_soil = vehicle_params_.sprocket_radius * diags[k].thrust;
        if (sim_options_.drive == DriveMode::Speed) {
            torques[k] = T_soil + T_int;
        } else {
            if (vehicle_params_.sprocket_inertia <= 0)
                throw std::invalid_argument("torque drive needs vehicle.sprocket_inertia > 0");
            torques[k] = cmd[k];
            sprocket_[k] += (cmd[k] - T_soil - T_int) / vehicle_params_.sprocket_inertia * dt;
        }
    }

    // rigid body, semi-implicit Euler
    const glm::dvec3 F_w = R_ * F_b + glm::dvec3(0.0, 0.0, -vehicle_params_.mass * sim_options_.gravity * g_scale_);
    vel_ += F_w * (dt / vehicle_params_.mass);
    const glm::dvec3 Iw(I_[0] * omega_.x, I_[1] * omega_.y, I_[2] * omega_.z);
    const glm::dvec3 rhs = M_b - glm::cross(omega_, Iw);
    omega_ += glm::dvec3(Iinv_[0] * rhs.x, Iinv_[1] * rhs.y, Iinv_[2] * rhs.z) * dt;
    p_ += vel_ * dt;
    R_ = Orthonormalise(R_ * RotExp(omega_ * dt));
    elapsed_time_ += dt;

    current_simulation_state_.t = elapsed_time_;
    current_simulation_state_.diags = diags;
    current_simulation_state_.torques = torques;
    for (int k = 0; k < 2; ++k) {
        const double belt = diags[k].belt_speed;
        current_simulation_state_.slips[k] = std::abs(belt) > 1e-6 ? 1 - diags[k].vx_track / belt : 0.0;
    }
    current_simulation_state_.v_body = glm::transpose(R_) * vel_;
    current_simulation_state_.F_body = F_b;
    current_simulation_state_.M_body = M_b;

}

void TrackedVehicle::Settle(double duration, double dt) {
    const DriveMode drive = sim_options_.drive;
    sim_options_.drive = DriveMode::Speed;
    const int n = static_cast<int>(duration / dt);
    for (int i = 0; i < n; ++i) {
        g_scale_ = std::min(1.0, 1.5 * (i + 1) / n);
        Step(dt, {0.0, 0.0});
    }
    g_scale_ = 1.0;
    sim_options_.drive = drive;
    elapsed_time_ = 0.0;
}

VehicleState TrackedVehicle::GetCurrentVehicleState() const {
    VehicleState s;
    RpyFromR(R_, s.roll, s.pitch, s.yaw);
    const glm::dvec3 vb = glm::transpose(R_) * vel_;
    s.t = elapsed_time_;
    s.x = p_.x; s.y = p_.y; s.z = p_.z;
    s.vx = vb.x; s.vy = vb.y; s.vz = vb.z;
    s.yaw_rate = omega_.z;
    s.omega_left = sprocket_[0];
    s.omega_right = sprocket_[1];
    return s;
}

std::vector<LogRow> TrackedVehicle::Run() {
    std::vector<LogRow> log;
    const long nsteps = std::lround(sim_options_.simulation_duration / sim_options_.dt);
    for (long i = 0; i < nsteps; ++i) {
        Step(sim_options_.dt, controller_(elapsed_time_, *this));
        const SimulationState& out = current_simulation_state_;
        if (i % sim_options_.log_every == 0 || i == nsteps - 1) {
            const TrackDiag& d0 = out.diags[0];
            const TrackDiag& d1 = out.diags[1];
            LogRow r;
            r.s = GetCurrentVehicleState();
            r.thrust_left = d0.thrust;          r.thrust_right = d1.thrust;
            r.torque_left = out.torques[0];     r.torque_right = out.torques[1];
            r.slip_left = out.slips[0];         r.slip_right = out.slips[1];
            r.sinkage_left = d0.sinkage;        r.sinkage_right = d1.sinkage;
            r.N_left = d0.N;                    r.N_right = d1.N;
            r.F_compaction = d0.F_compaction + d1.F_compaction;
            r.F_bulldoze_x = d0.F_bulldoze_x + d1.F_bulldoze_x;
            r.F_bulldoze_y = d0.F_bulldoze_y + d1.F_bulldoze_y;
            r.outside_map = d0.outside_map + d1.outside_map;
            log.push_back(r);
        }
    }
    return log;
}

}  // namespace tracked
} // namespace vehicle
} // namespace mavs