#include "utils.hpp"
#include <iomanip>
#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace mpc {
namespace utils {

std::pair<std::vector<double>, std::vector<double>>
extractPointsFromMarkerArray(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    std::vector<double> xs;
    std::vector<double> ys;

    if (!msg || msg->markers.empty()) {
        return {xs, ys};
    }

    size_t estimated_points = 0;
    for (const auto &marker : msg->markers) {
        estimated_points += marker.points.empty() ? 1U : marker.points.size();
    }
    xs.reserve(estimated_points);
    ys.reserve(estimated_points);

    for (const auto &marker : msg->markers) {
        if (!marker.points.empty()) {
            for (const auto &point : marker.points) {
                xs.push_back(point.x);
                ys.push_back(point.y);
            }
            continue;
        }

        xs.push_back(marker.pose.position.x);
        ys.push_back(marker.pose.position.y);
    }

    return {xs, ys};
}

visualization_msgs::msg::Marker createReferencePathMarker(
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    rclcpp::Time now)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = "mpc/reference";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.1;
    // Cyan color for reference path
    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker.points.reserve(xs.size());
    for (size_t i = 0; i < xs.size() && i < ys.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = xs[i];
        p.y = ys[i];
        p.z = 0.05;
        marker.points.push_back(p);
    }
    return marker;
}

visualization_msgs::msg::Marker createPredictedPathMarker(
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    rclcpp::Time now)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = "mpc/prediction";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.15;
    // Bright green for predicted MPC horizon
    marker.color.a = 1.0;
    marker.color.r = 0.2;
    marker.color.g = 1.0;
    marker.color.b = 0.2;

    marker.points.reserve(xs.size());
    for (size_t i = 0; i < xs.size() && i < ys.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = xs[i];
        p.y = ys[i];
        p.z = 0.12;
        marker.points.push_back(p);
    }
    return marker;
}

visualization_msgs::msg::Marker createPredictedSpheresMarker(
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    rclcpp::Time now)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = "mpc/predicted_spheres";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Sphere dimensions: 0.22m diameter "pallini"
    marker.scale.x = 0.22;
    marker.scale.y = 0.22;
    marker.scale.z = 0.22;

    size_t count = std::min(xs.size(), ys.size());
    marker.points.reserve(count);
    marker.colors.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        geometry_msgs::msg::Point p;
        p.x = xs[i];
        p.y = ys[i];
        p.z = 0.18; // Elevated to clearly float above track surface
        marker.points.push_back(p);

        // Smooth gradient: bright neon green at k=0 to amber/yellow at horizon end
        double frac = (count > 1) ? static_cast<double>(i) / static_cast<double>(count - 1) : 0.0;
        std_msgs::msg::ColorRGBA col;
        col.a = 0.95;
        col.r = 0.1 + 0.9 * frac;
        col.g = 1.0 - 0.15 * frac;
        col.b = 0.2 * (1.0 - frac);
        marker.colors.push_back(col);
    }
    return marker;
}

visualization_msgs::msg::Marker createReferenceSpheresMarker(
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    rclcpp::Time now)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = "mpc/reference_spheres";
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Sphere dimensions: 0.18m diameter cyan "pallini"
    marker.scale.x = 0.18;
    marker.scale.y = 0.18;
    marker.scale.z = 0.18;

    size_t count = std::min(xs.size(), ys.size());
    marker.points.reserve(count);
    marker.colors.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        geometry_msgs::msg::Point p;
        p.x = xs[i];
        p.y = ys[i];
        p.z = 0.08;
        marker.points.push_back(p);

        std_msgs::msg::ColorRGBA col;
        col.a = 0.85;
        col.r = 0.0;
        col.g = 0.75;
        col.b = 1.0;
        marker.colors.push_back(col);
    }
    return marker;
}

// ============================================================================
// MPCLogger Implementation
// ============================================================================

MPCLogger::MPCLogger() = default;
MPCLogger::~MPCLogger() { close(); }

bool MPCLogger::init(const std::string& log_dir) {
    try {
        std::filesystem::create_directories(log_dir);
    } catch (const std::exception& e) {
        std::cerr << "[MPCLogger] Failed to create log directory: " << e.what() << std::endl;
        return false;
    }

    main_log_.open(log_dir + "/mpc_main.log", std::ios::out | std::ios::trunc);
    state_log_.open(log_dir + "/mpc_state.csv", std::ios::out | std::ios::trunc);
    control_log_.open(log_dir + "/mpc_control.csv", std::ios::out | std::ios::trunc);
    detailed_log_.open(log_dir + "/mpc_detailed.csv", std::ios::out | std::ios::trunc);
    timing_log_.open(log_dir + "/mpc_timing.csv", std::ios::out | std::ios::trunc);
    telemetry_log_.open(log_dir + "/mpc_telemetry.csv", std::ios::out | std::ios::trunc);

    if (!main_log_.is_open() || !state_log_.is_open() || !control_log_.is_open() ||
        !detailed_log_.is_open() || !timing_log_.is_open() || !telemetry_log_.is_open()) {
        return false;
    }

    state_log_ << "time,x,y,psi,v,yaw_rate,s,e_y,e_psi,progress\n";
    state_log_ << std::fixed << std::setprecision(5);

    control_log_ << "time,a_cmd,delta_rad,steer_wheel_rad,t_fl,t_fr,t_rl,t_rr,solve_time_us\n";
    control_log_ << std::fixed << std::setprecision(5);

    detailed_log_ << "time,x,y,psi,v,yaw_rate,s,e_y,e_psi,kappa_ref,v_target,"
                  << "a_cmd,delta_cmd,steer_wheel_cmd,t_fl,t_fr,t_rl,t_rr,"
                  << "solver_status,solve_time_us,pred_ey_end,pred_v_end,friction_util\n";
    detailed_log_ << std::fixed << std::setprecision(5);

    timing_log_ << "iteration,time_sec,total_loop_ms,solver_ms,lin_time_ms,qp_time_ms,"
                << "proj_us,horizon_us,publish_us,qp_iter,qp_status,solver_status\n";
    timing_log_ << std::fixed << std::setprecision(5);

    telemetry_log_ << "time,lap_idx,s_lap,progress_pct,x,y,psi,v,yaw_rate,e_y,e_psi,kappa_ref,"
                   << "w_l,w_r,clearance_left,clearance_right,min_cone_clearance,v_target,delta_v,"
                   << "a_eff_max,gating_mode,a_lon,a_lat,a_total,friction_util_pct,friction_headroom,"
                   << "delta_cmd,steer_wheel_cmd,delta_dot,delta_dyn_offset,jerk_lon,jerk_steer,"
                   << "t_fl,t_fr,t_rl,t_rr,solver_status,solve_time_us,lin_time_ms,qp_time_ms,"
                   << "qp_iter,cost_value,pred_ey_end,pred_v_end,pred_ey_1,pred_v_1,error_pred_ey,error_pred_v\n";
    telemetry_log_ << std::fixed << std::setprecision(5);

    return true;
}

void MPCLogger::close() {
    if (main_log_.is_open()) main_log_.close();
    if (state_log_.is_open()) state_log_.close();
    if (control_log_.is_open()) control_log_.close();
    if (detailed_log_.is_open()) detailed_log_.close();
    if (timing_log_.is_open()) timing_log_.close();
    if (telemetry_log_.is_open()) telemetry_log_.close();
}

void MPCLogger::logMain(const std::string& msg, double time) {
    if (main_log_.is_open()) {
        main_log_ << "[" << std::fixed << std::setprecision(3) << time << "] " << msg << std::endl;
    }
}

void MPCLogger::logState(double t, double x, double y, double psi, double v, double yaw_rate,
                        double s, double e_y, double e_psi, double progress) 
{
    if (state_log_.is_open()) {
        state_log_ << t << "," << x << "," << y << "," << psi << ","
                   << v << "," << yaw_rate << "," << s << "," << e_y << ","
                   << e_psi << "," << progress << "\n";
    }
}

void MPCLogger::logControl(double t, double a_cmd, double delta_cmd, double steer_wheel_cmd,
                          double t_fl, double t_fr, double t_rl, double t_rr, double solve_time_us) 
{
    if (control_log_.is_open()) {
        control_log_ << t << "," << a_cmd << "," << delta_cmd << "," << steer_wheel_cmd << ","
                     << t_fl << "," << t_fr << "," << t_rl << "," << t_rr << ","
                     << solve_time_us << "\n";
    }
}

void MPCLogger::logDetailed(double t, double x, double y, double psi, double v, double yaw_rate,
                           double s, double e_y, double e_psi, double kappa_ref, double v_target,
                           double a_cmd, double delta_cmd, double steer_wheel_cmd,
                           double t_fl, double t_fr, double t_rl, double t_rr,
                           int solver_status, double solve_time_us,
                           double pred_ey_end, double pred_v_end, double friction_util)
{
    if (detailed_log_.is_open()) {
        detailed_log_ << t << "," << x << "," << y << "," << psi << ","
                      << v << "," << yaw_rate << "," << s << "," << e_y << ","
                      << e_psi << "," << kappa_ref << "," << v_target << ","
                      << a_cmd << "," << delta_cmd << "," << steer_wheel_cmd << ","
                      << t_fl << "," << t_fr << "," << t_rl << "," << t_rr << ","
                      << solver_status << "," << solve_time_us << ","
                      << pred_ey_end << "," << pred_v_end << ","
                      << friction_util << "\n";
    }
}

void MPCLogger::logTiming(size_t iteration, double t_sec, double total_loop_ms, double solver_ms,
                          double prep_lin_ms, double feedback_qp_ms, double proj_us, double horizon_us,
                          double publish_us, int qp_iter, int qp_status, int solver_status)
{
    if (timing_log_.is_open()) {
        timing_log_ << iteration << ","
                    << t_sec << ","
                    << total_loop_ms << ","
                    << solver_ms << ","
                    << prep_lin_ms << ","
                    << feedback_qp_ms << ","
                    << proj_us << ","
                    << horizon_us << ","
                    << publish_us << ","
                    << qp_iter << ","
                    << qp_status << ","
                    << solver_status << "\n";
    }
}

void MPCLogger::logTelemetry(const TelemetryData& d) {
    if (telemetry_log_.is_open()) {
        telemetry_log_ << d.time << ","
                       << d.lap_idx << ","
                       << d.s_lap << ","
                       << d.progress_pct << ","
                       << d.x << ","
                       << d.y << ","
                       << d.psi << ","
                       << d.v << ","
                       << d.yaw_rate << ","
                       << d.e_y << ","
                       << d.e_psi << ","
                       << d.kappa_ref << ","
                       << d.w_l << ","
                       << d.w_r << ","
                       << d.clearance_left << ","
                       << d.clearance_right << ","
                       << d.min_cone_clearance << ","
                       << d.v_target << ","
                       << d.delta_v << ","
                       << d.a_eff_max << ","
                       << d.gating_mode << ","
                       << d.a_lon << ","
                       << d.a_lat << ","
                       << d.a_total << ","
                       << d.friction_util_pct << ","
                       << d.friction_headroom << ","
                       << d.delta_cmd << ","
                       << d.steer_wheel_cmd << ","
                       << d.delta_dot << ","
                       << d.delta_dyn_offset << ","
                       << d.jerk_lon << ","
                       << d.jerk_steer << ","
                       << d.t_fl << ","
                       << d.t_fr << ","
                       << d.t_rl << ","
                       << d.t_rr << ","
                       << d.solver_status << ","
                       << d.solve_time_us << ","
                       << d.lin_time_ms << ","
                       << d.qp_time_ms << ","
                       << d.qp_iter << ","
                       << d.cost_value << ","
                       << d.pred_ey_end << ","
                       << d.pred_v_end << ","
                       << d.pred_ey_1 << ","
                       << d.pred_v_1 << ","
                       << d.error_pred_ey << ","
                       << d.error_pred_v << "\n";
    }
}

} // namespace utils
} // namespace mpc
