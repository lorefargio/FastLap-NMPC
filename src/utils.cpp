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

    if (!main_log_.is_open() || !state_log_.is_open() || !control_log_.is_open()) {
        return false;
    }

    state_log_ << "time,x,y,psi,v,yaw_rate,s,e_y,e_psi,progress\n";
    state_log_ << std::fixed << std::setprecision(5);

    control_log_ << "time,a_cmd,delta_rad,steer_wheel_rad,t_fl,t_fr,t_rl,t_rr,solve_time_us\n";
    control_log_ << std::fixed << std::setprecision(5);

    return true;
}

void MPCLogger::close() {
    if (main_log_.is_open()) main_log_.close();
    if (state_log_.is_open()) state_log_.close();
    if (control_log_.is_open()) control_log_.close();
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

} // namespace utils
} // namespace mpc
