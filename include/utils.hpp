#ifndef ETDV_MPC_UTILS_HPP
#define ETDV_MPC_UTILS_HPP

#include <vector>
#include <string>
#include <fstream>
#include <utility>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mpc {
namespace utils {

/**
 * @brief Extracts 2D (x, y) points from a MarkerArray message.
 */
std::pair<std::vector<double>, std::vector<double>>
extractPointsFromMarkerArray(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

/**
 * @brief Creates a Marker representing the track reference trajectory.
 */
visualization_msgs::msg::Marker createReferencePathMarker(
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    rclcpp::Time now);

/**
 * @brief Creates a Marker representing the MPC predicted horizon trajectory.
 */
visualization_msgs::msg::Marker createPredictedPathMarker(
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    rclcpp::Time now);

/**
 * @brief Creates a SPHERE_LIST Marker representing discrete predicted horizon states ("pallini").
 * Color shifts smoothly from green (stage 1) to amber/yellow (stage N).
 */
visualization_msgs::msg::Marker createPredictedSpheresMarker(
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    rclcpp::Time now);

/**
 * @brief Creates a SPHERE_LIST Marker representing discrete reference track points ("pallini").
 */
visualization_msgs::msg::Marker createReferenceSpheresMarker(
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    rclcpp::Time now);

/**
 * @brief Telemetry logger for MPC states, inputs, and solver solve times.
 */
class MPCLogger {
public:
    MPCLogger();
    ~MPCLogger();

    bool init(const std::string& log_dir);
    void close();

    void logMain(const std::string& msg, double time);
    void logState(double t, double x, double y, double psi, double v, double yaw_rate,
                  double s, double e_y, double e_psi, double progress);
    void logControl(double t, double a_cmd, double delta_cmd, double steer_wheel_cmd,
                    double t_fl, double t_fr, double t_rl, double t_rr, double solve_time_us);
    void logDetailed(double t, double x, double y, double psi, double v, double yaw_rate,
                     double s, double e_y, double e_psi, double kappa_ref, double v_target,
                     double a_cmd, double delta_cmd, double steer_wheel_cmd,
                     double t_fl, double t_fr, double t_rl, double t_rr,
                     int solver_status, double solve_time_us,
                     double pred_ey_end, double pred_v_end, double friction_util);

    struct TelemetryData {
        double time = 0.0;
        size_t lap_idx = 0;
        double s_lap = 0.0;
        double progress_pct = 0.0;
        double x = 0.0;
        double y = 0.0;
        double psi = 0.0;
        double v = 0.0;
        double yaw_rate = 0.0;
        double e_y = 0.0;
        double e_psi = 0.0;
        double kappa_ref = 0.0;
        double w_l = 0.0;
        double w_r = 0.0;
        double clearance_left = 0.0;
        double clearance_right = 0.0;
        double min_cone_clearance = 0.0;
        double v_target = 0.0;
        double delta_v = 0.0;
        double a_eff_max = 0.0;
        int gating_mode = 0;
        double a_lon = 0.0;
        double a_lat = 0.0;
        double a_total = 0.0;
        double friction_util_pct = 0.0;
        double friction_headroom = 0.0;
        double delta_cmd = 0.0;
        double steer_wheel_cmd = 0.0;
        double delta_dot = 0.0;
        double delta_dyn_offset = 0.0;
        double jerk_lon = 0.0;
        double jerk_steer = 0.0;
        double t_fl = 0.0, t_fr = 0.0, t_rl = 0.0, t_rr = 0.0;
        int solver_status = 0;
        double solve_time_us = 0.0;
        double lin_time_ms = 0.0;
        double qp_time_ms = 0.0;
        int qp_iter = 0;
        double cost_value = 0.0;
        double pred_ey_end = 0.0;
        double pred_v_end = 0.0;
        double pred_ey_1 = 0.0;
        double pred_v_1 = 0.0;
        double error_pred_ey = 0.0;
        double error_pred_v = 0.0;
    };

    void logTelemetry(const TelemetryData& d);
    void logTiming(size_t iteration, double t_sec, double total_loop_ms, double solver_ms,
                   double prep_lin_ms, double feedback_qp_ms, double proj_us, double horizon_us,
                   double publish_us, int qp_iter, int qp_status, int solver_status);

private:
    std::ofstream main_log_;
    std::ofstream state_log_;
    std::ofstream control_log_;
    std::ofstream detailed_log_;
    std::ofstream timing_log_;
    std::ofstream telemetry_log_;
};

} // namespace utils
} // namespace mpc

#endif // ETDV_MPC_UTILS_HPP
