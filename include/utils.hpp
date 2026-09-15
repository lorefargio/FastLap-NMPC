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

private:
    std::ofstream main_log_;
    std::ofstream state_log_;
    std::ofstream control_log_;
};

} // namespace utils
} // namespace mpc

#endif // ETDV_MPC_UTILS_HPP
