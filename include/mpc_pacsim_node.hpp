#ifndef ETDV_MPC_PACSIM_NODE_HPP
#define ETDV_MPC_PACSIM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PACSim Messages
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include "pacsim/msg/track.hpp"

// Internal Modules
#include "frenet_track.hpp"
#include "acados_mpc_solver.hpp"
#include "speed_governor.hpp"
#include "utils.hpp"

#include <memory>
#include <vector>
#include <string>

namespace mpc {

class MPCPacsimNode : public rclcpp::Node {
public:
    MPCPacsimNode();
    ~MPCPacsimNode();

private:
    // Callbacks
    void centerlineCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void landmarksCallback(const pacsim::msg::Track::SharedPtr msg);
    void velocityCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void controlLoop();

    // Helpers
    std::tuple<double, double, double, double> publishControls(double steering_wheel_rad, double acceleration, double solve_time_us);
    void publishZeroControls();
    void publishVisualizations(const std::vector<StateVector>& predicted_states, double current_s);
    void publishReferencePath();
    double declareAndGetDoubleParam(const std::string& name, double default_val);

    // ROS Publishers
    rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steering_pub_;
    rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr torques_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pred_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pred_spheres_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_spheres_pub_;

    // ROS Subscribers
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_sub_;
    rclcpp::Subscription<pacsim::msg::Track>::SharedPtr landmarks_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_sub_;

    // ROS Timer & Time
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time start_time_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Core Controllers & Track
    FrenetTrack track_;
    AcadosMpcSolver solver_;
    SpeedGovernor speed_governor_;
    utils::MPCLogger logger_;

    // Vehicle State
    double current_velocity_x_ = 0.0;
    double current_velocity_y_ = 0.0;
    double current_yaw_rate_ = 0.0;
    double current_speed_ = 0.0;
    double last_steering_angle_ = 0.0;
    double last_acceleration_cmd_ = 0.0;
    double last_s_ = 0.0;

    // Node Parameters
    double control_dt_ = 0.01;
    double mpc_dt_ = 0.05;
    double max_torque_ = 100.0;
    double steering_ratio_ = 0.23;
    double max_lateral_error_ = 3.0;
    double default_track_width_ = 3.0;
    double track_margin_ = 0.85;
    double effective_mu_ = 1.0;
    double max_accel_ = 3.5;
    double min_accel_ = -8.0;
    double speed_scale_ = 0.90;
    double max_straight_speed_ = 22.5;
    std::string speed_limits_csv_ = "";
    std::string centerline_topic_ = "/pacsim/track/centerline_smoothed";

    // Launch & Corner Exit Acceleration Governor
    double low_speed_threshold_ = 7.0;
    double high_speed_threshold_ = 13.0;
    double standing_launch_accel_ = 2.8;
    double low_speed_max_accel_ = 0.85;
    double corner_exit_steer_derate_ = 0.75;
    double max_accel_slew_rate_ = 6.0;
    double max_decel_slew_rate_ = 25.0;
    double a_brake_ = 5.8;
    double understeer_gradient_ = 0.0012; // rad / (m/s^2) for tire slip angle compensation

    // Counters & Status Flags
    size_t control_loop_count_ = 0;
    bool velocity_received_ = false;
    bool track_received_ = false;
    bool is_shutting_down_ = false;

    // Timing & Performance Statistics
    double loop_time_sum_ms_ = 0.0;
    double max_loop_time_ms_ = 0.0;
    double min_loop_time_ms_ = 1e6;
    double solve_time_sum_ms_ = 0.0;
    double max_solve_time_ms_ = 0.0;
    size_t overruns_count_ = 0;
};

} // namespace mpc

#endif // ETDV_MPC_PACSIM_NODE_HPP
