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

// Internal Modules
#include "frenet_track.hpp"
#include "acados_mpc_solver.hpp"
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
    void velocityCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void controlLoop();

    // Helpers
    void publishControls(double steering_wheel_rad, double acceleration);
    void publishZeroControls();
    void publishVisualizations(const std::vector<StateVector>& predicted_states, double current_s);

    // ROS Publishers
    rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steering_pub_;
    rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr torques_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pred_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_path_pub_;

    // ROS Subscribers
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_sub_;
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
    utils::MPCLogger logger_;

    // Vehicle State
    double current_velocity_x_ = 0.0;
    double current_velocity_y_ = 0.0;
    double current_yaw_rate_ = 0.0;
    double current_speed_ = 0.0;
    double last_steering_angle_ = 0.0;
    double last_acceleration_cmd_ = 0.0;

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

    // Counters & Status Flags
    size_t control_loop_count_ = 0;
    bool velocity_received_ = false;
    bool track_received_ = false;
    bool is_shutting_down_ = false;
};

} // namespace mpc

#endif // ETDV_MPC_PACSIM_NODE_HPP
