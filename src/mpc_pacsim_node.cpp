#include "mpc_pacsim_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <thread>
#include <iostream>

namespace mpc {

MPCPacsimNode::MPCPacsimNode() 
    : Node("mpc_pacsim_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
{
    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "=== Initializing ETDV NMPC PACSim Node ===");

    // Parameter declarations
    this->declare_parameter("control_rate", 100.0);
    this->declare_parameter("mpc_dt", 0.05);
    this->declare_parameter("max_torque_per_wheel", 100.0);
    this->declare_parameter("outer_steering_ratio", 0.23);
    this->declare_parameter("max_lateral_error", 3.0);
    this->declare_parameter("emergency_stop", false);
    this->declare_parameter("log_dir", "/workspace/MPC_logs");
    this->declare_parameter("default_track_width", 3.0);
    this->declare_parameter("track_margin", 0.85);
    this->declare_parameter("effective_mu", 1.0);
    this->declare_parameter("max_accel", 3.5);
    this->declare_parameter("min_accel", -8.0);
    this->declare_parameter("stop_on_trajectory_complete", false);

    // Get parameters
    double control_rate = this->get_parameter("control_rate").as_double();
    control_dt_ = 1.0 / control_rate;
    mpc_dt_ = this->get_parameter("mpc_dt").as_double();
    max_torque_ = this->get_parameter("max_torque_per_wheel").as_double();
    steering_ratio_ = this->get_parameter("outer_steering_ratio").as_double();
    max_lateral_error_ = this->get_parameter("max_lateral_error").as_double();
    default_track_width_ = this->get_parameter("default_track_width").as_double();
    track_margin_ = this->get_parameter("track_margin").as_double();
    effective_mu_ = this->get_parameter("effective_mu").as_double();
    max_accel_ = this->get_parameter("max_accel").as_double();
    min_accel_ = this->get_parameter("min_accel").as_double();

    std::string log_dir = this->get_parameter("log_dir").as_string();

    // Initialize telemetry logger
    if (!logger_.init(log_dir)) {
        RCLCPP_WARN(this->get_logger(), "Failed to open MPC telemetry logs at %s", log_dir.c_str());
    } else {
        logger_.logMain("=== ETDV NMPC PACSim Node Started ===", 0.0);
        logger_.logMain("Control rate: " + std::to_string(control_rate) + " Hz | MPC dt: " + std::to_string(mpc_dt_), 0.0);
    }

    // Initialize acados C solver
    if (!solver_.init()) {
        RCLCPP_FATAL(this->get_logger(), "CRITICAL: Could not initialize acados MPC solver capsule!");
    }

    // Publishers
    steering_pub_ = this->create_publisher<pacsim::msg::StampedScalar>(
        "/pacsim/steering_setpoint", 10);
    torques_pub_ = this->create_publisher<pacsim::msg::Wheels>(
        "/pacsim/torques_max", 10);
    pred_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/mpc/predicted_path", 1);
    ref_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/mpc/reference_path", 1);

    // Subscribers
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 10, std::bind(&MPCPacsimNode::velocityCallback, this, std::placeholders::_1));

    centerline_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/pacsim/track/centerline_smoothed_front", 10, 
        std::bind(&MPCPacsimNode::centerlineCallback, this, std::placeholders::_1));

    // Control timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000.0)),
        std::bind(&MPCPacsimNode::controlLoop, this));
    control_timer_->cancel(); // Started once track arrives

    RCLCPP_INFO(this->get_logger(), "✓ ETDV NMPC Node successfully initialized.");
}

MPCPacsimNode::~MPCPacsimNode() {
    double t = (this->now() - start_time_).seconds();
    logger_.logMain("=== ETDV NMPC Node Shutting Down ===", t);
    publishZeroControls();
    logger_.close();
}

void MPCPacsimNode::velocityCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    current_velocity_x_ = msg->twist.twist.linear.x;
    current_velocity_y_ = msg->twist.twist.linear.y;
    current_yaw_rate_   = msg->twist.twist.angular.z;
    current_speed_      = std::hypot(current_velocity_x_, current_velocity_y_);

    velocity_received_ = true;
}

void MPCPacsimNode::centerlineCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    if (!msg || msg->markers.empty()) return;

    auto [xs, ys] = utils::extractPointsFromMarkerArray(msg);
    if (xs.size() < 5 || xs.size() != ys.size()) return;

    std::string input_frame = msg->markers.front().header.frame_id;
    if (input_frame.empty()) input_frame = "map";

    // If marker points are published in car frame or another frame, transform to map
    if (input_frame != "map") {
        try {
            const auto tf_map_from_input = tf_buffer_.lookupTransform(
                "map", input_frame, tf2::TimePointZero);

            tf2::Quaternion q(
                tf_map_from_input.transform.rotation.x,
                tf_map_from_input.transform.rotation.y,
                tf_map_from_input.transform.rotation.z,
                tf_map_from_input.transform.rotation.w);
            tf2::Matrix3x3 rot(q);
            double roll, pitch, yaw;
            rot.getRPY(roll, pitch, yaw);

            const double c = std::cos(yaw);
            const double s = std::sin(yaw);
            const double tx = tf_map_from_input.transform.translation.x;
            const double ty = tf_map_from_input.transform.translation.y;

            for (size_t i = 0; i < xs.size(); ++i) {
                double x_local = xs[i];
                double y_local = ys[i];
                xs[i] = tx + c * x_local - s * y_local;
                ys[i] = ty + s * x_local + c * y_local;
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Centerline TF lookup failed: %s", ex.what());
            return;
        }
    }

    // Build or update reference track spline
    if (track_.build(xs, ys, false, default_track_width_)) {
        if (!track_received_) {
            track_received_ = true;
            control_timer_->reset();
            RCLCPP_INFO(this->get_logger(), "✓ Reference track received (Length: %.2f m). Control loop STARTED.", 
                track_.getTrackLength());
        }
        // Publish reference trajectory visualization
        ref_path_pub_->publish(utils::createReferencePathMarker(xs, ys, this->now()));
    }
}

void MPCPacsimNode::controlLoop() {
    double current_time = (this->now() - start_time_).seconds();

    if (!track_received_ || !velocity_received_ || !solver_.isInitialized()) {
        return;
    }

    if (this->get_parameter("emergency_stop").as_bool()) {
        publishZeroControls();
        if (!is_shutting_down_) {
            logger_.logMain("Emergency stop detected. Shutting down.", current_time);
            is_shutting_down_ = true;
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                rclcpp::shutdown();
            }).detach();
        }
        return;
    }

    // 1. Vehicle Pose from TF (map -> car)
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform("map", "car", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        return;
    }

    double x_cart = transform.transform.translation.x;
    double y_cart = transform.transform.translation.y;

    tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, psi_cart;
    m.getRPY(roll, pitch, psi_cart);

    // 2. Project vehicle pose onto reference track spline
    auto [s, e_y, e_psi, curvature] = track_.cartesianToFrenet(x_cart, y_cart, psi_cart);

    // Heading error wrap-around safety
    e_psi = FrenetTrack::normalizeAngle(e_psi);

    // Lateral boundary safety check
    if (std::abs(e_y) > max_lateral_error_) {
        RCLCPP_ERROR(this->get_logger(), "SAFETY: Lateral error limit exceeded: %.2f m! Emergency Stop.", e_y);
        logger_.logMain("SAFETY: Lateral error limit exceeded: " + std::to_string(e_y), current_time);
        this->set_parameter(rclcpp::Parameter("emergency_stop", true));
        publishZeroControls();
        return;
    }

    // 3. Set Initial State Constraint x0 in acados: [s_rel=0.0, e_y, e_psi, v, delta]
    // Using relative s=0.0 ensures the solver is invariant to rolling track segment resets!
    StateVector x0 = {0.0, e_y, e_psi, std::max(current_speed_, 0.5), last_steering_angle_};
    solver_.setInitialState(x0);

    // 4. Update track curvature and boundaries along prediction horizon
    double preview_speed = std::max(current_speed_, 3.0);
    for (int k = 0; k <= MPC_N; ++k) {
        double s_stage = s + k * preview_speed * mpc_dt_;
        auto [rx, ry, rpsi, kappa_k] = track_.getReferencePoint(s_stage);
        kappa_k = std::clamp(kappa_k, -2.5, 2.5);

        solver_.setStageParameters(k, kappa_k, track_.getLeftWidth(), track_.getRightWidth(), effective_mu_);
    }

    // 5. Solve the Optimal Control Problem (Real-Time Iteration)
    auto result = solver_.solve();

    // 6. Extract Optimal Actuation with robust fallback
    double a_opt = 0.0;
    double delta_target = last_steering_angle_;

    if (result.status == 0 || result.status == 2) {
        a_opt = result.optimal_u[0];
        delta_target = result.target_steering_angle;

        if (std::isnan(a_opt) || std::isinf(a_opt) || std::isnan(delta_target) || std::isinf(delta_target)) {
            RCLCPP_WARN(this->get_logger(), "Non-finite values from MPC solver! Using safe fallback.");
            a_opt = -1.5;
            delta_target = last_steering_angle_;
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "acados solver warning status: %d (solve time: %.1f us). Applying safe deceleration.", 
            result.status, result.solve_time_us);
        a_opt = -2.0; // Controlled safe braking
        delta_target = last_steering_angle_ * 0.95;
    }

    // Bound controls
    a_opt = std::clamp(a_opt, min_accel_, max_accel_);
    delta_target = std::clamp(delta_target, -0.52, 0.52);

    // Convert wheel steering angle to steering wheel command
    double steering_wheel_cmd = delta_target / steering_ratio_;

    // Publish to PACSim
    publishControls(steering_wheel_cmd, a_opt);

    // 7. Visualizations & Telemetry
    publishVisualizations(result.predicted_states, s);

    double progress = (track_.getTrackLength() > 0.0) ? (s / track_.getTrackLength()) : 0.0;
    logger_.logState(current_time, x_cart, y_cart, psi_cart, current_speed_, current_yaw_rate_, s, e_y, e_psi, progress);

    last_steering_angle_ = delta_target;
    last_acceleration_cmd_ = a_opt;
    control_loop_count_++;
}

void MPCPacsimNode::publishControls(double steering_wheel_rad, double acceleration) {
    auto stamp = this->now();
    double t = (stamp - start_time_).seconds();

    // 1. Steering setpoint
    pacsim::msg::StampedScalar steer_msg;
    steer_msg.stamp = stamp;
    steer_msg.value = steering_wheel_rad;
    steering_pub_->publish(steer_msg);

    // 2. Wheel Torques mapping (matching etdv_pid baseline)
    pacsim::msg::Wheels torque_msg;
    torque_msg.stamp = stamp;

    double t_fl = 0.0, t_fr = 0.0, t_rl = 0.0, t_rr = 0.0;

    if (acceleration > 0.0) {
        // Rear-Wheel Drive (RWD) acceleration
        double norm_acc = std::clamp(acceleration / max_accel_, 0.0, 1.0);
        double torque = norm_acc * max_torque_;
        t_rl = torque;
        t_rr = torque;
    } else {
        // 4-Wheel Braking (60/40 Front/Rear bias with brake gain 7.5)
        double norm_brake = std::clamp(std::abs(acceleration) / std::abs(min_accel_), 0.0, 1.0);
        double brake_torque = -norm_brake * max_torque_ * 7.5;
        t_fl = brake_torque * 0.3;
        t_fr = brake_torque * 0.3;
        t_rl = brake_torque * 0.2;
        t_rr = brake_torque * 0.2;
    }

    torque_msg.fl = t_fl;
    torque_msg.fr = t_fr;
    torque_msg.rl = t_rl;
    torque_msg.rr = t_rr;
    torques_pub_->publish(torque_msg);

    logger_.logControl(t, acceleration, last_steering_angle_, steering_wheel_rad, t_fl, t_fr, t_rl, t_rr, 0.0);
}

void MPCPacsimNode::publishZeroControls() {
    pacsim::msg::StampedScalar steer_msg;
    steer_msg.stamp = this->now();
    steer_msg.value = 0.0;
    steering_pub_->publish(steer_msg);

    pacsim::msg::Wheels torque_msg;
    torque_msg.stamp = this->now();
    torque_msg.fl = 0.0;
    torque_msg.fr = 0.0;
    torque_msg.rl = 0.0;
    torque_msg.rr = 0.0;
    torques_pub_->publish(torque_msg);
}

void MPCPacsimNode::publishVisualizations(const std::vector<StateVector>& predicted_states, double current_s) {
    if (predicted_states.empty() || !track_.isReady()) return;

    std::vector<double> x_pred, y_pred;
    x_pred.reserve(predicted_states.size());
    y_pred.reserve(predicted_states.size());

    for (const auto& x_k : predicted_states) {
        double s_rel = x_k[0]; // Relative distance ahead along prediction horizon
        double ey_k = x_k[1];
        double epsi_k = x_k[2];

        auto [cx, cy, cpsi] = track_.frenetToCartesian(current_s + s_rel, ey_k, epsi_k);
        x_pred.push_back(cx);
        y_pred.push_back(cy);
    }

    pred_path_pub_->publish(utils::createPredictedPathMarker(x_pred, y_pred, this->now()));
}

} // namespace mpc

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mpc::MPCPacsimNode>();

    std::signal(SIGINT, [](int) {
        rclcpp::shutdown();
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
