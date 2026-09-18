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

    // Parameter declarations & loading (type-safe for both float and integer YAML values)
    this->declare_parameter("emergency_stop", false);
    this->declare_parameter("stop_on_trajectory_complete", false);
    this->declare_parameter("log_dir", "MPC_logs");
    this->declare_parameter("centerline_topic", "/pacsim/track/centerline_smoothed");
    this->declare_parameter("speed_limits_csv", "");

    double control_rate = declareAndGetDoubleParam("control_rate", 100.0);
    control_dt_ = 1.0 / control_rate;
    mpc_dt_ = declareAndGetDoubleParam("mpc_dt", 0.05);
    max_torque_ = declareAndGetDoubleParam("max_torque_per_wheel", 100.0);
    steering_ratio_ = declareAndGetDoubleParam("outer_steering_ratio", 0.23);
    max_lateral_error_ = declareAndGetDoubleParam("max_lateral_error", 3.0);
    default_track_width_ = declareAndGetDoubleParam("default_track_width", 3.0);
    track_margin_ = declareAndGetDoubleParam("track_margin", 0.80);
    effective_mu_ = declareAndGetDoubleParam("effective_mu", 1.0);
    max_accel_ = declareAndGetDoubleParam("max_accel", 4.8);
    min_accel_ = declareAndGetDoubleParam("min_accel", -8.0);
    speed_scale_ = declareAndGetDoubleParam("speed_scale", 1.00);
    max_straight_speed_ = declareAndGetDoubleParam("max_straight_speed", 25.0);
    low_speed_threshold_ = declareAndGetDoubleParam("low_speed_threshold", 6.0);
    high_speed_threshold_ = declareAndGetDoubleParam("high_speed_threshold", 12.0);
    standing_launch_accel_ = declareAndGetDoubleParam("standing_launch_accel", 4.8);
    low_speed_max_accel_ = declareAndGetDoubleParam("low_speed_max_accel", 2.20);
    corner_exit_steer_derate_ = declareAndGetDoubleParam("corner_exit_steer_derate", 0.55);
    max_accel_slew_rate_ = declareAndGetDoubleParam("max_accel_slew_rate", 9.0);
    max_decel_slew_rate_ = declareAndGetDoubleParam("max_decel_slew_rate", 25.0);
    a_brake_ = declareAndGetDoubleParam("a_brake", 5.0);
    understeer_gradient_ = declareAndGetDoubleParam("understeer_gradient", 0.0008);

    centerline_topic_ = this->get_parameter("centerline_topic").as_string();
    speed_limits_csv_ = this->get_parameter("speed_limits_csv").as_string();

    // Configure SpeedGovernor (from Velocità limite.xlsx)
    speed_governor_.configure(speed_scale_, max_straight_speed_);
    if (!speed_limits_csv_.empty()) {
        if (speed_governor_.loadFromCsv(speed_limits_csv_)) {
            RCLCPP_INFO(this->get_logger(), "SpeedGovernor: Loaded custom CSV: %s (%zu points)",
                        speed_limits_csv_.c_str(), speed_governor_.getTableSize());
        } else {
            RCLCPP_WARN(this->get_logger(), "SpeedGovernor: Could not load %s, using embedded table (%zu points)",
                        speed_limits_csv_.c_str(), speed_governor_.getTableSize());
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "SpeedGovernor: Using embedded table from 'Velocità limite.xlsx' (%zu points, scale=%.2f, max_v=%.1f m/s)",
                    speed_governor_.getTableSize(), speed_scale_, max_straight_speed_);
    }

    std::string log_dir = this->get_parameter("log_dir").as_string();

    // Initialize telemetry logger
    if (!logger_.init(log_dir)) {
        RCLCPP_WARN(this->get_logger(), "Failed to open MPC telemetry logs at %s", log_dir.c_str());
    } else {
        logger_.logMain("=== ETDV NMPC PACSim Node Started ===", 0.0);
        logger_.logMain("Control rate: " + std::to_string(control_rate) + " Hz | MPC dt: " + std::to_string(mpc_dt_), 0.0);
        logger_.logMain("Centerline topic: " + centerline_topic_ + " | Track margin: " + std::to_string(track_margin_) +
                        " m | max_lateral_error: " + std::to_string(max_lateral_error_) + " m", 0.0);
        logger_.logMain("SpeedGovernor: scale=" + std::to_string(speed_scale_) +
                        " | max_straight=" + std::to_string(max_straight_speed_) + " m/s", 0.0);
    }

    RCLCPP_INFO(this->get_logger(), "Config: max_lateral_error=%.2f m | track_margin=%.2f m | speed_scale=%.2f | centerline=%s",
                max_lateral_error_, track_margin_, speed_scale_, centerline_topic_.c_str());

    // Initialize acados C solver
    if (!solver_.init()) {
        RCLCPP_FATAL(this->get_logger(), "CRITICAL: Could not initialize acados MPC solver capsule!");
    }

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    // Publishers
    steering_pub_ = this->create_publisher<pacsim::msg::StampedScalar>(
        "/pacsim/steering_setpoint", 10);
    torques_pub_ = this->create_publisher<pacsim::msg::Wheels>(
        "/pacsim/torques_max", 10);
    pred_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/mpc/predicted_path", 1);
    pred_spheres_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/mpc/predicted_spheres", 1);
    ref_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/mpc/reference_path", latched_qos);
    ref_spheres_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/mpc/reference_spheres", 1);

    // Subscribers
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 10, std::bind(&MPCPacsimNode::velocityCallback, this, std::placeholders::_1));

    if (centerline_topic_ == "/pacsim/track/landmarks") {
        landmarks_sub_ = this->create_subscription<pacsim::msg::Track>(
            centerline_topic_, 10,
            std::bind(&MPCPacsimNode::landmarksCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to Track landmarks on: %s", centerline_topic_.c_str());
    } else {
        centerline_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            centerline_topic_, latched_qos, 
            std::bind(&MPCPacsimNode::centerlineCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to Centerline MarkerArray on: %s", centerline_topic_.c_str());
    }

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

    // Spline update gating:
    // If we already have a long/global track (>80m), do not rebuild from front slices.
    // If it's a rolling front window, update whenever vehicle progressed >= 1.0m or remaining lookahead < 32.0m.
    if (track_received_) {
        double track_len = track_.getTrackLength();
        if (track_len > 80.0) {
            return; // Full global track already active!
        }
        double remaining = track_len - last_s_;
        if (last_s_ < 1.0 && remaining >= 32.0) {
            return; // Window is already fresh and has plenty of lookahead
        }
    }

    std::string input_frame = msg->markers.front().header.frame_id;
    if (input_frame.empty()) input_frame = "map";

    // If marker points are published in car frame or another frame, transform to map
    if (input_frame != "map") {
        try {
            rclcpp::Time marker_stamp = msg->markers.front().header.stamp;
            geometry_msgs::msg::TransformStamped tf_map_from_input;
            if (tf_buffer_.canTransform("map", input_frame, marker_stamp, tf2::durationFromSec(0.02))) {
                tf_map_from_input = tf_buffer_.lookupTransform("map", input_frame, marker_stamp);
            } else {
                tf_map_from_input = tf_buffer_.lookupTransform("map", input_frame, tf2::TimePointZero);
            }

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

    // Closed circuit testing: full track (>50 points) is always treated as a closed loop
    bool is_closed = (xs.size() > 50);

    // Never apply Gaussian smoothing to pre-smoothed PACSim centerline points!
    // Gaussian filtering on curves cuts the apex inward by 0.3-0.5m, pulling the car into cones.
    bool apply_smoothing = false;

    if (track_.build(xs, ys, is_closed, default_track_width_, apply_smoothing)) {
        if (!cached_blue_cones_.empty() && !cached_yellow_cones_.empty()) {
            track_.setBoundaryCones(cached_blue_cones_, cached_yellow_cones_);
        }
        if (!track_received_) {
            track_received_ = true;
            control_timer_->reset();
            RCLCPP_INFO(this->get_logger(), "✓ Reference track received (Length: %.2f m, Closed: %s). Control loop STARTED.", 
                track_.getTrackLength(), is_closed ? "YES" : "NO");
        }
        publishReferencePath();
    }
}

void MPCPacsimNode::publishReferencePath() {
    if (!track_.isReady()) return;
    std::vector<double> smooth_rx, smooth_ry;
    double tlen = track_.getTrackLength();
    size_t n_samples = static_cast<size_t>(tlen / 0.25) + 1;
    smooth_rx.reserve(n_samples);
    smooth_ry.reserve(n_samples);
    for (size_t i = 0; i < n_samples; ++i) {
        double s_samp = std::min(i * 0.25, tlen);
        auto [rx, ry, rpsi, rkappa] = track_.getReferencePoint(s_samp);
        smooth_rx.push_back(rx);
        smooth_ry.push_back(ry);
    }
    ref_path_pub_->publish(utils::createReferencePathMarker(smooth_rx, smooth_ry, this->now()));
}

void MPCPacsimNode::landmarksCallback(const pacsim::msg::Track::SharedPtr msg) {
    if (!msg || msg->left_lane.size() < 3 || msg->right_lane.size() < 3) return;

    std::vector<Eigen::Vector2d> blue_cones, yellow_cones;
    blue_cones.reserve(msg->left_lane.size());
    yellow_cones.reserve(msg->right_lane.size());

    for (const auto& lm : msg->left_lane) {
        blue_cones.emplace_back(lm.pose.pose.position.x, lm.pose.pose.position.y);
    }
    for (const auto& lm : msg->right_lane) {
        yellow_cones.emplace_back(lm.pose.pose.position.x, lm.pose.pose.position.y);
    }

    cached_blue_cones_ = blue_cones;
    cached_yellow_cones_ = yellow_cones;

    // If track is already initialized from centerline, directly update local boundary splines
    if (track_.isReady()) {
        track_.setBoundaryCones(blue_cones, yellow_cones);
        return;
    }

    std::vector<double> xs, ys;
    size_t n = blue_cones.size();
    xs.reserve(n);
    ys.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        size_t closest_yellow = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t j = 0; j < yellow_cones.size(); ++j) {
            double dist = (blue_cones[i] - yellow_cones[j]).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_yellow = j;
            }
        }
        xs.push_back(0.5 * (blue_cones[i].x() + yellow_cones[closest_yellow].x()));
        ys.push_back(0.5 * (blue_cones[i].y() + yellow_cones[closest_yellow].y()));
    }

    if (track_.build(xs, ys, true, default_track_width_, true)) {
        track_.setBoundaryCones(blue_cones, yellow_cones);
        if (!track_received_) {
            track_received_ = true;
            control_timer_->reset();
            RCLCPP_INFO(this->get_logger(), "✓ Global landmarks track received (Length: %.2f m). Control loop STARTED.", 
                track_.getTrackLength());
        }
        publishReferencePath();
    }
}

void MPCPacsimNode::controlLoop() {
    auto t_loop_start = std::chrono::high_resolution_clock::now();
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
    auto t_proj_start = std::chrono::high_resolution_clock::now();
    auto [s, e_y, e_psi, curvature] = track_.cartesianToFrenet(x_cart, y_cart, psi_cart);
    auto t_proj_end = std::chrono::high_resolution_clock::now();
    last_s_ = s;

    // Heading error wrap-around safety
    e_psi = FrenetTrack::normalizeAngle(e_psi);

    // Lateral boundary safety check
    if (std::abs(e_y) > max_lateral_error_) {
        RCLCPP_ERROR(this->get_logger(), "SAFETY: Lateral error limit exceeded: %.2f m (limit: %.2f m)! Emergency Stop.",
                     e_y, max_lateral_error_);
        logger_.logMain("SAFETY: Lateral error limit exceeded: " + std::to_string(e_y) +
                        " (limit: " + std::to_string(max_lateral_error_) + " m)", current_time);
        this->set_parameter(rclcpp::Parameter("emergency_stop", true));
        publishZeroControls();
        return;
    }

    // 3. Set Initial State Constraint x0 in acados: [s_rel=0.0, e_y, e_psi, v, delta]
    // Using relative s=0.0 ensures the solver is invariant to rolling track segment resets!
    StateVector x0 = {0.0, e_y, e_psi, std::max(current_speed_, 0.5), last_mpc_steering_};
    solver_.setInitialState(x0);

    // 4. Update track curvature, speed references, and boundaries along prediction horizon
    double preview_speed = std::max(current_speed_, 3.0);

    // Dynamic launch governor: allows energetic standing launch on straights and derates with steering angle
    double a_eff_max = speed_governor_.computeEffectiveMaxAccel(
        current_speed_, last_steering_angle_,
        low_speed_threshold_, high_speed_threshold_,
        low_speed_max_accel_, max_accel_,
        standing_launch_accel_,
        corner_exit_steer_derate_, 0.52);

    // Kamm friction circle protection: when lateral acceleration is high, cap longitudinal drive force
    // to prevent tire breakaway / snap oversteer on corner exits
    double a_lat_est = std::abs((current_speed_ * current_speed_ / 1.53) * std::tan(last_steering_angle_));
    double mu_g = effective_mu_ * 9.81;
    if (a_lat_est > 5.0) {
        double a_lat_safe = std::min(a_lat_est, mu_g * 0.92);
        double a_lon_kamm = std::sqrt(std::max(0.4, (mu_g * 0.92) * (mu_g * 0.92) - a_lat_safe * a_lat_safe));
        a_eff_max = std::min(a_eff_max, a_lon_kamm);
    }

    // Build spatial preview coordinates (N+1 horizon stages + extended lookahead points)
    std::vector<double> s_preview;
    std::vector<double> kappa_preview;
    std::vector<double> free_widths;
    s_preview.reserve(MPC_N + 1 + 25);
    kappa_preview.reserve(MPC_N + 1 + 25);
    free_widths.reserve(MPC_N + 1 + 25);

    for (int k = 0; k <= MPC_N; ++k) {
        double s_stage = s + k * preview_speed * mpc_dt_;
        auto [rx, ry, rpsi, kappa_k] = track_.getReferencePoint(s_stage);
        s_preview.push_back(s_stage);
        kappa_preview.push_back(std::clamp(kappa_k, -2.5, 2.5));

        double wl = track_.getLeftWidth(s_stage);
        double wr = track_.getRightWidth(s_stage);
        double w_free = std::max(0.0, std::min(wl, wr) - track_margin_);
        free_widths.push_back(w_free);
    }

    // Extended lookahead beyond stage N (up to 65 meters ahead of vehicle)
    // Ensures upcoming sharp corners are anticipated early for smooth straight-line braking from > 80 km/h
    double s_last = s_preview.back();
    double s_max_lookahead = std::max(s + 65.0, s_last + 25.0);
    double ds_lookahead = 1.5;
    for (double s_extra = s_last + ds_lookahead; s_extra <= s_max_lookahead; s_extra += ds_lookahead) {
        auto [rx, ry, rpsi, kappa_extra] = track_.getReferencePoint(s_extra);
        s_preview.push_back(s_extra);
        kappa_preview.push_back(std::clamp(kappa_extra, -2.5, 2.5));

        double wl = track_.getLeftWidth(s_extra);
        double wr = track_.getRightWidth(s_extra);
        double w_free = std::max(0.0, std::min(wl, wr) - track_margin_);
        free_widths.push_back(w_free);
    }

    // Compute dynamically-feasible speed profile with backward braking pass (a_brake_)
    // Passes free_widths so the speed envelope reflects achievable racing line radius
    std::vector<double> speed_profile = speed_governor_.computeFeasibleSpeedProfile(
        s_preview, kappa_preview, current_speed_, MPC_N + 1, a_brake_, a_eff_max, free_widths);

    double v_target_current = speed_profile[0];

    for (int k = 0; k <= MPC_N; ++k) {
        double s_k = s_preview[k];
        double kappa_k = kappa_preview[k];
        double v_ref_k = speed_profile[k];
        double mu_k = speed_governor_.computeEffectiveMu(kappa_k);
        double wl_k = track_.getLeftWidth(s_k);
        double wr_k = track_.getRightWidth(s_k);

        solver_.setStageParameters(k, kappa_k, wl_k, wr_k, mu_k);
        solver_.setStageReference(k, v_ref_k, 0.0, 0.0, 0.0, 0.0, 0.0);

        if (k < MPC_N) {
            solver_.setStageControlBounds(k, min_accel_, a_eff_max, -1.5, 1.5);
        }

        if (k > 0 && k < MPC_N) {
            // Stage-dependent lateral corridor bounds derived from track cones
            double bound_l_k = std::max(0.15, wl_k - track_margin_);
            double bound_r_k = std::max(0.15, wr_k - track_margin_);
            double v_bound_max = std::max(35.0, max_straight_speed_ * 1.3);
            solver_.setStageLateralBounds(k, -bound_r_k, bound_l_k, v_bound_max);
        }
    }
    auto t_horizon_end = std::chrono::high_resolution_clock::now();

    // 5. Solve the Optimal Control Problem (Real-Time Iteration)
    auto result = solver_.solve();

    // 6. Extract Optimal Actuation with robust fallback
    double a_opt = 0.0;
    double delta_mpc = last_mpc_steering_;
    double delta_dyn = 0.0;

    if (result.status == 0 || result.status == 2) {
        a_opt = result.optimal_u[0];
        delta_mpc = result.target_steering_angle;

        // Dynamic slip angle compensation for tire cornering compliance in high-g curves
        // Inactive on straightaways (|a_lat| <= 2.5 m/s^2) and low speeds (v <= 7.0 m/s) to ensure straight-line stability
        double a_lat_est = current_speed_ * current_speed_ * std::abs(curvature);
        if (current_speed_ > 7.0 && a_lat_est > 2.5) {
            double v_blend = std::clamp((current_speed_ - 7.0) / 3.0, 0.0, 1.0);
            double alat_blend = std::clamp((a_lat_est - 2.5) / 2.0, 0.0, 1.0);
            delta_dyn = v_blend * alat_blend * understeer_gradient_ * (current_speed_ * current_speed_ * curvature);
        }

        if (std::isnan(a_opt) || std::isinf(a_opt) || std::isnan(delta_mpc) || std::isinf(delta_mpc)) {
            RCLCPP_WARN(this->get_logger(), "Non-finite values from MPC solver! Using safe fallback.");
            a_opt = -1.5;
            delta_mpc = last_mpc_steering_;
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "acados solver warning status: %d (solve time: %.1f us). Applying safe deceleration.", 
            result.status, result.solve_time_us);
        a_opt = -2.0; // Controlled safe braking
        delta_mpc = last_mpc_steering_ * 0.95;
    }

    last_mpc_steering_ = delta_mpc;

    // Bound controls and apply dynamic feedforward to physical actuator command
    a_opt = std::clamp(a_opt, min_accel_, a_eff_max);
    double delta_target = std::clamp(delta_mpc + delta_dyn, -0.52, 0.52);

    // Convert wheel steering angle to steering wheel command
    double steering_wheel_cmd = delta_target / steering_ratio_;

    // Publish to PACSim
    auto t_pub_start = std::chrono::high_resolution_clock::now();
    auto [t_fl, t_fr, t_rl, t_rr] = publishControls(steering_wheel_cmd, a_opt, result.solve_time_us);

    // 7. Visualizations & Telemetry
    publishVisualizations(result.predicted_states, s);
    auto t_pub_end = std::chrono::high_resolution_clock::now();

    double track_len = track_.getTrackLength();
    double s_lap = (track_len > 0.0) ? std::fmod(s, track_len) : s;
    if (s_lap < 0.0 && track_len > 0.0) s_lap += track_len;
    double progress = (track_len > 0.0) ? (s_lap / track_len) : 0.0;

    double pred_ey_end = result.predicted_states.empty() ? 0.0 : result.predicted_states.back()[1];
    double pred_v_end  = result.predicted_states.empty() ? 0.0 : result.predicted_states.back()[3];
    double a_lat = current_speed_ * current_yaw_rate_;
    double a_lon = last_acceleration_cmd_;
    double a_total = std::hypot(a_lon, a_lat);
    double friction_util_pct = (mu_g > 1e-3) ? (a_total / mu_g * 100.0) : 0.0;
    double friction_headroom = mu_g - a_total;

    double w_l = track_.getLeftWidth(s);
    double w_r = track_.getRightWidth(s);
    double clearance_left = w_l - e_y;
    double clearance_right = w_r + e_y;
    double min_cone_clearance = std::min(clearance_left, clearance_right);

    // Real-Time Lap Timing & Terminal Banner Logging
    if (track_len > 0.0) {
        if (!lap_timer_started_ && current_speed_ > 0.5) {
            lap_timer_started_ = true;
            lap_start_time_ = current_time;
            current_lap_idx_ = 1;
            lap_max_speed_ = current_speed_;
            lap_max_ey_ = std::abs(e_y);
            lap_min_clearance_ = min_cone_clearance;
            RCLCPP_INFO(this->get_logger(), "⏱️  Lap timing started (Lap 1: Standing Start)...");
        }

        if (lap_timer_started_) {
            lap_max_speed_ = std::max(lap_max_speed_, current_speed_);
            lap_max_ey_ = std::max(lap_max_ey_, std::abs(e_y));
            lap_min_clearance_ = std::min(lap_min_clearance_, min_cone_clearance);

            // Wrap-around detection on s indicates crossing finish line
            if (s < last_s_raw_ - 0.5 * track_len) {
                double lap_time = current_time - lap_start_time_;
                if (lap_time > 5.0) {
                    RCLCPP_INFO(this->get_logger(),
                        "\n"
                        "╔═════════════════════════════════════════════════════════════════════════╗\n"
                        "║ 🏁 LAP %zu COMPLETED! %-49s ║\n"
                        "║    ⏱️  Lap Time:          %6.3f s                                      ║\n"
                        "║    🚀 Top Speed:         %6.2f m/s (%5.1f km/h)                        ║\n"
                        "║    📐 Max Lateral Error: %6.3f m                                      ║\n"
                        "║    🛡️  Min Clearance:     %6.3f m                                      ║\n"
                        "╚═════════════════════════════════════════════════════════════════════════╝",
                        current_lap_idx_,
                        (current_lap_idx_ == 1) ? "[STANDING START]" : "[FLYING LAP]",
                        lap_time,
                        lap_max_speed_, lap_max_speed_ * 3.6,
                        lap_max_ey_,
                        lap_min_clearance_);

                    current_lap_idx_++;
                    lap_start_time_ = current_time;
                    lap_max_speed_ = current_speed_;
                    lap_max_ey_ = std::abs(e_y);
                    lap_min_clearance_ = min_cone_clearance;
                }
            }
        }
        last_s_raw_ = s;
    }

    double dt = std::max(control_dt_, 1e-4);
    double jerk_lon = (last_acceleration_cmd_ - prev_filtered_accel_) / dt;
    double jerk_steer = (steering_wheel_cmd - prev_steering_wheel_cmd_) / dt;
    prev_filtered_accel_ = last_acceleration_cmd_;
    prev_steering_wheel_cmd_ = steering_wheel_cmd;

    double error_pred_ey = 0.0;
    double error_pred_v = 0.0;
    if (has_prev_prediction_) {
        error_pred_ey = std::abs(e_y - prev_predicted_x1_[1]);
        error_pred_v = std::abs(current_speed_ - prev_predicted_x1_[3]);
    }
    prev_predicted_x1_ = result.predicted_x1;
    has_prev_prediction_ = true;

    int gating_mode = 0;
    if (current_speed_ < 1.0 && std::abs(last_steering_angle_) < 0.08) {
        gating_mode = 2; // Standing launch
    } else if (current_speed_ < high_speed_threshold_) {
        gating_mode = 1; // Low-speed traction gating
    }

    utils::MPCLogger::TelemetryData td;
    td.time = current_time;
    td.lap_idx = current_lap_idx_;
    td.s_lap = s_lap;
    td.progress_pct = progress * 100.0;
    td.x = x_cart;
    td.y = y_cart;
    td.psi = psi_cart;
    td.v = current_speed_;
    td.yaw_rate = current_yaw_rate_;
    td.e_y = e_y;
    td.e_psi = e_psi;
    td.kappa_ref = curvature;
    td.w_l = w_l;
    td.w_r = w_r;
    td.clearance_left = clearance_left;
    td.clearance_right = clearance_right;
    td.min_cone_clearance = min_cone_clearance;
    td.v_target = v_target_current;
    td.delta_v = v_target_current - current_speed_;
    td.a_eff_max = a_eff_max;
    td.gating_mode = gating_mode;
    td.a_lon = a_lon;
    td.a_lat = a_lat;
    td.a_total = a_total;
    td.friction_util_pct = friction_util_pct;
    td.friction_headroom = friction_headroom;
    td.delta_cmd = delta_target;
    td.steer_wheel_cmd = steering_wheel_cmd;
    td.delta_dot = result.optimal_u[1];
    td.delta_dyn_offset = delta_dyn;
    td.jerk_lon = jerk_lon;
    td.jerk_steer = jerk_steer;
    td.t_fl = t_fl;
    td.t_fr = t_fr;
    td.t_rl = t_rl;
    td.t_rr = t_rr;
    td.solver_status = result.status;
    td.solve_time_us = result.solve_time_us;
    td.lin_time_ms = result.time_lin_ms;
    td.qp_time_ms = result.time_qp_ms;
    td.qp_iter = result.qp_iter;
    td.cost_value = result.cost_value;
    td.pred_ey_end = pred_ey_end;
    td.pred_v_end = pred_v_end;
    td.pred_ey_1 = result.predicted_x1[1];
    td.pred_v_1 = result.predicted_x1[3];
    td.error_pred_ey = error_pred_ey;
    td.error_pred_v = error_pred_v;

    logger_.logTelemetry(td);
    logger_.logState(current_time, x_cart, y_cart, psi_cart, current_speed_, current_yaw_rate_, s, e_y, e_psi, progress);
    logger_.logDetailed(current_time, x_cart, y_cart, psi_cart, current_speed_, current_yaw_rate_,
                        s, e_y, e_psi, curvature, v_target_current,
                        last_acceleration_cmd_, delta_target, steering_wheel_cmd,
                        t_fl, t_fr, t_rl, t_rr,
                        result.status, result.solve_time_us,
                        pred_ey_end, pred_v_end, friction_util_pct / 100.0);

    if (std::abs(e_y) > 1.0) {
        logger_.logMain("WARNING: High lateral error e_y = " + std::to_string(e_y) + " m | v = " + std::to_string(current_speed_) + " m/s", current_time);
    }

    last_steering_angle_ = delta_target;

    // 8. High-Resolution Per-Iteration Timing Metrics
    auto t_loop_end = std::chrono::high_resolution_clock::now();
    double total_loop_ms = std::chrono::duration<double, std::milli>(t_loop_end - t_loop_start).count();
    double solver_ms = result.solve_time_us / 1000.0;
    double proj_us = std::chrono::duration<double, std::micro>(t_proj_end - t_proj_start).count();
    double horizon_us = std::chrono::duration<double, std::micro>(t_horizon_end - t_proj_end).count();
    double publish_us = std::chrono::duration<double, std::micro>(t_pub_end - t_pub_start).count();

    logger_.logTiming(control_loop_count_, current_time, total_loop_ms, solver_ms,
                      result.time_lin_ms, result.time_qp_ms,
                      proj_us, horizon_us, publish_us,
                      result.qp_iter, result.qp_status, result.status);

    loop_time_sum_ms_ += total_loop_ms;
    max_loop_time_ms_ = std::max(max_loop_time_ms_, total_loop_ms);
    min_loop_time_ms_ = std::min(min_loop_time_ms_, total_loop_ms);
    solve_time_sum_ms_ += solver_ms;
    max_solve_time_ms_ = std::max(max_solve_time_ms_, solver_ms);

    if (total_loop_ms > (control_dt_ * 1000.0)) {
        overruns_count_++;
    }

    if (control_loop_count_ % 100 == 0) {
        publishReferencePath();
        double avg_loop = loop_time_sum_ms_ / (control_loop_count_ + 1);
        double avg_solve = solve_time_sum_ms_ / (control_loop_count_ + 1);
        RCLCPP_INFO(this->get_logger(),
            "[PERF #%zu] Loop: %.2f ms (avg: %.2f, max: %.2f) | Solver: %.2f ms (avg: %.2f, QP: %.2f, Lin: %.2f, iter: %d) | Budget: %.1f ms | Overruns: %zu",
            control_loop_count_, total_loop_ms, avg_loop, max_loop_time_ms_,
            solver_ms, avg_solve, result.time_qp_ms, result.time_lin_ms, result.qp_iter,
            control_dt_ * 1000.0, overruns_count_);
    }

    control_loop_count_++;
}

std::tuple<double, double, double, double> MPCPacsimNode::publishControls(
    double steering_wheel_rad, double acceleration, double solve_time_us) 
{
    auto stamp = this->now();
    double t = (stamp - start_time_).seconds();

    // 1. Acceleration slew-rate filtering (smooth throttle build-up, rapid braking onset)
    double max_delta_up = max_accel_slew_rate_ * control_dt_;
    double max_delta_down = max_decel_slew_rate_ * control_dt_;
    double a_filtered = std::clamp(acceleration, 
                                   last_acceleration_cmd_ - max_delta_down, 
                                   last_acceleration_cmd_ + max_delta_up);
    last_acceleration_cmd_ = a_filtered;

    // 2. Steering setpoint with physical rack slew-rate protection and high-frequency chatter rejection
    double max_steer_step = (max_steer_rate_ / steering_ratio_) * control_dt_;
    double steer_rate_limited = std::clamp(steering_wheel_rad, 
                                           last_handwheel_cmd_ - max_steer_step, 
                                           last_handwheel_cmd_ + max_steer_step);
    // 1st-order low-pass filter (cutoff ~15 Hz at 100 Hz sampling) eliminates numerical chatter on straightaways
    double steer_filtered = 0.60 * steer_rate_limited + 0.40 * last_handwheel_cmd_;
    last_handwheel_cmd_ = steer_filtered;

    pacsim::msg::StampedScalar steer_msg;
    steer_msg.stamp = stamp;
    steer_msg.value = steer_filtered;
    steering_pub_->publish(steer_msg);

    // 3. Wheel Torques mapping (matching etdv_pid baseline)
    pacsim::msg::Wheels torque_msg;
    torque_msg.stamp = stamp;

    double t_fl = 0.0, t_fr = 0.0, t_rl = 0.0, t_rr = 0.0;

    if (a_filtered > 0.0) {
        // Rear-Wheel Drive (RWD) acceleration
        double norm_acc = std::clamp(a_filtered / max_accel_, 0.0, 1.0);
        double torque = norm_acc * max_torque_;
        t_rl = torque;
        t_rr = torque;
    } else {
        // 4-Wheel Braking (60/40 Front/Rear bias with brake gain 7.5)
        double norm_brake = std::clamp(std::abs(a_filtered) / std::abs(min_accel_), 0.0, 1.0);
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

    logger_.logControl(t, a_filtered, last_steering_angle_, steering_wheel_rad, t_fl, t_fr, t_rl, t_rr, solve_time_us);

    return {t_fl, t_fr, t_rl, t_rr};
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
    if (!track_.isReady()) return;

    auto stamp = this->now();

    // 1. Predicted Horizon Line & Spheres ("pallini")
    if (!predicted_states.empty()) {
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

        pred_path_pub_->publish(utils::createPredictedPathMarker(x_pred, y_pred, stamp));
        pred_spheres_pub_->publish(utils::createPredictedSpheresMarker(x_pred, y_pred, stamp));
    }

    // 2. Reference Horizon Preview Spheres ("pallini")
    double preview_speed = std::max(current_speed_, 3.0);
    std::vector<double> x_ref_horizon, y_ref_horizon;
    x_ref_horizon.reserve(MPC_N + 1);
    y_ref_horizon.reserve(MPC_N + 1);

    for (int k = 0; k <= MPC_N; ++k) {
        double s_k = current_s + k * preview_speed * mpc_dt_;
        auto [rx, ry, rpsi, rkappa] = track_.getReferencePoint(s_k);
        x_ref_horizon.push_back(rx);
        y_ref_horizon.push_back(ry);
    }
    ref_spheres_pub_->publish(utils::createReferenceSpheresMarker(x_ref_horizon, y_ref_horizon, stamp));
}

double MPCPacsimNode::declareAndGetDoubleParam(const std::string& name, double default_val) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    if (!this->has_parameter(name)) {
        this->declare_parameter(name, rclcpp::ParameterValue(default_val), desc);
    }
    auto param = this->get_parameter(name);
    if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {
        return param.as_double();
    } else if (param.get_type() == rclcpp::PARAMETER_INTEGER) {
        return static_cast<double>(param.as_int());
    }
    return default_val;
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
