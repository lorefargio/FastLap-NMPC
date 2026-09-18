#ifndef SPEED_GOVERNOR_HPP_
#define SPEED_GOVERNOR_HPP_

#include <vector>
#include <string>
#include <utility>

namespace mpc {

/**
 * @brief Empirical Speed and Acceleration Governor derived from vehicle limit data (Velocità limite.xlsx).
 *
 * Provides safe maximum speed and lateral acceleration lookups as a function of path curvature kappa [1/m].
 * Extrapolates smoothly to sharp hairpins (R < 9m) using tire grip limit and to straights (R > 30m).
 */
class SpeedGovernor {
public:
    SpeedGovernor();

    /**
     * @brief Configure governor parameters.
     * @param speed_scale Multiplicative scale factor on speed limit (e.g. 0.90 for 10% safety margin).
     * @param max_straight_speed Maximum speed on straights [m/s].
     */
    void configure(double speed_scale = 1.00, double max_straight_speed = 25.0);

    /**
     * @brief Load curvature limits from a CSV file. If file cannot be read, falls back to embedded table.
     * @param csv_path Absolute path to CSV file.
     * @return true if successfully loaded from file, false if fallback used.
     */
    bool loadFromCsv(const std::string& csv_path);

    /**
     * @brief Compute maximum safe speed for a given curvature kappa.
     * @param kappa Centerline curvature [1/m].
     * @return Target speed [m/s].
     */
    double computeSafeSpeed(double kappa) const;

    /**
     * @brief Compute maximum allowable lateral acceleration for a given curvature kappa.
     * @param kappa Centerline curvature [1/m].
     * @return Maximum lateral acceleration [m/s^2].
     */
    double computeMaxLateralAccel(double kappa) const;

    /**
     * @brief Compute effective tire-road friction coefficient mu for acados stage parameters.
     * @param kappa Centerline curvature [1/m].
     * @return mu = a_y_max / 9.81.
     */
    double computeEffectiveMu(double kappa) const;

    /**
     * @brief Compute dynamic maximum allowable longitudinal acceleration.
     * Allows energetic standing starts on straights while preventing torque snaps and yaw instability on corner exits.
     * @param current_speed Current vehicle speed [m/s].
     * @param steering_angle Current wheel steering angle [rad].
     * @param low_speed_thresh Speed threshold below which low-speed limit applies [m/s] (e.g. 7.0 m/s).
     * @param high_speed_thresh Speed threshold above which full acceleration is unlocked [m/s] (e.g. 13.0 m/s).
     * @param low_speed_max_accel Maximum acceleration at low speed when steering is turned [m/s^2] (e.g. 0.85 m/s^2).
     * @param full_max_accel Full acceleration on high-speed straights [m/s^2] (e.g. 3.5 m/s^2).
     * @param standing_launch_accel Maximum acceleration on straight lines at low speed / launch [m/s^2] (e.g. 2.8 m/s^2).
     * @param steer_derate Derating factor based on steering angle (0.0 to 1.0, e.g. 0.75).
     * @param max_steer Maximum wheel steering angle [rad] (e.g. 0.52 rad).
     */
    double computeEffectiveMaxAccel(
        double current_speed, double steering_angle,
        double low_speed_thresh = 6.0, double high_speed_thresh = 12.0,
        double low_speed_max_accel = 2.20, double full_max_accel = 4.8,
        double standing_launch_accel = 4.8,
        double steer_derate = 0.55, double max_steer = 0.52) const;

    /**
     * @brief Compute a dynamically-feasible speed profile along preview coordinates using a backward braking pass.
     * Propagates deceleration backwards from upcoming corners so vehicle brakes on straights before entering turns.
     * @param s_stages Vector of arc-lengths s along the horizon and lookahead window.
     * @param kappas Vector of curvatures kappa corresponding to s_stages.
     * @param current_speed Current vehicle speed [m/s].
     * @param num_output_stages Number of stages to return (e.g. MPC_N + 1).
     * @param a_brake Maximum comfortable braking deceleration [m/s^2] (e.g. 5.0 m/s^2).
     * @param a_accel Maximum comfortable acceleration [m/s^2] (e.g. 4.8 m/s^2).
     * @return Vector of target speeds [m/s] for stages 0 .. num_output_stages - 1.
     */
    std::vector<double> computeFeasibleSpeedProfile(
        const std::vector<double>& s_stages,
        const std::vector<double>& kappas,
        double current_speed,
        size_t num_output_stages,
        double a_brake = 5.0,
        double a_accel = 4.8,
        const std::vector<double>& free_widths = {}) const;

    /**
     * @brief Get count of loaded empirical points.
     */
    size_t getTableSize() const { return speed_table_.size(); }

private:
    void initDefaultTables();

    double speed_scale_{1.00};
    double max_straight_speed_{25.0};

    // Table of (kappa [1/m], v_max [m/s]) sorted by kappa ascending
    std::vector<std::pair<double, double>> speed_table_;

    // Table of (kappa [1/m], a_y_max [m/s^2]) sorted by kappa ascending
    std::vector<std::pair<double, double>> accel_table_;
};

} // namespace mpc

#endif // SPEED_GOVERNOR_HPP_
