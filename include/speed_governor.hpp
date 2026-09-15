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
    void configure(double speed_scale = 0.90, double max_straight_speed = 22.5);

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
     * @brief Get count of loaded empirical points.
     */
    size_t getTableSize() const { return speed_table_.size(); }

private:
    void initDefaultTables();

    double speed_scale_{0.90};
    double max_straight_speed_{22.5};

    // Table of (kappa [1/m], v_max [m/s]) sorted by kappa ascending
    std::vector<std::pair<double, double>> speed_table_;

    // Table of (kappa [1/m], a_y_max [m/s^2]) sorted by kappa ascending
    std::vector<std::pair<double, double>> accel_table_;
};

} // namespace mpc

#endif // SPEED_GOVERNOR_HPP_
