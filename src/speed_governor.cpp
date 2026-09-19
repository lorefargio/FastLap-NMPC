#include "speed_governor.hpp"

#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

namespace mpc {

SpeedGovernor::SpeedGovernor() {
    initDefaultTables();
}

void SpeedGovernor::configure(double speed_scale, double max_straight_speed) {
    speed_scale_ = std::clamp(speed_scale, 0.5, 1.2);
    max_straight_speed_ = std::max(max_straight_speed, 5.0);
}

void SpeedGovernor::initDefaultTables() {
    // Exact data extracted from Velocità limite.xlsx
    // (kappa [1/m], v_max [m/s]) sorted by kappa ascending
    speed_table_ = {
        {0.033755, 22.4983}, // R = 29.63m -> 81.0 km/h
        {0.034934, 21.9455}, // R = 28.63m -> 79.0 km/h
        {0.036199, 21.1628}, // R = 27.63m -> 76.2 km/h
        {0.036866, 21.1101}, // R = 27.13m -> 76.0 km/h
        {0.039024, 20.2651}, // R = 25.63m -> 73.0 km/h
        {0.039801, 19.9808}, // R = 25.12m -> 71.9 km/h
        {0.040609, 19.6954}, // R = 24.63m -> 70.9 km/h
        {0.041451, 19.4095}, // R = 24.12m -> 69.9 km/h
        {0.042328, 19.0971}, // R = 23.63m -> 68.7 km/h
        {0.044199, 18.5644}, // R = 22.62m -> 66.8 km/h
        {0.045198, 18.2926}, // R = 22.12m -> 65.9 km/h
        {0.046243, 17.9693}, // R = 21.62m -> 64.7 km/h
        {0.047337, 17.6794}, // R = 21.13m -> 63.6 km/h
        {0.048485, 17.3865}, // R = 20.62m -> 62.6 km/h
        {0.050955, 16.7944}, // R = 19.63m -> 60.5 km/h
        {0.052288, 16.4984}, // R = 19.12m -> 59.4 km/h
        {0.053691, 16.1968}, // R = 18.63m -> 58.3 km/h
        {0.055172, 15.8991}, // R = 18.13m -> 57.2 km/h
        {0.056738, 15.7445}, // R = 17.62m -> 56.7 km/h
        {0.058394, 15.2924}, // R = 17.13m -> 55.1 km/h
        {0.060150, 15.1396}, // R = 16.63m -> 54.5 km/h
        {0.066116, 14.2013}, // R = 15.12m -> 51.1 km/h
        {0.070796, 13.5749}, // R = 14.13m -> 48.9 km/h
        {0.073394, 13.2161}, // R = 13.63m -> 47.6 km/h
        {0.076190, 12.9030}, // R = 13.13m -> 46.5 km/h
        {0.082474, 12.0120}, // R = 12.13m -> 43.2 km/h
        {0.089888, 11.5096}, // R = 11.12m -> 41.4 km/h
        {0.098765, 10.9080}, // R = 10.13m -> 39.3 km/h
        {0.103896, 10.6096}, // R =  9.63m -> 38.2 km/h
        {0.109589, 10.3028}  // R =  9.13m -> 37.1 km/h
    };

    // Lateral acceleration table from Velocità limite.xlsx column E
    // (kappa [1/m], a_y_max [m/s^2]) sorted by kappa ascending
    accel_table_ = {
        {0.053691, 16.9500}, // R = 18.63m -> 16.95 m/s^2 (1.73g)
        {0.055172, 16.8800}, // R = 18.13m -> 16.88 m/s^2 (1.72g)
        {0.056738, 16.8000}, // R = 17.62m -> 16.80 m/s^2 (1.71g)
        {0.058394, 16.7200}, // R = 17.13m -> 16.72 m/s^2 (1.70g)
        {0.060150, 16.6500}, // R = 16.63m -> 16.65 m/s^2 (1.70g)
        {0.064000, 16.5000}, // R = 15.62m -> 16.50 m/s^2 (1.68g)
        {0.066116, 16.4000}, // R = 15.12m -> 16.40 m/s^2 (1.67g)
        {0.068376, 16.3500}, // R = 14.63m -> 16.35 m/s^2 (1.67g)
        {0.070796, 16.2600}, // R = 14.13m -> 16.26 m/s^2 (1.66g)
        {0.073394, 16.2000}, // R = 13.63m -> 16.20 m/s^2 (1.65g)
        {0.076190, 16.1000}, // R = 13.13m -> 16.10 m/s^2 (1.64g)
        {0.079208, 16.0500}, // R = 12.62m -> 16.05 m/s^2 (1.64g)
        {0.082474, 15.9400}, // R = 12.13m -> 15.94 m/s^2 (1.62g)
        {0.089888, 15.7800}, // R = 11.12m -> 15.78 m/s^2 (1.61g)
        {0.098765, 15.6200}, // R = 10.13m -> 15.62 m/s^2 (1.59g)
        {0.109589, 15.4600}, // R =  9.13m -> 15.46 m/s^2 (1.58g)
        {0.115942, 15.3600}, // R =  8.63m -> 15.36 m/s^2 (1.57g)
        {0.123077, 15.2500}  // R =  8.12m -> 15.25 m/s^2 (1.55g)
    };
}

bool SpeedGovernor::loadFromCsv(const std::string& csv_path) {
    if (csv_path.empty()) {
        return false;
    }

    std::ifstream file(csv_path);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    // Skip header line
    if (!std::getline(file, line)) {
        return false;
    }

    std::vector<std::pair<double, double>> new_speed_table;
    std::vector<std::pair<double, double>> new_accel_table;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        if (tokens.size() < 3) continue;

        try {
            double kappa = std::stod(tokens[0]);
            if (!tokens[2].empty()) {
                double v_max = std::stod(tokens[2]);
                new_speed_table.emplace_back(kappa, v_max);
            }
            if (tokens.size() >= 4 && !tokens[3].empty()) {
                double ay = std::stod(tokens[3]);
                new_accel_table.emplace_back(kappa, ay);
            }
        } catch (...) {
            continue;
        }
    }

    if (!new_speed_table.empty()) {
        std::sort(new_speed_table.begin(), new_speed_table.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        speed_table_ = new_speed_table;
    }

    if (!new_accel_table.empty()) {
        std::sort(new_accel_table.begin(), new_accel_table.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        accel_table_ = new_accel_table;
    }

    return (!speed_table_.empty());
}

double SpeedGovernor::computeSafeSpeed(double kappa) const {
    double k = std::abs(kappa);

    // 1. Straightaways and wide curves (k near 0):
    // Smoothly transition from max_straight_speed at k=0 down to table max at k_min (R=29.6m)
    double k_min = speed_table_.front().first;
    double v_table_max = speed_table_.front().second;
    if (k <= k_min) {
        if (max_straight_speed_ <= v_table_max) {
            return max_straight_speed_ * speed_scale_;
        }
        double frac = (k_min > 1e-6) ? (k / k_min) : 0.0;
        double v_interp = max_straight_speed_ - frac * (max_straight_speed_ - v_table_max);
        return v_interp * speed_scale_;
    }

    // 2. Sharp hairpins (k > table max, R < 9.13m):
    // Transition smoothly using mechanical tire grip limit: v = sqrt(ay_eff / k).
    // For ordinary tight curves (k <= 0.30, R >= 3.3m), maintain full empirical baseline (11.632 m/s^2).
    // Only for extreme hairpins beyond physical steering lock capability (k > 0.30, R < 3.3m),
    // excessive tire slip angle reduces effective lateral grip towards pure mechanical grip (~9.42 m/s^2).
    if (k >= speed_table_.back().first) {
        constexpr double ay_base = 11.632;
        constexpr double ay_mech = 9.81 * 0.96; // 9.42 m/s^2 (0.96g mechanical grip)
        double ay_eff = ay_base;
        if (k > 0.30) {
            double blend = std::clamp((k - 0.30) / 0.15, 0.0, 1.0);
            ay_eff = ay_base - blend * (ay_base - ay_mech);
        }
        double v_phys = std::sqrt(ay_eff / k);
        return v_phys * speed_scale_;
    }

    // 3. Piecewise linear monotonic interpolation across empirical table points
    auto it = std::lower_bound(speed_table_.begin(), speed_table_.end(), k,
                               [](const std::pair<double, double>& pt, double val) {
                                   return pt.first < val;
                               });

    if (it == speed_table_.end()) {
        return speed_table_.back().second * speed_scale_;
    }
    if (it == speed_table_.begin()) {
        return speed_table_.front().second * speed_scale_;
    }

    auto prev = it - 1;
    double k0 = prev->first;
    double v0 = prev->second;
    double k1 = it->first;
    double v1 = it->second;

    double frac = (k - k0) / (k1 - k0);
    double v_interp = v0 + frac * (v1 - v0);

    return v_interp * speed_scale_;
}

double SpeedGovernor::computeMaxLateralAccel(double kappa) const {
    double k = std::abs(kappa);

    if (accel_table_.empty()) {
        return 15.0;
    }

    // 1. Straights and high speed (k < 0.053691, R > 18.6m):
    // Aero downforce increases max grip up to ~17.5 m/s^2
    if (k <= accel_table_.front().first) {
        constexpr double ay_max_aero = 17.50;
        double k_min = accel_table_.front().first;
        double frac = std::clamp(1.0 - (k / k_min), 0.0, 1.0);
        return accel_table_.front().second + frac * (ay_max_aero - accel_table_.front().second);
    }

    // 2. Sharp hairpins (k > 0.123077, R < 8.12m):
    // Continues linearly towards baseline mechanical grip ~10.6 m/s^2 at k=0.31
    if (k >= accel_table_.back().first) {
        // Slope = (15.25 - 16.95) / (0.123077 - 0.053691) = -24.5 m/s^2 / (1/m)
        constexpr double slope = -24.5;
        double k_last = accel_table_.back().first;
        double ay_last = accel_table_.back().second;
        double ay_ext = ay_last + slope * (k - k_last);
        return std::max(ay_ext, 9.81 * 0.95); // Minimum 0.95g mechanical grip
    }

    // 3. Piecewise linear interpolation
    auto it = std::lower_bound(accel_table_.begin(), accel_table_.end(), k,
                               [](const std::pair<double, double>& pt, double val) {
                                   return pt.first < val;
                               });

    if (it == accel_table_.end()) {
        return accel_table_.back().second;
    }
    if (it == accel_table_.begin()) {
        return accel_table_.front().second;
    }

    auto prev = it - 1;
    double k0 = prev->first;
    double a0 = prev->second;
    double k1 = it->first;
    double a1 = it->second;

    double frac = (k - k0) / (k1 - k0);
    return a0 + frac * (a1 - a0);
}

double SpeedGovernor::computeEffectiveMu(double kappa) const {
    double ay = computeMaxLateralAccel(kappa);
    return ay / 9.81;
}

double SpeedGovernor::computeEffectiveMaxAccel(
    double current_speed, double steering_angle,
    double low_speed_thresh, double high_speed_thresh,
    double low_speed_max_accel, double full_max_accel,
    double standing_launch_accel,
    double steer_derate, double max_steer) const
{
    // 1. Standing start launch boost: ONLY active from standstill (v < 1.2 m/s) on straight lines
    if (current_speed < 1.2 && std::abs(steering_angle) < 0.08) {
        return standing_launch_accel;
    }

    // 2. Continuous speed blend: smoothly transition from low-speed to high-speed regime
    double speed_ratio = 0.0;
    if (current_speed <= low_speed_thresh) {
        speed_ratio = 0.0;
    } else if (current_speed >= high_speed_thresh) {
        speed_ratio = 1.0;
    } else {
        speed_ratio = (current_speed - low_speed_thresh) / (high_speed_thresh - low_speed_thresh);
    }

    // 3. Progressive corner-exit derating based on steering angle (smooth friction circle approximation)
    // Steering derating protects against snap oversteer at medium/high cornering speeds.
    // At crawl speeds (v < 2.5 m/s), lateral g is negligible (< 0.5 m/s^2), so derating is
    // phased out to prevent vehicle stall against tire scrub resistance.
    double speed_derate_weight = std::clamp((current_speed - 1.2) / 3.0, 0.0, 1.0);
    double effective_derate = steer_derate * speed_derate_weight;
    double steer_norm = std::clamp(std::abs(steering_angle) / max_steer, 0.0, 1.0);
    double traction_factor = std::clamp(1.0 - effective_derate * (steer_norm * steer_norm), 0.30, 1.0);

    double low_speed_target = low_speed_max_accel + (full_max_accel - low_speed_max_accel) * (1.0 - steer_norm * speed_derate_weight);
    double a_eff = low_speed_target + speed_ratio * (full_max_accel - low_speed_target);

    double a_result = a_eff * traction_factor;
    // Anti-stall floor at crawl speed ensures vehicle always has enough torque to roll through tight turns
    double min_floor = (current_speed < 2.5) ? low_speed_max_accel : (low_speed_max_accel * 0.5);
    return std::clamp(a_result, min_floor, full_max_accel);
}

std::vector<double> SpeedGovernor::computeFeasibleSpeedProfile(
    const std::vector<double>& s_stages,
    const std::vector<double>& kappas,
    double current_speed,
    size_t num_output_stages,
    double a_brake,
    double a_accel,
    const std::vector<double>& free_widths) const
{
    (void)current_speed;
    (void)a_accel;
    size_t M = std::min(s_stages.size(), kappas.size());
    if (M == 0) {
        return std::vector<double>(num_output_stages, 5.0);
    }

    // 1. Compute steady-state cornering limit for each point
    // If corridor free_widths are available, compute effective racing line curvature:
    // kappa_eff = |kappa| / (1.0 + |kappa| * w_free)
    // reflecting the wider radius (out-in-out) achievable within the cone corridor.
    std::vector<double> v_prof(M);
    bool use_corridor = (!free_widths.empty() && free_widths.size() >= M);

    for (size_t i = 0; i < M; ++i) {
        double k_raw = kappas[i];
        double k_eff = std::abs(k_raw);
        if (use_corridor && free_widths[i] > 0.05 && k_eff > 1e-4) {
            double w_free = std::clamp(free_widths[i], 0.0, 0.85);
            // In extreme hairpins (k > 0.25, R < 4.0m), vehicle steering lock limits achievable apex cut
            if (k_eff > 0.25) {
                double tight_blend = std::clamp((k_eff - 0.25) / 0.15, 0.0, 1.0);
                w_free *= (1.0 - 0.70 * tight_blend);
            }
            k_eff = k_eff / (1.0 + k_eff * w_free);
        }
        v_prof[i] = computeSafeSpeed(k_eff);
    }

    // 2. Backward Braking Pass:
    // Ensures vehicle begins braking on straight ahead of an upcoming corner.
    // For ordinary curves and straights (k <= 0.25, R >= 4.0m), full braking deceleration (5.0 m/s^2)
    // is available, guaranteeing late and aggressive braking zones across FSE23, FSG21, and FSI24.
    // For extreme hairpins (k > 0.25), smooth scaling anticipates braking into the hairpin apex.
    for (size_t i = M - 1; i > 0; --i) {
        size_t prev = i - 1;
        double ds = std::max(s_stages[i] - s_stages[prev], 0.01);

        double k_i = std::abs(kappas[i]);
        double a_brake_eff = a_brake;
        if (k_i > 0.25) {
            double k_blend = std::clamp((k_i - 0.25) / 0.20, 0.0, 1.0);
            a_brake_eff = a_brake * (1.0 - 0.45 * k_blend); // Scales smoothly from 5.0 down to 2.75 m/s^2 for k >= 0.45
        }

        double v_max_brake = std::sqrt(v_prof[i] * v_prof[i] + 2.0 * a_brake_eff * ds);
        if (v_max_brake < v_prof[prev]) {
            v_prof[prev] = v_max_brake;
        }
    }

    // 3. Extract output stages (clamped to [3.0, max_straight_speed_])
    std::vector<double> result(num_output_stages);
    for (size_t k = 0; k < num_output_stages; ++k) {
        double val = (k < M) ? v_prof[k] : v_prof.back();
        result[k] = std::clamp(val, 3.0, max_straight_speed_);
    }
    return result;
}

} // namespace mpc
