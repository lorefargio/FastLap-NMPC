#include "frenet_track.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>

namespace mpc {

// ============================================================================
// CubicSpline1D Implementation
// ============================================================================

void CubicSpline1D::build(const Eigen::VectorXd& x, const Eigen::VectorXd& y) {
    if (x.size() != y.size() || x.size() < 2) {
        throw std::invalid_argument("CubicSpline1D requires at least 2 points.");
    }

    n_ = static_cast<int>(x.size());
    x_ = x;
    a_ = y;
    b_.resize(n_ - 1);
    c_.resize(n_);
    d_.resize(n_ - 1);

    int m = n_ - 1;
    Eigen::VectorXd h(m);
    for (int i = 0; i < m; ++i) {
        h[i] = x_[i + 1] - x_[i];
        if (h[i] <= 0.0) {
            h[i] = 1e-6; // Ensure strict positive spacing
        }
    }

    Eigen::VectorXd alpha(m);
    for (int i = 1; i < m; ++i) {
        alpha[i] = (3.0 / h[i]) * (a_[i + 1] - a_[i]) - (3.0 / h[i - 1]) * (a_[i] - a_[i - 1]);
    }

    Eigen::VectorXd l(n_), mu(n_), z(n_);
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (int i = 1; i < m; ++i) {
        l[i] = 2.0 * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n_ - 1] = 1.0;
    z[n_ - 1] = 0.0;
    c_[n_ - 1] = 0.0;

    for (int j = m - 1; j >= 0; --j) {
        c_[j] = z[j] - mu[j] * c_[j + 1];
        b_[j] = (a_[j + 1] - a_[j]) / h[j] - h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
        d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
    }
}

int CubicSpline1D::findSegment(double t) const {
    if (t <= x_[0]) return 0;
    if (t >= x_[n_ - 1]) return n_ - 2;

    auto it = std::upper_bound(x_.data(), x_.data() + n_, t);
    int idx = static_cast<int>(it - x_.data()) - 1;
    return std::clamp(idx, 0, n_ - 2);
}

double CubicSpline1D::operator()(double t) const {
    int i = findSegment(t);
    double dx = t - x_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
}

double CubicSpline1D::derivative(double t) const {
    int i = findSegment(t);
    double dx = t - x_[i];
    return b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
}

double CubicSpline1D::secondDerivative(double t) const {
    int i = findSegment(t);
    double dx = t - x_[i];
    return 2.0 * c_[i] + 6.0 * d_[i] * dx;
}

// ============================================================================
// FrenetTrack Implementation
// ============================================================================

bool FrenetTrack::build(const std::vector<double>& x_vals, 
                        const std::vector<double>& y_vals, 
                        bool is_closed, 
                        double default_track_width) 
{
    if (x_vals.size() < 3 || x_vals.size() != y_vals.size()) {
        std::cerr << "[FrenetTrack] ERROR: Invalid input points size: " << x_vals.size() << std::endl;
        return false;
    }

    size_t num_points = x_vals.size();
    is_closed_ = is_closed;
    nominal_half_width_ = default_track_width / 2.0;

    s_.resize(num_points);
    x_.resize(num_points);
    y_.resize(num_points);

    s_[0] = 0.0;
    x_[0] = x_vals[0];
    y_[0] = y_vals[0];

    for (size_t i = 1; i < num_points; ++i) {
        x_[i] = x_vals[i];
        y_[i] = y_vals[i];
        double ds = std::hypot(x_[i] - x_[i - 1], y_[i] - y_[i - 1]);
        s_[i] = s_[i - 1] + ds;
    }

    track_length_ = s_[num_points - 1];

    if (track_length_ <= 1.0) {
        std::cerr << "[FrenetTrack] ERROR: Path length too short: " << track_length_ << std::endl;
        return false;
    }

    try {
        spline_x_.build(s_, x_);
        spline_y_.build(s_, y_);
        is_initialized_ = true;
        last_closest_idx_ = 0;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[FrenetTrack] Exception building splines: " << e.what() << std::endl;
        return false;
    }
}

std::tuple<double, double, double, double> FrenetTrack::getReferencePoint(double s) const {
    if (!is_initialized_) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    // Wrap-around if track is closed
    if (is_closed_ && track_length_ > 0.0) {
        s = std::fmod(s, track_length_);
        if (s < 0.0) s += track_length_;
    } else {
        s = std::clamp(s, 0.0, track_length_);
    }

    double x = spline_x_(s);
    double y = spline_y_(s);

    double dx = spline_x_.derivative(s);
    double dy = spline_y_.derivative(s);

    double ddx = spline_x_.secondDerivative(s);
    double ddy = spline_y_.secondDerivative(s);

    double heading = std::atan2(dy, dx);

    double denom = std::pow(dx * dx + dy * dy, 1.5);
    double curvature = 0.0;
    if (denom > 1e-6) {
        curvature = (dx * ddy - dy * ddx) / denom;
    }

    return {x, y, heading, curvature};
}

int FrenetTrack::findClosestSegment(double x, double y) const {
    size_t n = x_.size();
    if (n < 2) return 0;

    size_t best_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::infinity();

    // Fast local window search around last_closest_idx_
    size_t window = 40;
    size_t start_idx = (last_closest_idx_ > window) ? (last_closest_idx_ - window) : 0;
    size_t end_idx = std::min(n, last_closest_idx_ + window);

    for (size_t i = start_idx; i < end_idx; ++i) {
        double d2 = (x_[i] - x) * (x_[i] - x) + (y_[i] - y) * (y_[i] - y);
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            best_idx = i;
        }
    }

    // If local search found a poor match, fall back to global scan
    if (min_dist_sq > 25.0) { // > 5 meters error
        for (size_t i = 0; i < n; ++i) {
            double d2 = (x_[i] - x) * (x_[i] - x) + (y_[i] - y) * (y_[i] - y);
            if (d2 < min_dist_sq) {
                min_dist_sq = d2;
                best_idx = i;
            }
        }
    }

    last_closest_idx_ = best_idx;
    return static_cast<int>(best_idx);
}

std::tuple<double, double, double, double> FrenetTrack::cartesianToFrenet(
    double x, double y, double psi) const 
{
    if (!is_initialized_) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    int idx = findClosestSegment(x, y);
    int n = static_cast<int>(x_.size());

    // Project onto segment (idx to idx+1)
    int next_idx = (idx + 1 < n) ? (idx + 1) : idx;

    double s_guess = s_[idx];
    if (next_idx != idx) {
        double vx = x_[next_idx] - x_[idx];
        double vy = y_[next_idx] - y_[idx];
        double seg_len = std::hypot(vx, vy);
        if (seg_len > 1e-4) {
            double proj = ((x - x_[idx]) * vx + (y - y_[idx]) * vy) / (seg_len * seg_len);
            proj = std::clamp(proj, 0.0, 1.0);
            s_guess = s_[idx] + proj * (s_[next_idx] - s_[idx]);
        }
    }

    auto [ref_x, ref_y, ref_psi, kappa] = getReferencePoint(s_guess);

    // Vector from reference point to car position
    double dx = x - ref_x;
    double dy = y - ref_y;

    // Normal unit vector pointing to the LEFT (+ey) of path tangent
    double nx = -std::sin(ref_psi);
    double ny =  std::cos(ref_psi);

    // Signed lateral error
    double e_y = dx * nx + dy * ny;

    // Heading error relative to path tangent
    double e_psi = normalizeAngle(psi - ref_psi);

    return {s_guess, e_y, e_psi, kappa};
}

std::tuple<double, double, double> FrenetTrack::frenetToCartesian(
    double s, double e_y, double e_psi) const 
{
    auto [ref_x, ref_y, ref_psi, kappa] = getReferencePoint(s);

    // Normal unit vector pointing left
    double nx = -std::sin(ref_psi);
    double ny =  std::cos(ref_psi);

    double cart_x = ref_x + e_y * nx;
    double cart_y = ref_y + e_y * ny;
    double cart_psi = normalizeAngle(ref_psi + e_psi);

    return {cart_x, cart_y, cart_psi};
}

} // namespace mpc
