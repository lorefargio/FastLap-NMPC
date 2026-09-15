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

namespace {

std::vector<double> applyGaussianSmoothing(const std::vector<double>& vals, bool is_closed, int passes = 2) {
    if (vals.size() < 5) return vals;
    std::vector<double> cur = vals;
    std::vector<double> next = vals;
    size_t n = vals.size();

    // 5-point binomial kernel: [1, 4, 6, 4, 1] / 16.0
    constexpr double k[5] = {1.0/16.0, 4.0/16.0, 6.0/16.0, 4.0/16.0, 1.0/16.0};

    for (int p = 0; p < passes; ++p) {
        for (size_t i = 0; i < n; ++i) {
            if (is_closed) {
                double v = 0.0;
                for (int j = -2; j <= 2; ++j) {
                    int idx = (static_cast<int>(i) + j + static_cast<int>(n)) % static_cast<int>(n);
                    v += k[j + 2] * cur[idx];
                }
                next[i] = v;
            } else {
                if (i >= 2 && i + 2 < n) {
                    next[i] = k[0] * cur[i - 2] + k[1] * cur[i - 1] + k[2] * cur[i] +
                              k[3] * cur[i + 1] + k[4] * cur[i + 2];
                } else if (i == 1 && n > 3) {
                    next[i] = 0.25 * cur[0] + 0.50 * cur[1] + 0.25 * cur[2];
                } else if (i == n - 2 && n > 3) {
                    next[i] = 0.25 * cur[n - 3] + 0.50 * cur[n - 2] + 0.25 * cur[n - 1];
                } else {
                    next[i] = cur[i];
                }
            }
        }
        cur = next;
    }
    return cur;
}

} // namespace

// ============================================================================
// FrenetTrack Implementation
// ============================================================================

bool FrenetTrack::build(const std::vector<double>& x_vals, 
                        const std::vector<double>& y_vals, 
                        bool is_closed, 
                        double default_track_width,
                        bool apply_smoothing) 
{
    if (x_vals.size() < 3 || x_vals.size() != y_vals.size()) {
        std::cerr << "[FrenetTrack] ERROR: Invalid input points size: " << x_vals.size() << std::endl;
        return false;
    }

    size_t num_points = x_vals.size();
    is_closed_ = is_closed;
    nominal_half_width_ = default_track_width / 2.0;

    std::vector<double> smooth_x = x_vals;
    std::vector<double> smooth_y = y_vals;
    if (apply_smoothing && num_points >= 5) {
        smooth_x = applyGaussianSmoothing(x_vals, is_closed, 2);
        smooth_y = applyGaussianSmoothing(y_vals, is_closed, 2);
    }

    s_.resize(num_points);
    x_.resize(num_points);
    y_.resize(num_points);

    s_[0] = 0.0;
    x_[0] = smooth_x[0];
    y_[0] = smooth_y[0];

    for (size_t i = 1; i < num_points; ++i) {
        x_[i] = smooth_x[i];
        y_[i] = smooth_y[i];
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

    // Check projection on both adjacent segments [idx-1, idx] and [idx, idx+1]
    double best_s = s_[idx];
    double min_dist_sq = (x_[idx] - x) * (x_[idx] - x) + (y_[idx] - y) * (y_[idx] - y);

    auto test_segment = [&](int i0, int i1) {
        if (i0 < 0 || i1 >= n || i0 == i1) return;
        double vx = x_[i1] - x_[i0];
        double vy = y_[i1] - y_[i0];
        double seg_len_sq = vx * vx + vy * vy;
        if (seg_len_sq < 1e-6) return;

        double proj = ((x - x_[i0]) * vx + (y - y_[i0]) * vy) / seg_len_sq;
        proj = std::clamp(proj, 0.0, 1.0);
        double qx = x_[i0] + proj * vx;
        double qy = y_[i0] + proj * vy;
        double d2 = (qx - x) * (qx - x) + (qy - y) * (qy - y);
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            best_s = s_[i0] + proj * (s_[i1] - s_[i0]);
        }
    };

    if (idx > 0) {
        test_segment(idx - 1, idx);
    }
    if (idx + 1 < n) {
        test_segment(idx, idx + 1);
    }

    // 2-step Newton-Raphson refinement on the continuous spline
    // Minimize f(s) = (x(s) - x) * x'(s) + (y(s) - y) * y'(s) = 0
    double s_opt = best_s;
    for (int iter = 0; iter < 2; ++iter) {
        double xs = spline_x_(s_opt);
        double ys = spline_y_(s_opt);
        double dxs = spline_x_.derivative(s_opt);
        double dys = spline_y_.derivative(s_opt);
        double ddxs = spline_x_.secondDerivative(s_opt);
        double ddys = spline_y_.secondDerivative(s_opt);

        double f = (xs - x) * dxs + (ys - y) * dys;
        double f_prime = (dxs * dxs + dys * dys) + (xs - x) * ddxs + (ys - y) * ddys;
        if (std::abs(f_prime) > 0.1) {
            double ds = -f / f_prime;
            s_opt = std::clamp(s_opt + ds, 0.0, track_length_);
        }
    }

    auto [ref_x, ref_y, ref_psi, kappa] = getReferencePoint(s_opt);

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

    return {s_opt, e_y, e_psi, kappa};
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
