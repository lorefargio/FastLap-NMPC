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
    // Continuous curvature (parabolic runout) boundary conditions:
    // Left boundary: c_0 = c_1
    l[0] = 1.0;
    mu[0] = 1.0;
    z[0] = 0.0;

    for (int i = 1; i < m; ++i) {
        l[i] = 2.0 * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    // Right boundary: c_{n-1} = c_{n-2} (eliminates artificial zero curvature collapse)
    c_[n_ - 1] = z[m - 1] / (1.0 + mu[m - 1]);

    for (int j = m - 1; j >= 0; --j) {
        c_[j] = z[j] - mu[j] * c_[j + 1];
        b_[j] = (a_[j + 1] - a_[j]) / h[j] - h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
        d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
    }
    c_[0] = c_[1];
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

    // 1. Deduplicate consecutive points to prevent division by zero in cubic spline
    std::vector<double> clean_x, clean_y;
    clean_x.reserve(x_vals.size());
    clean_y.reserve(y_vals.size());

    for (size_t i = 0; i < x_vals.size(); ++i) {
        if (clean_x.empty()) {
            clean_x.push_back(x_vals[i]);
            clean_y.push_back(y_vals[i]);
        } else {
            double d = std::hypot(x_vals[i] - clean_x.back(), y_vals[i] - clean_y.back());
            if (d >= 0.05) { // Skip duplicate or near-coincident points
                clean_x.push_back(x_vals[i]);
                clean_y.push_back(y_vals[i]);
            }
        }
    }

    if (clean_x.size() < 3) return false;

    // Optional Gaussian smoothing (only if explicitly requested)
    if (apply_smoothing && clean_x.size() >= 5) {
        clean_x = applyGaussianSmoothing(clean_x, is_closed, 2);
        clean_y = applyGaussianSmoothing(clean_y, is_closed, 2);
    }

    nominal_half_width_ = default_track_width / 2.0;
    has_local_widths_ = false;

    // Check if loop closure endpoint is already duplicate
    if (is_closed && clean_x.size() >= 4) {
        double d_ends = std::hypot(clean_x.back() - clean_x.front(), clean_y.back() - clean_y.front());
        if (d_ends < 0.05) {
            clean_x.pop_back();
            clean_y.pop_back();
        }
    }

    size_t N = clean_x.size();
    is_closed_ = is_closed;

    if (is_closed_ && N >= 10) {
        // Compute segment lengths around the closed loop
        std::vector<double> seg_len(N);
        double total_L = 0.0;
        for (size_t i = 0; i < N; ++i) {
            size_t next_i = (i + 1) % N;
            seg_len[i] = std::hypot(clean_x[next_i] - clean_x[i], clean_y[next_i] - clean_y[i]);
            total_L += seg_len[i];
        }
        track_length_ = total_L;

        // Store reference points for distance search: N points + closure point at s = track_length_
        s_.resize(N + 1);
        x_.resize(N + 1);
        y_.resize(N + 1);

        s_[0] = 0.0;
        x_[0] = clean_x[0];
        y_[0] = clean_y[0];

        for (size_t i = 0; i < N; ++i) {
            s_[i + 1] = s_[i] + seg_len[i];
            size_t next_i = (i + 1) % N;
            x_[i + 1] = clean_x[next_i];
            y_[i + 1] = clean_y[next_i];
        }

        // Build periodic padded spline: prepend points before s=0, append points after s=L
        // This ensures the spline polynomial across the lap line s=0/L is strictly C2 continuous!
        size_t pad = std::min(size_t(50), N / 3);
        size_t total_padded = N + 1 + 2 * pad;

        Eigen::VectorXd s_pad(total_padded);
        Eigen::VectorXd x_pad(total_padded);
        Eigen::VectorXd y_pad(total_padded);

        // Prepended points (s < 0)
        double s_accum = 0.0;
        for (size_t p = 1; p <= pad; ++p) {
            size_t idx = (N - (p % N)) % N;
            s_accum -= seg_len[idx];
            size_t insert_pos = pad - p;
            s_pad[insert_pos] = s_accum;
            x_pad[insert_pos] = clean_x[idx];
            y_pad[insert_pos] = clean_y[idx];
        }

        // Main loop points (0 <= s <= track_length_)
        for (size_t i = 0; i <= N; ++i) {
            s_pad[pad + i] = s_[i];
            x_pad[pad + i] = x_[i];
            y_pad[pad + i] = y_[i];
        }

        // Appended points (s > track_length_)
        s_accum = track_length_;
        for (size_t p = 1; p <= pad; ++p) {
            size_t idx = p % N;
            s_accum += seg_len[(idx + N - 1) % N];
            size_t insert_pos = pad + N + p;
            s_pad[insert_pos] = s_accum;
            x_pad[insert_pos] = clean_x[idx];
            y_pad[insert_pos] = clean_y[idx];
        }

        try {
            spline_x_.build(s_pad, x_pad);
            spline_y_.build(s_pad, y_pad);
            is_initialized_ = true;
            last_closest_idx_ = 0;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[FrenetTrack] Exception building closed splines: " << e.what() << std::endl;
            return false;
        }
    } else {
        // Open track (rolling window or line segment)
        s_.resize(N);
        x_.resize(N);
        y_.resize(N);

        s_[0] = 0.0;
        x_[0] = clean_x[0];
        y_[0] = clean_y[0];

        for (size_t i = 1; i < N; ++i) {
            x_[i] = clean_x[i];
            y_[i] = clean_y[i];
            double ds = std::hypot(x_[i] - x_[i - 1], y_[i] - y_[i - 1]);
            s_[i] = s_[i - 1] + ds;
        }

        track_length_ = s_[N - 1];
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
            std::cerr << "[FrenetTrack] Exception building open splines: " << e.what() << std::endl;
            return false;
        }
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
        double x = spline_x_(s);
        double y = spline_y_(s);
        double dx = spline_x_.derivative(s);
        double dy = spline_y_.derivative(s);
        double ddx = spline_x_.secondDerivative(s);
        double ddy = spline_y_.secondDerivative(s);
        double heading = std::atan2(dy, dx);
        double denom = std::pow(dx * dx + dy * dy, 1.5);
        double curvature = (denom > 1e-6) ? ((dx * ddy - dy * ddx) / denom) : 0.0;
        return {x, y, heading, curvature};
    }

    // Open track segment (rolling preview window):
    // Smoothly extrapolate along circular arc of tail curvature if s > track_length_
    if (s >= track_length_) {
        double s_tail = track_length_;
        double x_tail = spline_x_(s_tail);
        double y_tail = spline_y_(s_tail);
        double dx = spline_x_.derivative(s_tail);
        double dy = spline_y_.derivative(s_tail);
        double ddx = spline_x_.secondDerivative(s_tail);
        double ddy = spline_y_.secondDerivative(s_tail);
        double heading_tail = std::atan2(dy, dx);
        double denom = std::pow(dx * dx + dy * dy, 1.5);
        double kappa_tail = (denom > 1e-6) ? ((dx * ddy - dy * ddx) / denom) : 0.0;

        double delta_s = s - s_tail;
        double heading = heading_tail + kappa_tail * delta_s;
        double x, y;
        if (std::abs(kappa_tail) > 1e-4) {
            x = x_tail + (std::sin(heading) - std::sin(heading_tail)) / kappa_tail;
            y = y_tail - (std::cos(heading) - std::cos(heading_tail)) / kappa_tail;
        } else {
            x = x_tail + delta_s * std::cos(heading_tail);
            y = y_tail + delta_s * std::sin(heading_tail);
        }
        return {x, y, heading, kappa_tail};
    } else if (s <= 0.0) {
        double s_head = 0.0;
        double x_head = spline_x_(s_head);
        double y_head = spline_y_(s_head);
        double dx = spline_x_.derivative(s_head);
        double dy = spline_y_.derivative(s_head);
        double ddx = spline_x_.secondDerivative(s_head);
        double ddy = spline_y_.secondDerivative(s_head);
        double heading_head = std::atan2(dy, dx);
        double denom = std::pow(dx * dx + dy * dy, 1.5);
        double kappa_head = (denom > 1e-6) ? ((dx * ddy - dy * ddx) / denom) : 0.0;

        double delta_s = s; // negative
        double heading = heading_head + kappa_head * delta_s;
        double x, y;
        if (std::abs(kappa_head) > 1e-4) {
            x = x_head + (std::sin(heading) - std::sin(heading_head)) / kappa_head;
            y = y_head - (std::cos(heading) - std::cos(heading_head)) / kappa_head;
        } else {
            x = x_head + delta_s * std::cos(heading_head);
            y = y_head + delta_s * std::sin(heading_head);
        }
        return {x, y, heading, kappa_head};
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
    int window = 40;
    if (is_closed_) {
        int n_int = static_cast<int>(n);
        int center = static_cast<int>(last_closest_idx_);
        for (int offset = -window; offset <= window; ++offset) {
            int i = (center + offset + n_int) % n_int;
            double d2 = (x_[i] - x) * (x_[i] - x) + (y_[i] - y) * (y_[i] - y);
            if (d2 < min_dist_sq) {
                min_dist_sq = d2;
                best_idx = static_cast<size_t>(i);
            }
        }
    } else {
        size_t start_idx = (last_closest_idx_ > static_cast<size_t>(window)) ? (last_closest_idx_ - window) : 0;
        size_t end_idx = std::min(n, last_closest_idx_ + window);

        for (size_t i = start_idx; i < end_idx; ++i) {
            double d2 = (x_[i] - x) * (x_[i] - x) + (y_[i] - y) * (y_[i] - y);
            if (d2 < min_dist_sq) {
                min_dist_sq = d2;
                best_idx = i;
            }
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
            if (is_closed_ && track_length_ > 0.0 && best_s >= track_length_) {
                best_s -= track_length_;
            }
        }
    };

    if (idx > 0) {
        test_segment(idx - 1, idx);
    } else if (is_closed_ && n >= 2) {
        test_segment(n - 2, n - 1);
    }

    if (idx + 1 < n) {
        test_segment(idx, idx + 1);
    } else if (is_closed_ && n >= 2) {
        test_segment(0, 1);
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
            if (is_closed_ && track_length_ > 0.0) {
                s_opt = std::fmod(s_opt + ds, track_length_);
                if (s_opt < 0.0) s_opt += track_length_;
            } else {
                s_opt = std::clamp(s_opt + ds, 0.0, track_length_);
            }
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

double FrenetTrack::getLeftWidth(double s) const {
    if (!has_local_widths_ || !is_initialized_) {
        return nominal_half_width_;
    }
    if (is_closed_ && track_length_ > 0.0) {
        s = std::fmod(s, track_length_);
        if (s < 0.0) s += track_length_;
    } else {
        s = std::clamp(s, 0.0, track_length_);
    }
    return std::clamp(spline_wl_(s), 0.8, 3.5);
}

double FrenetTrack::getRightWidth(double s) const {
    if (!has_local_widths_ || !is_initialized_) {
        return nominal_half_width_;
    }
    if (is_closed_ && track_length_ > 0.0) {
        s = std::fmod(s, track_length_);
        if (s < 0.0) s += track_length_;
    } else {
        s = std::clamp(s, 0.0, track_length_);
    }
    return std::clamp(spline_wr_(s), 0.8, 3.5);
}

void FrenetTrack::setBoundaryCones(const std::vector<Eigen::Vector2d>& left_cones,
                                  const std::vector<Eigen::Vector2d>& right_cones)
{
    if (!is_initialized_ || track_length_ <= 1.0 || left_cones.size() < 3 || right_cones.size() < 3) {
        return;
    }

    double ds_target = 0.5;
    size_t num_intervals = static_cast<size_t>(std::ceil(track_length_ / ds_target));
    if (num_intervals < 5) return;
    size_t num_pts = num_intervals + 1;
    double actual_ds = track_length_ / static_cast<double>(num_intervals);

    Eigen::VectorXd s_samples(num_pts);
    Eigen::VectorXd wl_samples(num_pts);
    Eigen::VectorXd wr_samples(num_pts);

    for (size_t i = 0; i < num_pts; ++i) {
        double s_curr = (i == num_intervals) ? track_length_ : (i * actual_ds);
        s_samples[i] = s_curr;
        auto [rx, ry, rpsi, rkappa] = getReferencePoint(s_curr);
        (void)rkappa;

        // Normal pointing left (+ey)
        double nx = -std::sin(rpsi);
        double ny =  std::cos(rpsi);

        // Find closest left cone
        double min_dist_l = std::numeric_limits<double>::max();
        for (const auto& cone : left_cones) {
            double dx = cone.x() - rx;
            double dy = cone.y() - ry;
            double lat = dx * nx + dy * ny;
            if (lat > -0.2) {
                double d = std::hypot(dx, dy);
                if (d < min_dist_l) {
                    min_dist_l = d;
                }
            }
        }
        if (min_dist_l == std::numeric_limits<double>::max()) {
            min_dist_l = nominal_half_width_;
        }

        // Find closest right cone
        double min_dist_r = std::numeric_limits<double>::max();
        for (const auto& cone : right_cones) {
            double dx = cone.x() - rx;
            double dy = cone.y() - ry;
            double lat = dx * nx + dy * ny;
            if (lat < 0.2) {
                double d = std::hypot(dx, dy);
                if (d < min_dist_r) {
                    min_dist_r = d;
                }
            }
        }
        if (min_dist_r == std::numeric_limits<double>::max()) {
            min_dist_r = nominal_half_width_;
        }

        wl_samples[i] = std::clamp(min_dist_l, 0.9, 3.5);
        wr_samples[i] = std::clamp(min_dist_r, 0.9, 3.5);
    }

    if (is_closed_) {
        double wl_avg = 0.5 * (wl_samples[0] + wl_samples[num_pts - 1]);
        double wr_avg = 0.5 * (wr_samples[0] + wr_samples[num_pts - 1]);
        wl_samples[0] = wl_avg;
        wl_samples[num_pts - 1] = wl_avg;
        wr_samples[0] = wr_avg;
        wr_samples[num_pts - 1] = wr_avg;
    }

    try {
        spline_wl_.build(s_samples, wl_samples);
        spline_wr_.build(s_samples, wr_samples);
        has_local_widths_ = true;
    } catch (const std::exception& e) {
        std::cerr << "[FrenetTrack] Warning: Failed to build boundary splines: " << e.what() << std::endl;
        has_local_widths_ = false;
    }
}

} // namespace mpc
