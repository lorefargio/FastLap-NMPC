#ifndef ETDV_MPC_FRENET_TRACK_HPP
#define ETDV_MPC_FRENET_TRACK_HPP

#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <cmath>
#include <memory>
#include <limits>
#include <string>

namespace mpc {

/**
 * @brief 1D Cubic Spline for smooth parameterization of x(s), y(s), and path properties.
 */
class CubicSpline1D {
public:
    CubicSpline1D() = default;
    void build(const Eigen::VectorXd& x, const Eigen::VectorXd& y);
    double operator()(double t) const;
    double derivative(double t) const;
    double secondDerivative(double t) const;

private:
    Eigen::VectorXd x_;
    Eigen::VectorXd a_, b_, c_, d_;
    int n_ = 0;
    int findSegment(double t) const;
};

/**
 * @brief Represents a reference track with arc-length parameterization,
 * curvature interpolation, track widths, and Frenet coordinate projection.
 */
class FrenetTrack {
public:
    FrenetTrack() = default;
    
    /**
     * @brief Builds splines from a sequence of 2D centerline waypoints.
     * @param x_vals X coordinates in map frame
     * @param y_vals Y coordinates in map frame
     * @param is_closed Whether the track is a closed circuit (wraps around)
     * @param default_track_width Nominal total track width [m]
     */
    bool build(const std::vector<double>& x_vals, 
               const std::vector<double>& y_vals, 
               bool is_closed = false, 
               double default_track_width = 3.0,
               bool apply_smoothing = true);

    /**
     * @brief Transforms global vehicle pose (x, y, psi) to Frenet frame.
     * @return std::tuple<double s, double e_y, double e_psi, double kappa>
     */
    std::tuple<double, double, double, double> cartesianToFrenet(
        double x, double y, double psi) const;

    /**
     * @brief Transforms Frenet coordinates (s, e_y, e_psi) back to global Cartesian (x, y, psi).
     * @return std::tuple<double x, double y, double psi>
     */
    std::tuple<double, double, double> frenetToCartesian(
        double s, double e_y, double e_psi) const;

    /**
     * @brief Evaluates reference point properties at arc-length s.
     * @return std::tuple<double x, double y, double heading, double curvature>
     */
    std::tuple<double, double, double, double> getReferencePoint(double s) const;

    /**
     * @brief Returns total length of the parameterized track segment [m].
     */
    double getTrackLength() const { return track_length_; }

    /**
     * @brief Returns whether the track has been successfully initialized.
     */
    bool isReady() const { return is_initialized_; }

    /**
     * @brief Gets the nominal half-width for left and right boundaries [m].
     */
    double getLeftWidth() const { return nominal_half_width_; }
    double getRightWidth() const { return nominal_half_width_; }

    /**
     * @brief Gets the local corridor half-width at arc-length s [m].
     */
    double getLeftWidth(double s) const;
    double getRightWidth(double s) const;

    /**
     * @brief Gets the local left and right corridor half-widths at arc-length s [m].
     * @return std::pair<double w_left, double w_right>
     */
    std::pair<double, double> getCorridorBounds(double s) const {
        return { getLeftWidth(s), getRightWidth(s) };
    }

    /**
     * @brief Incorporates track boundary cones to fit continuous local corridor half-widths.
     */
    void setBoundaryCones(const std::vector<Eigen::Vector2d>& left_cones,
                          const std::vector<Eigen::Vector2d>& right_cones);

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

private:
    Eigen::VectorXd s_;      // Arc length along centerline
    Eigen::VectorXd x_;      // Cartesian X
    Eigen::VectorXd y_;      // Cartesian Y
    Eigen::VectorXd dx_;     // First derivative dx/ds
    Eigen::VectorXd dy_;     // First derivative dy/ds

    CubicSpline1D spline_x_;
    CubicSpline1D spline_y_;
    CubicSpline1D spline_wl_;
    CubicSpline1D spline_wr_;

    double track_length_ = 0.0;
    double nominal_half_width_ = 1.5;
    bool is_closed_ = false;
    bool is_initialized_ = false;
    bool has_local_widths_ = false;

    mutable size_t last_closest_idx_ = 0;

    int findClosestSegment(double x, double y) const;
};

} // namespace mpc

#endif // ETDV_MPC_FRENET_TRACK_HPP
