#ifndef ETDV_MPC_ACADOS_MPC_SOLVER_HPP
#define ETDV_MPC_ACADOS_MPC_SOLVER_HPP

#include <vector>
#include <array>
#include <memory>
#include <chrono>

namespace mpc {

// Problem dimensions matching Python model
constexpr int MPC_N  = 30; // Prediction nodes
constexpr int MPC_NX = 5;  // States: [s, e_y, e_psi, v, delta]
constexpr int MPC_NU = 2;  // Controls: [a, v_delta]
constexpr int MPC_NP = 4;  // Parameters: [kappa, w_l, w_r, mu]

using StateVector   = std::array<double, MPC_NX>;
using ControlVector = std::array<double, MPC_NU>;
using ParamVector   = std::array<double, MPC_NP>;

struct MpcSolveResult {
    int status = 0;                     // 0 = Success, 1 = Failure
    double solve_time_us = 0.0;         // Execution latency in microseconds
    ControlVector optimal_u{0.0, 0.0};  // [a_opt, v_delta_opt]
    double target_steering_angle = 0.0; // Steering angle predicted for next step [rad]
    std::vector<StateVector> predicted_states;
};

/**
 * @brief High-performance C++ RAII wrapper around acados C-generated RTI solver.
 */
class AcadosMpcSolver {
public:
    AcadosMpcSolver();
    ~AcadosMpcSolver();

    // Prevent copying
    AcadosMpcSolver(const AcadosMpcSolver&) = delete;
    AcadosMpcSolver& operator=(const AcadosMpcSolver&) = delete;

    /**
     * @brief Allocates acados capsule, initializes solver memory and matrices.
     */
    bool init();

    /**
     * @brief Sets the initial state constraint (stage 0: lbx, ubx).
     */
    void setInitialState(const StateVector& x0);

    /**
     * @brief Sets dynamic track parameters for stage k (curvature, lane widths, grip).
     */
    void setStageParameters(int stage, double kappa, double w_l, double w_r, double mu);

    /**
     * @brief Updates lateral error constraints (lbx, ubx on e_y) for stage k.
     */
    void setStageLateralBounds(int stage, double e_y_min, double e_y_max);

    /**
     * @brief Executes the acados Real-Time Iteration (RTI) step.
     */
    MpcSolveResult solve();

    /**
     * @brief Returns whether the solver capsule is initialized and ready.
     */
    bool isInitialized() const { return is_initialized_; }

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    bool is_initialized_ = false;
};

} // namespace mpc

#endif // ETDV_MPC_ACADOS_MPC_SOLVER_HPP
