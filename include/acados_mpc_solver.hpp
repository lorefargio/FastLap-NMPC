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
    int qp_status = 0;                  // QP solver status
    int qp_iter = 0;                    // Number of QP iterations
    double solve_time_us = 0.0;         // Wall-clock solve latency in microseconds
    double time_lin_ms = 0.0;           // Linearization / preparation phase latency in milliseconds
    double time_qp_ms = 0.0;            // QP solution / feedback phase latency in milliseconds
    double cost_value = 0.0;            // OCP optimal objective function value J
    ControlVector optimal_u{0.0, 0.0};  // [a_opt, v_delta_opt]
    double target_steering_angle = 0.0; // Steering angle predicted for next step [rad]
    StateVector predicted_x1{0.0, 0.0, 0.0, 0.0, 0.0}; // 1-step-ahead predicted state
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
     * @brief Updates state bounds (lbx, ubx on e_y, v, delta) for stage k.
     */
    void setStageLateralBounds(int stage, double e_y_min, double e_y_max, double v_max = 35.0);

    /**
     * @brief Updates control bounds (lbu, ubu on a and v_delta) for stage k.
     */
    void setStageControlBounds(int stage, double a_min, double a_max, 
                               double v_delta_min = -1.5, double v_delta_max = 1.5);

    /**
     * @brief Updates reference targets (yref) for stage k (e.g. curvature-based target velocity).
     */
    void setStageReference(int stage, double v_ref, double e_y_ref = 0.0, double e_psi_ref = 0.0, 
                           double delta_ref = 0.0, double a_ref = 0.0, double v_delta_ref = 0.0);

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
