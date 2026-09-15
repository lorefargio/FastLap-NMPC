#include "acados_mpc_solver.hpp"
#include <iostream>
#include <cstring>

// acados generated C headers
extern "C" {
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_kinematic_frenet.h"
}

namespace mpc {

struct AcadosMpcSolver::Impl {
    kinematic_frenet_solver_capsule* capsule = nullptr;
    ocp_nlp_config* nlp_config = nullptr;
    ocp_nlp_dims* nlp_dims = nullptr;
    ocp_nlp_in* nlp_in = nullptr;
    ocp_nlp_out* nlp_out = nullptr;
    ocp_nlp_solver* nlp_solver = nullptr;

    ~Impl() {
        if (capsule) {
            kinematic_frenet_acados_free(capsule);
            kinematic_frenet_acados_free_capsule(capsule);
            capsule = nullptr;
        }
    }
};

AcadosMpcSolver::AcadosMpcSolver() : pimpl_(std::make_unique<Impl>()) {}

AcadosMpcSolver::~AcadosMpcSolver() = default;

bool AcadosMpcSolver::init() {
    pimpl_->capsule = kinematic_frenet_acados_create_capsule();
    if (!pimpl_->capsule) {
        std::cerr << "[AcadosMpcSolver] ERROR: Failed to allocate acados capsule." << std::endl;
        return false;
    }

    int status = kinematic_frenet_acados_create(pimpl_->capsule);
    if (status != 0) {
        std::cerr << "[AcadosMpcSolver] ERROR: Failed to create acados solver, code: " << status << std::endl;
        return false;
    }

    pimpl_->nlp_config = kinematic_frenet_acados_get_nlp_config(pimpl_->capsule);
    pimpl_->nlp_dims   = kinematic_frenet_acados_get_nlp_dims(pimpl_->capsule);
    pimpl_->nlp_in     = kinematic_frenet_acados_get_nlp_in(pimpl_->capsule);
    pimpl_->nlp_out    = kinematic_frenet_acados_get_nlp_out(pimpl_->capsule);
    pimpl_->nlp_solver = kinematic_frenet_acados_get_nlp_solver(pimpl_->capsule);

    is_initialized_ = true;
    std::cout << "[AcadosMpcSolver] ✓ acados C-solver initialized successfully." << std::endl;
    return true;
}

void AcadosMpcSolver::setInitialState(const StateVector& x0) {
    if (!is_initialized_) return;

    // Set initial condition box constraint on stage 0
    ocp_nlp_constraints_model_set(
        pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, pimpl_->nlp_out, 0, "lbx", const_cast<double*>(x0.data()));
    ocp_nlp_constraints_model_set(
        pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, pimpl_->nlp_out, 0, "ubx", const_cast<double*>(x0.data()));
}

void AcadosMpcSolver::setStageParameters(int stage, double kappa, double w_l, double w_r, double mu) {
    if (!is_initialized_ || stage < 0 || stage > MPC_N) return;

    double p[4] = {kappa, w_l, w_r, mu};
    kinematic_frenet_acados_update_params(pimpl_->capsule, stage, p, 4);
}

void AcadosMpcSolver::setStageLateralBounds(int stage, double e_y_min, double e_y_max) {
    if (!is_initialized_ || stage <= 0 || stage > MPC_N) return;

    // Update box constraint on e_y (idxbx = [1, 3, 4] -> e_y is first entry)
    // Note: Acados constraints allow setting lbx / ubx subvectors
    double lbx_stage[3] = {e_y_min, 0.0, -0.52};
    double ubx_stage[3] = {e_y_max, 25.0, 0.52};
    ocp_nlp_constraints_model_set(
        pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, pimpl_->nlp_out, stage, "lbx", lbx_stage);
    ocp_nlp_constraints_model_set(
        pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, pimpl_->nlp_out, stage, "ubx", ubx_stage);
}

MpcSolveResult AcadosMpcSolver::solve() {
    MpcSolveResult result;
    if (!is_initialized_) {
        result.status = -1;
        return result;
    }

    auto t0 = std::chrono::high_resolution_clock::now();

    // Execute RTI solver
    result.status = kinematic_frenet_acados_solve(pimpl_->capsule);

    auto t1 = std::chrono::high_resolution_clock::now();
    result.solve_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    // Extract optimal control input at stage 0: u = [a, v_delta]
    ocp_nlp_out_get(
        pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_out, 0, "u", result.optimal_u.data());

    // Extract predicted states across horizon
    result.predicted_states.reserve(MPC_N + 1);
    for (int k = 0; k <= MPC_N; ++k) {
        StateVector x_k;
        ocp_nlp_out_get(
            pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_out, k, "x", x_k.data());
        result.predicted_states.push_back(x_k);
    }

    // Next predicted steering angle: state index 4 (delta) at stage 1
    result.target_steering_angle = result.predicted_states[1][4];

    return result;
}

} // namespace mpc
