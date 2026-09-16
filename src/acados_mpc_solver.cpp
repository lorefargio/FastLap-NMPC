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

void AcadosMpcSolver::setStageLateralBounds(int stage, double e_y_min, double e_y_max, double v_max) {
    if (!is_initialized_ || stage <= 0 || stage >= MPC_N) return;

    // Update box constraints: idxbx = [1, 3, 4] -> e_y (0), v (1), delta (2)
    double lbx_stage[3] = {e_y_min, 0.0, -0.52};
    double ubx_stage[3] = {e_y_max, v_max, 0.52};
    ocp_nlp_constraints_model_set(
        pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, pimpl_->nlp_out, stage, "lbx", lbx_stage);
    ocp_nlp_constraints_model_set(
        pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, pimpl_->nlp_out, stage, "ubx", ubx_stage);
}

void AcadosMpcSolver::setStageControlBounds(int stage, double a_min, double a_max, 
                                           double v_delta_min, double v_delta_max) {
    if (!is_initialized_ || stage < 0 || stage >= MPC_N) return;

    double lbu_stage[2] = {a_min, v_delta_min};
    double ubu_stage[2] = {a_max, v_delta_max};
    ocp_nlp_constraints_model_set(
        pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, pimpl_->nlp_out, stage, "lbu", lbu_stage);
    ocp_nlp_constraints_model_set(
        pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, pimpl_->nlp_out, stage, "ubu", ubu_stage);
}

void AcadosMpcSolver::setStageReference(int stage, double v_ref, double e_y_ref, double e_psi_ref, 
                                       double delta_ref, double a_ref, double v_delta_ref) {
    if (!is_initialized_ || stage < 0 || stage > MPC_N) return;

    if (stage == MPC_N) {
        double yref_e[4] = {v_ref, e_y_ref, e_psi_ref, delta_ref};
        ocp_nlp_cost_model_set(
            pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, stage, "yref", yref_e);
    } else {
        double yref[6] = {v_ref, e_y_ref, e_psi_ref, delta_ref, a_ref, v_delta_ref};
        ocp_nlp_cost_model_set(
            pimpl_->nlp_config, pimpl_->nlp_dims, pimpl_->nlp_in, stage, "yref", yref);
    }
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

    // Query acados internal stage timings and QP statistics
    double time_lin_s = 0.0;
    double time_qp_s = 0.0;
    double cost_val = 0.0;
    int qp_iter = 0;
    int qp_status = 0;
    ocp_nlp_get(pimpl_->nlp_solver, "time_lin", &time_lin_s);
    ocp_nlp_get(pimpl_->nlp_solver, "time_qp", &time_qp_s);
    ocp_nlp_get(pimpl_->nlp_solver, "nlp_iter", &qp_iter);
    ocp_nlp_get(pimpl_->nlp_solver, "qp_status", &qp_status);
    ocp_nlp_get(pimpl_->nlp_solver, "cost_value", &cost_val);

    result.time_lin_ms = time_lin_s * 1000.0;
    result.time_qp_ms = time_qp_s * 1000.0;
    result.qp_iter = qp_iter;
    result.qp_status = qp_status;
    result.cost_value = cost_val;

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
    if (result.predicted_states.size() > 1) {
        result.target_steering_angle = result.predicted_states[1][4];
        result.predicted_x1 = result.predicted_states[1];
    }

    return result;
}

} // namespace mpc
