// ============================================================================
// pick_ik_c — C ABI implementation (thin adapter over pick_ik::IkSolver)
//
// No solver code here: this file adapts C callbacks / POD structs to the
// C++ contract (handles, row-major 4x4 poses, option structs). See
// include/pick_ik_c/pickik_c.h for the interface contract.
// ============================================================================

#include <pick_ik_c/pickik_c.h>

#include <pick_ik/robot.hpp>
#include <pick_ik/solver.hpp>
#include <pick_ik/solvers.hpp>

#include "arm7/arm7.hpp"  // built-in arm7 model (Design B)

#include <Eigen/Geometry>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Complete the opaque types declared (incomplete) in pickik_c.h. These must
// be defined in the global namespace so they complete the header's tags.
struct pickik_robot_s {
    int n = 0;
    std::vector<Eigen::Vector3d> local_axes;
    pick_ik::Robot robot;
};

struct pickik_solver_s {
    std::unique_ptr<pick_ik::IkSolver> impl;
};

namespace {

// Build a pick_ik::Isometry3d from 16 row-major doubles (row stride 4).
Eigen::Isometry3d pose_from_row_major(double const* p16) {
    Eigen::Matrix4d m;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            m(r, c) = p16[4 * r + c];
    Eigen::Isometry3d f;
    f.setIdentity();
    f.linear() = m.topLeftCorner(3, 3);
    f.translation() = m.col(3).head(3);
    return f;
}

// Adapt a C link-FK callback to pick_ik::LinkFkFn. Throws on failure so the
// solve aborts with PICKIK_E_FK instead of chasing garbage frames.
pick_ik::LinkFkFn adapt_link_fk(pickik_link_fk_fn fk, void* user, int n) {
    return [fk, user, n](std::vector<double> const& q) -> std::vector<Eigen::Isometry3d> {
        std::vector<double> buf((static_cast<size_t>(n) + 1u) * 16u);
        int const rc = fk(user, n, q.data(), buf.data());
        if (rc != PICKIK_OK)
            throw std::runtime_error("host FK callback returned " + std::to_string(rc));
        std::vector<Eigen::Isometry3d> frames;
        frames.reserve(static_cast<size_t>(n) + 1u);
        for (int i = 0; i <= n; ++i)
            frames.push_back(pose_from_row_major(buf.data() + 16 * i));
        return frames;
    };
}

}  // namespace

// ---------------------------------------------------------------------------
// Robot
// ---------------------------------------------------------------------------

PICKIK_C_API int pickik_robot_create(int n_joints, double const* lower, double const* upper,
                                     double const* max_velocity, double const* local_axes,
                                     pickik_robot** out) {
    if (n_joints <= 0 || !lower || !upper || !max_velocity || !local_axes || !out)
        return PICKIK_E_BADARG;
    try {
        auto* r = new pickik_robot_s();
        r->n = n_joints;
        r->local_axes.resize(n_joints);
        std::vector<pick_ik::Robot::JointSpec> specs;
        specs.reserve(n_joints);
        for (int i = 0; i < n_joints; ++i) {
            specs.push_back({lower[i], upper[i], true, max_velocity[i]});
            r->local_axes[i] = Eigen::Vector3d(local_axes[3 * i], local_axes[3 * i + 1],
                                               local_axes[3 * i + 2]);
        }
        r->robot = pick_ik::Robot::make(specs);
        *out = r;
        return PICKIK_OK;
    } catch (...) {
        return PICKIK_E_INTERNAL;
    }
}

PICKIK_C_API int pickik_robot_is_valid(pickik_robot* robot, double const* q, int* out_valid) {
    if (!robot || !q || !out_valid) return PICKIK_E_BADARG;
    std::vector<double> v(q, q + robot->n);
    *out_valid = robot->robot.is_valid_configuration(v) ? 1 : 0;
    return PICKIK_OK;
}

PICKIK_C_API int pickik_robot_free(pickik_robot* robot) {
    delete robot;
    return PICKIK_OK;
}

// ---------------------------------------------------------------------------
// Solvers
// ---------------------------------------------------------------------------

PICKIK_C_API int pickik_ccd_create(int max_passes, double damping, double epsilon,
                                   pickik_solver** out) {
    if (!out) return PICKIK_E_BADARG;
    try {
        auto* s = new pickik_solver_s();
        s->impl = std::make_unique<pick_ik::CcdSolver>(max_passes, damping, epsilon);
        *out = s;
        return PICKIK_OK;
    } catch (...) {
        return PICKIK_E_INTERNAL;
    }
}

PICKIK_C_API int pickik_gradient_create(double step_size, double min_cost_delta, double max_time,
                                        int max_iterations, int stop_on_valid_solution,
                                        pickik_solver** out) {
    if (!out) return PICKIK_E_BADARG;
    try {
        pick_ik::GradientIkParams p;
        p.step_size = step_size;
        p.min_cost_delta = min_cost_delta;
        p.max_time = max_time;
        p.max_iterations = max_iterations;
        p.stop_optimization_on_valid_solution = stop_on_valid_solution != 0;
        auto* s = new pickik_solver_s();
        s->impl = std::make_unique<pick_ik::PickIkGradientSolver>(p);
        *out = s;
        return PICKIK_OK;
    } catch (...) {
        return PICKIK_E_INTERNAL;
    }
}

PICKIK_C_API int pickik_memetic_create(int elite_size, int population_size,
                                       double wipeout_fitness_tol, int max_generations,
                                       double max_time, int num_threads,
                                       int stop_on_valid_solution, int stop_on_first_soln,
                                       pickik_solver** out) {
    if (!out) return PICKIK_E_BADARG;
    try {
        pick_ik::MemeticIkParams p;
        p.elite_size = static_cast<size_t>(elite_size);
        p.population_size = static_cast<size_t>(population_size);
        p.wipeout_fitness_tol = wipeout_fitness_tol;
        p.max_generations = max_generations;
        p.max_time = max_time;
        p.num_threads = static_cast<size_t>(num_threads);
        p.stop_optimization_on_valid_solution = stop_on_valid_solution != 0;
        p.stop_on_first_soln = stop_on_first_soln != 0;
        auto* s = new pickik_solver_s();
        s->impl = std::make_unique<pick_ik::PickIkMemeticSolver>(p);
        *out = s;
        return PICKIK_OK;
    } catch (...) {
        return PICKIK_E_INTERNAL;
    }
}

PICKIK_C_API int pickik_solver_free(pickik_solver* solver) {
    delete solver;
    return PICKIK_OK;
}

// ---------------------------------------------------------------------------
// Options + solve
// ---------------------------------------------------------------------------

PICKIK_C_API void pickik_options_default(pickik_options* o) {
    if (!o) return;
    o->position_threshold = 1e-3;
    o->orientation_threshold = 1e-3;
    o->cost_threshold = 1e-3;
    o->position_scale = 1.0;
    o->rotation_scale = 0.0;  // position-only by default; ~0.5 for full-pose goals
    o->minimal_displacement_weight = 0.0;
    o->joint_target_weight = 0.0;
    o->joint_target_values = nullptr;
    o->joint_target_has = nullptr;
    o->has_look_at = 0;
    o->look_at_point[0] = o->look_at_point[1] = o->look_at_point[2] = 0.0;
    o->look_at_axis[0] = 1.0;
    o->look_at_axis[1] = o->look_at_axis[2] = 0.0;
    o->look_at_weight = 0.0;
}

PICKIK_C_API int pickik_solve(pickik_solver* solver, pickik_robot* robot, pickik_link_fk_fn fk,
                              void* fk_user, double const* q_seed, double const* target,
                              pickik_options const* options, double* out_q,
                              pickik_result* out_result) {
    if (!solver || !robot || !q_seed || !target || !options || !out_q || !out_result)
        return PICKIK_E_BADARG;
    if (fk == nullptr) return PICKIK_E_BADARG;  // all standard solvers need FK feedback
    auto const n = robot->n;
    try {
        auto const link_fk = adapt_link_fk(fk, fk_user, n);
        std::vector<double> seed(q_seed, q_seed + n);
        std::vector<Eigen::Isometry3d> targets{pose_from_row_major(target)};

        pick_ik::SolveOptions opt;
        opt.position_threshold = options->position_threshold;
        if (options->orientation_threshold >= 0.0)
            opt.orientation_threshold = options->orientation_threshold;
        else
            opt.orientation_threshold = std::nullopt;  // position-only goal
        opt.cost_threshold = options->cost_threshold;
        opt.position_scale = options->position_scale;
        opt.rotation_scale = options->rotation_scale;
        opt.minimal_displacement_weight = options->minimal_displacement_weight;
        if (options->joint_target_weight > 0.0 && options->joint_target_values &&
            options->joint_target_has) {
            opt.joint_angle_targets.resize(n);
            for (int i = 0; i < n; ++i) {
                if (options->joint_target_has[i])
                    opt.joint_angle_targets[i] = options->joint_target_values[i];
                // else: default nullopt (no target for this joint)
            }
            opt.joint_target_weight = options->joint_target_weight;
        }
        if (options->has_look_at && options->look_at_weight > 0.0) {
            opt.look_at = pick_ik::LookAtTarget{
                Eigen::Vector3d(options->look_at_point[0], options->look_at_point[1],
                                options->look_at_point[2]),
                Eigen::Vector3d(options->look_at_axis[0], options->look_at_axis[1],
                                options->look_at_axis[2])};
            opt.look_at_weight = options->look_at_weight;
        }

        auto const t0 = std::chrono::steady_clock::now();
        pick_ik::IkResult const result =
            solver->impl->solve(robot->robot, link_fk, robot->local_axes, seed, targets, opt);
        auto const t1 = std::chrono::steady_clock::now();

        for (int i = 0; i < n; ++i)
            out_q[i] = result.q[i];
        out_result->success = result.success ? 1 : 0;
        out_result->position_error = result.position_error;
        out_result->orientation_error = result.orientation_error;
        out_result->time_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0)
                .count();
        return PICKIK_OK;
    } catch (std::runtime_error const& e) {
        return std::string(e.what()).rfind("host FK callback", 0) == 0 ? PICKIK_E_FK
                                                                       : PICKIK_E_INTERNAL;
    } catch (...) {
        return PICKIK_E_INTERNAL;
    }
}

// ---------------------------------------------------------------------------
// Built-in arm7 model
// ---------------------------------------------------------------------------

PICKIK_C_API int pickik_arm7_robot_create(pickik_robot** out) {
    if (!out) return PICKIK_E_BADARG;
    try {
        auto* r = new pickik_robot_s();
        r->n = static_cast<int>(arm7::joints().size());
        r->local_axes = arm7::make_local_axes();
        r->robot = arm7::make_robot();
        *out = r;
        return PICKIK_OK;
    } catch (...) {
        return PICKIK_E_INTERNAL;
    }
}

PICKIK_C_API int pickik_arm7_link_fk(void* /*user*/, int n_joints, double const* q,
                                     double* frames) {
    if (n_joints != PICKIK_ARM7_JOINTS || !q || !frames) return PICKIK_E_BADARG;
    try {
        std::vector<double> const vv(q, q + n_joints);
        auto const frames_e = arm7::link_frames(vv);
        for (size_t i = 0; i < frames_e.size(); ++i) {
            Eigen::Matrix4d const m = frames_e[i].matrix();
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    frames[16 * i + 4 * r + c] = m(r, c);  // row-major out
        }
        return PICKIK_OK;
    } catch (...) {
        return PICKIK_E_INTERNAL;
    }
}

PICKIK_C_API int pickik_arm7_local_axes(double* out) {
    if (!out) return PICKIK_E_BADARG;
    for (Eigen::Vector3d const& a : arm7::make_local_axes()) {
        out[0] = a.x();
        out[1] = a.y();
        out[2] = a.z();
        out += 3;
    }
    return PICKIK_OK;
}
