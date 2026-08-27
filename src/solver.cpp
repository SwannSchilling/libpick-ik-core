// Generic IkSolver implementations:
//   - CcdSolver:            faithful C++ port of the p5.js POC's solveCCD
//   - PickIkGradientSolver: wrapper around the upstream ik_gradient
//   - PickIkMemeticSolver:  wrapper around the upstream ik_memetic
//
// See include/pick_ik/solvers.hpp for the contract documentation.

#include <pick_ik/solvers.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pick_ik {

namespace {

/// Tip FK derived from a LinkFkFn (keeps only the last frame).
auto make_tip_frames_fn(LinkFkFn const& link_fk) -> FkFn {
    auto const lk = link_fk;
    return [lk = std::move(lk)](std::vector<double> const& q) -> std::vector<Eigen::Isometry3d> {
        auto const frames = lk(q);
        return std::vector<Eigen::Isometry3d>{frames.back()};
    };
}

/// Geodesic distance between two rotations [rad].
auto orientation_error(Eigen::Matrix3d const& a, Eigen::Matrix3d const& b) -> double {
    Eigen::Matrix3d const rel = a.transpose() * b;
    double const c = std::clamp(0.5 * (rel.trace() - 1.0), -1.0, 1.0);
    return std::acos(c);
}

/// Per-joint angle targets: `sum_i (q_i - target_i)^2` over the specified
/// joints (`std::nullopt` entries are skipped).
auto make_joint_angle_targets_cost_fn(std::vector<std::optional<double>> const& targets)
    -> CostFn {
    return [targets](std::vector<double> const& q) -> double {
        double cost = 0.0;
        for (size_t i = 0; i < targets.size() && i < q.size(); ++i) {
            if (!targets[i].has_value()) continue;
            double const dq = q[i] - *targets[i];
            cost += dq * dq;
        }
        return cost;
    };
}

/// Look-at: `(1 - (R_tip * axis) . dir_to_point)^2`, 0.0 = perfectly
/// aligned. Re-evaluates the tip frame, so it costs one extra FK call per
/// cost evaluation.
auto make_look_at_cost_fn(FkFn const& tip_fk, LookAtTarget const& target) -> CostFn {
    Eigen::Vector3d axis = target.axis;
    if (axis.norm() < 1e-12) axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    axis.normalize();
    return [tip_fk, target, axis](std::vector<double> const& q) -> double {
        auto const frames = tip_fk(q);
        auto const& tip = frames.back();
        Eigen::Vector3d const to_point = target.point - tip.translation();
        double const dist = to_point.norm();
        if (dist < 1e-9) return 0.0;  // tip at the point: trivially aligned
        Eigen::Vector3d const dir = to_point / dist;
        double const dot = (tip.rotation() * axis).dot(dir);  // in [-1, 1]
        double const err = 1.0 - dot;
        return err * err;
    };
}

/// Fill `result` with the position/orientation errors of `q` vs. `targets`.
void fill_errors(IkResult& result,
                 FkFn const& tip_fk,
                 std::vector<double> const& q,
                 std::vector<Eigen::Isometry3d> const& targets,
                 bool evaluate_orientation) {
    auto const tips = tip_fk(q);
    if (tips.size() != targets.size()) {
        throw std::invalid_argument("tip frame count != target count");
    }
    result.position_error = 0.0;
    result.orientation_error = evaluate_orientation ? 0.0 : -1.0;
    for (size_t i = 0; i < targets.size(); ++i) {
        result.position_error =
            std::max(result.position_error,
                     (tips[i].translation() - targets[i].translation()).norm());
        if (evaluate_orientation) {
            result.orientation_error =
                std::max(result.orientation_error,
                         orientation_error(targets[i].rotation(), tips[i].rotation()));
        }
    }
}

/// Shared plumbing for the PickIK wrappers: build frame tests, pose costs,
/// solution test and cost function from the unified options, run the
/// upstream solver, and package the result.
template <typename SolveFn>
auto solve_with_pickik(Robot const& robot,
                       FkFn const& tip_fk,
                       std::vector<double> const& q_seed,
                       std::vector<Eigen::Isometry3d> const& targets,
                       SolveOptions const& options,
                       SolveFn const& solve_fn) -> IkResult {
    auto const frame_tests =
        make_frame_tests(targets, options.position_threshold, options.orientation_threshold);
    std::vector<Goal> goals;
    if (options.minimal_displacement_weight > 0.0) {
        goals.push_back(
            Goal{make_minimal_displacement_cost_fn(robot, q_seed),
                 options.minimal_displacement_weight});
    }
    if (!options.joint_angle_targets.empty()) {
        if (options.joint_angle_targets.size() != robot.variables.size()) {
            throw std::invalid_argument(
                "joint_angle_targets must have one entry per joint");
        }
        bool const any_target = std::any_of(options.joint_angle_targets.begin(),
                                            options.joint_angle_targets.end(),
                                            [](auto const& t) { return t.has_value(); });
        if (any_target && options.joint_target_weight > 0.0) {
            goals.push_back(Goal{make_joint_angle_targets_cost_fn(
                                  options.joint_angle_targets),
                                 options.joint_target_weight});
        }
    }
    if (options.look_at.has_value() && options.look_at_weight > 0.0) {
        goals.push_back(Goal{make_look_at_cost_fn(tip_fk, *options.look_at),
                             options.look_at_weight});
    }
    auto const solution_fn =
        make_is_solution_test_fn(frame_tests, goals, options.cost_threshold, tip_fk);
    auto const pose_costs =
        make_pose_cost_functions(targets, options.position_scale, options.rotation_scale);
    auto const cost_fn = make_cost_fn(pose_costs, goals, tip_fk);

    IkResult result;
    auto const maybe = solve_fn(robot, cost_fn, solution_fn);
    result.q = maybe.value_or(q_seed);
    fill_errors(result, tip_fk, result.q, targets, options.orientation_threshold.has_value());
    result.success = maybe.has_value() && result.position_error < options.position_threshold &&
                     (!options.orientation_threshold.has_value() ||
                      result.orientation_error < options.orientation_threshold.value());
    return result;
}

}  // namespace

auto make_tip_fk(LinkFkFn link_fk) -> FkFn {
    return make_tip_frames_fn(link_fk);
}

// ---------------------------------------------------------------------------
// CcdSolver
// ---------------------------------------------------------------------------

CcdSolver::CcdSolver(int max_passes, double damping, double epsilon)
    : max_passes_(max_passes), damping_(damping), epsilon_(epsilon) {
    if (max_passes_ <= 0 || damping_ <= 0.0 || epsilon_ <= 0.0) {
        throw std::invalid_argument("CcdSolver: max_passes/damping/epsilon must be positive");
    }
}

auto CcdSolver::solve(Robot const& robot,
                      LinkFkFn const& link_fk,
                      std::vector<Eigen::Vector3d> const& local_axes,
                      std::vector<double> const& q_seed,
                      std::vector<Eigen::Isometry3d> const& targets,
                      SolveOptions const& options) const -> IkResult {
    size_t const n = q_seed.size();
    if (n != robot.variables.size() || n != local_axes.size() || targets.empty()) {
        throw std::invalid_argument("CcdSolver: seed/axes/robot/targets size mismatch");
    }

    // Position-only, single target (POC semantics).
    Eigen::Vector3d const target_pos = targets.front().translation();

    std::vector<double> q = q_seed;
    {
        auto const frames = link_fk(q_seed);
        if (frames.size() != n + 1) {
            throw std::invalid_argument("CcdSolver: link_fk must return n + 1 frames "
                                        "(joint frames + tip frame)");
        }
    }
    for (int pass = 0; pass < max_passes_; ++pass) {
        for (int j = static_cast<int>(n) - 1; j >= 0; --j) {
            // Re-read the configuration at every joint update (the POC
            // re-reads its sliders at every joint step).
            auto const frames = link_fk(q);
            if (frames.size() != n + 1) {
                throw std::invalid_argument("CcdSolver: link_fk must return n + 1 frames "
                                            "(joint frames + tip frame)");
            }
            Eigen::Vector3d const ee = frames.back().translation();
            Eigen::Vector3d const joint_pos = frames[static_cast<size_t>(j)].translation();
            Eigen::Vector3d const world_axis =
                (frames[static_cast<size_t>(j)].rotation() * local_axes[static_cast<size_t>(j)])
                    .normalized();

            Eigen::Vector3d const ee_vec = ee - joint_pos;
            Eigen::Vector3d const target_vec = target_pos - joint_pos;

            // Project both vectors onto the plane perpendicular to the joint axis.
            Eigen::Vector3d p_ee =
                ee_vec - world_axis * ee_vec.dot(world_axis);
            Eigen::Vector3d p_target =
                target_vec - world_axis * target_vec.dot(world_axis);
            if (p_ee.squaredNorm() < epsilon_ * epsilon_ ||
                p_target.squaredNorm() < epsilon_ * epsilon_) {
                continue;
            }
            p_ee.normalize();
            p_target.normalize();

            double const cos_angle = std::clamp(p_ee.dot(p_target), -1.0, 1.0);
            double angle_delta = std::acos(cos_angle);
            if (world_axis.dot(p_ee.cross(p_target)) < 0.0) {
                angle_delta = -angle_delta;
            }
            q[static_cast<size_t>(j)] =
                robot.variables[static_cast<size_t>(j)].clamp_to_limits(q[static_cast<size_t>(j)] +
                                                                        damping_ * angle_delta);
        }
    }

    IkResult result;
    result.q = std::move(q);
    result.position_error =
        (link_fk(result.q).back().translation() - target_pos).norm();
    result.orientation_error = -1.0;  // position-only solver
    result.success = result.position_error < options.position_threshold;
    return result;
}

// ---------------------------------------------------------------------------
// PickIkGradientSolver
// ---------------------------------------------------------------------------

PickIkGradientSolver::PickIkGradientSolver(GradientIkParams params) : params_(params) {}

auto PickIkGradientSolver::solve(Robot const& robot,
                                 LinkFkFn const& link_fk,
                                 std::vector<Eigen::Vector3d> const& /*local_axes*/,
                                 std::vector<double> const& q_seed,
                                 std::vector<Eigen::Isometry3d> const& targets,
                                 SolveOptions const& options) const -> IkResult {
    auto const tip_fk = make_tip_frames_fn(link_fk);
    return solve_with_pickik(robot, tip_fk, q_seed, targets, options,
                             [this, q_seed](Robot const& r, CostFn const& c,
                                            SolutionTestFn const& s) {
                                 return ik_gradient(q_seed, r, c, s, this->params_,
                                                    /*approx_solution=*/false);
                             });
}

// ---------------------------------------------------------------------------
// PickIkMemeticSolver
// ---------------------------------------------------------------------------

PickIkMemeticSolver::PickIkMemeticSolver(MemeticIkParams params) : params_(params) {}

auto PickIkMemeticSolver::solve(Robot const& robot,
                                LinkFkFn const& link_fk,
                                std::vector<Eigen::Vector3d> const& /*local_axes*/,
                                std::vector<double> const& q_seed,
                                std::vector<Eigen::Isometry3d> const& targets,
                                SolveOptions const& options) const -> IkResult {
    auto const tip_fk = make_tip_frames_fn(link_fk);
    return solve_with_pickik(robot, tip_fk, q_seed, targets, options,
                             [this, q_seed](Robot const& r, CostFn const& c,
                                            SolutionTestFn const& s) {
                                 return ik_memetic(q_seed, r, c, s, this->params_,
                                                  /*approx_solution=*/false,
                                                  /*print_debug=*/false);
                             });
}

}  // namespace pick_ik
