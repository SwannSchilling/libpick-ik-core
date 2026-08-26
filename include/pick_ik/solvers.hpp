#pragma once

#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/solver.hpp>

namespace pick_ik {

/**
 * @brief Cyclic Coordinate Descent — a faithful C++ port of the p5.js POC's
 *        `solveCCD` (RobotArm_2026_08_25_10_03_56/sketch.js).
 *
 * Behavior matches the POC exactly:
 *   - positions joints J7 -> J1, one joint at a time;
 *   - each step projects the joint->end-effector and joint->target vectors
 *     onto the plane perpendicular to the joint's world axis and applies a
 *     damped rotation `q_j += damping * angle_delta`;
 *   - updates are clamped to the joint limits;
 *   - the configuration is re-read from FK at every single joint update
 *     (the POC re-reads its sliders the same way).
 *
 * The POC runs this as 5 full passes per animation frame (~60 fps); a
 * `max_passes` of 300 therefore corresponds to ~1 second of POC runtime,
 * 600 to ~2 seconds.
 *
 * Position-only solver: it chases `targets[0]`'s position with the tip frame
 * and ignores orientation (`IkResult::orientation_error` is reported as -1).
 * Like the POC, it is a local method: it improves from the seed and stops in
 * a (local) minimum, which on hard targets can be centimeters from the
 * target.
 */
class CcdSolver final : public IkSolver {
   public:
    explicit CcdSolver(int max_passes = 300, double damping = 0.1, double epsilon = 1e-8);

    auto name() const -> std::string override { return "ccd"; }

    auto solve(Robot const& robot,
               LinkFkFn const& link_fk,
               std::vector<Eigen::Vector3d> const& local_axes,
               std::vector<double> const& q_seed,
               std::vector<Eigen::Isometry3d> const& targets,
               SolveOptions const& options) const -> IkResult override;

   private:
    int max_passes_;
    double damping_;
    double epsilon_;
};

/**
 * @brief `IkSolver` wrapper around the upstream `pick_ik::ik_gradient`
 *        (local gradient-descent solver; deterministic).
 *
 * The wrapper only adapts the interface (tip FK derived from `LinkFkFn`,
 * unified options/result types); the solver code path is the untouched
 * upstream implementation.
 */
class PickIkGradientSolver final : public IkSolver {
   public:
    explicit PickIkGradientSolver(GradientIkParams params = {});

    auto name() const -> std::string override { return "gradient"; }

    auto solve(Robot const& robot,
               LinkFkFn const& link_fk,
               std::vector<Eigen::Vector3d> const& local_axes,
               std::vector<double> const& q_seed,
               std::vector<Eigen::Isometry3d> const& targets,
               SolveOptions const& options) const -> IkResult override;

   private:
    GradientIkParams params_;
};

/**
 * @brief `IkSolver` wrapper around the upstream `pick_ik::ik_memetic`
 *        (global memetic / evolutionary solver with gradient exploitation).
 *
 * Wrapper-only, as for `PickIkGradientSolver`. Note: the memetic solver's
 * population is seeded randomly, so results are not bit-reproducible run to
 * run (same as the upstream free function).
 */
class PickIkMemeticSolver final : public IkSolver {
   public:
    explicit PickIkMemeticSolver(MemeticIkParams params = {});

    auto name() const -> std::string override { return "memetic"; }

    auto solve(Robot const& robot,
               LinkFkFn const& link_fk,
               std::vector<Eigen::Vector3d> const& local_axes,
               std::vector<double> const& q_seed,
               std::vector<Eigen::Isometry3d> const& targets,
               SolveOptions const& options) const -> IkResult override;

   private:
    MemeticIkParams params_;
};

}  // namespace pick_ik
