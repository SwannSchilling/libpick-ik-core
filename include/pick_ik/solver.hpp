#pragma once

#include <pick_ik/fk.hpp>
#include <pick_ik/robot.hpp>

#include <Eigen/Geometry>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace pick_ik {

/**
 * @brief Forward-kinematics callback returning every joint frame plus the tip frame.
 *
 * For n active joints, `link_fk(q)` returns n + 1 frames, in the base frame:
 *   - `frames[i]` for 0 <= i < n: the frame of joint i's child link (the pivot
 *     frame, i.e. parent frame * joint origin * R(axis, q_i));
 *   - `frames[n]`: the tip frame (e.g. tool0).
 *
 * This is the richer FK contract required by solvers that inspect individual
 * joints (the CCD solver needs each joint pivot and world axis). The
 * tip-only `pick_ik::FkFn` is derivable by taking the last frame — see
 * `make_tip_fk`.
 *
 * Implementations must be re-entrant, exactly like `FkFn`.
 */
using LinkFkFn = std::function<std::vector<Eigen::Isometry3d>(std::vector<double> const&)>;

/** @brief Derive a tip-only `FkFn` from a `LinkFkFn` (keeps the last frame). */
auto make_tip_fk(LinkFkFn link_fk) -> FkFn;

/** @brief Common options for `IkSolver::solve`. */
struct SolveOptions {
    /// Position error threshold [m] at which a configuration counts as a solution.
    double position_threshold = 1e-3;
    /// Orientation error threshold [rad]; `std::nullopt` solves position-only.
    std::optional<double> orientation_threshold = 1e-3;
    /// Cost-function threshold used by the PickIK solvers' solution test [arbitrary cost units].
    double cost_threshold = 1e-3;
    /// Position weight of the pose cost function.
    double position_scale = 1.0;
    /// Orientation weight of the pose cost function.
    double rotation_scale = 0.5;
};

/** @brief Result of an `IkSolver::solve` call. */
struct IkResult {
    /// True if the configuration satisfies the position (and, when enabled,
    /// orientation) thresholds.
    bool success = false;
    /// Solution configuration; on failure the seed (PickIK solvers) or the
    /// final (best-effort) configuration (CCD).
    std::vector<double> q;
    /// Max position error vs. the targets [m]; -1 if not computed.
    double position_error = -1.0;
    /// Max orientation error vs. the targets [rad]; -1 when not evaluated
    /// (position-only solve, or failure).
    double orientation_error = -1.0;
};

/**
 * @brief Generic inverse-kinematics solver interface.
 *
 * All solver implementations (CCD, PickIK gradient, PickIK memetic, and
 * future solvers) share this contract so any frontend (p5.js via transport
 * layer, Python, Unity, C++ robot controller, ...) can use or swap solvers
 * without changing its code:
 *
 * @code
 *   pick_ik::CcdSolver ccd;
 *   auto result = ccd.solve(robot, link_fk, axes, seed, {target}, options);
 * @endcode
 *
 * @note v1 supports a single tip frame (the last frame returned by
 *       `link_fk`) and one target pose.
 */
class IkSolver {
   public:
    virtual ~IkSolver() = default;

    /// @brief Short identifier, e.g. "ccd", "gradient", "memetic".
    virtual auto name() const -> std::string = 0;

    /**
     * @brief Whether `solve` evaluates `link_fk` on native worker threads in
     *        addition to the calling thread.
     *
     * Front-ends that hand the solver a thread-aware FK callback (e.g. the
     * Python binding, whose callback must hold the interpreter GIL) use this to
     * decide whether to release the GIL for the duration of `solve`. Solvers
     * that spawn no FK-calling native threads report `false` and the callback
     * runs only on the calling thread.
     *
     * @note The memetic solver reports `true` even for a single species thread,
     *       because its gradient-descent exploitation always runs on separate
     *       threads.
     */
    virtual auto spawns_fk_worker_threads() const -> bool { return false; }

    /**
     * @brief Solve inverse kinematics for `targets` starting from `q_seed`.
     * @param robot      Kinematic limits / velocity specs of the n variables.
     * @param link_fk     Joint-frame + tip-frame forward kinematics.
     * @param local_axes  Joint i's rotation axis, expressed in joint i's frame
     *                    (e.g. the URDF joint axis).
     * @param q_seed      Initial configuration, size n.
     * @param targets     Target pose(s) in the base frame; v1: exactly one.
     * @param options     Thresholds and cost scaling.
     */
    virtual auto solve(Robot const& robot,
                      LinkFkFn const& link_fk,
                      std::vector<Eigen::Vector3d> const& local_axes,
                      std::vector<double> const& q_seed,
                      std::vector<Eigen::Isometry3d> const& targets,
                      SolveOptions const& options) const -> IkResult = 0;
};

}  // namespace pick_ik
