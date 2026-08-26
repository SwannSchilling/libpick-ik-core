// ============================================================================
// arm7_cross_check — standalone demo/cross-check for libpick_ik_core
// ============================================================================
//
// Uses the 7-DOF arm of the p5.js POC project
// (RobotArm_2026_08_25_10_03_56/sketch.js) as-is:
//
//   * joint table transcribed 1:1 from the POC's embedded URDF
//     (joints 1..7 revolute, joint7_to_tool0 fixed, tool0 offset z = 0.126 m)
//   * forward kinematics ported 1:1 from the POC's `computeURDFFK`:
//         base at identity;
//         for each joint (URDF document order):
//             child = parent * T(origin.xyz) * R(origin.rpy) * R(axis, q)
//         R(origin.rpy) = Rz(yaw) * Ry(pitch) * Rx(roll)  (URDF fixed-axis)
//     The POC's display-only SCALE factor (= 250) is NOT applied — all values
//     here are in meters/radians, exactly what the POC divides by SCALE to
//     show its "mm" readout.
//
// Part 1 — FK cross-check:
//   For a set of known joint configurations, prints the tool0 pose
//   (position + fixed-axis RPY) at high precision so it can be compared
//   against the sketch's FK executed in the browser:
//
//       const m  = computeURDFFK(thetas).tool0;      // 4x4, translation*250
//       const pos = [m[0][3], m[1][3], m[2][3]].map(v => v / 250);
//
// Part 2 — PickIK solve:
//   target = FK(q_true) with q_true a known configuration;
//   seed   = all-zero configuration (a different pose);
//   run ik_gradient and ik_memetic;
//   print the 7 resulting joint values, position/orientation error, and
//   verify that FK(solution) reaches the target.
//
// Part 3 — IK against the p5.js POC's external targets:
//   A (deep fold): target sliders 300/200/450 mm = (0.30, 0.20, 0.45) m;
//   B (moderate):  target sliders 450/250/450 mm = (0.45, 0.25, 0.45) m.
//   Same (quantized) seed as the sketch's "all zero" slider state,
//   position-only goal (the POC's CCD is position-only); both solvers.
//
// No ROS, no MoveIt, no solver modifications.

#include <pick_ik/fk.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>
#include <pick_ik/solvers.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <vector>

// ----------------------------------------------------------------------------
// 7-DOF arm definition (POC URDF)
// ----------------------------------------------------------------------------
namespace arm7 {

constexpr double kPi = 3.14159265358979323846;

// POC URDF limit values, verbatim (note: "-3.14159265" is the URDF text value).
constexpr double kLimPi = 3.14159265;
constexpr double kLimShoulder = 2.09;
constexpr double kVelUpper = 2.17;
constexpr double kVelWrist = 2.61;

struct Joint {
    Eigen::Vector3d origin;  ///< Joint origin translation (m), in parent frame
    Eigen::Matrix3d rotation;  ///< Origin RPY rotation (fixed-axis Rz*Ry*Rx)
    Eigen::Vector3d axis;      ///< Unit joint axis in the joint frame
};

inline Eigen::Matrix3d rpy_matrix(double roll, double pitch, double yaw) {
    // URDF RPY = fixed-axis rotations: Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Matrix3d const rx =
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d const ry =
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d const rz =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return rz * ry * rx;
}

inline Joint make_joint(Eigen::Vector3d const& origin, double roll, double pitch, double yaw,
                        Eigen::Vector3d const& axis) {
    Joint j;
    j.origin = origin;
    j.rotation = rpy_matrix(roll, pitch, yaw);
    j.axis = axis.normalized();
    return j;
}

inline std::vector<Joint> const& joints() {
    // J2/J4/J6 origins z = 0.34 / 0.40 / 0.40 m; all axes are the joint z axis;
    // alternating origins: J2, J4, J6 rotated -90 deg about x, J3, J5, J7 +90.
    static auto const table = [] {
        std::vector<Joint> j;
        Eigen::Vector3d const z(0.0, 0.0, 1.0);
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.00), 0.0, 0.0, 0.0, z));        // J1
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.34), -kPi / 2.0, 0.0, 0.0, z)); // J2
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.00), kPi / 2.0, 0.0, 0.0, z));  // J3
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.40), -kPi / 2.0, 0.0, 0.0, z)); // J4
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.00), kPi / 2.0, 0.0, 0.0, z));  // J5
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.40), -kPi / 2.0, 0.0, 0.0, z)); // J6
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.00), kPi / 2.0, 0.0, 0.0, z));  // J7
        return j;
    }();
    return table;
}

inline Eigen::Vector3d const& tool_offset() {
    static Eigen::Vector3d const t(0.0, 0.0, 0.126);  // fixed joint link7 -> tool0
    return t;
}

/// All joint child frames plus tool0, in the base frame, for q = [J1..J7]:
/// frames[0..6] = joint pivots, frames[7] = tool0.
inline std::vector<Eigen::Isometry3d> link_frames(std::vector<double> const& q) {
    std::vector<Eigen::Isometry3d> frames;
    frames.reserve(joints().size() + 1);
    Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
    size_t i = 0;
    for (Joint const& j : joints()) {
        Eigen::Isometry3d const origin =
            Eigen::Translation3d(j.origin) * Eigen::Quaterniond(j.rotation);
        frame = frame * origin * Eigen::AngleAxisd(q[i++], j.axis);
        frames.push_back(frame);
    }
    frames.push_back(frame * Eigen::Translation3d(tool_offset()));
    return frames;
}

/// tool0 pose in the base frame for q = [J1..J7] (meters/radians).
inline Eigen::Isometry3d tool0_pose(std::vector<double> const& q) {
    return link_frames(q).back();
}

/// pick_ik::FkFn returning the single tool0 frame. Pure function of q:
/// re-entrant / thread-safe.
inline pick_ik::FkFn make_fk() {
    return [](std::vector<double> const& q) {
        return std::vector<Eigen::Isometry3d>{tool0_pose(q)};
    };
}

/// pick_ik::LinkFkFn returning all joint frames + tool0.
inline pick_ik::LinkFkFn make_link_fk() {
    return [](std::vector<double> const& q) {
        return link_frames(q);
    };
}

/// Local joint axes (all z, per the POC URDF).
inline std::vector<Eigen::Vector3d> make_local_axes() {
    std::vector<Eigen::Vector3d> axes;
    for (int i = 0; i < 7; ++i) axes.push_back(Eigen::Vector3d::UnitZ());
    return axes;
}

}  // namespace arm7

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------
namespace {

/// Inverse of the POC's fixed-axis convention R = Rz(yaw)*Ry(pitch)*Rx(roll).
Eigen::Vector3d rpy_from_matrix(Eigen::Matrix3d const& r) {
    double const roll = std::atan2(r(2, 1), r(2, 2));
    double const pitch = std::atan2(-r(2, 0), std::hypot(r(0, 0), r(1, 0)));
    double const yaw = std::atan2(r(1, 0), r(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
}

void print_pose(char const* label, Eigen::Isometry3d const& pose) {
    Eigen::Vector3d const p = pose.translation();
    Eigen::Vector3d const rpy = rpy_from_matrix(pose.rotation());
    std::printf("%s\n", label);
    std::printf("  pos [m]  = [ %16.12f, %16.12f, %16.12f ]\n", p.x(), p.y(), p.z());
    std::printf("  rpy [rad]= [ %16.12f, %16.12f, %16.12f ]   (roll, pitch, yaw)\n",
                rpy.x(), rpy.y(), rpy.z());
}

void print_vec(char const* label, std::vector<double> const& v) {
    std::printf("%s = [", label);
    for (double const d : v) std::printf(" %12.9f", d);
    std::printf(" ]\n");
}

double position_error(Eigen::Isometry3d const& pose, Eigen::Isometry3d const& target) {
    return (pose.translation() - target.translation()).norm();
}

double orientation_error(Eigen::Isometry3d const& pose, Eigen::Isometry3d const& target) {
    Eigen::Quaterniond const a(pose.rotation());
    Eigen::Quaterniond const b(target.rotation());
    return 2.0 * std::acos(std::min(1.0, std::abs(a.dot(b))));
}

/// Runs one solver, prints the outcome, and verifies FK(solution) ~ target.
/// For position-only goals the orientation of the (arbitrary) solution is
/// not checked against the target.
void report(char const* name, Eigen::Isometry3d const& target, pick_ik::FkFn const& fk,
            pick_ik::Robot const& robot, std::optional<std::vector<double>> const& maybe,
            bool position_only = false) {
    std::printf("\n=== %s ===\n", name);
    if (!maybe.has_value()) {
        std::printf("  NO SOLUTION (seed not improved into the target neighborhood)\n");
        return;
    }
    std::vector<double> const& q = maybe.value();
    print_vec("  solution [rad]", q);
    std::printf("  solution [deg]= [");
    for (double const d : q) std::printf(" %12.6f", d * 180.0 / M_PI);
    std::printf(" ]\n");

    Eigen::Isometry3d const final_pose = fk(q)[0];
    double const pos_err = position_error(final_pose, target);
    double const rot_err = orientation_error(final_pose, target);
    std::printf("  position error  = %.9f m\n", pos_err);
    std::printf("  orientation err = %.9f rad\n", rot_err);
    std::printf("  within joint limits: %s\n",
                robot.is_valid_configuration(q) ? "yes" : "NO");
    bool const reached = position_only ? (pos_err < 1e-3) : (pos_err < 1e-3 && rot_err < 1e-3);
    std::printf("  FK(solution) reaches target: %s\n", reached ? "YES" : "no");
}

}  // namespace

using arm7::kLimPi;
using arm7::kLimShoulder;
using arm7::kVelUpper;
using arm7::kVelWrist;

int main() {
    pick_ik::FkFn const fk = arm7::make_fk();

    // =========================================================================
    // Part 1 — FK cross-check configurations
    // =========================================================================
    std::printf("=== Part 1: tool0 FK for known joint configurations ===\n");
    std::printf("(compare against the p5.js sketch: computeURDFFK(q).tool0 / SCALE)\n\n");

    std::vector<std::vector<double>> const configs = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, -0.3, 0.2, -0.5, 0.1, 0.4, -0.2},
        {1.0, -0.7, 0.3, -0.9, 0.4, -0.6, 0.2},
        {0.7853981633974483, 0.5235987755982988, -0.7853981633974483, 0.1, -0.2617993877991494,
         0.5, 0.3},
        {-2.0, 0.8, -1.5, 1.7, -0.9, 1.9, 2.5},
    };

    for (size_t i = 0; i < configs.size(); ++i) {
        std::printf("-- q%d: [", i);
        for (double const d : configs[i]) std::printf(" %9.6f", d);
        std::printf(" ]\n");
        print_pose("   tool0", fk(configs[i])[0]);
    }

    // =========================================================================
    // Part 2 — PickIK solve
    // =========================================================================
    std::printf("\n=== Part 2: PickIK solve (target from q_true, seed = all zeros) ===\n");

    // Robot: POC URDF limits and max velocities.
    pick_ik::Robot const robot = pick_ik::Robot::make(
        {{-kLimPi,  kLimPi,  true, kVelUpper},   // J1
         {-kLimShoulder, kLimShoulder, true, kVelUpper},  // J2
         {-kLimPi,  kLimPi,  true, kVelUpper},   // J3
         {-kLimShoulder, kLimShoulder, true, kVelUpper},  // J4
         {-kLimPi,  kLimPi,  true, kVelWrist},   // J5
         {-kLimShoulder, kLimShoulder, true, kVelWrist},  // J6
         {-kLimPi,  kLimPi,  true, kVelWrist}});  // J7

    std::vector<double> const q_true = {0.5, -0.3, 0.2, -0.5, 0.1, 0.4, -0.2};
    Eigen::Isometry3d const target = fk(q_true)[0];
    std::vector<double> const seed(7, 0.0);

    std::printf("target = FK(q_true)\n");
    print_vec("  q_true [rad]", q_true);
    print_pose("  target tool0", target);
    print_vec("  seed [rad]", seed);

    double constexpr position_threshold = 1e-3;
    double constexpr orientation_threshold = 1e-3;
    double constexpr cost_threshold = 1e-3;

    auto const frame_tests =
        pick_ik::make_frame_tests({target}, position_threshold, orientation_threshold);
    std::vector<pick_ik::Goal> const goals;
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, cost_threshold, fk);
    auto const pose_costs =
        pick_ik::make_pose_cost_functions({target}, /*position_scale=*/1.0,
                                          /*rotation_scale=*/0.5);
    auto const cost_fn = pick_ik::make_cost_fn(pose_costs, goals, fk);

    // --- 1) ik_gradient ---
    pick_ik::GradientIkParams gd_params;
    gd_params.max_time = 0.5;
    gd_params.max_iterations = 500;
    auto const maybe_gradient =
        pick_ik::ik_gradient(seed, robot, cost_fn, solution_fn, gd_params,
                             /*approx_solution=*/false);
    report("ik_gradient", target, fk, robot, maybe_gradient);

    // --- 2) ik_memetic ---
    pick_ik::MemeticIkParams memetic_params;
    memetic_params.num_threads = 4;
    memetic_params.max_time = 2.0;
    auto const maybe_memetic =
        pick_ik::ik_memetic(seed, robot, cost_fn, solution_fn, memetic_params,
                            /*approx_solution=*/false,
                            /*print_debug=*/false);
    report("ik_memetic", target, fk, robot, maybe_memetic);

    // =========================================================================
    // Part 3 — IK against external targets (p5.js POC cross-check)
    // =========================================================================
    // The p5.js sketch chases these same positions with its own CCD solver:
    // target sliders in mm, converted by
    //   targetPos = sliderValue * SCALE / 1000   (SCALE = 250)
    // i.e. slider mm -> meters. Both targets are position-only, matching the
    // POC's CCD (which drives tool0 position, ignoring orientation).
    //
    //   A = "deep fold": 300/200/450 mm. Sits deep in the workspace: every
    //       solution pins J4 or J6 at the 2.09 rad limit. The POC's CCD
    //       stalls ~30 mm short of this target from the zero seed.
    //   B = "moderate":  450/250/450 mm. Reached with J2~0.5, J4~1.3,
    //       J6~1.8 — no joint pinned at its limit.
    //
    // NOTE: an earlier candidate (100/150/450) mm was OUT of the arm's
    // workspace: with the +-2.09 rad J2/J4 limits the tool can never come
    // closer than ~0.275 m to the J2 joint at (0, 0, 0.34); that point sat
    // 0.211 m away. Both solvers correctly reported NO SOLUTION for it.
    //
    // The seed matches the sketch's quantized "all zero" slider state: p5
    // sliders snap to 0.01 rad steps anchored at the joint's lower limit
    // (value = round((v - min)/step)*step + min), so filling 0 gives 0.0 for
    // the 2.09-limited joints and -0.00159265 for the pi-limited ones.
    std::printf("\n=== Part 3: IK against external targets (p5.js cross-check) ===\n");
    {
        auto make_pos_target = [](double x, double y, double z) {
            Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
            t.translation() = Eigen::Vector3d(x, y, z);
            return t;
        };
        std::vector<std::pair<char const*, Eigen::Isometry3d>> const targets3 = {
            {"A (deep fold) 300/200/450 mm", make_pos_target(0.30, 0.20, 0.45)},
            {"B (moderate) 450/250/450 mm",  make_pos_target(0.45, 0.25, 0.45)}};
        std::vector<double> const seed3 = {-0.00159265, 0.0, -0.00159265, 0.0, -0.00159265, 0.0,
                                            -0.00159265};

        for (auto const& [label, target3] : targets3) {
            std::printf("\n--- Target %s ---\n", label);
            print_vec("  seed [rad]", seed3);
            print_pose("  target (position only)", target3);

            // Position-only goal: the POC's CCD is position-only (tool0 position).
            auto const frame_tests3 =
                pick_ik::make_frame_tests({target3}, /*position_threshold=*/1e-3,
                                          /*orientation_threshold=*/std::nullopt);
            std::vector<pick_ik::Goal> const goals3;
            auto const solution_fn3 =
                pick_ik::make_is_solution_test_fn(frame_tests3, goals3, /*cost_threshold=*/1e-3,
                                                  fk);
            auto const pose_costs3 =
                pick_ik::make_pose_cost_functions({target3}, /*position_scale=*/1.0,
                                                  /*rotation_scale=*/0.0);
            auto const cost_fn3 = pick_ik::make_cost_fn(pose_costs3, goals3, fk);

            pick_ik::GradientIkParams gd3;
            gd3.max_time = 0.5;
            gd3.max_iterations = 500;
            auto const maybe_gd3 =
                pick_ik::ik_gradient(seed3, robot, cost_fn3, solution_fn3, gd3,
                                     /*approx_solution=*/false);
            report("ik_gradient (position only)", target3, fk, robot, maybe_gd3,
                   /*position_only=*/true);

            pick_ik::MemeticIkParams me3;
            me3.num_threads = 4;
            me3.max_time = 2.0;
            auto const maybe_me3 =
                pick_ik::ik_memetic(seed3, robot, cost_fn3, solution_fn3, me3,
                                    /*approx_solution=*/false,
                                    /*print_debug=*/false);
            report("ik_memetic (position only)", target3, fk, robot, maybe_me3,
                   /*position_only=*/true);
        }
    }

    // ---------------------------------------------------------------------
    // Part 4: all three solvers through the unified IkSolver contract
    // (pick_ik/solvers.hpp) — position-only, same seed as Part 3.
    //
    //   q = solver.solve(robot, link_fk, axes, seed, {target}, options)
    //
    // This is the generic entry point any frontend (transport layer, Python,
    // C++) uses; the POC's CCD is included as a first-class implementation
    // alongside the PickIK solvers.
    // ---------------------------------------------------------------------
    {
        auto make_pos_target = [](double x, double y, double z) {
            Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
            t.translation() = Eigen::Vector3d(x, y, z);
            return t;
        };
        std::vector<std::pair<char const*, Eigen::Isometry3d>> const targets4 = {
            {"A (deep fold) 300/200/450 mm", make_pos_target(0.30, 0.20, 0.45)},
            {"B (moderate) 450/250/450 mm",  make_pos_target(0.45, 0.25, 0.45)}};
        std::vector<double> const seed4 = {-0.00159265, 0.0, -0.00159265, 0.0, -0.00159265, 0.0,
                                            -0.00159265};
        auto link_fk4 = arm7::make_link_fk();
        auto axes4 = arm7::make_local_axes();

        pick_ik::CcdSolver ccd(600);  // ~2 s of POC runtime
        pick_ik::GradientIkParams gd4;
        gd4.max_time = 2.0;  // target A's limit-pinned solution sits on a near-flat
        gd4.max_iterations = 2000;  // cost face: give the local search a wide budget
        pick_ik::PickIkGradientSolver gradient(gd4);
        pick_ik::MemeticIkParams me4;
        me4.num_threads = 4;
        me4.max_time = 2.0;
        pick_ik::PickIkMemeticSolver memetic(me4);
        std::vector<pick_ik::IkSolver const*> const solvers = {&ccd, &gradient, &memetic};

        pick_ik::SolveOptions pos_only;
        pos_only.orientation_threshold = std::nullopt;  // position-only goal
        pos_only.rotation_scale = 0.0;                  // ...and no orientation in the cost

        std::printf("\n================ Part 4: unified solver API ================\n");
        for (auto const& solver : solvers) {
            for (auto const& [label, target4] : targets4) {
                std::printf("\n--- %s | target %s ---\n", solver->name().c_str(), label);
                auto const t0 = std::chrono::steady_clock::now();
                auto const result =
                    solver->solve(robot, link_fk4, axes4, seed4, {target4}, pos_only);
                auto const t1 = std::chrono::steady_clock::now();
                auto const ms = std::chrono::duration_cast<std::chrono::microseconds>(
                                    t1 - t0)
                                    .count() /
                                1000.0;
                std::printf("  success: %s | position error: %.4f mm | solve time: %.1f ms\n",
                            result.success ? "true" : "false",
                            result.position_error * 1000.0, ms);
                print_vec("  q [rad]", result.q);
                std::vector<double> q_deg;
                for (double const d : result.q) q_deg.push_back(d * 180.0 / M_PI);
                print_vec("  q [deg]", q_deg);
                if (result.success) {
                    print_pose("  FK(q)", arm7::tool0_pose(result.q));
                }
            }
        }
    }

    std::printf("\nDone.\n");
    return (maybe_gradient.has_value() && maybe_memetic.has_value()) ? 0 : 1;
}
