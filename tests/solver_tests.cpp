// Tests for the generic IkSolver contract (pick_ik/solver.hpp) and its three
// implementations (pick_ik/solvers.hpp):
//   - CcdSolver:            faithful C++ port of the p5.js POC's solveCCD
//   - PickIkGradientSolver: wrapper around the upstream ik_gradient
//   - PickIkMemeticSolver:  wrapper around the upstream ik_memetic
//
// Driven by the POC 7-DOF reference FK (arm7_fk.hpp).

#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/solver.hpp>
#include <pick_ik/solvers.hpp>

#include <catch2/catch_test_macros.hpp>

#include "arm7_fk.hpp"

#include <optional>
#include <utility>
#include <vector>

namespace {

pick_ik::Robot const& arm7_robot() {
    static pick_ik::Robot const r = pick_ik::Robot::make({
        {-3.14159265, 3.14159265, true, 2.17},  // J1
        {-2.09, 2.09, true, 2.17},              // J2
        {-3.14159265, 3.14159265, true, 2.17},  // J3
        {-2.09, 2.09, true, 2.17},              // J4
        {-3.14159265, 3.14159265, true, 2.61},  // J5
        {-2.09, 2.09, true, 2.61},              // J6
        {-3.14159265, 3.14159265, true, 2.61},  // J7
    });
    return r;
}

pick_ik::SolveOptions pos_only() {
    pick_ik::SolveOptions o;
    o.position_threshold = 1e-3;
    o.orientation_threshold = std::nullopt;  // position-only, like the POC CCD
    o.cost_threshold = 1e-3;
    o.position_scale = 1.0;
    o.rotation_scale = 0.0;
    return o;
}

std::vector<double> const& quantized_zero_seed() {
    // The p5 POC's "all zero" slider state: p5 sliders snap to 0.01 rad
    // steps anchored at each joint's lower limit, so filling 0 yields
    // -0.00159265 on the pi-limited joints and 0.0 on the 2.09-limited ones.
    static std::vector<double> const s = {-0.00159265, 0.0, -0.00159265, 0.0, -0.00159265, 0.0,
                                          -0.00159265};
    return s;
}

Eigen::Isometry3d target_a() {
    // "Deep fold" target: every solution pins J4 or J6 at the 2.09 rad limit.
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(0.30, 0.20, 0.45);
    return t;
}

Eigen::Isometry3d target_b() {
    // "Moderate" target: reachable with no joint at its limit.
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(0.45, 0.25, 0.45);
    return t;
}

}  // namespace

TEST_CASE("IkSolver contract: solver names", "[solver]") {
    pick_ik::CcdSolver ccd;
    pick_ik::PickIkGradientSolver gd;
    pick_ik::PickIkMemeticSolver me;
    REQUIRE(ccd.name() == "ccd");
    REQUIRE(gd.name() == "gradient");
    REQUIRE(me.name() == "memetic");
}

TEST_CASE("make_tip_fk returns the last link frame", "[solver]") {
    auto link_fk = arm7::make_link_fk();
    auto tip_fk = pick_ik::make_tip_fk(link_fk);
    std::vector<double> const q = {0.3, -0.2, 0.5, -0.4, 0.1, 0.6, -0.3};
    auto const tips = tip_fk(q);
    auto const links = link_fk(q);
    REQUIRE(tips.size() == 1);
    REQUIRE(links.size() == 8);
    REQUIRE(tips[0].translation().isApprox(links.back().translation()));
}

TEST_CASE("CCD reaches a nearby target", "[solver][ccd]") {
    auto arm = arm7::Arm7::make();
    auto link_fk = arm7::make_link_fk();
    auto axes = arm7::make_local_axes();

    std::vector<double> const q0 = {0.20, -0.40, 0.10, -0.80, 0.10, 0.30, -0.20};
    std::vector<double> const q1 = {0.35, -0.50, 0.15, -0.90, 0.15, 0.40, -0.30};
    Eigen::Isometry3d const target = arm.tool0_pose(q1);

    pick_ik::CcdSolver ccd;  // default: 300 passes, damping 0.1
    auto const result = ccd.solve(arm7_robot(), link_fk, axes, q0, {target}, pos_only());

    REQUIRE(result.success);
    REQUIRE(result.position_error < 1e-3);
    REQUIRE(result.orientation_error == -1.0);  // position-only solver
    REQUIRE(arm7_robot().is_valid_configuration(result.q));
}

TEST_CASE("CCD solves both POC cross-check targets", "[solver][ccd]") {
    // The POC's browser CCD stalls ~15-35 mm short of these targets, but that
    // stall is an artifact of the p5 slider's 0.01 rad quantization (every
    // joint update is snapped to a grid, which pins the search in local
    // minima). The C++ port operates on continuous joint values and reaches
    // both targets sub-millimeter from the POC's quantized-zero seed —
    // including the deep-fold target A, whose solutions pin J6 at the
    // +2.09 rad limit exactly as the PickIK solvers found in the live check.
    auto link_fk = arm7::make_link_fk();
    auto axes = arm7::make_local_axes();
    auto const seed = quantized_zero_seed();
    auto const robot = arm7_robot();

    pick_ik::CcdSolver ccd(600);  // ~2 s of POC runtime
    for (auto const& [label, target] :
         std::vector<std::pair<char const*, Eigen::Isometry3d>>{
             {"A", target_a()}, {"B", target_b()}}) {
        auto const result = ccd.solve(robot, link_fk, axes, seed, {target}, pos_only());
        REQUIRE(result.position_error < 1e-3);
        REQUIRE(result.success);
        REQUIRE(robot.is_valid_configuration(result.q));
    }
}

TEST_CASE("Gradient (moderate target) and memetic (deep fold) reach what a "
          "single local search may not", "[solver]") {
    // Gradient: target B (moderate) — reliably inside its wall-clock budget.
    // (Target A's limit-pinned solutions sit on a near-flat cost face and are
    // only marginally inside the default 0.5 s budget: load-dependent.)
    // Memetic: target A (deep fold) — the population reliably finds the
    // limit-pinned solution early in the 2 s budget (observed ~0.4 s).
    auto link_fk = arm7::make_link_fk();
    auto axes = arm7::make_local_axes();
    auto const seed = quantized_zero_seed();
    auto const robot = arm7_robot();

    pick_ik::GradientIkParams gd_params;
    gd_params.max_time = 2.0;
    gd_params.max_iterations = 2000;
    pick_ik::PickIkGradientSolver gd(gd_params);
    auto const gd_result = gd.solve(robot, link_fk, axes, seed, {target_b()}, pos_only());

    pick_ik::MemeticIkParams me_params;
    me_params.num_threads = 4;
    me_params.max_time = 2.0;
    pick_ik::PickIkMemeticSolver me(me_params);
    auto const me_result = me.solve(robot, link_fk, axes, seed, {target_a()}, pos_only());

    REQUIRE(gd_result.success);
    REQUIRE(gd_result.position_error < 1e-3);
    REQUIRE(robot.is_valid_configuration(gd_result.q));

    REQUIRE(me_result.success);
    REQUIRE(me_result.position_error < 1e-3);
    REQUIRE(robot.is_valid_configuration(me_result.q));
}

TEST_CASE("Gradient wrapper matches the upstream free function", "[solver][gradient]") {
    auto link_fk = arm7::make_link_fk();
    auto axes = arm7::make_local_axes();
    auto const target = target_b();
    auto const seed = quantized_zero_seed();
    auto const options = pos_only();
    auto const robot = arm7_robot();

    // Direct upstream call (same plumbing as the example program).
    auto const tip_fk = pick_ik::make_tip_fk(link_fk);
    auto const frame_tests =
        pick_ik::make_frame_tests({target}, options.position_threshold,
                                  options.orientation_threshold);
    std::vector<pick_ik::Goal> const goals;
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, options.cost_threshold, tip_fk);
    auto const pose_costs =
        pick_ik::make_pose_cost_functions({target}, options.position_scale,
                                          options.rotation_scale);
    auto const cost_fn = pick_ik::make_cost_fn(pose_costs, goals, tip_fk);

    pick_ik::GradientIkParams params;
    params.max_time = 2.0;
    params.max_iterations = 2000;
    auto const direct =
        pick_ik::ik_gradient(seed, robot, cost_fn, solution_fn, params, false);

    pick_ik::PickIkGradientSolver gd(params);
    auto const via_contract = gd.solve(robot, link_fk, axes, seed, {target}, options);

    REQUIRE(direct.has_value());
    REQUIRE(via_contract.success);
    REQUIRE(via_contract.q == direct.value());  // deterministic: identical path
}
