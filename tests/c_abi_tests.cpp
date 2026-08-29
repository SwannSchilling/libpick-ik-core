// ============================================================================
// pick_ik_c C ABI — validation suite (integration-roadmap §3.0c)
//
// This is the "protocol each host re-runs": it exercises the C ABI exactly as
// a native host would (opaque handles, row-major 4x4 poses, C FK callbacks)
// and pins the results against the spec:
//   - spec §5 anchor poses through the built-in arm7 C FK;
//   - the Design B cross-check targets A (deep fold 200/100/300 mm) and B
//     (moderate 300/150/300 mm) through all three solvers via pickik_solve;
//   - the out-of-workspace case (no solution, cleanly reported);
//   - joint-limit validity through pickik_robot_is_valid.
//
// The C++ ports stay pinned by the same anchors in arm7_fk.hpp; this file
// proves the C boundary itself is transparent (no drift at the ABI).
// ============================================================================

#include <pick_ik_c/pickik_c.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstring>
#include <vector>

#include "arm7/arm7.hpp"  // kChain for the zero-pose check

using Catch::Approx;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Row-major 4x4 helpers -----------------------------------------------------
void pose_identity(double p16[16]) {
    std::memset(p16, 0, 16 * sizeof(double));
    p16[0] = p16[5] = p16[10] = p16[15] = 1.0;
}
void pose_set_position(double p16[16], double x, double y, double z) {
    // Standard homogeneous row-major: translation is column 3.
    p16[3] = x;
    p16[7] = y;
    p16[11] = z;
}
void pose_set_z_axis_90deg(double p16[16]) {
    // Rz(+90deg): x->y, y->-x, z->z (row-major matrix)
    std::memset(p16, 0, 16 * sizeof(double));
    p16[0] = 0.0;
    p16[1] = -1.0;  // col 0 is x-axis (0,-1,0)
    p16[4] = 1.0;   // col 1 is y-axis (1,0,0)
    p16[10] = 1.0;
    p16[15] = 1.0;
}

// Spec §5 anchor fixtures (Design B). Configuration -> expected tool0 position.
struct Anchor {
    char const* name;
    std::vector<double> q;
    double pos[3];
};

std::vector<Anchor> const& anchors() {
    static auto const a = std::vector<Anchor>{
        {"zero pose (all joints 0)",
         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
         {0.0, 0.0, 0.675}},
        {"J1 +90 deg (base yaw)",
         {kPi / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
         {0.0, 0.0, 0.675}},
        {"J2 +90 deg (shoulder forward)",
         {0.0, kPi / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0},
         {0.495, 0.0, 0.180}},
        {"J2 -90 deg (shoulder back)",
         {0.0, -kPi / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0},
         {-0.495, 0.0, 0.180}},
        {"J4 +90 deg (elbow forward)",
         {0.0, 0.0, 0.0, kPi / 2.0, 0.0, 0.0, 0.0},
         {0.280, 0.0, 0.395}},
        {"J6 +90 deg (wrist forward)",
         {0.0, 0.0, 0.0, 0.0, 0.0, kPi / 2.0, 0.0},
         {0.065, 0.0, 0.610}},
    };
    return a;
}

// Run the built-in arm7 C FK for q; returns tool0 (frame 7) position.
std::array<double, 3> arm7_tool0_pos(std::vector<double> const& q) {
    double frames[(PICKIK_ARM7_JOINTS + 1) * 16];
    int const rc = pickik_arm7_link_fk(nullptr, PICKIK_ARM7_JOINTS, q.data(), frames);
    REQUIRE(rc == PICKIK_OK);
    double const* t0 = frames + 7 * 16;
    return {t0[3], t0[7], t0[11]};  // column 3 of the row-major 4x4
}

// Solve helper: gradient solver, position-only goal, all-zero seed.
pickik_result solve_gradient(double x, double y, double z) {
    pickik_robot* robot = nullptr;
    REQUIRE(pickik_arm7_robot_create(&robot) == PICKIK_OK);
    pickik_solver* solver = nullptr;
    REQUIRE(pickik_gradient_create(1e-4, 1e-12, 2.0, 2000, 1, &solver) == PICKIK_OK);

    pickik_options options;
    pickik_options_default(&options);
    options.orientation_threshold = -1.0;  // position-only

    double target[16];
    pose_identity(target);
    pose_set_position(target, x, y, z);
    double const seed[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double q_out[7];
    pickik_result result{};
    int const rc = pickik_solve(solver, robot, pickik_arm7_link_fk, nullptr, seed, target,
                                &options, q_out, &result);
    pickik_solver_free(solver);
    pickik_robot_free(robot);
    REQUIRE(rc == PICKIK_OK);
    return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// §3.0c protocol 1 — FK anchors through the C ABI
// ---------------------------------------------------------------------------
TEST_CASE("pick_ik_c: arm7 FK reproduces spec section-5 anchor poses") {
    SECTION("every anchor") {
        for (Anchor const& a : anchors()) {
            INFO(a.name);
            auto const p = arm7_tool0_pos(a.q);
            REQUIRE(p[0] == Approx(a.pos[0]).margin(1e-9));
            REQUIRE(p[1] == Approx(a.pos[1]).margin(1e-9));
            REQUIRE(p[2] == Approx(a.pos[2]).margin(1e-9));
        }
    }
    SECTION("zero-pose chain length") {
        auto const p = arm7_tool0_pos({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        REQUIRE(std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]) ==
                Approx(arm7::kChain).margin(1e-12));
    }
    SECTION("bad arguments are rejected") {
        double frames[(PICKIK_ARM7_JOINTS + 1) * 16];
        double const q[7] = {0, 0, 0, 0, 0, 0, 0};
        REQUIRE(pickik_arm7_link_fk(nullptr, 6, q, frames) == PICKIK_E_BADARG);
        REQUIRE(pickik_arm7_link_fk(nullptr, 7, nullptr, frames) == PICKIK_E_BADARG);
        REQUIRE(pickik_arm7_link_fk(nullptr, 7, q, nullptr) == PICKIK_E_BADARG);
    }
}

// ---------------------------------------------------------------------------
// §3.0c protocol 2 — the Design B cross-check targets through pickik_solve
// ---------------------------------------------------------------------------
TEST_CASE("pick_ik_c: Design B cross-check targets via C ABI") {
    SECTION("target B (moderate 300/150/300 mm) — gradient") {
        auto const r = solve_gradient(0.300, 0.150, 0.300);
        REQUIRE(r.success == 1);
        REQUIRE(r.position_error < 1e-3);
        REQUIRE(r.orientation_error < 0.0);  // -1: not evaluated (position-only)
        REQUIRE(r.time_ms >= 0.0);
    }

    SECTION("target A (deep fold 200/100/300 mm) — memetic") {
        pickik_robot* robot = nullptr;
        REQUIRE(pickik_arm7_robot_create(&robot) == PICKIK_OK);
        pickik_solver* solver = nullptr;
        REQUIRE(pickik_memetic_create(4, 16, 1e-5, 100, 4.0, 1, 1, 1, &solver) == PICKIK_OK);
        pickik_options options;
        pickik_options_default(&options);
        options.orientation_threshold = -1.0;
        double target[16];
        pose_identity(target);
        pose_set_position(target, 0.200, 0.100, 0.300);
        double const seed[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double q_out[7];
        pickik_result result{};
        REQUIRE(pickik_solve(solver, robot, pickik_arm7_link_fk, nullptr, seed, target,
                             &options, q_out, &result) == PICKIK_OK);
        pickik_solver_free(solver);
        pickik_robot_free(robot);
        REQUIRE(result.success == 1);
        REQUIRE(result.position_error < 1e-3);
    }

    SECTION("both targets — CCD") {
        double const targets[2][3] = {{0.200, 0.100, 0.300}, {0.300, 0.150, 0.300}};
        for (int k = 0; k < 2; ++k) {
            double const t[3] = {targets[k][0], targets[k][1], targets[k][2]};
            pickik_robot* robot = nullptr;
            REQUIRE(pickik_arm7_robot_create(&robot) == PICKIK_OK);
            pickik_solver* solver = nullptr;
            REQUIRE(pickik_ccd_create(600, 0.1, 1e-8, &solver) == PICKIK_OK);
            pickik_options options;
            pickik_options_default(&options);
            options.orientation_threshold = -1.0;
            double target[16];
            pose_identity(target);
            pose_set_position(target, t[0], t[1], t[2]);
            double const seed[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double q_out[7];
            pickik_result result{};
            REQUIRE(pickik_solve(solver, robot, pickik_arm7_link_fk, nullptr, seed, target,
                                 &options, q_out, &result) == PICKIK_OK);
            pickik_solver_free(solver);
            pickik_robot_free(robot);
            INFO("CCD target " << t[0] << ", " << t[1] << ", " << t[2]);
            REQUIRE(result.success == 1);
            REQUIRE(result.position_error < 1e-3);
        }
    }
}

// ---------------------------------------------------------------------------
// §3.0c protocol 3 — out-of-workspace targets fail cleanly
// ---------------------------------------------------------------------------
TEST_CASE("pick_ik_c: out-of-workspace target reports no solution") {
    // (50, 0, 180) mm: inside the arm's inner boundary (~151 mm horizontal
    // minimum at this height family) — unreachable (spec §6).
    auto const r = solve_gradient(0.050, 0.0, 0.180);
    REQUIRE(r.success == 0);
    REQUIRE(r.position_error > 0.1);  // far from the target, not "close enough"
}

// ---------------------------------------------------------------------------
// §3.0c protocol 4 — robot handle: limit validity
// ---------------------------------------------------------------------------
TEST_CASE("pick_ik_c: robot handle validates configurations") {
    pickik_robot* robot = nullptr;
    REQUIRE(pickik_arm7_robot_create(&robot) == PICKIK_OK);
    int valid = -1;
    double const inside[7] = {0.0, 1.0, 0.0, -1.0, 0.5, 0.0, -0.5};
    REQUIRE(pickik_robot_is_valid(robot, inside, &valid) == PICKIK_OK);
    REQUIRE(valid == 1);
    double const outside[7] = {0.0, 3.0, 0.0, -3.0, 0.5, 2.5, -0.5};  // J2/J4/J6 past ±2.09
    REQUIRE(pickik_robot_is_valid(robot, outside, &valid) == PICKIK_OK);
    REQUIRE(valid == 0);
    pickik_robot_free(robot);
}

// ---------------------------------------------------------------------------
// §3.0c protocol 5 — options plumbing (secondary objectives accepted)
// ---------------------------------------------------------------------------
TEST_CASE("pick_ik_c: secondary options are plumbed to the solver") {
    // Deterministic plumbing check: pinning all joints to a configuration that
    // already solves the goal must leave the solution where it started (the
    // pose cost and the joint-target cost agree at that point).
    pickik_robot* robot = nullptr;
    REQUIRE(pickik_arm7_robot_create(&robot) == PICKIK_OK);
    pickik_solver* solver = nullptr;
    REQUIRE(pickik_gradient_create(1e-4, 1e-12, 2.0, 2000, 1, &solver) == PICKIK_OK);

    double target[16];
    pose_identity(target);
    pose_set_position(target, 0.300, 0.150, 0.300);

    // 1) base solve, all-zero seed.
    pickik_options base_options;
    pickik_options_default(&base_options);
    base_options.orientation_threshold = -1.0;
    double const zero[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double q1[7];
    pickik_result base{};
    REQUIRE(pickik_solve(solver, robot, pickik_arm7_link_fk, nullptr, zero, target,
                         &base_options, q1, &base) == PICKIK_OK);
    REQUIRE(base.success == 1);

    // 2) same goal, seeded at q1, with q1 as the joint targets (weight 0.2) and
    // a minimal-displacement anchor (0.01): the pinned configuration is the
    // joint of the two cost minima.
    pickik_options options;
    pickik_options_default(&options);
    options.orientation_threshold = -1.0;
    options.minimal_displacement_weight = 0.01;
    options.joint_target_values = q1;
    int const has[7] = {1, 1, 1, 1, 1, 1, 1};
    options.joint_target_has = has;
    options.joint_target_weight = 0.2;
    double q2[7];
    pickik_result result{};
    REQUIRE(pickik_solve(solver, robot, pickik_arm7_link_fk, nullptr, q1, target, &options, q2,
                         &result) == PICKIK_OK);
    pickik_solver_free(solver);
    pickik_robot_free(robot);

    REQUIRE(result.success == 1);
    REQUIRE(result.position_error < 1e-3);
    double drift = 0.0;
    for (int i = 0; i < 7; ++i) drift += std::abs(q2[i] - q1[i]);
    REQUIRE(drift < 0.05);  // the joint targets kept the solution in place
}

// ---------------------------------------------------------------------------
// §3.0c protocol 6 — host-supplied C FK callback (generic path, not just the
// built-in arm7 one)
// ---------------------------------------------------------------------------
namespace {
// A host "model" that is just the arm7 FK wrapped through the C ABI itself —
// proves the callback adaptation (row-major in, Eigen out) is lossless.
int host_fk(void* /*user*/, int n_joints, double const* q, double* frames) {
    return pickik_arm7_link_fk(nullptr, n_joints, q, frames);
}
}  // namespace

TEST_CASE("pick_ik_c: host-provided FK callback path") {
    pickik_robot* robot = nullptr;
    REQUIRE(pickik_arm7_robot_create(&robot) == PICKIK_OK);
    pickik_solver* solver = nullptr;
    REQUIRE(pickik_gradient_create(1e-4, 1e-12, 2.0, 2000, 1, &solver) == PICKIK_OK);
    pickik_options options;
    pickik_options_default(&options);
    options.orientation_threshold = -1.0;
    double target[16];
    pose_identity(target);
    pose_set_position(target, 0.300, 0.150, 0.300);
    double const seed[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double q_out[7];
    pickik_result result{};
    REQUIRE(pickik_solve(solver, robot, host_fk, nullptr, seed, target, &options, q_out,
                         &result) == PICKIK_OK);
    pickik_solver_free(solver);
    pickik_robot_free(robot);
    REQUIRE(result.success == 1);
    REQUIRE(result.position_error < 1e-3);
}

// ---------------------------------------------------------------------------
// Bad-argument robustness at the boundary
// ---------------------------------------------------------------------------
TEST_CASE("pick_ik_c: bad arguments rejected at the boundary") {
    pickik_robot* robot = nullptr;
    pickik_solver* solver = nullptr;
    REQUIRE(pickik_arm7_robot_create(&robot) == PICKIK_OK);
    REQUIRE(pickik_gradient_create(1e-4, 1e-12, 2.0, 2000, 1, &solver) == PICKIK_OK);
    double const seed[7] = {0, 0, 0, 0, 0, 0, 0};
    double const target[16] = {1, 0, 0, 0.3, 0, 1, 0, 0.15, 0, 0, 1, 0.3, 0, 0, 0, 1};
    double q_out[7];
    pickik_result result{};
    REQUIRE(pickik_solve(nullptr, robot, pickik_arm7_link_fk, nullptr, seed, target,
                         nullptr, q_out, &result) == PICKIK_E_BADARG);
    REQUIRE(pickik_solve(solver, nullptr, pickik_arm7_link_fk, nullptr, seed, target,
                         nullptr, q_out, &result) == PICKIK_E_BADARG);
    pickik_options options;
    pickik_options_default(&options);
    options.orientation_threshold = -1.0;
    REQUIRE(pickik_solve(solver, robot, nullptr, nullptr, seed, target, &options, q_out,
                         &result) == PICKIK_E_BADARG);
    pickik_solver_free(solver);
    pickik_robot_free(robot);
    // Null handle frees are safe no-ops.
    REQUIRE(pickik_solver_free(nullptr) == PICKIK_OK);
    REQUIRE(pickik_robot_free(nullptr) == PICKIK_OK);
}
