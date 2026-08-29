// IK tests for libpick_ik_core — MoveIt-free ports of the upstream
// pick_ik tests plus 7-DOF arm tests driven by the POC reference FK.
//
// Validation strategy (deliberate isolation):
//   known q  ->  hand-written FK  ->  target pose  ->  PickIK core  ->
//   q_solution  ->  FK(q_solution)  ->  compare against target.
//
// Position-only first, then orientation, then secondary objectives, then
// memetic concurrency.

#include <pick_ik/fk.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "arm7_fk.hpp"

#include <cmath>
#include <optional>
#include <vector>

// ===========================================================================
// RR 2-DOF planar arm (ported from upstream pick_ik tests/ik_tests.cpp; the
// MoveIt RobotModelBuilder model is replaced by a hand-written FK).
//
//   base --revolute(z)--> a --(2,0,0) revolute(z)--> b --(1,0,0) fixed--> ee
// ===========================================================================

namespace {

Eigen::Isometry3d rr_ee_pose(std::vector<double> const& q) {
    Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
    // base -> a: revolute z, origin = identity
    frame = frame * Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ());
    // a -> b: revolute z, origin = (2, 0, 0) in frame a
    frame = frame *
            (Eigen::Translation3d(2.0, 0.0, 0.0) *
             Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitZ()));
    // b -> ee: fixed, origin = (1, 0, 0) in frame b
    frame = frame * Eigen::Translation3d(1.0, 0.0, 0.0);
    return frame;
}

pick_ik::Robot rr_robot() {
    return pick_ik::Robot::make({{-M_PI, M_PI, true, 0.0}, {-M_PI, M_PI, true, 0.0}});
}

struct IkTestParams {
    double position_threshold = 0.0001;
    double orientation_threshold = 0.001;
    double cost_threshold = 0.0001;
    double position_scale = 1.0;
    double rotation_scale = 1.0;
};

auto solve_rr_gradient(std::vector<double> const& initial_guess,
                       Eigen::Isometry3d const& goal_frame,
                       IkTestParams const& params = IkTestParams())
    -> std::optional<std::vector<double>> {
    pick_ik::FkFn fk = [](std::vector<double> const& q) {
        return std::vector<Eigen::Isometry3d>{rr_ee_pose(q)};
    };

    auto robot = rr_robot();

    std::optional<double> position_threshold = std::nullopt;
    if (params.position_scale > 0.0) position_threshold = params.position_threshold;
    std::optional<double> orientation_threshold = std::nullopt;
    if (params.rotation_scale > 0.0) orientation_threshold = params.orientation_threshold;
    auto const frame_tests =
        pick_ik::make_frame_tests({goal_frame}, position_threshold, orientation_threshold);

    std::vector<pick_ik::Goal> const goals;
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, params.cost_threshold, fk);

    auto const pose_cost_functions =
        pick_ik::make_pose_cost_functions({goal_frame}, params.position_scale,
                                          params.rotation_scale);
    auto const cost_fn = pick_ik::make_cost_fn(pose_cost_functions, goals, fk);

    pick_ik::GradientIkParams gd_params;
    return pick_ik::ik_gradient(initial_guess, robot, cost_fn, solution_fn, gd_params,
                                /*approx_solution=*/false);
}

}  // namespace

TEST_CASE("RR FK reference") {
    SECTION("zero joint positions -> ee at (3, 0, 0)") {
        auto const pose = rr_ee_pose({0.0, 0.0});
        CHECK(pose.translation().x() == Catch::Approx(3.0).margin(1e-12));
        CHECK(pose.translation().y() == Catch::Approx(0.0).margin(1e-12));
        CHECK(pose.translation().z() == Catch::Approx(0.0).margin(1e-12));
    }

    SECTION("non-zero joint positions") {
        auto const pose = rr_ee_pose({M_PI_4, -M_PI_4});
        CHECK(pose.translation().x() == Catch::Approx(2.0 * std::cos(M_PI_4) + 1.0).margin(1e-12));
        CHECK(pose.translation().y() == Catch::Approx(2.0 * std::sin(M_PI_4)).margin(1e-12));
    }
}

TEST_CASE("RR model IK (gradient)") {
    SECTION("Zero joint angles with close initial guess") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond::Identity();
        std::vector<double> const expected_joint_angles = {0.0, 0.0};
        std::vector<double> const initial_guess = {0.1, -0.1};

        auto const maybe_solution = solve_rr_gradient(initial_guess, goal_frame);

        REQUIRE(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] ==
              Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] ==
              Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }

    SECTION("Zero joint angles with far initial guess") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond::Identity();
        std::vector<double> const expected_joint_angles = {0.0, 0.0};
        std::vector<double> const initial_guess = {M_PI_2, -M_PI_2};

        auto const maybe_solution = solve_rr_gradient(initial_guess, goal_frame);

        REQUIRE(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] ==
              Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] ==
              Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }

    SECTION("Nonzero joint angles with near initial guess") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(std::sin(M_PI_4), 3.0 * std::sin(M_PI_4), 0.0) *
            Eigen::AngleAxisd(0.75 * M_PI, Eigen::Vector3d::UnitZ());
        std::vector<double> const expected_joint_angles = {M_PI_4, M_PI_2};
        std::vector<double> const initial_guess = {M_PI_4 + 0.1, M_PI_2 - 0.1};

        auto const maybe_solution = solve_rr_gradient(initial_guess, goal_frame);

        REQUIRE(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] ==
              Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] ==
              Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }

    SECTION("Nonzero joint angles with far initial guess") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(std::sin(M_PI_4), 3.0 * std::sin(M_PI_4), 0.0) *
            Eigen::AngleAxisd(0.75 * M_PI, Eigen::Vector3d::UnitZ());
        std::vector<double> const expected_joint_angles = {M_PI_4, M_PI_2};
        std::vector<double> const initial_guess = {0.0, 0.0};

        auto const maybe_solution = solve_rr_gradient(initial_guess, goal_frame);

        REQUIRE(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] ==
              Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] ==
              Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }

    SECTION("Unreachable position") {
        auto const goal_frame = Eigen::Isometry3d::Identity();
        std::vector<double> const initial_guess = {0.0, 0.0};

        auto const maybe_solution = solve_rr_gradient(initial_guess, goal_frame);

        CHECK(!maybe_solution.has_value());
    }

    SECTION("Reachable position, but not orientation") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(std::sin(M_PI_4), 3.0 * std::sin(M_PI_4), 0.0) *
            Eigen::Quaterniond::Identity();
        std::vector<double> const initial_guess = {0.0, 0.0};

        auto const maybe_solution = solve_rr_gradient(initial_guess, goal_frame);

        CHECK(!maybe_solution.has_value());
    }

    SECTION("Reachable position, but not orientation -- zero rotation scale") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(std::sin(M_PI_4), 3.0 * std::sin(M_PI_4), 0.0) *
            Eigen::Quaterniond::Identity();
        std::vector<double> const expected_joint_angles = {M_PI_4, M_PI_2};
        std::vector<double> const initial_guess = {M_PI_4 + 0.1, M_PI_2 - 0.1};
        IkTestParams params;
        params.rotation_scale = 0.0;

        auto const maybe_solution = solve_rr_gradient(initial_guess, goal_frame, params);

        CHECK(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] ==
              Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] ==
              Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }
}

// ===========================================================================
// 7-DOF arm (POC URDF) with the POC reference FK
// ===========================================================================

namespace {

pick_ik::Robot arm7_robot() { return arm7::make_robot(); }

// A comfortably reachable, non-degenerate target configuration.
std::vector<double> const arm7_q_true = {0.5, -0.3, 0.2, -0.5, 0.1, 0.4, -0.2};

}  // namespace

TEST_CASE("Arm7 FK reference") {
    auto const arm = arm7::Arm7::make();

    SECTION("zero configuration -> tool0 at (0, 0, 0.675) with identity orientation") {
        // All joints at zero: arm stands vertically, tool0 pointing +z.
        // 0.180 + 0.215 + 0.215 + 0.065 = 0.675 (Design B desktop dimensions)
        auto const pose = arm.tool0_pose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        CHECK(pose.translation().x() == Catch::Approx(0.0).margin(1e-9));
        CHECK(pose.translation().y() == Catch::Approx(0.0).margin(1e-9));
        CHECK(pose.translation().z() == Catch::Approx(0.675).margin(1e-9));
        CHECK(pose.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
    }

    SECTION("fk callback returns exactly one frame (tool0)") {
        auto const fk = arm7::make_fk();
        auto const frames = fk({0.1, -0.2, 0.3, -0.1, 0.2, -0.3, 0.1});
        CHECK(frames.size() == 1);
        CHECK(frames[0].isApprox(arm.tool0_pose({0.1, -0.2, 0.3, -0.1, 0.2, -0.3, 0.1}), 1e-12));
    }
}

TEST_CASE("Arm7 IK (gradient) -- position only") {
    auto const fk = arm7::make_fk();
    auto const goal = fk(arm7_q_true)[0];

    std::vector<double> seed = arm7_q_true;
    std::vector<double> const offsets = {0.03, -0.03, 0.03, -0.03, 0.03, -0.03, 0.03};
    for (size_t i = 0; i < seed.size(); ++i) seed[i] += offsets[i];

    pick_ik::FkFn const fk_copy = fk;
    auto robot = arm7_robot();
    auto const frame_tests =
        pick_ik::make_frame_tests({goal}, /*position_threshold=*/1e-3,
                                  /*orientation_threshold=*/std::nullopt);
    std::vector<pick_ik::Goal> const goals;
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, 1e-3, fk_copy);
    auto const pose_costs =
        pick_ik::make_pose_cost_functions({goal}, /*position_scale=*/1.0,
                                          /*rotation_scale=*/0.0);
    auto const cost_fn = pick_ik::make_cost_fn(pose_costs, goals, fk_copy);

    pick_ik::GradientIkParams gd_params;
    gd_params.max_time = 0.2;
    gd_params.max_iterations = 200;

    auto const maybe = pick_ik::ik_gradient(seed, robot, cost_fn, solution_fn, gd_params,
                                             /*approx_solution=*/false);

    REQUIRE(maybe.has_value());
    auto const final_pose = fk(*maybe);
    CHECK((final_pose[0].translation() - goal.translation()).norm() < 1e-3);
    CHECK(robot.is_valid_configuration(*maybe));
}

TEST_CASE("Arm7 IK (gradient) -- full pose") {
    auto const fk = arm7::make_fk();
    auto const goal = fk(arm7_q_true)[0];

    std::vector<double> seed = arm7_q_true;
    std::vector<double> const offsets = {0.03, -0.03, 0.03, -0.03, 0.03, -0.03, 0.03};
    for (size_t i = 0; i < seed.size(); ++i) seed[i] += offsets[i];

    pick_ik::FkFn const fk_copy = fk;
    auto robot = arm7_robot();
    auto const frame_tests =
        pick_ik::make_frame_tests({goal}, /*position_threshold=*/1e-3,
                                  /*orientation_threshold=*/1e-3);
    std::vector<pick_ik::Goal> const goals;
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, 1e-3, fk_copy);
    auto const pose_costs =
        pick_ik::make_pose_cost_functions({goal}, /*position_scale=*/1.0,
                                          /*rotation_scale=*/0.5);
    auto const cost_fn = pick_ik::make_cost_fn(pose_costs, goals, fk_copy);

    pick_ik::GradientIkParams gd_params;
    gd_params.max_time = 0.2;
    gd_params.max_iterations = 200;

    auto const maybe = pick_ik::ik_gradient(seed, robot, cost_fn, solution_fn, gd_params,
                                             /*approx_solution=*/false);

    REQUIRE(maybe.has_value());
    auto const final_pose = fk(*maybe);
    CHECK((final_pose[0].translation() - goal.translation()).norm() < 1e-3);
    Eigen::Quaterniond const q_goal(goal.rotation());
    Eigen::Quaterniond const q_final(final_pose[0].rotation());
    CHECK(2.0 * std::acos(std::min(1.0, std::abs(q_goal.dot(q_final)))) < 1e-3);
}

TEST_CASE("Arm7 IK (memetic) -- position only, far seed, 2 threads") {
    auto const fk = arm7::make_fk();
    auto const goal = fk(arm7_q_true)[0];

    std::vector<double> const seed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    pick_ik::FkFn const fk_copy = fk;
    auto robot = arm7_robot();
    auto const frame_tests =
        pick_ik::make_frame_tests({goal}, /*position_threshold=*/1e-2,
                                  /*orientation_threshold=*/std::nullopt);
    std::vector<pick_ik::Goal> const goals;
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, 1e-2, fk_copy);
    auto const pose_costs =
        pick_ik::make_pose_cost_functions({goal}, /*position_scale=*/1.0,
                                          /*rotation_scale=*/0.0);
    auto const cost_fn = pick_ik::make_cost_fn(pose_costs, goals, fk_copy);

    pick_ik::MemeticIkParams params;
    params.num_threads = 2;
    params.max_time = 1.0;

    auto const maybe = pick_ik::ik_memetic(seed, robot, cost_fn, solution_fn, params,
                                           /*approx_solution=*/false,
                                           /*print_debug=*/false);

    REQUIRE(maybe.has_value());
    auto const final_pose = fk(*maybe);
    CHECK((final_pose[0].translation() - goal.translation()).norm() < 1e-2);
    CHECK(robot.is_valid_configuration(*maybe));
}

TEST_CASE("Arm7 IK (memetic) -- full pose, far seed, 4 threads (concurrency smoke)") {
    auto const fk = arm7::make_fk();
    auto const goal = fk(arm7_q_true)[0];

    std::vector<double> const seed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    pick_ik::FkFn const fk_copy = fk;
    auto robot = arm7_robot();
    auto const frame_tests =
        pick_ik::make_frame_tests({goal}, /*position_threshold=*/1e-2,
                                  /*orientation_threshold=*/5e-2);
    std::vector<pick_ik::Goal> const goals;
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, 5e-3, fk_copy);
    auto const pose_costs =
        pick_ik::make_pose_cost_functions({goal}, /*position_scale=*/1.0,
                                          /*rotation_scale=*/0.5);
    auto const cost_fn = pick_ik::make_cost_fn(pose_costs, goals, fk_copy);

    pick_ik::MemeticIkParams params;
    params.num_threads = 4;
    params.max_time = 1.5;

    auto const maybe = pick_ik::ik_memetic(seed, robot, cost_fn, solution_fn, params,
                                           /*approx_solution=*/false,
                                           /*print_debug=*/false);

    REQUIRE(maybe.has_value());
    auto const final_pose = fk(*maybe);
    CHECK((final_pose[0].translation() - goal.translation()).norm() < 1e-2);
    CHECK(robot.is_valid_configuration(*maybe));
}

TEST_CASE("Arm7 secondary objectives -- factory behavior") {
    auto robot = arm7_robot();
    auto const seed = std::vector<double>(7, 0.0);

    SECTION("minimal displacement is zero at the seed") {
        auto const cost_fn = pick_ik::make_minimal_displacement_cost_fn(robot, seed);
        CHECK(cost_fn(seed) == Catch::Approx(0.0).margin(1e-15));
        CHECK(cost_fn(arm7_q_true) > 0.0);
    }

    SECTION("joint centering is zero at the mid positions") {
        auto const cost_fn = pick_ik::make_center_joints_cost_fn(robot);
        CHECK(cost_fn(seed) == Catch::Approx(0.0).margin(1e-15));  // all limits symmetric
        CHECK(cost_fn(arm7_q_true) > 0.0);
    }

    SECTION("limit avoidance is zero at the mid and positive near a limit") {
        auto const cost_fn = pick_ik::make_avoid_joint_limits_cost_fn(robot);
        CHECK(cost_fn(seed) == Catch::Approx(0.0).margin(1e-15));
        std::vector<double> near_limit = seed;
        near_limit[1] = 2.08;  // J2 limit is 2.09
        CHECK(cost_fn(near_limit) > 0.0);
    }
}

TEST_CASE("Arm7 IK (memetic) -- with secondary objectives (smoke)") {
    // Mirrors the upstream "Panda model IK, with joint centering and limits
    // avoiding" test: secondary objectives must not prevent convergence.
    auto const fk = arm7::make_fk();
    auto const goal = fk(arm7_q_true)[0];

    std::vector<double> const seed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    pick_ik::FkFn const fk_copy = fk;
    auto robot = arm7_robot();
    auto const frame_tests =
        pick_ik::make_frame_tests({goal}, /*position_threshold=*/1e-2,
                                  /*orientation_threshold=*/5e-2);
    std::vector<pick_ik::Goal> goals;
    goals.push_back({pick_ik::make_center_joints_cost_fn(robot), 0.01});
    goals.push_back({pick_ik::make_avoid_joint_limits_cost_fn(robot), 0.01});
    goals.push_back({pick_ik::make_minimal_displacement_cost_fn(robot, seed), 0.01});
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, /*cost_threshold=*/0.01, fk_copy);
    auto const pose_costs =
        pick_ik::make_pose_cost_functions({goal}, /*position_scale=*/1.0,
                                          /*rotation_scale=*/0.5);
    auto const cost_fn = pick_ik::make_cost_fn(pose_costs, goals, fk_copy);

    pick_ik::MemeticIkParams params;
    params.max_time = 1.5;

    auto const maybe = pick_ik::ik_memetic(seed, robot, cost_fn, solution_fn, params,
                                           /*approx_solution=*/false,
                                           /*print_debug=*/false);

    REQUIRE(maybe.has_value());
    auto const final_pose = fk(*maybe);
    CHECK((final_pose[0].translation() - goal.translation()).norm() < 1e-2);
}
