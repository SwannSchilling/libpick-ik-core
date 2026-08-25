#pragma once

#include <pick_ik/fk.hpp>
#include <pick_ik/robot.hpp>

#include <Eigen/Geometry>
#include <functional>
#include <numeric>  // std::accumulate (used by make_cost_fn; also required by ik_gradient.cpp)
#include <optional>
#include <string>
#include <vector>

namespace pick_ik {

// Frame equality tests
using FrameTestFn = std::function<bool(Eigen::Isometry3d const& tip_frame)>;
auto make_frame_tests(std::vector<Eigen::Isometry3d> goal_frames,
                      std::optional<double> position_threshold = std::nullopt,
                      std::optional<double> orientation_threshold = std::nullopt)
    -> std::vector<FrameTestFn>;

// Pose cost functions
using PoseCostFn = std::function<double(std::vector<Eigen::Isometry3d> const& tip_frames)>;
auto make_pose_cost_fn(Eigen::Isometry3d goal,
                       size_t goal_link_index,
                       double position_scale,
                       double rotation_scale) -> PoseCostFn;

auto make_pose_cost_functions(std::vector<Eigen::Isometry3d> goal_frames,
                              double position_scale,
                              double rotation_scale) -> std::vector<PoseCostFn>;

// Goal Function type
using CostFn = std::function<double(std::vector<double> const& active_positions)>;

struct Goal {
    CostFn eval;
    double weight;
};

auto make_center_joints_cost_fn(Robot robot) -> CostFn;

auto make_avoid_joint_limits_cost_fn(Robot robot) -> CostFn;

auto make_minimal_displacement_cost_fn(Robot robot, std::vector<double> initial_guess) -> CostFn;

// Create a solution test function from frame tests and goals
using SolutionTestFn = std::function<bool(std::vector<double> const& active_positions)>;

auto make_is_solution_test_fn(std::vector<FrameTestFn> frame_tests,
                              std::vector<Goal> goals,
                              double cost_threshold,
                              FkFn fk) -> SolutionTestFn;

auto make_cost_fn(std::vector<PoseCostFn> pose_cost_functions, std::vector<Goal> goals, FkFn fk)
    -> CostFn;

}  // namespace pick_ik
