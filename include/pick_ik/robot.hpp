#pragma once

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace pick_ik {

/**
 * @brief Kinematic description of the optimized variables.
 *
 * This struct is the pure (non-MoveIt) part of the upstream PickIK `Robot`
 * type: per-variable bounds, sampling, and clamping. Construction from a
 * MoveIt `RobotModel` remains in the ROS adapter (`robot_moveit.hpp`);
 * standalone users construct a `Robot` via `Robot::make` from plain joint
 * specifications.
 */
struct Robot {
    struct Variable {
        /// @brief Min, max, and middle position values of the variable.
        double min, max, mid;

        /// @brief Whether the variable's position is bounded.
        bool bounded;

        /// @brief The half-span (max - min) / 2.0 of the variable, or a default value if
        /// unbounded.
        double half_span;

        double max_velocity_rcp;
        double minimal_displacement_factor;

        /// @brief Generates a valid variable value given an optional initial value (for
        /// unbounded joints).
        auto generate_valid_value(double init_val = 0.0) const -> double;

        /// @brief Returns true if a value is valid given the variable bounds.
        auto is_valid(double val) const -> bool;

        /// @brief Clamps a configuration to joint limits.
        auto clamp_to_limits(double val) const -> double;
    };
    std::vector<Variable> variables;

    /// @brief Per-joint specification for standalone `Robot` construction.
    struct JointSpec {
        double min;      ///< Lower position limit.
        double max;      ///< Upper position limit.
        bool bounded = true;     ///< False for unbounded (e.g. floating) variables.
        double max_velocity = 0.0;  ///< Max joint velocity; <= 0 disables velocity weighting.
    };

    /**
     * @brief Create a `Robot` from plain joint specifications.
     * @details Reproduces, exactly, the per-variable initialization performed by the
     * MoveIt-based `Robot::from`, including the velocity-weighted
     * `minimal_displacement_factor`. Unbounded variables get `mid = 0.0` and
     * `half_span = pi` (the MoveIt path leaves an unused NaN mid there; `mid` is never
     * read for unbounded variables by the cost functions).
     */
    static auto make(std::vector<JointSpec> const& joints) -> Robot;

    /**
     * @brief Sets a variable vector to a random configuration.
     * @details Here, "valid" denotes that the joint values are within their specified
     * limits.
     */
    auto set_random_valid_configuration(std::vector<double>& config) const -> void;

    /** @brief Check if a configuration is valid. */
    auto is_valid_configuration(std::vector<double> const& config) const -> bool;
};

}  // namespace pick_ik
