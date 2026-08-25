#include <pick_ik/robot.hpp>

#include <rsl/random.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>

namespace {
constexpr double kUnboundedVariableHalfSpan = M_PI;
constexpr double kUnboundedJointSampleSpread = M_PI;
}  // namespace

namespace pick_ik {

auto Robot::Variable::generate_valid_value(double init_val /* = 0.0*/) const -> double {
    if (bounded) {
        return rsl::uniform_real(min, max);
    } else {
        return rsl::uniform_real(init_val - kUnboundedJointSampleSpread,
                                 init_val + kUnboundedJointSampleSpread);
    }
}

auto Robot::Variable::is_valid(double val) const -> bool {
    return (!bounded) || (val <= max && val >= min);
}

auto Robot::Variable::clamp_to_limits(double val) const -> double {
    if (bounded) {
        return std::clamp(val, min, max);
    } else {
        return std::clamp(val, val - half_span, val + half_span);
    }
}

auto Robot::make(std::vector<JointSpec> const& joints) -> Robot {
    auto robot = Robot{};

    auto const variable_count = joints.size();

    auto const minimal_displacement_divisor = [&joints]() {
        double sum = 0.0;
        for (auto const& joint : joints) {
            auto const max_velocity = joint.max_velocity;
            sum += max_velocity > 0.0 ? 1.0 / max_velocity : 0.0;
        }
        return sum;
    }();

    for (auto const& joint : joints) {
        auto var = Variable{};

        var.bounded = joint.bounded;
        var.min = joint.min;
        var.max = joint.max;
        // The MoveIt-based Robot::from computes 0.5 * (min + max) for all variables, which is
        // NaN for unbounded variables (min = -inf, max = +inf). That value is never read for
        // unbounded variables (the centering and limit-avoidance cost functions skip them),
        // so use 0.0 here to avoid the NaN.
        var.mid = joint.bounded ? 0.5 * (joint.min + joint.max) : 0.0;
        var.half_span = var.bounded ? (var.max - var.min) / 2.0 : kUnboundedVariableHalfSpan;

        auto const max_velocity = joint.max_velocity;
        var.max_velocity_rcp = max_velocity > 0.0 ? 1.0 / max_velocity : 0.0;

        var.minimal_displacement_factor = 1.0 / static_cast<double>(variable_count);

        robot.variables.push_back(var);
    }

    // Calculate minimal displacement factors
    if (minimal_displacement_divisor > 0.0) {
        for (auto& var : robot.variables) {
            var.minimal_displacement_factor = var.max_velocity_rcp / minimal_displacement_divisor;
        }
    }

    return robot;
}

auto Robot::set_random_valid_configuration(std::vector<double>& config) const -> void {
    auto const num_vars = variables.size();
    if (config.size() != num_vars) {
        config.resize(num_vars);
    }
    for (size_t idx = 0; idx < num_vars; ++idx) {
        config[idx] = variables[idx].generate_valid_value(config[idx]);
    }
}

auto Robot::is_valid_configuration(std::vector<double> const& config) const -> bool {
    auto const num_vars = variables.size();
    for (size_t idx = 0; idx < num_vars; ++idx) {
        if (!variables[idx].is_valid(config[idx])) {
            return false;
        }
    }
    return true;
}

}  // namespace pick_ik
