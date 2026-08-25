#include <pick_ik/robot.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace {
// The 7-DOF arm joint specs from the POC URDF (RobotArm_2026_08_25_10_03_56).
std::vector<pick_ik::Robot::JointSpec> arm7_specs() {
    return {
        {-M_PI, M_PI, true, 2.17},    // J1
        {-2.09, 2.09, true, 2.17},    // J2
        {-M_PI, M_PI, true, 2.17},    // J3
        {-2.09, 2.09, true, 2.17},    // J4
        {-M_PI, M_PI, true, 2.61},    // J5
        {-2.09, 2.09, true, 2.61},    // J6
        {-M_PI, M_PI, true, 2.61},    // J7
    };
}
}  // namespace

TEST_CASE("pick_ik::Robot::make -- 7-DOF arm") {
    auto const robot = pick_ik::Robot::make(arm7_specs());

    SECTION("seven bounded variables") {
        CHECK(robot.variables.size() == 7);
        for (auto const& variable : robot.variables) {
            CHECK(variable.bounded);
        }
    }

    SECTION("min/max/mid/half_span") {
        auto const& v = robot.variables[1];  // J2: limits [-2.09, 2.09]
        CHECK(v.min == Catch::Approx(-2.09));
        CHECK(v.max == Catch::Approx(2.09));
        CHECK(v.mid == Catch::Approx(0.0));
        CHECK(v.half_span == Catch::Approx(2.09));
    }

    SECTION("velocity reciprocals") {
        CHECK(robot.variables[0].max_velocity_rcp == Catch::Approx(1.0 / 2.17));
        CHECK(robot.variables[4].max_velocity_rcp == Catch::Approx(1.0 / 2.61));
    }

    SECTION("velocity-weighted minimal displacement factors") {
        double const inv_v[7] = {1.0 / 2.17,
                                 1.0 / 2.17,
                                 1.0 / 2.17,
                                 1.0 / 2.17,
                                 1.0 / 2.61,
                                 1.0 / 2.61,
                                 1.0 / 2.61};
        double sum = 0.0;
        for (auto x : inv_v) sum += x;

        for (size_t i = 0; i < 7; ++i) {
            CHECK(robot.variables[i].minimal_displacement_factor ==
                  Catch::Approx(inv_v[i] / sum));
        }

        double total = 0.0;
        for (auto const& variable : robot.variables) total += variable.minimal_displacement_factor;
        CHECK(total == Catch::Approx(1.0));
    }
}

TEST_CASE("pick_ik::Robot::make -- unweighted when no velocities given") {
    auto const robot =
        pick_ik::Robot::make({{-1.0, 1.0, true, 0.0}, {-2.0, 2.0, true, 0.0}, {-3.0, 3.0, true, 0.0}});

    for (auto const& v : robot.variables) {
        CHECK(v.max_velocity_rcp == 0.0);
        CHECK(v.minimal_displacement_factor == Catch::Approx(1.0 / 3.0));
    }
}

TEST_CASE("pick_ik::Robot::make -- unbounded variable") {
    auto const robot = pick_ik::Robot::make(
        {{-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
          false, 0.0}});
    auto const& v = robot.variables[0];

    CHECK_FALSE(v.bounded);
    CHECK(v.mid == 0.0);  // clean instead of NaN
    CHECK(v.half_span == Catch::Approx(M_PI));

    CHECK(v.is_valid(1.0e300));
    CHECK(v.is_valid(-1.0e300));

    // Unbounded clamp is a window around the value: the value itself stays.
    CHECK(v.clamp_to_limits(0.5) == Catch::Approx(0.5));

    auto const sample = v.generate_valid_value(0.0);
    CHECK(sample >= -M_PI - 1.0e-12);
    CHECK(sample <= M_PI + 1.0e-12);
}

TEST_CASE("pick_ik::Robot -- clamping and validity") {
    auto const robot = pick_ik::Robot::make(arm7_specs());

    SECTION("in-limit values pass") {
        CHECK(robot.is_valid_configuration({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
        CHECK(robot.is_valid_configuration({-3.14, -2.09, 3.14, 2.09, -3.14, 2.09, 3.14}));
    }

    SECTION("out-of-limit values fail") {
        CHECK_FALSE(robot.is_valid_configuration({0.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0}));
        CHECK_FALSE(robot.is_valid_configuration({0.0, 0.0, 0.0, -2.5, 0.0, 0.0, 0.0}));
    }

    SECTION("clamp_to_limits") {
        CHECK(robot.variables[1].clamp_to_limits(3.0) == Catch::Approx(2.09));
        CHECK(robot.variables[1].clamp_to_limits(-3.0) == Catch::Approx(-2.09));
        CHECK(robot.variables[0].clamp_to_limits(0.5) == Catch::Approx(0.5));
    }
}

TEST_CASE("pick_ik::Robot -- random valid configuration") {
    auto const robot = pick_ik::Robot::make(arm7_specs());

    SECTION("resizes the config vector") {
        std::vector<double> config(3, 0.0);
        robot.set_random_valid_configuration(config);
        CHECK(config.size() == 7);
    }

    SECTION("samples stay within limits") {
        std::vector<double> config(7, 0.0);
        for (int i = 0; i < 100; ++i) {
            robot.set_random_valid_configuration(config);
            CHECK(robot.is_valid_configuration(config));
        }
    }
}
