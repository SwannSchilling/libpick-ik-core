#pragma once

// ============================================================================
// arm7 — 7-DOF desktop arm (Design B), shared C++ model
// ============================================================================
//
// Single source for the C++ side of the arm7 model: the joint table
// (Design B dimensions), forward kinematics, and the joint specs for
// `pick_ik::Robot::make`. Every C++ consumer (arm7_cross_check, the ctest
// ports, the `pick_ik_c` C ABI, future integrations) links against this
// header instead of carrying its own copy of the constants.
//
// Human/CAD source of truth: docs/arm7-kinematic-spec.md (Design B desktop
// dimensions, 2026-08 rescale; anchor poses in its section 5 pin every FK
// port). Design rationale: docs/desktop-arm-design-study.md.
//
// FK convention (URDF standard, identical across all ports):
//   base at identity;
//   for each joint (document order):
//       child frame = parent frame * T(origin.xyz) * R(origin.rpy) * R(axis, q)
//   R(origin.rpy) = Rz(yaw) * Ry(pitch) * Rx(roll)   (URDF fixed-axis order)
//   tool0 = link7 + 0.065 m along link7 z (fixed joint).
//
// Units: meters / radians.

#include <pick_ik/robot.hpp>
#include <pick_ik/solver.hpp>

#include <Eigen/Geometry>

#include <vector>

namespace arm7 {

constexpr double kPi = 3.14159265358979323846;

// POC URDF limit values, verbatim (note: "-3.14159265" is the URDF text
// value, not M_PI).
constexpr double kLimPi = 3.14159265;
constexpr double kLimShoulder = 2.09;
constexpr double kVelUpper = 2.17;
constexpr double kVelWrist = 2.61;

/// Design B linear dimensions [m] (base->J2, J2->J4, J4->J6, J6->tool0).
constexpr double kL1 = 0.180;
constexpr double kL2 = 0.215;
constexpr double kL3 = 0.215;
constexpr double kTool = 0.065;
/// Straight-chain length: 0.675 m.
constexpr double kChain = kL1 + kL2 + kL3 + kTool;

struct Joint {
    Eigen::Vector3d origin;      ///< Joint origin translation (m), in parent frame
    Eigen::Matrix3d rotation;    ///< Origin RPY rotation (fixed-axis Rz*Ry*Rx)
    Eigen::Vector3d axis;        ///< Unit joint axis in the joint frame
    double min = 0.0;            ///< Lower position limit [rad]
    double max = 0.0;            ///< Upper position limit [rad]
    double max_velocity = 0.0;   ///< Max joint velocity [rad/s]
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
                        Eigen::Vector3d const& axis, double min, double max, double max_velocity) {
    Joint j;
    j.origin = origin;
    j.rotation = rpy_matrix(roll, pitch, yaw);
    j.axis = axis.normalized();
    j.min = min;
    j.max = max;
    j.max_velocity = max_velocity;
    return j;
}

/// The 7 joints, document order (J1..J7).
/// J2/J4/J6 origins z = 0.18 / 0.215 / 0.215 m (Design B); all axes are the
/// joint z axis; alternating origins: J2, J4, J6 rotated -90 deg about x,
/// J3, J5, J7 +90.
inline std::vector<Joint> const& joints() {
    static auto const table = [] {
        std::vector<Joint> j;
        Eigen::Vector3d const z(0.0, 0.0, 1.0);
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.00), 0.0, 0.0, 0.0, z,
                               -kLimPi, kLimPi, kVelUpper));            // J1 base yaw
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, kL1), -kPi / 2.0, 0.0, 0.0, z,
                               -kLimShoulder, kLimShoulder, kVelUpper)); // J2 shoulder pitch
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.00), kPi / 2.0, 0.0, 0.0, z,
                               -kLimPi, kLimPi, kVelUpper));            // J3 shoulder roll
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, kL2), -kPi / 2.0, 0.0, 0.0, z,
                               -kLimShoulder, kLimShoulder, kVelUpper)); // J4 elbow pitch
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.00), kPi / 2.0, 0.0, 0.0, z,
                               -kLimPi, kLimPi, kVelWrist));           // J5 forearm roll
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, kL3), -kPi / 2.0, 0.0, 0.0, z,
                               -kLimShoulder, kLimShoulder, kVelWrist)); // J6 wrist pitch
        j.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.00), kPi / 2.0, 0.0, 0.0, z,
                               -kLimPi, kLimPi, kVelWrist));           // J7 tool roll
        return j;
    }();
    return table;
}

inline Eigen::Vector3d const& tool_offset() {
    static Eigen::Vector3d const t(0.0, 0.0, kTool);  // fixed joint link7 -> tool0
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
    return [](std::vector<double> const& q) { return link_frames(q); };
}

/// Local joint axes (all z, per the POC URDF).
inline std::vector<Eigen::Vector3d> make_local_axes() {
    std::vector<Eigen::Vector3d> axes;
    axes.reserve(joints().size());
    for (size_t i = 0; i < joints().size(); ++i) axes.push_back(Eigen::Vector3d::UnitZ());
    return axes;
}

/// Per-joint specs for `pick_ik::Robot::make` (limits + velocities from the
/// table; all joints bounded, all axes z — pass `make_local_axes()` as the
/// solver's `local_axes`).
inline std::vector<pick_ik::Robot::JointSpec> joint_specs() {
    std::vector<pick_ik::Robot::JointSpec> specs;
    specs.reserve(joints().size());
    for (Joint const& j : joints()) {
        specs.push_back({j.min, j.max, true, j.max_velocity});
    }
    return specs;
}

/// The arm7 `pick_ik::Robot` (velocity-weighted minimal-displacement factors).
inline pick_ik::Robot make_robot() {
    return pick_ik::Robot::make(joint_specs());
}

}  // namespace arm7
