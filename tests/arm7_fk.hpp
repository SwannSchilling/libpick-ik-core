#pragma once

// Reference forward kinematics for the 7-DOF arm defined by the POC URDF
// (RobotArm_2026_08_25_10_03_56/sketch.js), ported 1:1 from the POC's
// computeURDFFK:
//
//   base_link at identity;
//   for each joint (document order):
//       child frame = parent frame * joint origin transform * R(axis, q)
//   tool0 = link7 + 0.065 along link7 z (fixed joint, Design B desktop
//   dimensions — see docs/desktop-arm-design-study.md; the POC 1.266 m
//   scaffolding was rescaled 2026-08 to 0.675 m).
//
// Real-world units (meters/radians) — the POC's display SCALE factor is NOT
// applied here.

#include <pick_ik/fk.hpp>
#include <pick_ik/solver.hpp>

#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <vector>

namespace arm7 {

constexpr double kPi = 3.14159265358979323846;

struct Joint {
    Eigen::Vector3d origin;    ///< Joint origin translation (m), in parent frame
    Eigen::Matrix3d rotation;  ///< Joint origin RPY rotation (fixed-axis: Rz(yaw)*Ry(pitch)*Rx(roll))
    Eigen::Vector3d axis;      ///< Joint axis in joint frame (unit vector)
    double min = 0.0;
    double max = 0.0;
    double max_velocity = 0.0;
    bool bounded = true;
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

inline Joint make_joint(Eigen::Vector3d const& origin,
                        double roll,
                        double pitch,
                        double yaw,
                        Eigen::Vector3d const& axis,
                        double min,
                        double max,
                        double max_velocity) {
    Joint j;
    j.origin = origin;
    j.rotation = rpy_matrix(roll, pitch, yaw);
    j.axis = axis.normalized();
    j.min = min;
    j.max = max;
    j.max_velocity = max_velocity;
    return j;
}

class Arm7 {
   public:
    static Arm7 make() {
        Arm7 arm;
        Eigen::Vector3d const z(0.0, 0.0, 1.0);

        // J1: base yaw
        arm.joints_.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, z,
                                         -kPi, kPi, 2.17));
        // J2: shoulder pitch
        arm.joints_.push_back(
            make_joint(Eigen::Vector3d(0.0, 0.0, 0.18), -kPi / 2.0, 0.0, 0.0, z, -2.09, 2.09, 2.17));
        // J3: shoulder roll
        arm.joints_.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.0), kPi / 2.0, 0.0, 0.0, z,
                                         -kPi, kPi, 2.17));
        // J4: elbow pitch
        arm.joints_.push_back(
            make_joint(Eigen::Vector3d(0.0, 0.0, 0.215), -kPi / 2.0, 0.0, 0.0, z, -2.09, 2.09, 2.17));
        // J5: forearm roll
        arm.joints_.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.0), kPi / 2.0, 0.0, 0.0, z,
                                         -kPi, kPi, 2.61));
        // J6: wrist pitch
        arm.joints_.push_back(
            make_joint(Eigen::Vector3d(0.0, 0.0, 0.215), -kPi / 2.0, 0.0, 0.0, z, -2.09, 2.09, 2.61));
        // J7: tool roll
        arm.joints_.push_back(make_joint(Eigen::Vector3d(0.0, 0.0, 0.0), kPi / 2.0, 0.0, 0.0, z,
                                         -kPi, kPi, 2.61));

        arm.tool_offset_ = Eigen::Vector3d(0.0, 0.0, 0.065);  // fixed joint link7 -> tool0
        return arm;
    }

    /**
     * @brief All joint child frames plus the tool0 frame.
     * @return frames[i] for 0 <= i < 7: frame of joint i's child link (pivot
     *         frame including origin and current joint rotation);
     *         frames[7]: tool0. All in the base frame.
     */
    auto link_frames(std::vector<double> const& q) const -> std::vector<Eigen::Isometry3d> {
        assert(q.size() == joints_.size());
        std::vector<Eigen::Isometry3d> frames;
        frames.reserve(joints_.size() + 1);
        Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
        for (size_t i = 0; i < joints_.size(); ++i) {
            Joint const& j = joints_[i];
            Eigen::Isometry3d const origin =
                Eigen::Translation3d(j.origin) * Eigen::Quaterniond(j.rotation);
            frame = frame * origin * Eigen::AngleAxisd(q[i], j.axis);
            frames.push_back(frame);
        }
        frames.push_back(frame * Eigen::Translation3d(tool_offset_));
        return frames;
    }

    /// @brief Pose of the tool0 frame in the base frame for joint positions q = [J1..J7].
    auto tool0_pose(std::vector<double> const& q) const -> Eigen::Isometry3d {
        return link_frames(q).back();
    }

    auto joint_count() const -> size_t { return joints_.size(); }

    auto joint(size_t i) const -> Joint const& { return joints_[i]; }

   private:
    std::vector<Joint> joints_;
    Eigen::Vector3d tool_offset_;
};

/// @brief pick_ik::FkFn callback returning the tool0 pose (single tip frame).
/// Pure function of q, no shared mutable state: safe under concurrent calls.
inline auto make_fk() -> pick_ik::FkFn {
    auto arm = Arm7::make();
    return [arm = std::move(arm)](std::vector<double> const& q)
        -> std::vector<Eigen::Isometry3d> {
        return std::vector<Eigen::Isometry3d>{arm.tool0_pose(q)};
    };
}

/// @brief pick_ik::LinkFkFn: all joint child frames + tool0 (8 frames).
inline auto make_link_fk() -> pick_ik::LinkFkFn {
    auto arm = Arm7::make();
    return [arm = std::move(arm)](std::vector<double> const& q)
        -> std::vector<Eigen::Isometry3d> {
        return arm.link_frames(q);
    };
}

/// @brief Local joint axes (all z, per the POC URDF).
inline auto make_local_axes() -> std::vector<Eigen::Vector3d> {
    std::vector<Eigen::Vector3d> axes;
    for (int i = 0; i < 7; ++i) {
        axes.push_back(Eigen::Vector3d::UnitZ());
    }
    return axes;
}

}  // namespace arm7
