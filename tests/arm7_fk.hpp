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

#include "arm7/arm7.hpp"  // the shared model (Design B constants + FK)

namespace arm7 {

// (The Joint struct, rpy_matrix / make_joint helpers, make_fk / make_link_fk /
//  make_local_axes all live in the shared model header arm7/arm7.hpp — the
//  single source for the C++ side. The Arm7 class below keeps the test-facing
//  facade: make() / link_frames / tool0_pose / joint_count / joint(i).)

class Arm7 {
   public:
    static Arm7 make() {
        // Built from the shared model header (examples/arm7/arm7.hpp) so the
        // ctest ports and every other C++ consumer share one constant set.
        Arm7 arm;
        for (arm7::Joint const& j : arm7::joints()) {
            Joint t;
            t.origin = j.origin;
            t.rotation = j.rotation;
            t.axis = j.axis;
            t.min = j.min;
            t.max = j.max;
            t.max_velocity = j.max_velocity;
            arm.joints_.push_back(std::move(t));
        }
        arm.tool_offset_ = arm7::tool_offset();  // fixed joint link7 -> tool0
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

// (make_fk / make_link_fk / make_local_axes are provided by the shared model
//  header arm7/arm7.hpp.)

}  // namespace arm7
