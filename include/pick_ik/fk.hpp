#pragma once

#include <Eigen/Geometry>
#include <functional>
#include <vector>

namespace pick_ik {

/**
 * @brief Forward-kinematics callback.
 *
 * Maps a vector of active joint positions to the poses of the configured tip
 * frames, expressed in the base (world) frame. The returned vector contains
 * one pose per tip frame, in the same order in which the tip frames were
 * configured.
 *
 * Implementations must be re-entrant: the solvers may invoke this callback
 * concurrently from multiple threads (memetic solver: one worker thread per
 * species, plus concurrent gradient-descent threads per species).
 *
 * This type is a byte-for-byte port of `pick_ik/fk_moveit.hpp` from the
 * upstream PickIK repository; the MoveIt-based factory `make_fk_fn` is
 * intentionally not part of this interface.
 */
using FkFn = std::function<std::vector<Eigen::Isometry3d>(std::vector<double> const&)>;

}  // namespace pick_ik
