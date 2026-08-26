# libpick_ik_core

Standalone, **ROS/MoveIt-free** core of the [PickIK](https://github.com/PickNikRobotics/pick_ik)
inverse-kinematics solver (reimplementation of
[bio_ik](https://github.com/TAMS-Group/bio_ik): local gradient-descent + global
memetic/evolutionary solver).

This library has **zero** dependencies on ROS 2, MoveIt 2, rclcpp, pluginlib,
geometry_msgs, moveit_core, tf2, or generate_parameter_library. Its only
dependencies are Eigen (PUBLIC), fmt (PRIVATE, debug-print path), and the
vendored RSL `random`/`queue` sources.

## Layout

```
include/pick_ik/
    fk.hpp           FkFn: std::function<vector<Isometry3d>(vector<double> const&)>
    robot.hpp        Robot / Variable + pure Robot::make(std::vector<JointSpec>)
    goal.hpp         frame tests, pose costs, secondary objectives, cost/solution assembly
    ik_gradient.hpp  local solver entry point: ik_gradient(...)
    ik_memetic.hpp   global solver entry point: ik_memetic(...)
    solver.hpp       generic IkSolver contract: LinkFkFn, SolveOptions, IkResult
    solvers.hpp      CcdSolver, PickIkGradientSolver, PickIkMemeticSolver
src/
    robot.cpp        pure parts of upstream robot.cpp + Robot::make
    goal.cpp         pure parts of upstream goal.cpp (make_ik_cost_fn stays in the ROS adapter)
    ik_gradient.cpp  byte-identical to upstream
    ik_memetic.cpp   byte-identical to upstream
    solver.cpp       the three IkSolver implementations (wrapper + CCD port)
extern/rsl/          vendored byte-identical RSL random/queue sources (see LICENSE note)
tests/               Catch2 tests, no MoveIt (hand-written FK models)
```

## Example: `arm7_cross_check`

`examples/arm7_cross_check` is a standalone, dependency-free test/example
program (no ROS, no server, no DLL) that drives the solver with a 7-DOF
`FkFn` ported 1:1 from a p5.js POC sketch's URDF forward kinematics
(fixed-axis RPY = Rz(yaw)·Ry(pitch)·Rx(roll), `SCALE` display factor divided
out to meters). It runs three parts:

1. **FK cross-check** — tool0 pose for five pinned joint configurations
   (print-and-compare against the sketch's `computeURDFFK(q).tool0 / SCALE`;
   the two agree to machine precision).
2. **PickIK self-test** — target = FK of a known configuration, seed =
   all zeros; runs `ik_gradient` and `ik_memetic` and verifies
   FK(solution) reaches the target.
3. **IK against external targets** — the same positions the p5.js sketch's
   own CCD solver chases (target sliders in mm), same quantized "all zero"
   seed, position-only goal (the POC's CCD is position-only):
   a "deep fold" target where every solution pins J4/J6 at the 2.09 rad
   limit, and a "moderate" target.
4. **Unified solver API demo** — the same two targets and seed through the
   `IkSolver` contract: CCD, gradient, and memetic side by side, with solve
   times. Notably, the de-quantized C++ CCD reaches both targets sub-millimeter
   (~2 ms) where the POC's browser CCD stalled ~15–35 mm short: that stall was
   an artifact of the p5 sliders' 0.01 rad quantization, not of the CCD math.

```sh
cmake --build build --config RelWithDebInfo --target arm7_cross_check
build/examples/arm7_cross_check/RelWithDebInfo/arm7_cross_check.exe
```

## Provenance

- `ik_gradient.cpp`, `ik_memetic.cpp` and the test `goal_tests.cpp` are
  byte-identical copies of the corresponding upstream files
  (PickNikRobotics/pick_ik, BSD-3-Clause — see `LICENSE`).
- `robot.cpp` / `goal.cpp` are the pure parts of the upstream files: the
  MoveIt-bound factories (`Robot::from`, `get_link_indices`,
  `get_active_variable_indices`, `get_variables`, `transform_poses_to_frames`,
  `make_ik_cost_fn`) are **not** included; they remain in the ROS adapter
  package. The new `Robot::make` reproduces `Robot::from`'s per-variable
  arithmetic exactly (including the velocity-weighted
  `minimal_displacement_factor`); the only deliberate difference is
  `mid = 0.0` for unbounded variables (the MoveIt path leaves an unused NaN
  there, which is never read).
- The two dead MoveIt `#include`s in `ik_gradient.hpp` / `ik_memetic.hpp` are
  removed; all other declarations are unchanged.
- RSL (`random.hpp`, `queue.hpp`, `random.cpp`) is vendored byte-identical
  from [PickNikRobotics/RSL](https://github.com/PickNikRobotics/RSL)
  (BSD-3-Clause — see `extern/rsl/LICENSE`) because upstream RSL's CMake
  itself depends on `rclcpp`. A local `rsl/export.hpp` shim replaces the
  CMake-generated export header (empty macro; the core is a static library).

## Building (plain CMake, no ROS)

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build
```

Missing dependencies (Eigen 3.4, fmt, Catch2 3) are fetched automatically via
FetchContent when not found on the system (Eigen from its canonical GitLab
repository; `github.com/eigen/eigen` is a stale mirror).

### Pinning to local dependency checkouts

FetchContent accepts `FETCHCONTENT_SOURCE_DIR_<UCNAME>` overrides to use
pre-cloned sources instead of git-cloning at configure time (useful where
network/git access is restricted). The working tree in this workspace uses
shallow clones under `../.deps/`:

```sh
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 \
  -DFETCHCONTENT_SOURCE_DIR_EIGEN=.../URDF_BIO_IK/.deps/eigen \
  -DFETCHCONTENT_SOURCE_DIR_FMT=.../URDF_BIO_IK/.deps/fmt \
  -DFETCHCONTENT_SOURCE_DIR_CATCH2=.../URDF_BIO_IK/.deps/catch2
```

### MSVC notes

- `_USE_MATH_DEFINES` is set for all targets so `M_PI`/`M_PI_4` from
  `<cmath>` exist under MSVC (upstream is developed against GCC).
- `ik_gradient.cpp` is byte-identical to upstream but relies on
  `std::accumulate` without including `<numeric>` itself (GCC's headers
  provide it transitively). The core header `goal.hpp` therefore includes
  `<numeric>` so MSVC compiles the untouched upstream source.

## Usage sketch

```cpp
#include <pick_ik/fk.hpp>
#include <pick_ik/robot.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_memetic.hpp>

auto robot = pick_ik::Robot::make({
    {-M_PI,  M_PI,  true, 2.17},   // J1
    {-2.09,  2.09,  true, 2.17},   // J2
    {-M_PI,  M_PI,  true, 2.17},   // J3
    {-2.09,  2.09,  true, 2.17},   // J4
    {-M_PI,  M_PI,  true, 2.61},   // J5
    {-2.09,  2.09,  true, 2.61},   // J6
    {-M_PI,  M_PI,  true, 2.61},   // J7
});

pick_ik::FkFn fk = my_robot_fk;   // returns {tool0 pose in base frame}

Eigen::Isometry3d const goal = ...;  // in the base frame
std::vector<double> const seed = ...;

auto frame_tests = pick_ik::make_frame_tests({goal}, 1e-3, 1e-3);
auto pose_costs  = pick_ik::make_pose_cost_functions({goal}, 1.0, 0.5);
std::vector<pick_ik::Goal> goals;
goals.push_back({pick_ik::make_minimal_displacement_cost_fn(robot, seed), 0.05});

auto solution_fn = pick_ik::make_is_solution_test_fn(frame_tests, goals, 1e-3, fk);
auto cost_fn     = pick_ik::make_cost_fn(pose_costs, goals, fk);

pick_ik::MemeticIkParams params;  // num_threads etc.
auto maybe = pick_ik::ik_memetic(seed, robot, cost_fn, solution_fn, params);
```

## Solver API (`IkSolver` contract)

`include/pick_ik/solver.hpp` defines a generic solver interface so any
frontend (transport layer, Python, Unity, C++ controller) can use or swap
solvers without changing its code:

```cpp
#include <pick_ik/solvers.hpp>

pick_ik::CcdSolver ccd(600);               // POC CCD, ~2 s of POC runtime
pick_ik::PickIkGradientSolver gradient;    // upstream ik_gradient
pick_ik::PickIkMemeticSolver memetic;      // upstream ik_memetic

pick_ik::SolveOptions options;             // position-only:
options.orientation_threshold = std::nullopt;
options.rotation_scale = 0.0;              // no orientation in the cost

auto result = memetic.solve(robot, link_fk, local_axes, seed, {target}, options);
// result: success, q, position_error [m], orientation_error [rad]
```

- `CcdSolver` is a **faithful C++ port of the p5.js POC's `solveCCD`**
  (J7→J1, damped angle update, limit clamping, configuration re-read at every
  joint step). Position-only: it chases `targets[0]`'s position with the tip
  frame and reports `orientation_error = -1`. Because the POC ran on 0.01 rad
  slider quantization, the continuous port outperforms it: on the arm7
  cross-check targets it reaches ~1e-6 m in ~2 ms where the browser CCD
  stalled ~15–35 mm short.
- `PickIkGradientSolver` / `PickIkMemeticSolver` are thin wrappers around the
  upstream free functions (which remain byte-identical); the wrapper builds
  the frame-test/cost plumbing from `SolveOptions` and packages the unified
  `IkResult`. Tests assert the gradient wrapper is bit-identical to the direct
  free-function call.
- `LinkFkFn` is the richer FK callback: `n` joint child frames (pivots) plus
  the tip frame; `pick_ik::make_tip_fk(link_fk)` derives the tip-only `FkFn`.
  v1 supports a single tip frame and a single target pose.

## FK contract

`FkFn` maps the active joint vector (same order as `Robot::variables`, i.e.
`[J1..J7]`) to **one `Eigen::Isometry3d` per configured tip frame, in the
base/world frame**. `LinkFkFn` does the same plus the `n` joint child frames
in front. Implementations must be re-entrant: the memetic solver calls the
cost function — hence FK — concurrently from
`num_threads × elite_size` threads in the worst case.
