# PickIK → `libpick_ik_core` — Source-Level Analysis Report

**Scope:** analysis only; no PickIK file was modified.
**Repo inspected:** `C:\Users\swann.schilling\Documents\URDF_BIO_IK\pick_ik` (v1.1.3)
**POC inspected:** `RobotArm_2026_08_25_10_03_56\sketch.js` (p5.js, URDF + FK + CCD)

---

## 1. Repository structure (PickIK-relevant)

```
pick_ik/
├── CMakeLists.txt                        # ament; ONE shared target: pick_ik_plugin
├── CMakePresets.json                     # build presets (Ninja)
├── package.xml                           # ROS 3-format metadata
├── pick_ik_kinematics_description.xml    # pluginlib registration (KinematicsBase)
├── cmake/FindCatch2.cmake
├── include/pick_ik/
│   ├── robot.hpp                         # Robot struct (pure data) + MoveIt factories
│   ├── goal.hpp                          # all cost/test factories + 1 MoveIt factory
│   ├── fk_moveit.hpp                     # FkFn typedef (PURE) + make_fk_fn (MoveIt)
│   ├── forward_kinematics.hpp            # MoveIt FK primitives (JointModel/LinkModel/tf2)
│   ├── ik_gradient.hpp                   # local solver (pure; dead MoveIt includes)
│   ├── ik_memetic.hpp                    # global solver (pure + RSL; dead MoveIt includes)
│   └── pick_ik_plugin.hpp                # PickIKPlugin : kinematics::KinematicsBase (adapter)
├── src/
│   ├── robot.cpp                         # pure Variable methods + MoveIt Robot::from + helpers
│   ├── goal.cpp                          # 8 pure factories + make_ik_cost_fn (MoveIt)
│   ├── fk_moveit.cpp                     # make_fk_fn — MoveIt RobotState FK (adapter)
│   ├── forward_kinematics.cpp            # MoveIt FK helpers (adapter)
│   ├── ik_gradient.cpp                   # PURE (fmt include unused)
│   ├── ik_memetic.cpp                    # PURE + RSL random/queue + fmt (debug prints only)
│   ├── pick_ik_plugin.cpp                # ROS/MoveIt orchestration (adapter)
│   └── pick_ik_parameters.yaml           # → generated pick_ik_parameters.hpp (ParamListener)
└── tests/
    ├── CMakeLists.txt                    # links pick_ik_plugin + moveit_test_utils
    ├── goal_tests.cpp                    # MoveIt-free in practice (pure Eigen tests)
    ├── robot_tests.cpp                   # MoveIt RobotModelBuilder / panda
    ├── ik_tests.cpp                      # MoveIt model + make_fk_fn
    └── ik_memetic_tests.cpp              # MoveIt model + make_fk_fn
```

Build system facts (from `CMakeLists.txt` / `package.xml`):
- Single shared library `pick_ik_plugin` containing **all** .cpp files — core and adapter are currently welded together.
- `generate_parameter_library(pick_ik_parameters src/pick_ik_parameters.yaml)` produces `pick_ik/pick_ik_parameters.hpp` (`ParamListener`), included **only** by `pick_ik_plugin.cpp`.
- Dependencies: `moveit_core` (robot_model, robot_state, kinematics_base), `rclcpp`, `pluginlib`, `tf2_geometry_msgs`, `tf2_kdl`, `tf2`, `fmt`, `rsl`, `range-v3`, `tl-expected`, `Eigen` (implicit via MoveIt).
- `range-v3` is declared but **never used** (verified by grep: zero `ranges::` occurrences). Dead dependency — drop.
- `target_compile_features(... cxx_std_17)` — core needs C++17.

---

## 2. Dependency graph

### Header include graph

```
pick_ik_plugin.hpp ──► robot.hpp ─────────────► moveit/robot_model/{robot_model,joint_model,joint_model_group}.h
               │                └────────────► tl/expected.hpp
               └──► moveit/kinematics_base/kinematics_base.h

goal.hpp ──► fk_moveit.hpp ──► moveit/robot_model/{robot_model,joint_model_group}.h
              └► robot.hpp
              └► moveit/kinematics_base/kinematics_base.h      (only for make_ik_cost_fn)
              └► moveit/robot_model/{robot_model,joint_model_group}.h (only for make_ik_cost_fn)

ik_gradient.hpp ──► goal.hpp, robot.hpp
              └► moveit/robot_model/{joint_model_group,robot_model}.h   ← UNUSED (vestigial)

ik_memetic.hpp ──► goal.hpp, ik_gradient.hpp, robot.hpp
              └► rsl/random.hpp                                   ← USED (uniform_real, uniform_int)
              └► moveit/robot_model/{joint_model_group,robot_model}.h ← UNUSED (vestigial)

forward_kinematics.hpp ──► moveit/robot_model/{robot_model,joint_model}.h
                        └► tf2/LinearMath/Vector3.hpp            (used as axis type)

fk_moveit.hpp ──► moveit/robot_model/{robot_model,joint_model_group}.h   (only for make_fk_fn)
```

### Symbol-level use (what each .cpp actually *uses*)

| File | MoveIt symbols | RSL | fmt | tl:: | tf2 | geometry_msgs |
|---|---|---|---|---|---|---|
| `robot.cpp` | `RobotModel`, `JointModelGroup`, `RobotState` (in `Robot::from`, `get_active_variable_indices`, `get_variables`, `transform_poses_to_frames`) | `uniform_real` (in `Variable::generate_valid_value`) | `fmt::format` (in `get_link_indices` only) | `expected` (return of `get_link_indices`) | `tf2::fromMsg` (in `transform_poses_to_frames`) | `msg::Pose` (in `transform_poses_to_frames`) |
| `goal.cpp` | **only** in `make_ik_cost_fn` (`RobotState`, `IKCostFn`, `RobotModel`, `JMG`, `msg::Pose`) | — | — | — | — | `msg::Pose` (only there) |
| `fk_moveit.cpp` | `RobotState`, `RobotModel`, `JMG` (entire file) | — | — | — | — | — |
| `forward_kinematics.cpp` | `RobotModel`, `JointModel`, `LinkModel` (entire file) | — | — | — | `tf2::Vector3` | — |
| `ik_gradient.cpp` | **none** | — | include only, never used | — | — | — |
| `ik_memetic.cpp` | **none** | `uniform_real`, `uniform_int`, `Queue` | `fmt::print` (all inside `if (print_debug)` / `printPopulation`) | — | — | — |
| `pick_ik_plugin.cpp` | `KinematicsBase`, `RobotModel`, `RobotState`, `JMG`, `moveit_msgs` | — | — | — | — | `msg::Pose` |

**Key structural finding:** every MoveIt/ROS symbol in the codebase lives in exactly four places:
1. `robot.cpp/hpp` — `Robot::from` + 4 helper functions
2. `goal.cpp/hpp` — the single function `make_ik_cost_fn`
3. `fk_moveit.cpp` + `forward_kinematics.cpp` — MoveIt FK (two whole files)
4. `pick_ik_plugin.cpp/hpp` — the KinematicsBase adapter

The two optimizers (`ik_gradient.cpp`, `ik_memetic.cpp`) contain **zero** MoveIt/ROS usage. The MoveIt includes in their headers are dead includes that only compile because the MoveIt include paths exist.

---

## 3. Exact core candidate files

| File | Verdict |
|---|---|
| `include/pick_ik/ik_gradient.hpp` + `src/ik_gradient.cpp` | **CORE, whole file** (drop 2 dead MoveIt includes) |
| `include/pick_ik/ik_memetic.hpp` + `src/ik_memetic.cpp` | **CORE, whole file** (drop 2 dead MoveIt includes; keep RSL or swap ~15 lines) |
| `include/pick_ik/robot.hpp` + `src/robot.cpp` | **CORE, PARTIAL** — `Robot` struct, `Variable` + its 3 methods, `set_random_valid_configuration`, `is_valid_configuration` are pure. Move `Robot::from`, `get_link_indices`, `get_active_variable_indices`, `get_variables`, `transform_poses_to_frames` to the adapter. Add one new pure factory (see §5). |
| `include/pick_ik/goal.hpp` + `src/goal.cpp` | **CORE, PARTIAL** — 8 of 9 functions are pure (`make_frame_test_fn`, `make_frame_tests`, `make_pose_cost_fn`, `make_pose_cost_functions`, `make_center_joints_cost_fn`, `make_avoid_joint_limits_cost_fn`, `make_minimal_displacement_cost_fn`, `make_is_solution_test_fn`, `make_cost_fn`). Only `make_ik_cost_fn` is MoveIt-bound → move to adapter. |
| `include/pick_ik/fk_moveit.hpp` | **SPLIT** — the `FkFn` typedef is pure (move to new core header `fk.hpp`); `make_fk_fn` is adapter-only. |

### 4. Exact ROS/MoveIt adapter files (retained in the ROS package)

- `include/pick_ik/pick_ik_plugin.hpp` + `src/pick_ik_plugin.cpp` (pluginlib + `KinematicsBase` + `rclcpp` + `ParamListener`)
- `src/fk_moveit.cpp` (`make_fk_fn`) + the MoveIt portion of `fk_moveit.hpp`
- `include/pick_ik/forward_kinematics.hpp` + `src/forward_kinematics.cpp` (entirely)
- MoveIt portion of `robot.{hpp,cpp}`: `Robot::from`, `get_link_indices`, `get_active_variable_indices`, `get_variables`, `transform_poses_to_frames` → new `robot_moveit.{hpp,cpp}`
- MoveIt portion of `goal.{hpp,cpp}`: `make_ik_cost_fn` → new `goal_moveit.{hpp,cpp}`
- `src/pick_ik_parameters.yaml` (+ generated header)
- `pick_ik_kinematics_description.xml`, `package.xml` export stanza
- Tests `robot_tests.cpp`, `ik_tests.cpp`, `ik_memetic_tests.cpp` (use `moveit_test_utils`); `goal_tests.cpp` is MoveIt-free and can be reused as-is for the core.

---

## 5. Exact modifications required for each core candidate

### `robot.hpp`
1. Remove `#include <tl/expected.hpp>`, `#include <moveit/robot_model/joint_model.h>`, `joint_model_group.h`, `robot_model.h`.
2. Remove declarations of: `Robot::from`, `get_link_indices`, `get_active_variable_indices`, `get_variables`, `transform_poses_to_frames` (they move to `robot_moveit.hpp`).
3. Add a pure construction path. `Robot` is currently an aggregate whose only constructor is `Robot::from(RobotModel, JMG, tips)`. Add:
   ```cpp
   struct JointSpec {
       double min;
       double max;
       bool bounded = true;
       double max_velocity = 0.0;   // 0.0 ⇒ velocity-unweighted fallback (exactly what Robot::from does)
   };
   static Robot make(std::vector<JointSpec> const& joints);
   ```
   `make()` must reproduce `Robot::from`'s per-variable math (lines 60–82 of `robot.cpp`):
   `mid = 0.5*(min+max)`; `half_span = bounded ? (max-min)/2 : M_PI`; `max_velocity_rcp = vmax > 0 ? 1/vmax : 0`; `minimal_displacement_factor` initially `1/N`, then re-weighted to `max_velocity_rcp / Σ(max_velocity_rcp)` when `Σ > 0`.
   *(Detail to preserve: for unbounded joints set `mid = 0.0` — the existing MoveIt path computes `0.5*(−inf+inf) = NaN` there, but `mid` is never read for unbounded variables because both centering and limit-avoidance factories skip `!bounded`. No behavioral change.)*
4. Keep `Variable::generate_valid_value`, `is_valid`, `clamp_to_limits`, `set_random_valid_configuration`, `is_valid_configuration` unchanged.

### `robot.cpp`
1. Keep: `Variable::generate_valid_value` (uses `rsl::uniform_real`), `is_valid`, `clamp_to_limits`, `set_random_valid_configuration`, `is_valid_configuration`.
2. Move out: `Robot::from`, `get_link_indices`, `get_active_variable_indices`, `get_variables`, `transform_poses_to_frames` → adapter `robot_moveit.cpp` (byte-for-byte relocation; their includes `rsl`, `tf2_eigen`, `tf2_geometry_msgs`, `tl/expected`, `fmt`, MoveIt stay with them).
3. Implement new `Robot::make`.
4. Resulting includes: Eigen (via header), `<cmath>`, `<cfloat>`, `<algorithm>`, RSL (or std random after §RSL below). No MoveIt/ROS/tl/fmt/tf2.

### `goal.hpp`
1. Remove `#include <moveit/kinematics_base/kinematics_base.h>`, `robot_model.h`, `joint_model_group.h`.
2. Replace `#include <pick_ik/fk_moveit.hpp>` with `#include <pick_ik/fk.hpp>` (new core header holding only the `FkFn` typedef — see below).
3. Remove the `make_ik_cost_fn` declaration → `goal_moveit.hpp`.
4. Cosmetic: `CostFn` is declared twice (lines 35 and 62) — remove the duplicate.

### `goal.cpp`
1. Remove `make_ik_cost_fn` (lines 146–161) and its four MoveIt includes → `goal_moveit.cpp`.
2. **Everything else is byte-identical core code.** All 8 remaining factories use only Eigen + the core type aliases + `Robot`.

### `fk_moveit.hpp` → split
- New core header `include/pick_ik/fk.hpp`:
  ```cpp
  #pragma once
  #include <Eigen/Geometry>
  #include <functional>
  #include <vector>
  namespace pick_ik {
  using FkFn = std::function<std::vector<Eigen::Isometry3d>(std::vector<double> const&)>;
  }
  ```
- Adapter keeps `fk_moveit.hpp` (now `#include <pick_ik/fk.hpp>` + MoveIt model headers, declaring only `make_fk_fn`). `src/fk_moveit.cpp` stays adapter-only.

### `ik_gradient.hpp`
1. Remove `#include <moveit/robot_model/joint_model_group.h>` and `#include <moveit/robot_model/robot_model.h>` (unused — no declaration in the file references MoveIt types).
2. No other change. `GradientIk`, `GradientIkParams`, `step`, `ik_gradient` are already pure.

### `ik_gradient.cpp`
- No functional change. Optional cosmetic: drop unused `#include <fmt/core.h>`.

### `ik_memetic.hpp`
1. Remove the two unused MoveIt includes. Keep `#include <rsl/random.hpp>`.
2. No other change.

### `ik_memetic.cpp`
- No functional change. Two options for RSL:
  - **Keep RSL** (recommended first): it is a standalone PickNik "ROS Support Library" (C++17 headers; the two headers used — `random.hpp`, `queue.hpp` — are pure C++, the rclcpp-dependent part is `parameter_validators.hpp`, which pick_ik never includes). Vendor it via `FetchContent` in the core CMake; zero code changes. *Verify the vendored version's `random.hpp` has no rclcpp include before committing.*
  - **Replace** (fully-zero-third-party, ~15-line diff):
    - `rsl::uniform_real(a,b)` → thread-local `std::mt19937` + `std::uniform_real_distribution<double>(a,b)`
    - `rsl::uniform_int<size_t>(a,b)` → `std::uniform_int_distribution<size_t>(a,b)`
    - `rsl::Queue<std::optional<Individual>>` → a ~20-line local class over `std::deque` + `std::mutex` + `std::condition_variable` with `push`, `pop(timeout)`, `empty`.
    - **Correctness note:** RSL's random uses **per-thread engines**; `ik_memetic` calls these from up to `elite_size × num_threads` concurrent threads. Any std replacement must likewise use thread-local engines (or guard a shared one) or the mutation stream becomes correlated.
  - Optionally strip the `fmt::print` debug paths (`print_debug == false` by default) to remove the last fmt dependency; fmt itself is a plain C++ library and an acceptable core dependency if kept.

### `forward_kinematics.{hpp,cpp}`
- **Not ported.** Entirely MoveIt FK (`JointModel`/`LinkModel`, `tf2::Vector3`). Your C++ FK replaces it. The one concept worth copying: `get_frame` for REVOLUTE builds `Quaterniond(fcos, axis.x()*fsin, axis.y()*fsin, axis.z()*fsin)` with `half_angle = q/2` — the same Rodrigues half-angle you already use in the POC (`rotationAroundAxis`), confirming the two FK conventions are mathematically identical.

---

## 6. Exact FK interface required

PickIK's actual FK abstraction (from `fk_moveit.hpp`, used throughout):

```cpp
using FkFn = std::function<std::vector<Eigen::Isometry3d>(std::vector<double> const&)>;
```

Your guessed interface was **correct in shape**. Precise contract, verified from call sites:

| Aspect | Requirement |
|---|---|
| Input | `std::vector<double>` of **active** joint positions, same order as `Robot.variables` (for your arm: `[J1 … J7]`, the order the POC's `thetas[jointIndex]` walks). |
| Output | `std::vector<Eigen::Isometry3d>` — **one pose per configured tip frame, returned in full on every call**. There is no per-call frame selection. For your arm: a single-element vector containing the **tool0 pose**. |
| Frame convention | Poses in the **base/world frame** (root link at identity) — the same convention as MoveIt's `getGlobalLinkTransform` and the POC's `frames` map with `base_link = I`. Goal pose must be expressed in that same frame. |
| Units | Meters / radians (MoveIt's `updateLinkTransforms` is metric; note the POC's `SCALE = 250` px/m is display-only and is divided out before IK at sketch.js lines 917–931 — your C++ FK must stay in meters). |
| Orientation | Always present in the `Isometry3d`, but its *influence* is optional: `rotation_scale = 0.0` disables orientation in the pose cost, and `orientation_threshold = std::nullopt` disables it in the solution test. Position-only solving is supported. |
| Number of frames | Fixed at construction (length of `tip_link_indices` / `goal_frames`). All frames every time. |
| Joint axes | **Not required** — only the MoveIt FK implementation needs them. |
| Joint positions | Required (as input to FK; limits/velocities come via `Robot`, not FK). |
| Callback | Yes — a plain `std::function`. Invoked hundreds–thousands of times per solve (each gradient `step()` = `2N + 2` evaluations; memetic adds population evals). |
| Thread safety | **Must be re-entrant.** In `ik_memetic_impl`, the `elite_size` (default 4) gradient-descent threads call `cost_fn` → FK **concurrently**, and `ik_memetic` itself runs up to `num_threads` full copies. Worst case ≈ `4 × num_threads` simultaneous FK calls. Your POC FK is a pure function of `q` — trivially thread-safe as long as the C++ port allocates its frame array locally (no shared mutable state), or you add a mutex and pass the solver a locked wrapper. |
| POC compatibility | Direct. `computeURDFFK`'s tree walk `child = parent · origin · R(axis, q)` is exactly `RobotState::updateLinkTransforms`; the ported callback is `return { fkTool0(q) };`. |

The `std::mutex& mx` parameter of `make_fk_fn` exists only because MoveIt reuses one shared `RobotState` inside its lambda — it is **not** part of the `FkFn` signature and is irrelevant to your callback.

---

## 7. Exact Robot interface required

After the split, the solver's entire view of the robot is:

```cpp
namespace pick_ik {
struct Robot {
    struct Variable {
        double min, max, mid;
        bool bounded;
        double half_span;
        double max_velocity_rcp;            // 1/max_velocity, 0 if unknown
        double minimal_displacement_factor; // velocity-weighted, else 1/N
        double generate_valid_value(double init_val = 0.0) const; // random valid sample
        bool   is_valid(double val) const;
        double clamp_to_limits(double val) const;
    };
    std::vector<Variable> variables;

    static Robot make(std::vector<JointSpec> const& joints);  // NEW pure factory

    void set_random_valid_configuration(std::vector<double>& config) const;
    bool is_valid_configuration(std::vector<double> const& config) const;
};
}
```

What the algorithms actually read (grep-verified):
- `ik_gradient.cpp` → `variables.size()` (assert), `variables[i].clamp_to_limits(...)`
- `ik_memetic.cpp` → `variables.size()`, `set_random_valid_configuration` (init population, mating-pool-empty re-roll), `variables[j].half_span`, `variables[j].clamp_to_limits(...)`
- `goal.cpp` (the three secondary objectives) → `variables.size()`, `bounded`, `min`, `max`, `mid`, `half_span`, `minimal_displacement_factor`

So the kinematic information your app must hand to the core is: **per joint — min, max, max_velocity (optional)**; joint *names* are not needed by the core at all (only by your own bookkeeping and by the MoveIt adapter). No joint axes, no link frames, no URDF — the core stays kinematics-agnostic by design, and your app supplies limits/velocities straight from the URDF it already parses.

For your arm the `Robot::make` call data (from the POC URDF):

| Joint | min | max | max_velocity |
|---|---|---|---|
| J1 | −π | +π | 2.17 |
| J2 | −2.09 | +2.09 | 2.17 |
| J3 | −π | +π | 2.17 |
| J4 | −2.09 | +2.09 | 2.17 |
| J5 | −π | +π | 2.61 |
| J6 | −2.09 | +2.09 | 2.61 |
| J7 | −π | +π | 2.61 |

All bounded; velocity-weighted `minimal_displacement_factor` = `(1/v_i)/Σ(1/v_j)` (J1–J4 ≈ 0.1474, J5–J7 ≈ 0.1636 each, normalized) — identical to what `Robot::from` would compute from MoveIt's `VariableBounds`.

---

## 8. Exact Goal/CostFn interface required

All pure, all in `goal.hpp` (post-split):

```cpp
using FrameTestFn    = std::function<bool(Eigen::Isometry3d const& tip_frame)>;
using PoseCostFn     = std::function<double(std::vector<Eigen::Isometry3d> const& tip_frames)>;
using CostFn         = std::function<double(std::vector<double> const& active_positions)>;
using SolutionTestFn = std::function<bool(std::vector<double> const& active_positions)>;

struct Goal { CostFn eval; double weight; };
```

Factories:

| Factory | Signature | Behavior (source-verified) |
|---|---|---|
| `make_frame_tests` | `(vector<Isometry3d> goal_frames, optional<double> pos_thr, optional<double> rot_thr) → vector<FrameTestFn>` | Pass iff `‖p_goal − p_tip‖ ≤ pos_thr` (if set) **and** `angular_distance(quats) ≤ rot_thr` (if set). `nullopt` disables that axis. |
| `make_pose_cost_fn` | `(Isometry3d goal, size_t goal_link_index, double pos_scale, double rot_scale) → PoseCostFn` | `(‖Δp‖·pos_scale)² + (angular_distance·rot_scale)²` on `tip_frames[goal_link_index]`. Scale 0 ⇒ that term off. |
| `make_pose_cost_functions` | `(vector<Isometry3d> goal_frames, pos_scale, rot_scale) → vector<PoseCostFn>` | One per goal frame; frame `i` ↔ cost `i`. |
| `make_center_joints_cost_fn` | `(Robot) → CostFn` | `Σ (q_i − mid_i)² · minimal_displacement_factor_i`, bounded joints only. |
| `make_avoid_joint_limits_cost_fn` | `(Robot) → CostFn` | `Σ fmax(0, 2·\|q_i − mid_i\| − half_span_i)² · weight_i` — zero until a joint enters the outer half of its range. |
| `make_minimal_displacement_cost_fn` | `(Robot, vector<double> initial_guess) → CostFn` | `Σ (q_i − seed_i)² · weight_i`. |
| `make_is_solution_test_fn` | `(vector<FrameTestFn>, vector<Goal>, double cost_threshold, FkFn) → SolutionTestFn` | Valid iff **all** frame tests pass **and** every goal's `eval(q)·weight² < threshold²`. |
| `make_cost_fn` | `(vector<PoseCostFn>, vector<Goal>, FkFn) → CostFn` | Total = `Σ pose_cost_i(tip_frames)` `+ Σ goal_i.eval(q)·weight_i²`. This is the multi-objective composition point. |
| ~~`make_ik_cost_fn`~~ | — | **Adapter-only** (MoveIt `IKCostFn`). Standalone equivalent: wrap any user `CostFn` lambda directly as a `Goal{fn, weight}` — strictly more flexible, no MoveIt needed. |

Note: goal weights enter **squared** in both `make_cost_fn` and the solution test — keep that convention when choosing weights (the ROS parameter file uses e.g. `0.01` weights, i.e. effective `1e-4`).

---

## 9. Exact solver entry points to call

```cpp
// src/ik_gradient.cpp  — local solver
std::optional<std::vector<double>> ik_gradient(
    std::vector<double> const& initial_guess,
    Robot const& robot,
    CostFn const& cost_fn,
    SolutionTestFn const& solution_fn,
    GradientIkParams const& params,   // step_size, min_cost_delta, max_time, max_iterations,
                                      // stop_optimization_on_valid_solution
    bool approx_solution);

// src/ik_memetic.cpp   — global memetic solver (evolution + gradient exploitation)
std::optional<std::vector<double>> ik_memetic(
    std::vector<double> const& initial_guess,
    Robot const& robot,
    CostFn const& cost_fn,
    SolutionTestFn const& solution_fn,
    MemeticIkParams const& params,    // elite_size, population_size, wipeout_fitness_tol,
                                       // max_generations, max_time, num_threads,
                                       // stop_optimization_on_valid_solution, stop_on_first_soln,
                                       // + embedded gd_params (GradientIkParams)
    bool approx_solution = false,
    bool print_debug = false);
```

Both return the active-joint vector (`[J1…J7]`) or `std::nullopt`. `ik_memetic` internally drives `ik_gradient`'s `GradientIk::from` + `step` for elite exploitation — you never call those separately.

The plugin's `searchPositionIK` (pick_ik_plugin.cpp lines 73–294) is **pure glue around these two calls**: parameter read → assemble frame tests / pose costs / goals / solution fn / cost fn → sanitize seed (`is_valid_configuration`, else `set_random_valid_configuration`) → pick `local` vs `global` mode → call `ik_gradient`/`ik_memetic` → on failure re-roll seed and retry with remaining time → optional approximate-solution gating (looser thresholds + per-joint displacement threshold vs seed) → callbacks/error codes. That ~80-line loop is the entire "minimal natural wrapper" — it contains no algorithm, so a standalone `PickIkSolver::solve(IkRequest)` that mirrors it is a faithful, thin wrapper (no unnecessary abstraction layers).

---

## 10. Proposed `libpick_ik_core` directory structure

```
libpick_ik_core/
├── CMakeLists.txt
├── LICENSE                              # BSD-3-Clause (from PickIK)
├── include/pick_ik/
│   ├── fk.hpp                           # NEW: FkFn typedef (moved from fk_moveit.hpp)
│   ├── robot.hpp                        # SPLIT: Robot/Variable + JointSpec + Robot::make
│   ├── goal.hpp                         # SPLIT: all factories except make_ik_cost_fn
│   ├── ik_gradient.hpp                  # UNCHANGED minus 2 dead MoveIt includes
│   └── ik_memetic.hpp                   # UNCHANGED minus 2 dead MoveIt includes
├── src/
│   ├── robot.cpp                        # PARTIAL: pure parts + new Robot::make
│   ├── goal.cpp                         # PARTIAL: 8 pure factories
│   ├── ik_gradient.cpp                  # UNCHANGED
│   └── ik_memetic.cpp                   # UNCHANGED (keep RSL or std swap)
└── tests/
    ├── CMakeLists.txt                   # Catch2, links pick_ik_core only
    ├── goal_tests.cpp                   # REUSED as-is (already MoveIt-free)
    ├── robot_tests_core.cpp             # NEW: Robot::make math, clamping, random validity
    └── ik_tests_core.cpp                # NEW: your 7-DOF FK as a plain lambda,
                                          # mirroring POC FK; RR + 7-DOF solve cases
```

ROS package keeps (adapter): `pick_ik_plugin.{hpp,cpp}`, `fk_moveit.{hpp,cpp}`, `forward_kinematics.{hpp,cpp}`, `robot_moveit.{hpp,cpp}`, `goal_moveit.{hpp,cpp}`, `pick_ik_parameters.yaml`, `pick_ik_kinematics_description.xml`, `package.xml`.

---

## 11. Proposed CMake target structure

```cmake
# ===== libpick_ik_core: plain CMake, no ament =====
cmake_minimum_required(VERSION 3.21)
project(pick_ik_core LANGUAGES CXX)

find_package(Eigen3 REQUIRED NO_MODULE)

# Option A (recommended, minimal diff):
find_package(rsl QUIET)                 # system RSL if available, else:
if(NOT rsl_FOUND)
  include(FetchContent)
  FetchContent_Declare(rsl
    GIT_REPOSITORY https://github.com/PickNikRobotics/RSL.git)
  FetchContent_MakeAvailable(rsl)
endif()
# Option B: delete rsl, patch ik_memetic.cpp + robot.cpp to std::mt19937
# (thread-local!) + a local mutex queue — see §5.

# fmt only if the debug-print paths in ik_memetic.cpp are kept:
find_package(fmt REQUIRED)

add_library(pick_ik_core STATIC
  src/robot.cpp
  src/goal.cpp
  src/ik_gradient.cpp
  src/ik_memetic.cpp)
target_compile_features(pick_ik_core PUBLIC cxx_std_17)
target_include_directories(pick_ik_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(pick_ik_core
  PUBLIC  Eigen3::Eigen
  PRIVATE fmt::fmt)          # + rsl::rsl if Option A
# no rclcpp / moveit / pluginlib / tf2 / generate_parameter_library anywhere

# ===== pick_ik_ros_adapter: ament package =====
add_library(pick_ik_plugin SHARED
  src/pick_ik_plugin.cpp
  src/fk_moveit.cpp
  src/forward_kinematics.cpp
  src/robot_moveit.cpp
  src/goal_moveit.cpp)
target_link_libraries(pick_ik_plugin
  PUBLIC  pick_ik_core
  PRIVATE moveit_core::moveit_robot_model
          moveit_core::moveit_robot_state
          moveit_core::moveit_kinematics_base
          tf2_geometry_msgs::tf2_geometry_msgs
          tf2_kdl::tf2_kdl
          tl::expected
          rsl::rsl
          fmt::fmt
          pluginlib::pluginlib
          rclcpp::rclcpp
          pick_ik_parameters)        # generate_parameter_library(...)
pluginlib_export_plugin_description_file(moveit_core pick_ik_kinematics_description.xml)
```

Layering (as you specified): `moveit2 ← pick_ik_ros_adapter ← libpick_ik_core ← your C++ robot API`. Core has **no** ROS dependency; the adapter links core and adds `make_fk_fn`, `Robot::from`, `make_ik_cost_fn`, `transform_poses_to_frames` back on top. Drop `range-v3` from both (unused).

---

## 12. Minimal pseudo-code: MyRobot → FK callback → PickIK core → solution

```cpp
#include <pick_ik/fk.hpp>
#include <pick_ik/robot.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/ik_gradient.hpp>

// ---- 1) Robot: limits/velocities from the URDF your app already owns ----
auto robot = pick_ik::Robot::make({
    {  .min = -M_PI, .max =  M_PI, .max_velocity = 2.17},  // J1
    {  .min = -2.09, .max =  2.09, .max_velocity = 2.17},  // J2
    {  .min = -M_PI, .max =  M_PI, .max_velocity = 2.17},  // J3
    {  .min = -2.09, .max =  2.09, .max_velocity = 2.17},  // J4
    {  .min = -M_PI, .max =  M_PI, .max_velocity = 2.61},  // J5
    {  .min = -2.09, .max =  2.09, .max_velocity = 2.61},  // J6
    {  .min = -M_PI, .max =  M_PI, .max_velocity = 2.61},  // J7
});

// ---- 2) FK callback: your existing URDF FK, thread-safe, meters ----
//    POC algorithm: base=I; for each joint: child = parent * origin(rpy,xyz) * R(axis, q_i)
//    returns pose of tool0 (fixed joint z=0.126 from link7) in base frame.
pick_ik::FkFn fk = [](std::vector<double> const& q) {
    Eigen::Isometry3d tool0 = /* your FK, no shared mutable state */;
    return std::vector<Eigen::Isometry3d>{tool0};
};

// ---- 3) Goal: target pose in base frame ----
Eigen::Isometry3d goal =
    Eigen::Translation3d(0.35, -0.05, 0.59) * Eigen::Quaterniond(0.707, 0.0, 0.0, 0.707);

std::vector<double> seed = {0.0, -0.4, 0.1, -1.2, 0.0, 1.3, 0.4};

// ---- 4) Assemble (this is exactly what pick_ik_plugin.cpp does, minus ROS) ----
auto frame_tests = pick_ik::make_frame_tests({goal},
                                             /*pos_thr=*/1e-3, /*rot_thr=*/1e-3);
auto pose_costs  = pick_ik::make_pose_cost_functions({goal},
                        /*pos_scale=*/1.0, /*rot_scale=*/0.5);

std::vector<pick_ik::Goal> goals;
goals.push_back({pick_ik::make_minimal_displacement_cost_fn(robot, seed), /*w=*/0.05});
// redundant-DOF secondary objectives, as needed:
// goals.push_back({pick_ik::make_center_joints_cost_fn(robot),  0.01});
// goals.push_back({pick_ik::make_avoid_joint_limits_cost_fn(robot), 0.01});

auto solution_fn = pick_ik::make_is_solution_test_fn(frame_tests, goals,
                                                     /*cost_threshold=*/1e-3, fk);
auto cost_fn     = pick_ik::make_cost_fn(pose_costs, goals, fk);

// ---- 5) Solve ----
// global (redundancy-capable; seed may be far) — multithreaded, FK must be thread-safe:
pick_ik::MemeticIkParams mp;
mp.num_threads = 4;  mp.max_time = 0.5;
mp.population_size = 16;  mp.elite_size = 4;
mp.gd_params.max_time = 0.005;  mp.gd_params.max_iterations = 25;

auto maybe = pick_ik::ik_memetic(seed, robot, cost_fn, solution_fn, mp,
                                 /*approx_solution=*/false);
// local (fast; seed must be near the goal):
// pick_ik::GradientIkParams gp;  gp.max_time = 0.05;
// auto maybe = pick_ik::ik_gradient(seed, robot, cost_fn, solution_fn, gp, false);

if (maybe) {
    // maybe.value() == [J1..J7] that reaches goal within thresholds.
    // Optional: retry loop with re-rolled seed like the plugin does
    // (if !robot.is_valid_configuration(seed) robot.set_random_valid_configuration(seed);
    //  on timeout failure: robot.set_random_valid_configuration(seed); retry with remaining time).
}
```

The *smallest natural wrapper* is exactly step 4+5 (≈ 80 lines, mirroring `PickIKPlugin::searchPositionIK` with the ROS seams cut): nothing more is needed before your `PickIkSolver solver(robot, fk); auto r = solver.solve(request);` API — that API is a rename of this, not new abstraction.

---

## 13. Hidden / non-obvious dependencies blocking a ROS/MoveIt-free build

1. **`goal.hpp` includes `fk_moveit.hpp` solely for the `FkFn` typedef** — so "core" headers currently drag `moveit/robot_model/*` into every consumer of the cost types. Splitting `FkFn` into `fk.hpp` is the single most important cut.
2. **Dead MoveIt includes in `ik_gradient.hpp` / `ik_memetic.hpp`** — they hide the fact that both optimizers are pure; they only compile because MoveIt include paths exist on the build.
3. **RSL is a real (but non-ROS) dependency** of the core (`uniform_real`, `uniform_int`, `Queue`). It's PickNik's "ROS Support Library", C++17, and the two headers used are plain C++ — but it must be vendored/located for a standalone build. Thread-locality of its RNG matters (see §5).
4. **Concurrent FK** — up to `elite_size × num_threads` simultaneous FK calls inside `ik_memetic`; the POC's single-threaded JS has no such requirement. Your C++ FK must be re-entrant or mutex-wrapped.
5. **`fmt` is pulled into `ik_memetic.cpp`** — but only behind `print_debug` (off by default). Strippable if you want zero third-party beyond Eigen (and RSL).
6. **`tl/expected` is confined to `get_link_indices`** (adapter). Core never needs it.
7. **`range-v3` is declared in CMake/package.xml but used nowhere** — remove; it's a hidden "gotcha" for anyone diffing deps.
8. **`make_ik_cost_fn` is the one core-file function that cannot move**: it takes MoveIt's `KinematicsBase::IKCostFn`, `RobotModel`, `JMG`, `geometry_msgs::Pose`. Its *function* (user-defined extra cost) is replaced in standalone by adding any `CostFn` lambda as a `Goal` — capability preserved, MoveIt types removed.
9. **`transform_poses_to_frames` / `get_variables`** (robot.cpp) convert MoveIt `RobotState` → frames; the plugin uses them to express target poses at the seed state. Standalone, your app simply supplies the goal in the base frame — no equivalent needed, but note the plugin does *not* re-target poses relative to a moving base; keep that assumption.
10. **`assert()`s in `goal.cpp`** (`frame_tests.size() == tip_frames.size()`, size checks in the cost fns) — active in debug builds; your FK must always return exactly the configured frame count.
11. **Tests are entangled with MoveIt** (`moveit_test_utils`, `RobotModelBuilder`, panda URDF). Only `goal_tests.cpp` transfers as-is; `ik_*`/`robot_*` tests need a manual-FK variant (trivial for your rigid 7-DOF arm — the POC FK is the reference implementation).
12. **C++17** is the baseline (`cxx_std_17`); nothing in the core requires C++20.
13. **ament** wraps the package (`ament_package()`, `pluginlib_export...`); none of it appears in core code, but the top-level CMake must not assume `find_package(ament_cmake_ros)`.
14. **Unbounded-joint `mid = NaN` quirk** in `Robot::from` (0.5·(−inf+inf)): harmless today (unbounded vars are skipped by the centering/limit-avoidance factories) but your pure `Robot::make` should set `mid = 0` for unbounded joints to be safe.

---

## 14. Verdict: small refactor, not a fork

**The current PickIK source supports your architecture with a small, mechanical refactor.** No algorithm change, no new IK, no Eigen replacement, no URDF parser needed. Evidence:

- `ik_gradient.cpp` and `ik_memetic.cpp` contain **zero** MoveIt/ROS symbols; their headers' MoveIt includes are provably unused.
- `goal.cpp` is 8-of-9 pure functions; `robot.cpp`'s `Variable`/validation layer is pure.
- The FK boundary is already a single std::function type alias with exactly the signature you guessed; your POC FK (URDF tree walk, `child = parent · origin · R(axis, q)`, base at identity) satisfies it after returning a one-element `tool0` vector in meters, thread-safe.
- Everything 7-DOF-relevant is core: **joint centering**, **joint-limit avoidance**, **minimal displacement** (all three in pure `goal.cpp`), **pose error** (pure `make_pose_cost_fn`), **multiple weighted goals** (`Goal{eval, weight}` + `make_cost_fn`, weights enter squared), **solution validity** (`make_is_solution_test_fn` with independent position/orientation/cost thresholds), **redundant-DOF handling** (memetic population diversity + per-elite gradient exploitation; local gradient as a near-goal fallback).
- The only capability that is *currently tied to MoveIt* and has no core counterpart is `make_ik_cost_fn` (MoveIt's `IKCostFn` registration), and it is subsumed by the `Goal` mechanism in standalone.

Estimated refactor size: ~150 lines of header surgery (5 headers), ~180 lines of function relocation (2 .cpp splits), ~80 lines of new pure `Robot::make` + `JointSpec` (reusing `Robot::from`'s exact arithmetic), optional ~15–35 lines for the RSL→std swap, plus one new 80-line `solve()` wrapper replicating the plugin's assembly/retry loop. Solver behavior remains byte-identical; the MoveIt adapter becomes the thin consumer of `libpick_ik_core` instead of the other way round.

One caution: `RSL` is a third-party (PickNik) library — it is *not* ROS/MoveIt, but if your "zero dependencies" list eventually means "no non-stdlib, non-Eigen deps", the Option-B swap in §5 is ready. Verify the vendored `rsl/random.hpp`/`rsl/queue.hpp` include lists when you integrate.

---

## 15. Implementation results (appendix, post-analysis)

`libpick_ik_core` was implemented per §5/§11 and validated on this machine
(Windows, MSVC 14.44, VS2022 generator, plain CMake).

**Layout** — `libpick_ik_core/`: `include/pick_ik/{fk,robot,goal,ik_gradient,ik_memetic}.hpp`,
`src/{robot,goal,ik_gradient,ik_memetic}.cpp`, `extern/rsl/` (vendored),
`tests/{goal_tests,robot_tests_core,ik_tests_core}.cpp`, `arm7_fk.hpp` (POC
reference FK), `CMakeLists.txt`, `tests/CMakeLists.txt`, `README.md`, `LICENSE`.

**Provenance (hash-verified):**
- byte-identical to upstream: `src/ik_gradient.cpp`, `src/ik_memetic.cpp`,
  `tests/goal_tests.cpp`, and `extern/rsl/{include/rsl/random.hpp,
  include/rsl/queue.hpp, src/random.cpp}` (from PickNikRobotics/RSL; BSD-3,
  `extern/rsl/LICENSE`).
- `ik_gradient.hpp` / `ik_memetic.hpp`: exactly the two dead MoveIt includes
  removed, nothing else.
- `robot.cpp` / `goal.cpp`: pure parts only; new `Robot::make` reproduces
  `Robot::from` arithmetic (velocity-weighted `minimal_displacement_factor`,
  `mid = 0.0` for unbounded joints as per §13.14).
- New pure headers `fk.hpp` (FkFn) and `robot.hpp`/`goal.hpp` (factories).

**Deviation from the original plan (justified):** upstream RSL's CMake
requires `rclcpp`, so "keep RSL" was realized as byte-identical vendoring of
the pure C++ parts plus an empty `RSL_EXPORT` shim
(`extern/rsl/include/rsl/export.hpp`). The Option-B `std::mt19937` swap from
§5 remains available if a non-Eigen-zero policy is later required.

**MSVC portability fixes (no solver-logic change):**
- `_USE_MATH_DEFINES` (M_PI under MSVC).
- `goal.hpp` includes `<numeric>`: byte-identical `ik_gradient.cpp` uses
  `std::accumulate` without including `<numeric>` (GCC provides it
  transitively; MSVC does not).
- `extern/rsl/include` is a PUBLIC include dir (public `ik_memetic.hpp`
  includes `<rsl/random.hpp>`; installed to `<prefix>/include/rsl`).
- Eigen FetchContent points at `gitlab.com/libeigen/eigen` (the GitHub mirror
  is stale/dead for tag fetches).

**Validation (user's order: position → orientation → secondary → concurrency):**
17/17 Catch2 tests pass (0.5 s, RelWithDebInfo):
- `Robot::make` 7-DOF (bounds, `1/vmax` reciprocals, velocity-weighted
  factors, unbounded variable, clamping, random-valid config) + pure goal
  factory tests (upstream `goal_tests.cpp`).
- RR 2-DOF port of upstream `ik_tests.cpp` (hand FK; zero/nonzero poses,
  near/far seeds, unreachable position, unreachable orientation,
  zero-rotation-scale) — all pass.
- Arm7 POC FK reference: FK(0) = (0, 0, 1.266) with identity orientation
  (analytic guard for composition/RPY order).
- Arm7 gradient IK: position-only and full-pose, near seed
  (q_true + ±0.03 offsets) → converged within 1e-3.
- Arm7 memetic: position-only far-seed (all-zeros seed, 2 threads) and
  full-pose far-seed (4 threads — concurrency smoke); both converge within
  1e-2 and return valid configurations.
- Secondary objectives: zero-cost at reference points, positive elsewhere;
  memetic run with centering + limit-avoidance + minimal-displacement goals
  (smoke, mirroring the upstream panda-with-secondary-objectives test).

**Environment notes:** local shallow clones in `../.deps/{eigen,fmt,catch2}`
with `FETCHCONTENT_SOURCE_DIR_*` overrides (documented in the README);
`BUILD_TESTING=OFF` for subprojects so CTest only sees the 17 core tests
(Eigen's ~900-test suite otherwise leaks in via `subdirs()`).

---

End of report.
