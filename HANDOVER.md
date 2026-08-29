# HANDOVER — libpick_ik_core

> Machine-agnostic resume for this repo. The project-level handover (state,
> next up, cross-machine protocol) lives in the sibling repo:
> `../ik_service/HANDOVER.md` — **read that first.** This file holds the
> core-specific state.
>
> Last updated: 2026-08-29 — `pick_ik_c` C ABI + shared arm7 header + C-ABI
> ctest suite landed (roadmap §3.0a/b/c); next: the Blender add-on (§3.1,
> lives in its own folder/repo, not here).

## One paragraph

Standalone, ROS/MoveIt-free core of the [PickIK](https://github.com/PickNikRobotics/pick_ik)
IK solver. Provenance (README "Provenance"): `ik_gradient.cpp` /
`ik_memetic.cpp` are **byte-identical** to upstream (BSD-3); `robot.cpp` /
`goal.cpp` are the pure, non-MoveIt parts with a new `Robot::make`; RSL
`random`/`queue` vendored byte-identical from PickNikRobotics/RSL. Our
additions: `fk.hpp`, `solver.hpp` (the `IkSolver` contract), `solvers.hpp`
(`CcdSolver` — a faithful C++ port of the p5 POC's CCD — plus thin
gradient/memetic wrappers), the `pickik` pybind11 binding (FK-pump design),
the `pick_ik_c` C ABI shared library (thin adapter, no solver code),
`examples/arm7/arm7.hpp` (the shared C++ arm7 model), and
`examples/arm7_cross_check`.

Only build dependencies: Eigen 3.4 (PUBLIC), fmt (PRIVATE), vendored RSL.
No ROS, no MoveIt, no rclcpp.

## Build / test

```sh
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config RelWithDebInfo
ctest --test-dir build -C RelWithDebInfo
```

FetchContent clones Eigen/fmt/Catch2 when not found; on machines with a
`../.deps` cache use `FETCHCONTENT_SOURCE_DIR_*` overrides instead.

## Current state / next

- Core is stable; **33/33 ctest green** (26 original + 7 C-ABI protocol
  tests); nothing pending in this repo.
- [x] 2026-08-28 build fix: RSL includes moved behind
  `$<BUILD_INTERFACE:>`/`$<INSTALL_INTERFACE:rsl>` — the raw source path in
  the install interface made CMake's `install(EXPORT)` generation fatal
  ("prefixed in the source directory") on out-of-source configure with any
  CMake ≥ 3.22; found during second-machine onboarding (CMake 3.27.0-rc3).
  Install consumers now get `<prefix>/include` + `<prefix>/include/rsl`,
  matching the existing `install(DIRECTORY ...)` rules.
- [x] 2026-08-29 roadmap §3.0 shared blocks (full notes in the roadmap):
  1. **`pick_ik_c` C ABI — done.** `include/pick_ik_c/pickik_c.h` +
     `src/pickik_c.cpp` → `pick_ik_c` shared library (ON by default,
     `PICK_IK_CORE_BUILD_C_ABI`). Opaque handles, POD options/result, C FK
     callback, plus the arm7 model compiled in (`pickik_arm7_robot_create`,
     `pickik_arm7_link_fk`, `pickik_arm7_local_axes`). Conventions:
     standard row-major homogeneous 4×4 (translation in column 3);
     position-only goal = `orientation_threshold < 0` with the default
     `rotation_scale = 0.0` (stack-wide convention — matches `/solve` and
     the C++ `pos_only` helper; a nonzero rotation scale on a position-only
     goal measurably stalls the gradient solver on target B).
  2. **Shared C++ arm7 model — done.** `examples/arm7/arm7.hpp` (Design B
     table + limits/velocities, FK, `make_robot`/`joint_specs`); linked by
     `arm7_cross_check`, all ctest ports, and `pick_ik_c`.
  3. **Validation protocol — done.** `tests/c_abi_tests.cpp`: §5 anchors
     through the C FK, targets A/B through all three solvers,
     out-of-workspace case, limit validity, options plumbing, host FK
     callback path. The ctest suite links `pick_ik_c` (DLL copied next to
     the test exe by a POST_BUILD command).
- Next core work: driven from the service side — the Blender add-on
  (roadmap §3.1) consumes `pick_ik_c.dll` via ctypes; nothing more is owed
  by this repo for it.
- Docs: `docs/api-reference.md` (API/options),
  `docs/integration-roadmap.md` (integration plan + performance numbers),
  `docs/arm7-kinematic-spec.md` (arm7 model, CAD source of truth, MATH
  CONFIRMED — single source of truth for geometry/limits),
  `docs/pick-ik-core-analysis.md` (source-level analysis of the upstream
  extraction).

## Gotchas

- Keep `ik_gradient.cpp` / `ik_memetic.cpp` byte-identical to upstream —
  that's the provenance promise. Fixes go into `solver.cpp` / the wrappers /
  the binding.
- MSVC: `_USE_MATH_DEFINES` is set in the top-level CMake; `goal.hpp`
  includes `<numeric>` so the untouched upstream `ik_gradient.cpp` compiles
  (it relies on `std::accumulate` without including it).
- Binding: all CPython hazards are handled inside `pickik_module.cpp` —
  the FK pump (native threads post FK requests, the calling thread pumps
  under the GIL), borrowed `PyObject*` in solver lambdas, `std::mutex`
  instead of CPython C-level locks (the MS-Store CPython build returns
  `PY_LOCK_FAILURE` for uncontended C-level locks). Two full crash
  post-mortems are in the README — do not redesign casually.
- Memetic + Python FK serializes through the GIL pump: `num_threads > 1`
  costs time, it does not parallelize (Python FK hosts keep 1; native hosts
  should raise it).
- Arm7 model: the C++ side is now the shared header
  `examples/arm7/arm7.hpp` (single source; `tests/arm7_fk.hpp` is a thin
  facade over it; `arm7_cross_check` and `pick_ik_c` link it directly). The
  Python port (`arm7.py` in ik_service) and the URDF stay separate ports of
  `docs/arm7-kinematic-spec.md`. Change the model → update the spec first →
  update all ports → re-run cross-checks and anchor tests.
- `pick_ik_c`: the DLL must sit next to any exe that loads it (MSVC
  searches the exe directory first) — the ctest POST_BUILD copy and the
  Blender add-on's DLL-path property exist for this reason. Row-major 4×4
  is **standard homogeneous** (translation column 3) — the p5 POC used the
  transposed layout; do not copy its index math.
