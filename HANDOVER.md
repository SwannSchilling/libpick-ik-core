# HANDOVER — libpick_ik_core

> Machine-agnostic resume for this repo. The project-level handover (state,
> next up, cross-machine protocol) lives in the sibling repo:
> `../ik_service/HANDOVER.md` — **read that first.** This file holds the
> core-specific state.
>
> Last updated: 2026-08-28 — core stable, nothing pending; `pick_ik_c` C ABI
> is the next core work item (prerequisite of the Blender add-on).

## One paragraph

Standalone, ROS/MoveIt-free core of the [PickIK](https://github.com/PickNikRobotics/pick_ik)
IK solver. Provenance (README "Provenance"): `ik_gradient.cpp` /
`ik_memetic.cpp` are **byte-identical** to upstream (BSD-3); `robot.cpp` /
`goal.cpp` are the pure, non-MoveIt parts with a new `Robot::make`; RSL
`random`/`queue` vendored byte-identical from PickNikRobotics/RSL. Our
additions: `fk.hpp`, `solver.hpp` (the `IkSolver` contract), `solvers.hpp`
(`CcdSolver` — a faithful C++ port of the p5 POC's CCD — plus thin
gradient/memetic wrappers), the `pickik` pybind11 binding (FK-pump design),
and `examples/arm7_cross_check`.

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

- Core is stable; all tests pass; nothing pending.
- Next core work (driven from the service side), per
  `docs/integration-roadmap.md` §3.0:
  1. **`pick_ik_c`** — thin C ABI over the `IkSolver` contract (opaque
     handles, POD 4×4 poses, FK as a C function pointer). Entry point for
     Blender (ctypes) and Unity (P/Invoke). No new solver code.
  2. **Shared C++ arm7 model** — extract the C++ FK/joint-table from
     `examples/arm7_cross_check/main.cpp` into `examples/arm7/arm7.hpp` so
     every integration links one model. `arm7.py` (in ik_service) stays the
     Python reference pinned to the spec's anchor poses.
  3. Validation protocol (§3.0c) reused as each integration's acceptance.
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
- Arm7 model: `tests/arm7_fk.hpp` + `examples/arm7_cross_check` are ports of
  `docs/arm7-kinematic-spec.md`. Change the model → update the spec first →
  update all ports (this repo + ik_service's `arm7.py`) → re-run
  cross-checks and anchor tests.
