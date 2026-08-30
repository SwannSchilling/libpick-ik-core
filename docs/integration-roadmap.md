# PickIK Integration Roadmap & Performance Report

Status: 2026-07. Covers where the stack is today, what each integration
target (Blender, Unity, PyBullet, IsaacSim, ROS 2/robot, browser/p5) will
look like, and a measured performance report answering *"what speeds do we
get with a native integration of the pick_ik API instead of the service?"*

Companion: **`api-reference.md`** — the durable option/API reference
(SolveOptions, all solver parameters, binding + service option tables,
p5 integration, threading invariants, build cheatsheet). Options-rollout
Phases 0–3 (forwarding every solver option, `minimal_displacement_weight`,
`joint_angle_targets`, `look_at`, orientation goals end-to-end, web-demo
mirror) are implemented; see `api-reference.md` §10.

## 1. What we have today

```
p5.js sketch / browser / Python / Blender / Unity / PyBullet / IsaacSim / robot
        │  (per-app frontends, see §3)
        ▼
   libpick_ik_core (C++17, this repo) — ALL solver code lives here
   CcdSolver · PickIkGradientSolver · PickIkMemeticSolver  (IkSolver contract)
   upstream ik_gradient.cpp / ik_memetic.cpp / solver.cpp: byte-identical to PickIK v1.1.3
        │
        ├── pickik  — pybind11 binding (optional, FK-pump threading; see README)
        ├── ik_service/ — FastAPI dev/test transport + web demo (port 8081)
        └── C ABI layer (planned, §3.0 a) — for Unity / native hosts
```

- Model spec: `ARM7_KINEMATIC_SPEC.md` (workspace root) — the URDF constants
  are the design spec; every FK port (Python in `ik_service/service/arm7.py`,
  C++ in `examples/arm7_cross_check/main.cpp`, JS in the p5 sketch) is pinned
  to its §5 anchor poses by tests.
- **The service's role (short answer: yes, it is the debugging tool).**
  `ik_service` is a dev/test/debug *transport* and a reference client. It
  contains no solver code and is not on any production performance path.
  Every product integration below embeds the C++ core (natively, or
  in-process via the binding) and uses the service for side-by-side
  cross-checking.

## 2. Performance report

### 2.1 Environment

- Windows 11 (build 26200), 12 logical processors.
- MSVC 14.44, **RelWithDebInfo** (O2 + debug info).
- Python 3.12.28 (MS Store), numpy 1.26.4.
- Targets: **A** "deep fold" 300/200/450 mm (every solution pins J4/J6 at the
  2.09 rad limit) and **B** "moderate" 450/250/450 mm — the p5 POC's own
  cross-check targets. Position-only goals, the POC's quantized "all zero"
  seed. Memetic runs with `num_threads = 4` (4 independent parallel species)
  everywhere below.

### 2.2 Measured numbers

| Host path | FK | CCD (600 passes) | Gradient (2 s / 2000 it) | Memetic (4 species) |
|---|---|---|---|---|
| p5 sketch, browser | JS, in-page | ~35 FK calls/frame, iterative: converges, but stalls **~15–35 mm short** on target A — an artifact of the p5 sliders' 0.01 rad quantization, not the CCD math | — | — |
| `ik_service` (HTTP + Python/numpy FK through the GIL pump) | Python, ~49 µs/call measured | **235–345 ms** | **~11 ms** | **55–640 ms** (option-dependent; 4-species baseline 285–490 ms) |
| Native C++ host (`arm7_cross_check` Part 4, C++ FK) | C++, ~0.5 µs/call | **1.9–2.3 ms** | **0.3–0.6 ms** | **2.7–22.6 ms** |

> Phase 0 note: the service's default `num_threads` changed from 4 to 1
> after the §2.5 sweep (with a Python FK, extra species only add pump
> traffic). The single-species service memetic baseline on the same
> targets is ~55–190 ms at elite 1–4.

Supporting measurements:

- One Python `arm7.fk_callback` call, in-process (numpy): **~48–49 µs**.
  Through the service's FK pump (native thread posts a request → GIL owner
  runs the FK → thread is woken) it becomes ~60–70 µs/call end to end.
- CCD does up to 600 passes × 7 joints ≈ **4200 FK calls**. Service CCD
  time ÷ calls ≈ 65–80 µs/call → **the FK language boundary is ~100 % of the
  CCD service time**; the solver math itself is ~2 ms.
- Memetic at 4 species: the run stops at the *first* solution found, so
  native wall time ≈ the fastest species (~3–23 ms). Under the Python pump,
  every FK call from every species/gradient-descent thread is serialized
  through one Python thread (the GIL): thousands of calls × ~65 µs
  ≈ 0.3–0.5 s.

### 2.3 Where the service time goes (memetic, ~350 ms)

| Chunk | Time |
|---|---|
| FK evaluation in Python (numpy) + pump queue round-trips | ~0.3–0.5 s (**dominant**) |
| Solver computation itself (native, parallel across species) | ~1–5 ms |
| HTTP + JSON round-trip (measured wall − `time_ms`) | ~50–150 ms |

The solver math is never the bottleneck at these DOF counts; the FK's
language boundary is.

### 2.4 What to expect from native integration of the pick_ik API

| Host | Expected solve time (A/B targets) | What that enables |
|---|---|---|
| **Native C++ host** (Unity via C ABI, Blender with native FK, ROS 2 node, batch tools) | CCD **~2 ms** · gradient **~0.3–1 ms** · memetic(4) **~3–25 ms** | CCD/gradient re-solved **every frame at 60–120 Hz**; memetic at ~40 Hz or on demand |
| **In-process Python host, FK stays Python** (Blender/PyBullet/IsaacSim via `pickik` binding) | CCD **~200–300 ms** · memetic **~300–500 ms** (≈ service numbers minus the HTTP chunk) | event-driven solves (target changed, "recover" button); not per-frame |
| **Browser** (p5 sketch, web demo — service-backed) | ≈ service numbers | event-driven solves, which is exactly how the UIs use them |

Rule of thumb: **every 1000 FK calls cost ~50–70 ms with a Python FK and
~0.5–1 ms with a native FK.** To get millisecond-class times from a
Python-hosted app, the FK has to move into C++ (a compiled arm7 model,
§3.0 b) — the solver API stays the same either way.

### 2.5 Memetic option sweep (elite_size × num_threads, Phase 0)

All solver options are now forwardable through the service and the p5
sketch (§3.5), so a sweep of `elite_size` × `num_threads` was measured
through the live service (5 repeats per point, median; position-only,
quantized-zero seed, `max_time` 2 s). Key points of the 3×4×2 grid:

| target | nt | elite | success | median solve | max pos err |
|---|---|---|---|---|---|
| A | 1 | 1 | 5/5 | 109 ms | 0.99 mm |
| A | 1 | 2 | 5/5 | 163 ms | 0.90 mm |
| A | 1 | 4 | 5/5 | 116 ms | 0.07 mm |
| A | 4 | 4 | 5/5 | 284 ms | 0.58 mm |
| B | 1 | 1 | 5/5 | 56 ms | 0.05 mm |
| B | 1 | 2 | 5/5 | 58 ms | 0.44 mm |
| B | 1 | 4 | 5/5 | 117 ms | 0.60 mm |
| B | 4 | 4 | 5/5 | 361 ms | 0.67 mm |

Readings:

1. **24/24 combinations succeed** — quality (position error) tracks
   `elite_size` (more elites → more gradient exploitation → tighter
   error), speed tracks it inversely, and `stop_on_first_soln` makes the
   fastest species win the race.
2. **With a Python FK, `num_threads > 1` is counterproductive**: every
   species' FK calls serialize through the GIL pump, so extra species add
   queue traffic instead of parallelism (B: 56 ms at nt=1 → 361 ms at
   nt=4, same elite). The service default was therefore changed from 4 to
   1; **native FK hosts (Unity, ROS 2, batch) should keep raising it.**
3. Interactive sweet spot: **nt=1, elite 1–2** (~55–165 ms, well under a
   frame budget if needed, plenty of margin for the sketch's 150 ms
   cooldown). Elite 4 gives the tightest error on hard targets.

Full grid captured at `URDF_BIO_IK/.tmp/elite_sweep.json` (transient);
the option tables live in `api-reference.md` §4/§7.

## 3. Integration roadmap

### 3.0 Shared building blocks (do once, use everywhere)

- **a. C ABI layer (`pick_ik_c`) — done (2026-08).** Thin C interface over
  the `IkSolver` contract (`include/pick_ik_c/pickik_c.h`,
  `src/pickik_c.cpp`, built as the `pick_ik_c` shared library, ON by
  default). Opaque robot/solver handles, POD options + result structs, FK
  as a C function pointer, **and the arm7 model compiled in**
  (`pickik_arm7_robot_create` / `pickik_arm7_link_fk`) so hosts get native
  ms-class FK with no callbacks of their own. Conventions: meters/radians;
  poses are standard row-major homogeneous 4×4 (`[R | t; 0 0 0 1]`,
  translation in column 3); position-only goals use `orientation_threshold
  < 0` and the default `rotation_scale = 0.0` (the stack-wide convention —
  matches the service's `/solve` and the C++ `pos_only` test helper).
  Handles are thread-safe and re-entrant; free each handle with its
  matching `*_free`.
- **b. One shared C++ arm7 model — done (2026-08).**
  `examples/arm7/arm7.hpp`: the Design B joint table (limits/velocities
  included), FK (`link_frames` / `tool0_pose`), `make_fk` / `make_link_fk`
  / `make_local_axes`, and `make_robot` / `joint_specs` for
  `pick_ik::Robot::make`. Every C++ consumer links it: `arm7_cross_check`,
  the ctest ports (`arm7_fk.hpp` facade, `solver_tests`,
  `ik_tests_core`, `robot_tests_core`), and `pick_ik_c`. `arm7.py` stays
  the Python reference pinned to the spec's anchor poses.
  `ARM7_KINEMATIC_SPEC.md` remains the single source of truth.
- **c. The validation protocol — done (2026-08).** `tests/c_abi_tests.cpp`
  (33-test ctotal suite) encodes it through the C boundary exactly as a
  native host would: spec §5 anchor poses through the built-in C FK, the
  Design B cross-check targets A/B through all three solvers, the
  out-of-workspace case (clean `success = 0`), joint-limit validity, the
  secondary-options plumbing, and a host-supplied FK-callback path.
  Each host re-runs the same gates against its own path (Blender: the
  add-on's headless acceptance script).
- **b. One shared C++ arm7 model** — today the arm7 joint table + FK +
  limits exist in three ports (Python `arm7.py`, C++ in
  `arm7_cross_check/main.cpp`, JS in the p5 sketch). Extract the C++ one
  into a reusable header (`examples/arm7/arm7.hpp`) that every integration
  links against; `arm7.py` stays the Python reference pinned to the spec's
  anchor poses. `ARM7_KINEMATIC_SPEC.md` remains the single source of
  truth.
- **c. The validation protocol** — reuse the existing gates as each
  integration's acceptance test: FK anchor cross-check (spec §5), IK
  cross-check on targets A/B plus the known out-of-workspace case, joint
  limit sweeps. The native ctest suite and the service pytest already encode
  these; each host just re-runs them against its own FK/solve path.

### 3.1 Blender add-on — *done (2026-08-30, v1)*

- Own repo: [`SwannSchilling/blender_ik_addon`](https://github.com/SwannSchilling/blender_ik_addon)
  (BSD-3, first commit `fe3c162`, 2026-08-30): `__init__.py` (Blender 3.4+
  add-on: rig builder, target empty, solver dropdown, Solve, continuous
  timer, status readout), `ik_core.py` (ctypes wrapper, DLL auto-discovery),
  `arm7_rig.py` (joint table + empty hierarchy), `test_acceptance.py`
  (headless, 5 gates).
- **Acceptance passed on Blender 4.5.3 headless** (all 5 gates, 2026-08-30):
  spec §5 anchors through the rig FK and the C ABI FK (worst 1e-7 m),
  target B via gradient (0.68 mm), target A via memetic **on a background
  thread** (0.8 µm, 56 ms), the out-of-workspace case (clean no-solution),
  main-thread stall budget (gradient p90 ~2.1 ms, CCD p90 ~3.8 ms).
- The add-on's CCD default is 100 passes (~2.2 ms), not the C-ABI 600
  (~15 ms): CCD is local, 100 passes converges from a nearby seed; the
  C default stays the conservative upper bound.
- **Plugin-ABI runtime fix** (core side): `pick_ik_c` now builds from
  `pick_ik_core_plugin` with the static MSVC CRT (`/MT`). Blender pins its
  own older `MSVCP140.dll` for the whole process (blender.crt, 14.29 in
  4.5); thread-waiting into the host's runtime dereferenced null in the
  memetic harvest (crash only inside Blender). The static CRT makes the
  DLL's thread primitives self-contained — also the shape Unity wants.

- Shape: Blender 4.x Python add-on that loads the `pick_ik_c` shared library
  via ctypes (§3.0 a) and drives the shared arm7 FK. UI mirrors the p5
  sketch: target gizmo (an empty you move), solver dropdown
  (CCD / gradient / memetic), current pose as seed, "Solve" button plus an
  optional "continuous" mode on a background job.
- Performance (§2.4, native): CCD ~2 ms → continuous tracking at 60 fps is
  comfortable; memetic ~25 ms → use it as the "recover / global" action
  (or run it in the background and snap the result).
- Acceptance: spec anchors + targets A/B reproduced in Blender; no
  main-thread stall > 4 ms.

### 3.2 Unity (native)

- Shape: `pick_ik_c.dll` + C# P/Invoke wrapper. Solving runs on a C++ worker
  thread — never on Unity's main thread; Unity posts targets
  (slider/gizmo/UI) into a double-buffered request and pulls the resulting
  `q` into the joint transforms at frame start.
- Performance: CCD ~2 ms → per-frame IK at any frame rate; memetic ~25 ms →
  every 2–4 frames, or background + pose blending.
- Acceptance: the §3.0 c protocol; sustained 60/120 fps with CCD tracking.

### 3.3 PyBullet / IsaacSim (simulation)

- Shape: in-process `pickik` binding (both are Python hosts) or the C ABI;
  apply solved `q` to the articulation (`setJointMotorControlVector` /
  Isaac articulation APIs).
- Performance (§2.4, Python FK in-process): CCD ~200–300 ms, memetic
  ~300–500 ms → **event-driven** (teleop target updates, recovery from
  singular/far poses), not per-physics-step. For a closed-loop tracking
  simulation use the C ABI + native FK (§2.4 first row): ms-class.
- Acceptance: FK parity with the simulator's own FK < 0.1 mm.

### 3.4 ROS 2 / the real robot (PickIK's original home)

- Shape: a thin ROS 2 *adapter* package around this core (the core stays
  ROS-free): `FkFn` from the URDF (KDL or the shared C++ arm7 model), a
  `solve_once` service/action plus an optional rate-limited continuous
  node, `num_threads`/`max_time` exposed as parameters — the same shape the
  upstream PickIK plugin had with `generate_parameter_library`.
- This is "Step 8: ROS adapter" from the original project plan.
- Acceptance: parity with the upstream PickIK plugin on the same
  model/targets (byte-identical solvers guarantee algorithmic parity; the
  adapter must not alter the cost function).

### 3.5 Browser / p5 — *done*

- p5 POC sketch: solver dropdown (in-browser CCD + the three service
  solvers), interactive IK target sliders, joint sliders act as the seed,
  live status/error readout, service health indicator.
- Options panel (Phase 0): `threads`/`elite`/`mem s` sliders forwarded to
  the service; orientation field (`x y z w`) for full-pose goals; status
  shows position + orientation error. Defaults follow the §2.5 sweep
  (nt=1, elite=2).
- `minimal_displacement_weight` (Phase 1) available for all service
  solvers — the tool to test the "arm jumps to a random pose" fix.
- Secondary objectives (Phase 2/3): per-joint angle targets (J1–J7
  inputs, rad) + look-at point (x/y/z mm) with a tip-axis dropdown
  (+X/+Y/+Z), each with its own weight slider — all off by default.
- Web demo (`ik_service/web`): solver dropdown + interactive 3D view
  (p5.js WebGL, drag-orbit / wheel-zoom / top–front–side–angled presets,
  vendored p5.js served from the service) + FK readout; options mirrored
  (memetic threads/elite/time, all three secondary-objective weights,
  joint targets, look-at + axis). The 3D view is URDF-driven: the robot
  is loaded from `ik_service/robot_description/` (`GET /model/*`,
  default `arm7`, `?model=name` to switch; STL meshes under
  `robot_description/meshes/`).
- Both are service consumers: performance ≈ §2.2 row 2, which is fine for
  event-driven UI solves.

### 3.6 Offline / CAD batch tools

- Self-collision sweep + workspace map (from the original plan): a native
  batch tool (core + shared arm7 header + collision model) that sweeps the
  joint/target workspace, runs FK (+ optionally IK) per sample, and maps
  self-collision regions and the reachable workspace. Native-only —
  millions of FKs at ~0.5 µs make this practical; a Python loop would
  take hours for what runs in minutes.

### Suggested order (rough effort)

1. Shared blocks a + b + c — 1–2 days
2. Blender add-on — 2–3 days (incl. UX)
3. Unity — 2–3 days
4. ROS 2 adapter — 2–4 days
5. PyBullet / IsaacSim — 1–2 days each (mostly reuses the above)
6. Batch collision/workspace tool — 2–3 days

## 4. Invariants that must not regress

- `ik_gradient.cpp` / `ik_memetic.cpp` / `solver.cpp` remain byte-identical
  to upstream; everything is composed through the `IkSolver` contract.
- No CPython GIL API and no Python refcount operations on native threads
  (binding invariants — README § "Python binding threading model").
- Model = `ARM7_KINEMATIC_SPEC.md`; every FK port is pinned by the anchor
  tests, and the p5 POC's targets stay the common cross-check fixture.
- `ik_service` remains a transport: it must never gain solver code.
