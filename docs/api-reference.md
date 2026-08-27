# pick_ik API reference

> Durable reference for the PickIK v1.1.3 core, its C++ contract, the
> `pickik` Python binding, the `ik_service` transport, and the p5.js POC
> integration. Keep this file updated whenever the option surface changes —
> it is the file future sessions should consult first.
>
> Companion docs: `integration-roadmap.md` (per-app plan + performance
> report), `../README.md` (threading model + crash post-mortems).
>
> **Status: 2026-07 — Phases 0–3 of the options rollout implemented**
> (all solver options forwarded, `minimal_displacement_weight`,
> `joint_angle_targets`, `look_at`, orientation goals end-to-end incl. the
> quaternion-normalization fix; see §10).

---

## 1. Stack

```
┌──────────────────────────────────────────────────────────────────┐
│  Frontends                                                       │
│  p5.js POC (RobotArm_2026_08_25_10_03_56)   web demo (service/)  │
│  Blender · Unity · PyBullet · IsaacSim · ROS 2   (roadmap)       │
└──────────────┬───────────────────────────────────────────────────┘
               │ HTTP (dev/test transport)          │ in-process / C ABI
┌──────────────▼──────────────┐          ┌──────────▼──────────────┐
│ ik_service (FastAPI, 8081)  │          │ pickik (pybind11)       │
│ POST /solve · /fk · GET …   │          │  + FK GIL pump          │
└──────────────┬──────────────┘          └──────────┬──────────────┘
               └──────────────┬─────────────────────┘
                     ┌────────▼────────┐
                     │ libpick_ik_core │  pick_ik::IkSolver contract
                     │  (this repo)    │  CcdSolver / PickIkGradientSolver
                     │  C++17 + Eigen  │  / PickIkMemeticSolver
                     └─────────────────┘
```

Model-first doctrine: the arm model (ARM7_KINEMATIC_SPEC.md) is the source
of truth; `arm7.py` (service), `arm7_fk.hpp` (tests/examples) and the p5 JS
constants are pinned to it. The service is a **dev/test transport**, never
the production performance path.

---

## 2. C++ contract (`include/pick_ik/`)

### 2.1 `IkSolver::solve`

```cpp
pick_ik::CcdSolver ccd;
auto result = ccd.solve(robot, link_fk, axes, q_seed, {target}, options);
```

| Argument | Type | Meaning |
|---|---|---|
| `robot` | `Robot const&` | n joint specs (min/max, bounded, max velocity, `minimal_displacement_factor`) |
| `link_fk` | `LinkFkFn` | `q → n+1 Isometry3d` frames (n joint child/pivot frames + tip), base frame, meters |
| `local_axes` | `vector<Vector3d>` | joint i's rotation axis in its joint frame (URDF axes) |
| `q_seed` | `vector<double>` | initial configuration, size n |
| `targets` | `vector<Isometry3d>` | target pose(s) in the base frame; **v1: exactly one** |
| `options` | `SolveOptions` | thresholds + cost scaling (§2.2) |

`IkResult`: `success`, `q` (seed on PickIK failure; final config for CCD),
`position_error` [m] (-1 if not computed), `orientation_error` [rad]
(-1 when position-only or on failure).

`IkSolver::spawns_fk_worker_threads()`: false for CCD/gradient (FK runs on
the calling thread), **true for memetic** (gradient-descent exploitation
always runs on native threads — the binding releases the GIL / pumps FK
accordingly; see README "threading model").

Tip-only FK: `make_tip_fk(link_fk)` keeps the last frame.

### 2.2 `SolveOptions` (all fields)

| Field | Default | Meaning |
|---|---|---|
| `position_threshold` | 1e-3 m | position error counting as "solved" |
| `orientation_threshold` | 1e-3 rad (`nullopt` = position-only) | orientation error threshold |
| `cost_threshold` | 1e-3 | solution-test cost threshold [cost units] |
| `position_scale` | 1.0 | position weight in the pose cost |
| `rotation_scale` | 0.5 | orientation weight in the pose cost (0.0 for position-only) |
| `minimal_displacement_weight` | 0.0 | secondary "stay near the seed" objective (§2.4). 0.0 = original PickIK behavior; CCD ignores it |
| `joint_angle_targets` | empty | **Phase 2**: one entry per joint, `nullopt` = no target; cost `Σ (qᵢ − targetᵢ)²` over specified joints, scaled by `joint_target_weight` |
| `joint_target_weight` | 0.0 | weight of the joint-angle-targets goal (0.0 = off; CCD ignores) |
| `look_at` | `nullopt` | **Phase 2**: `LookAtTarget{point, axis}` — point (base frame, m) + tip-frame axis to align with it; cost `(1 − axis_tip·dir)²`, scaled by `look_at_weight` |
| `look_at_weight` | 0.0 | weight of the look-at goal (0.0 = off; CCD ignores) |

### 2.3 Solvers

**`CcdSolver(max_passes=300, damping=0.1, epsilon=1e-8)`** — faithful port
of the p5 POC's `solveCCD` (J7→J1 cyclic coordinate descent, damped,
limit-clamped, FK re-read per joint update). Position-only by design:
chases `targets[0]`'s position, `orientation_error = -1`, no cost-based
search. 300 passes ≈ 1 s of POC runtime, 600 ≈ 2 s.

**`PickIkGradientSolver(GradientIkParams)`** — upstream `ik_gradient`
(local, deterministic):

| Parameter | Default |
|---|---|
| `step_size` | 0.0001 |
| `min_cost_delta` | 1e-12 |
| `max_time` | 0.05 s (service uses 2.0) |
| `max_iterations` | 100 (service uses 2000) |
| `stop_optimization_on_valid_solution` | true |

**`PickIkMemeticSolver(MemeticIkParams)`** — upstream `ik_memetic`
(global, population seeded randomly → not bit-reproducible run to run):

| Parameter | Default | Note |
|---|---|---|
| `elite_size` | 4 | "keep-alive" members; each runs gradient descent |
| `population_size` | 16 | total population |
| `wipeout_fitness_tol` | 1e-5 | reinit threshold |
| `max_generations` | 100 | |
| `max_time` | 1.0 s (service uses 2.0) | |
| `num_threads` | 1 | independent parallel **species** (each a full memetic run + its GD threads) |
| `stop_optimization_on_valid_solution` | true | |
| `stop_on_first_soln` | true | first valid solution vs. best after join |
| `gd_params` | upstream defaults | embedded GradientIkParams — **not yet exposed** in the binding |

### 2.4 Goal layer — secondary objectives

All wired into the PickIK wrappers (Phase 1–2); the CCD solver performs no
cost-based search and ignores every goal:

| Goal | Cost | Option | Status |
|---|---|---|---|
| minimal displacement (upstream `goal.cpp`) | `Σ (q_i − seed_i)² · factor_i` | `minimal_displacement_weight` | **wired** |
| joint angle targets | `Σ (q_i − target_i)²` over specified joints | `joint_angle_targets` + `joint_target_weight` | **wired** (Phase 2) |
| look-at | `(1 − (R_tip·axis)·dir_to_point)²` | `look_at` + `look_at_weight` | **wired** (Phase 2) |
| `make_avoid_joint_limits_cost_fn(robot)` | penalizes proximity to limits | — | available, not yet an option |
| `make_center_joints_cost_fn(robot)` | penalizes deviation from joint midpoints | — | available, not yet an option |
| `make_pose_cost_fn(goal, link_index, pos_scale, rot_scale)` | per-frame pose cost (multi-frame capable) | — | tip frame in v1 |

**Practical guidance (measured):**
* Joint targets on **load-bearing joints** (J2/J3/J4/J6 for this arm) are
  weak constraints: moving them off their position-only optimum costs
  position error, and the strict position threshold then rejects the
  shifted multi-objective minimum (J4 → 0.5 rad at target B failed for
  weights up to 0.3). **Near-null joints** (J1/J5/J7 roll/yaw) track their
  targets almost exactly — the ctest/pytest use J5 +0.5 rad @ weight 0.3
  and land within 0.06 rad (typically < 1e-4).
* Look-at is a soft alignment objective: the residual angle trades off
  against position via the weight. Weight 0.05 at target B kept position
  < 1 mm with ~12° axis residual; raise the weight for tighter alignment.

**Cost-scale note (important):** goals enter the cost as
`goal.eval(q) · w²` and the solution test requires each goal term to stay
below `cost_threshold²`. The squared position cost near convergence is
~1e-6 (meters²), so realistic weights are: `minimal_displacement_weight`
**~1e-3 (tie-breaker) to ~1e-2 (visible anchoring)** (the upstream
plugin's example uses 0.001); `joint_target_weight` **~0.1–0.5** for
visible single-joint pull (the joint residual is O(1) rad, not O(m));
`look_at_weight` **~0.01–0.1** (the alignment residual is O(0–2)). For
secondary objectives far from their natural value, raise `cost_threshold`
(it only widens the goal check) — e.g. the core tests use w = 0.01 /
0.3 / 0.05 with `cost_threshold` 0.05–0.2.

---

## 3. Python binding `pickik`

```python
import pickik

robot = pickik.make_robot([
    # (min, max, bounded, max_velocity) × 7 — see arm7.py
])

fk = lambda q: [...8 4x4 frames (numpy or lists)...]  # joint pivots + tool0

solver = pickik.PickIkMemeticSolver(elite_size=4, population_size=16,
                                    num_threads=4, max_time=2.0)
options = pickik.SolveOptions()
options.minimal_displacement_weight = 0.001
options.joint_angle_targets = [None, None, None, None, 0.5, None, None]
options.joint_target_weight = 0.3
options.look_at = pickik.LookAtTarget([1.5, 0.0, 0.45], [1.0, 0.0, 0.0])
options.look_at_weight = 0.05
result = solver.solve(robot, fk, local_axes, seed, [target_pose], options)
# result: .success, .q, .position_error, .orientation_error
```

* `make_robot(joints)` — plain-list constructor.
* `LookAtTarget(point, axis=[1,0,0])` — `point` in base frame (m),
  `axis` in tip-frame coordinates; read back via `.point` / `.axis`
  (plain `[x, y, z]` lists).

* `make_robot(joints)` — plain-list constructor.
* Solvers: `CcdSolver(max_passes, damping, epsilon)`,
  `PickIkGradientSolver(step_size, min_cost_delta, max_time,
  max_iterations, stop_optimization_on_valid_solution)`,
  `PickIkMemeticSolver(elite_size, population_size, wipeout_fitness_tol,
  max_generations, max_time, num_threads,
  stop_optimization_on_valid_solution, stop_on_first_soln)` — all keyword
  arguments with the §2 defaults.
* FK callable: `q → n+1` 4×4 frames (plain lists or numpy). Thread-safety
  contract: see README — the binding pumps all native-thread FK calls
  through the calling Python thread (GIL held); `num_threads` values are
  all safe, bounded only by CPU cores.

---

## 4. `ik_service` API (FastAPI, port **8081**)

CORS: `*` (the p5 sketch may call from `file://`, Origin: null).

| Endpoint | Body / params |
|---|---|
| `POST /solve` | below |
| `POST /fk` | `{"q": [7 floats]}` → 8 frames + tool0 readout (mm) |
| `GET /solvers` | `{"solvers": ["ccd","gradient","memetic"]}` |
| `GET /health` | status + arm + FK backend |
| `GET /` | web demo page |
| `GET /lib/{filename}` | static assets for the web demo (vendored `p5.js` — no CDN) |
| `GET /model/{path}` | robot description assets (`ik_service/robot_description/`: URDFs + STL meshes), same path-traversal guard as `/lib/*` |

**`/solve` request**

```jsonc
{
  "solver": "memetic",
  "seed":   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   // optional
  "target": {"position": [0.45, 0.25, 0.45],        // meters, required
             "quaternion": [0, 0, 0, 1]},           // optional, [x, y, z, w] — empty/absent = position-only
  "options": { … free-form, everything below … }
}
```

The service **normalizes the quaternion** before building the target
rotation. Rounded user input (e.g. `0.7071` for `1/√2`) would otherwise
yield a slightly non-orthogonal target matrix (~0.006 rad off for
4-decimal input) — unreachable under the default 1e-3 rad orientation
threshold, which made *every* full-pose goal report "no solution" (Phase-0
bug, regression-tested in `tests/test_api.py`).

**`/solve` options (all forwarded to the binding; service defaults in
parentheses)**

| Option | Applies to | Binding default |
|---|---|---|
| `position_threshold` | all | 1e-3 |
| `orientation_threshold` | all | 1e-3 (auto-None when no quaternion) |
| `cost_threshold` | all | 1e-3 |
| `position_scale` / `rotation_scale` | all | 1.0 / 0.5 |
| `minimal_displacement_weight` | gradient, memetic | 0.0 |
| `joint_angle_targets` (7 values, `null` = no target) + `joint_target_weight` | gradient, memetic | off / 0.0 |
| `look_at` `{"point": [x,y,z] m, "axis": [x,y,z] (default +X)}` + `look_at_weight` | gradient, memetic | off / 0.0 |
| `max_passes` / `damping` / `epsilon` | ccd | 300 / 0.1 / 1e-8 (service: 600 / 0.1) |
| `step_size` / `min_cost_delta` / `max_time` / `max_iterations` / `stop_optimization_on_valid_solution` | gradient | 1e-4 / 1e-12 / 2.0 s / 2000 / true |
| `elite_size` / `population_size` / `wipeout_fitness_tol` / `max_generations` / `max_time` / `num_threads` / `stop_optimization_on_valid_solution` / `stop_on_first_soln` | memetic | 4 / 16 / 1e-5 / 100 / 2.0 s / **1** / true / true |

> `num_threads` service default is **1** (binding default is 1): with the
> Python FK on the GIL pump, extra species only add FK traffic — measured
> nt=1/elite≈2 ≈ 55–165 ms vs nt=4 ≈ 280–640 ms on the same targets
> (§7). Native FK hosts should raise it.

**`/solve` response**

```json
{ "success": true, "q": [7 floats], "position_error": 0.0004,
  "orientation_error": -1.0, "time_ms": 132.5, "solver": "memetic" }
```

`orientation_error` is -1 for position-only solves.

**Validation protocol** (used by every integration): spec §5 FK anchors
(zero/yaw → (0,0,1266) mm etc., see `ARM7_KINEMATIC_SPEC.md`) + target A
(300/200/450, "deep fold", pins a joint at its 2.09 rad limit) and target B
(450/250/450, "moderate").

---

## 5. p5.js POC integration (`RobotArm_2026_08_25_10_03_56`)

* `IK_SERVICE_URL = "http://127.0.0.1:8081"` in CONFIG.
* Solver dropdown: `CCD (in-browser, POC)` (per-frame JS CCD, the original
  POC) · `memetic / gradient / ccd (C++ service)`.
* Target X/Y/Z sliders drive the goal position; joint sliders are the seed
  and are mirrored by the solved pose (quantized to 0.01 rad).
* **Options panel** (Phase 0): `threads` 1–4 (default 1), `elite` 1–8
  (default 2), `mem s` 0.1–2.0 (default 0.6) — forwarded in `/solve`
  options; an **orientation field** accepts `x y z w` (empty = position-only).
* **Secondary objectives** (Phase 2): seven **joint target** inputs
  (J1–J7, rad, empty = off) + a **look-at point** (x/y/z in mm, empty =
  off) with a **tip-axis dropdown** (+X/+Y/+Z, default +X), plus weight
  sliders `md w` / `jt w` / `la w` for minimal displacement / joint
  targets / look-at (all default 0 = off).
* Re-solve triggers: target/joint/options/orientation change + 150 ms
  cooldown, one request in flight; status readout shows solver, error (mm
  + ° when orientation is tracked), solve time, service online/offline.
* Vendored p5 v2.3.2 / p5.sound v0.4.1 are git-ignored; fetch URLs in the
  sketch's README.
* **Web demo** (`ik_service/web`, served at `GET /`): same option surface
  as the sketch, plus an interactive **3D view** — p5.js WebGL scene
  (arm chain from the `/fk` frames, tool cross, target ring) with
  drag-orbit / wheel-zoom and **top / front / side / angled** preset
  buttons. p5 v2.3.2 is vendored at `web/lib/p5.js` and served via
  `GET /lib/{filename}` (no CDN dependency).
  The robot is **URDF-driven**: the viewer loads
  `GET /model/{model}.urdf` from `ik_service/robot_description/`
  (default model `arm7`, override via `?model=name`) and places each
  link's visuals (box / sphere / cylinder / cone / STL mesh) at the
  matching `/fk` frame. STLs (binary or ASCII, meters, `scale`
  honored) live in `robot_description/meshes/`; the folder is the
  drop-in point for real mesh files — no viewer or service code
  changes needed (see `robot_description/README.md`). Caveat: the
  URDF drives the *visuals*; the solver's kinematics stay hardcoded
  in the C++ core (`arm7_fk.hpp`) until the core is URDF-driven.
  Note: p5's WebGL world is pixel-scale at the default camera with a
  near-clip plane a few units ahead of the camera — the demo therefore
  draws the model at 300 px/m and keeps the camera at pixel-scale
  distances (the same trick the p5 POC sketch uses); a camera placed a
  couple of "meters" away clips the entire scene. Two more p5 v2.3.2
  quirks the demo works around: this build inverts the camera up
  vector relative to standard GL (world +Z needs up = (0,0,-1)), and
  `stroke()` on closed 3D surfaces renders nothing (the target ring is
  filled instead).

---

## 6. Known limitations (v1)

1. One target pose (tip frame) per solve — multi-frame goals exist in the
   goal layer but the contract takes a single target.
2. CCD is position-only (POC parity); no cost, no orientation.
3. Memetic `gd_params` (embedded gradient-exploitation knobs) are not
   exposed in the binding yet.
4. Memetic results are not bit-reproducible (random population seed).
5. `minimal_displacement_weight` acts on all joints uniformly
   (`minimal_displacement_factor` per joint exists in `Robot` but the
   service builds specs with factor 1).
6. The service serializes all solving on one worker (one concurrent solve
   in flight is the normal operating mode; the p5 sketch enforces it).
7. Full-pose goals on the hard "deep fold" target (A) can sit at the edge
   of the search basin: e.g. target A + orientation is reachable (a 20 s
   memetic lands within ~4 cm / 2°), but the strict 1 mm / 0.6° solve is
   not reached within 20 s from the quantized-zero seed. Moderate target B
   + orientation solves in ~0.3–2 s. Raise `max_time`/`max_generations`
   or seed from a nearby pose for the hard cases.
8. Stacking several secondary objectives (e.g. minimal displacement +
   joint target + look-at at once) can exceed what a short memetic budget
   finds — the objectives are soft cost terms, and the strict
   position/orientation test still gates success. Use the gradient solver
   or a larger budget when combining them.

---

## 7. Performance summary (measured)

Environment: Windows 11 build 26200, 12 logical procs, MSVC 14.44
RelWithDebInfo, Python 3.12.28 (MS Store), numpy 1.26.4. Details and
decomposition in `integration-roadmap.md` §2.

| Host path | CCD | Gradient | Memetic |
|---|---|---|---|
| **Native C++ FK** (in-process, e.g. Unity/ROS) | ~2 ms | ~0.3–1 ms | ~3–25 ms |
| **Python FK, in-process** (Blender/PyBullet/IsaacSim) | ~200–300 ms | ~10–50 ms | ~150–500 ms |
| **`ik_service` (HTTP + Python FK)** | 235–345 ms | ~11 ms | 55–640 ms (option-dependent) |

Rule of thumb: every **1000 FK calls ≈ 50–70 ms** on a Python FK vs
**~0.5–1 ms** native; the solver math itself is ~2 ms (CCD).

**Memetic option sweep (Phase 0, through the service, 5 repeats/point,
median, position-only, quantized-zero seed):**

| target | nt | elite | success | median solve | max pos err |
|---|---|---|---|---|---|
| A (deep fold) | 1 | 1 | 5/5 | 109 ms | 0.99 mm |
| A | 1 | 2 | 5/5 | 163 ms | 0.90 mm |
| A | 1 | 4 | 5/5 | 116 ms | 0.07 mm |
| A | 1 | 8 | 5/5 | 126 ms | 0.51 mm |
| A | 2 | 4 | 5/5 | 134 ms | 0.80 mm |
| A | 4 | 4 | 5/5 | 284 ms | 0.58 mm |
| B (moderate) | 1 | 1 | 5/5 | 56 ms | 0.05 mm |
| B | 1 | 2 | 5/5 | 58 ms | 0.44 mm |
| B | 1 | 4 | 5/5 | 117 ms | 0.60 mm |
| B | 1 | 8 | 5/5 | 188 ms | 0.95 mm |
| B | 2 | 4 | 5/5 | 185 ms | 0.21 mm |
| B | 4 | 4 | 5/5 | 361 ms | 0.67 mm |

(Full 3×4×2 grid in `.tmp/elite_sweep.json` at capture time.)

Readings:
* All 24/24 combinations succeed; quality (err) varies with elite,
  speed with elite size and — inversely — with `num_threads`.
* **With a Python FK, `num_threads > 1` is counterproductive** (FKs are
  serialized on the GIL pump): B goes 56 ms (nt=1, elite=1) → 361 ms
  (nt=4, elite=4). Keep species parallelism for native FK hosts.
* Small elite (1–2) is the interactive sweet spot; elite 4 gives the
  tightest position error on hard targets.

**Secondary-objective overhead (Phase 3, through the service, target B,
3 repeats, median):**

| Solve | median | note |
|---|---|---|
| gradient plain | 17 ms | |
| gradient + minimal displacement (w 0.01) | 17 ms | no extra FK — arithmetic goal |
| gradient + joint target (J5, w 0.3) | 18 ms | no extra FK — arithmetic goal |
| gradient + look-at (+X, w 0.05) | 126 ms | +1 FK per cost evaluation |
| memetic plain (nt=1, elite=2) | 57 ms | |
| memetic + minimal displacement (w 0.01) | 56 ms | no extra FK |

---

## 8. Threading invariants (do not regress)

* No CPython GIL API / no refcount operations on native solver threads;
  FK callbacks are pumped through the calling thread (borrowed
  `PyObject*` only; the pump thread holds the GIL for the whole solve).
* `std::mutex` for cross-thread coordination — the C-level
  `PyThread_acquire_lock` is broken on MS Store Python 3.12.x.
* Upstream `ik_gradient.cpp` / `ik_memetic.cpp` / `solver.cpp` stay
  byte-identical; only the binding + contract headers evolve.
* The service never gains solver code; all solving happens in
  `libpick_ik_core`.

---

## 9. Verification matrix (last green run)

* Core ctest: **26 cases / 266 assertions** (incl. Phase 1
  `minimal_displacement_weight`, Phase 2 joint-angle-targets + look-at
  cases, wrong-size target rejection).
* Service pytest: **41 tests** (FK anchors, binding surface, API, option
  forwarding, orientation goals incl. the quaternion-normalization
  regression + zero-quaternion rejection, joint-target + look-at
  pass-through/effect + 422 validation).
* Crash regression: memetic storm matrix 0 bad across ~40 runs; API burst
  0/10; browser demo 0.2 mm / ~330 ms.
* p5 sketch end-to-end (Phase 3): target B + rounded yaw-90 quaternion
  `memetic OK 0.83 mm +0.04° in 593 ms`; J5 joint target from the sketch
  `gradient OK 0.82 mm in 24 ms` (q5 = 0.4942 vs target 0.4935); look-at
  point (1500, 0, 450) mm with the +Z axis dropdown `gradient OK 0.13 mm
  in 852 ms` (tip +Z · dir = 0.98).
* Web demo (Phase 3): options mirror verified live (J5 target 0.49 rad →
  `gradient OK 0.83 mm in 23 ms`, q5 = 28.07° ≈ 0.49 rad); interactive
  3D view verified (drag-orbit/zoom/presets, arm follows solves —
  tool0 lands at target in the scene, `/lib/p5.js` served).

---

## 10. Options rollout status

| Phase | Content | Status |
|---|---|---|
| 0 | Forward every existing solver option through the service; p5 options panel + orientation input; elite × threads sweep; **quaternion normalization fix** (service + sketch client) | ✅ done (this file's §4–§7) |
| 1 | `SolveOptions.minimal_displacement_weight` (contract → binding → service → sketch); ctest + pytest | ✅ done |
| 2 | Per-joint angle targets (`joint_angle_targets` + weight) + look-at goal (`look_at` + weight) as `Goal` costs in the wrapper; binding `LookAtTarget`; ctest + pytest | ✅ done |
| 3 | Re-measure perf with new options; mirror options panel in the web demo + sketch secondary-objective UI; update docs | ✅ done |

---

## 11. Build & run cheatsheet

```powershell
# binding (from ik_service)
cmd /c '"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" && cmake --build build --target pickik --config RelWithDebInfo'
Copy-Item build\pick_ik_core\bindings\python\RelWithDebInfo\pickik.cp312-win_amd64.pyd(.pdb) dist\   # service must be stopped (file lock)

# core tests
cmd /c '...vcvars64.bat && cmake --build ..\libpick_ik_core\build --target test-pick_ik_core --config RelWithDebInfo'
libpick_ik_core\build\tests\RelWithDebInfo\test-pick_ik_core.exe

# service (port 8081; stop any running instance before restarting)
python -m service.main        # from ik_service

# native timings reference
libpick_ik_core\build\examples\arm7_cross_check\RelWithDebInfo\arm7_cross_check.exe
```
