# Desktop 7-DOF Arm — Design Study (rescale of the arm7 POC)

> **Status: design study only — no constants have been changed yet.** Per the
> design brief this document is the review gate before touching C++ FK,
> Python, URDF, viewer, or tests.
>
> Companion docs: `arm7-kinematic-spec.md` (current POC spec, single source
> of truth), `integration-roadmap.md`. Service-side state:
> `ik-service/HANDOVER.md`.

## 0. TL;DR — the 12 recommendations

1. **Target maximum straight-chain reach: 675 mm (Design B)** — middle of
   the desired 650–700 mm window. Design C (695 mm) gives ~6 % worse
   payload capacity at the binding joint for no benefit; Design A (633 mm)
   is *below* the stated window (it would win ~16 % payload — only relevant
   if the window is relaxed).
2. Base → J2 (shoulder): **180 mm**.
3. J2 → J4 (upper arm): **215 mm**.
4. J4 → J6 (forearm): **215 mm**.
5. J6 → tool (wrist/tool): **65 mm**.
6. **Actuator assignment:** J2 = CubeMars AK10-9 **V2.0 KV60** (given);
   J4 = CubeMars AK70-10 (given, revision to verify, §10); J1/J3/J5/J6/J7
   **parametric, not yet chosen** — proposed fill-in: J1 ≥ AK70-class
   (stiff base yaw), J3/J5 AK70-class, J6/J7 smaller joints (≤ AK60-class).
   The payload conclusion in §6 depends only on the two *known* actuators
   (J2, J4), so this does not block the decision.
7. **Mass budget (base case, B):** arm ≈ 3.2 kg + base 2.5–3.0 kg ≈
   **5.5–6.0 kg total on the desk** (4.5–5.0 kg without base) — §4.
8. **Static torque (B, worst-case horizontal pose):** J2 arm-only ≈ 6.7 Nm;
   J4 ≈ 1.9 Nm; J6 ≈ 0.06 Nm. Per kg of payload: J2 +4.86 Nm, J4 +2.75 Nm,
   J6 +0.64 Nm. J2 (shoulder) is the binding joint for all three designs.
9. **Payload range (B):** ≈ **1.0 kg continuous** at SF 1.5 on rated
   (18 Nm AK10); ≈ 2.3 kg at rated with no safety factor; ≈ 5 kg in
   short-duration peak bursts (48 Nm peak). Consistent with the reBot-DevArm
   data point (650 mm 6-DOF: 1.5+ kg, ~4 kg arm).
10. **Margin assumptions:** SF 1.5 on *rated* torque for
    continuous/quasi-static operation (desktop, smooth profiles, low speed).
    Dynamics at α = 3 rad/s² add ~1.4 Nm (1 kg payload, B) — covered;
    re-check for α > ~8 rad/s². No-load speed (16.75 rad/s @ 24 V) is far
    above the POC's 2.17–2.61 rad/s velocity limits — speed is not a
    constraint.
11. **Keep the 7-DOF frame convention unchanged** (coincident pitch/roll
    pairs, all local z axes, ±90° frame rotations, zero pose = arm up). It
    is consistent across spec, URDF, C++/Python/JS ports, and IK; the
    Franka/Gen3 differences (lateral roll-axis placement, sunk pitch axes)
    are mechanical packaging, not kinematic requirements.
12. **No joint-axis offsets to redesign now.** One kinematically-neutral
    option for the mechanical pass: roll-joint origins may slide along
    their own (vertical) axis to match where the motor housings physically
    sit — zero effect on FK/IK/limits. In-plane lateral offsets
    (Franka-style ±82.5 mm) would *change* the kinematics and are not
    needed.

## 1. Scope and method

Inputs: the design brief (candidates A/B/C, actuator data), the POC code
(`ik-service/service/arm7.py` + `arm7-kinematic-spec.md`, both read and
re-verified this session), reference architectures extracted **this session**
from public GitHub URDFs via DeepWiki, and the brief's class-level reference
data (FR3, xArm7, Gen3).

| Source | Status this session |
|---|---|
| Franka Panda/FR3 xacro (frankarobotics/franka_ros) | **verified** — joint origins extracted from source |
| Kinova Gen3 URDF (applied-ai-lab/compliant_controllers) | **verified** — full joint table extracted |
| reBot-DevArm specs (Seeed-Projects/reBot-DevArm) | **verified** — spec sheet data |
| xArm7 URDF | **not found** — no DeepWiki-indexed public repo; search down → class-level reference only (flagged, §3.4) |
| CubeMars AK70-10 exact revision | **unverified** — search engines down; values from the brief (flagged, §2.2) |
| FR3 payload/reach | brief's values; consistent with the verified xacro geometry |

All structural masses, fill-in actuator masses, safety factor, and
acceleration are **explicit assumptions** (§2.3), parameterized so the model
re-runs in one line each when CAD/BOM values land.

## 2. Inputs

### 2.1 Design candidates (from the brief)

| | A compact | **B middle** | C larger |
|---|---|---|---|
| Base → J2 | 170 | **180** | 185 |
| J2 → J4 | 200 | **215** | 220 |
| J4 → J6 | 200 | **215** | 220 |
| J6 → tool | 63 | **65** | 70 |
| **Straight-chain total** | **633** | **675** | **695** |
| J2 → tool (distal chain) | 463 | **495** | 510 |

### 2.2 Actuators

| Joint | Actuator | Mass | Envelope | Rated / peak torque | Status |
|---|---|---|---|---|---|
| J2 (shoulder pitch) | CubeMars AK10-9 **V2.0** KV60, 9:1, 160 rpm no-load @ 24 V / 320 @ 48 V, integrated driver, dual encoder | **0.960 kg** | **Ø98 × 61.7 mm** | **18 / 48 Nm** | given (do not confuse with V3.0) |
| J4 (elbow pitch) | CubeMars AK70-10, 10:1 | **0.521 kg** | **Ø89 × 50.25 mm** | **8.3 / 24.8 Nm** | from brief — **verify revision** |
| J1, J3, J5, J6, J7 | — parameterized `m_Ji`, `τ_r,i`, `τ_p,i` — | see fill-in | — | — | **open** |

Fill-in used for the base case (assumption, replace when chosen):
`m_J1 = m_J3 = m_J5 = 0.521 kg` (AK70-class), `m_J6 = m_J7 = 0.30 kg`
(smaller joint). The J6 check in §6 uses a 4 Nm rated placeholder.

### 2.3 Assumptions (all explicit — replace with CAD values)

- g = 9.81 m/s². Worst-case gravity pose: distal chain **horizontal**,
  fully extended forward (max lever arms for every pitch joint).
- Structure: upper arm `m_u = 0.40 kg` (COM at L2/2), forearm
  `m_f = 0.35 kg` (COM at L3/2), wrist housing `m_w = 0.20 kg`
  (COM at tool/2). Cross-sections ~Ø50–60 mm aluminum/steel tube or
  carbon; wiring allowance +0.1–0.2 kg per link included in these.
- Base (below J2): 2.5–3.0 kg; its horizontal lever arm at J2 is small and
  excluded from τ_J2 (checked separately for tip-over, §8).
- Actuator COMs on or within ~20 mm of their axes (eccentricity neglected).
- **Safety factor 1.5 on rated torque** for the continuous payload claim
  (quasi-static desktop operation, smooth acceleration profiles).
- Dynamics check at α = 3 rad/s² (modest); flagged for faster motion.

## 3. Reference architectures

### 3.1 Franka Panda / FR3 (xacro verified this session)

Chain (joint origins from `franka_arm.xacro`): j1 z **0.333** (shoulder
height) · j3 y **0.316** (upper arm) · j4 x **0.0825** · j5
(−0.0825, **0.384**, 0) (forearm, lateral return) · j7 x **0.088** · fixed
j8 z **0.107** (flange).

Lessons:
- Pitch joints are **not all on one line**: the ±82.5 mm lateral offsets
  place the roll axes along the links and shape the wrist. j5/j6 are
  coincident (a wrist center).
- All axes are local z with frame rotations — **same convention as our POC**.
- Spec reach (855 mm) is far below the ~1.32 m chain sum: **never compare
  chain sums to spec reach**; use the manufacturer's workspace definition.

### 3.2 Kinova Gen3 (URDF verified this session)

| joint | origin xyz (m) | type | role |
|---|---|---|---|
| j1 | 0 0 **0.15643** | continuous | base yaw |
| j2 | 0 0.005375 **−0.12838** | revolute | shoulder pitch (axis **sunk 128 mm** into shoulder) |
| j3 | 0 **−0.21038** −0.006375 | continuous | upper-arm roll (half-arm split) |
| j4 | 0 0.006375 **−0.21038** | revolute | elbow pitch |
| j5 | 0 **−0.20843** −0.006375 | continuous | forearm roll |
| j6 | 0 0.00017505 **−0.10593** | revolute | wrist pitch |
| j7 | 0 **−0.10593** −0.00017505 | continuous | wrist roll |

Lessons: pitch-axis spacings **210 / 208 / 106 mm** for ~900 mm reach —
a 700 mm-class arm lands at ~150–190 / 180–210 / 80–110, consistent with
candidates A–C. Proximal joints (j1–j4) carry the high-torque actuators,
distal (j5–j7) the small ones; roll joints are continuous (unlimited).
The sunk shoulder axis and half-arm split are **packaging** decisions, not
kinematic ones.

### 3.3 reBot-DevArm (specs verified this session)

6-DOF + gripper, **650 mm reach, 1.5+ kg payload, ~4.0 kg arm, 24 V** —
a real desktop-scale data point: a 650 mm 6-DOF arm at ~4 kg moves 1.5+ kg.
Our Design B (675 mm, ~4.5–5.0 kg excluding base, ~1.0 kg continuous at
SF 1.5) is in the same class. (Full URDF/STEP planned by upstream; joint
origins not yet extractable.)

### 3.4 xArm7

**No public URDF was reachable this session** (no DeepWiki-indexed repo
found; search down). Used strictly as the brief's "700 mm-class 7-DOF"
scale reference. If a public model appears, extract its joint origins and
confirm the pitch spacings match the Gen3/Franka class.

### 3.5 FR3 (brief data; consistent with the verified xacro above)

855 mm, links 333/316/384 — the "too big" reference; out of scope for the
desktop target.

## 4. Current POC convention (analysis of `service/arm7.py` + spec)

Verified constants (as of this study):

| joint | origin z | frame roll | limits | max vel |
|---|---|---|---|---|
| J1 base yaw | 0.000 | 0 | ±π | 2.17 rad/s |
| J2 shoulder pitch | **0.340** | −π/2 | ±2.09 | 2.17 |
| J3 shoulder roll | 0.000 | +π/2 | ±π | 2.17 |
| J4 elbow pitch | **0.400** | −π/2 | ±2.09 | 2.17 |
| J5 forearm roll | 0.000 | +π/2 | ±π | 2.61 |
| J6 wrist pitch | **0.400** | −π/2 | ±2.09 | 2.61 |
| J7 tool roll | 0.000 | +π/2 | ±π | 2.61 |
| tool0 | **0.126** (fixed) | — | — | — |

- Zero-pose axes: J1 ẑ · J2 ŷ · J3 ẑ · J4 ŷ · J5 ẑ · J6 ŷ · J7 ẑ —
  **alternating vertical/horizontal**, all local z, ±90° frame rotations.
- All origins are pure z translations: **no lateral offsets**.
- Zero pose = arm vertical (as in the Franka zero pose).
- Properties: the tool point is **invariant under J7** (the roll axis passes
  through the J6 point → spherical-wrist behavior); each pitch/roll pair is
  a coaxial intersecting pair; FK = planar 3-pitch chain × 3 rolls.
- Comparison with the references: Franka/Gen3 put roll axes *along* the
  links and sink pitch axes *into* housings; the POC keeps every origin
  coincident. That is a **packaging simplification, not a loss of kinematic
  capability** — orientation coverage is identical for intersecting
  pitch/roll pairs, and the workspace is set by the pitch spacings. The
  Franka's ±82.5 mm in-plane offsets are a *different* (kinematically
  active) choice and are not needed here.
- **Recommendation: keep the convention unchanged** (brief item 11 ✓).
  Kinematically-neutral option for the mechanical pass: roll-joint origins
  may slide along their own vertical axis to match housing positions in the
  URDF (zero effect on FK/IK/limits — a joint can be declared at any point
  of its own axis).
- POC limits (±π / ±2.09 rad) are generous at the new scale; physical stops
  are a CAD decision (§10.5), after which IK limits tighten to
  stop − 5–10° clearance.

## 5. Mass model (parametric)

Static: `τ_i = g · Σ_j m_j · r_{j,i}`, worst-case horizontal pose;
`r` = horizontal distance from the joint axis to each downstream COM.

Design B (levers from J2 / from J4 / from J6):

| downstream mass | from J2 | from J4 | from J6 |
|---|---|---|---|
| upper-arm structure `m_u` | L2/2 = 0.1075 | — | — |
| J4 + J5 (0.521 + 0.30 kg) | L2 = 0.215 | 0 | — |
| forearm structure `m_f` | L2 + L3/2 = 0.3225 | L3/2 = 0.1075 | — |
| J6 + J7 (0.30 + 0.30 kg) | L2 + L3 = 0.430 | L3 = 0.215 | 0 |
| wrist housing `m_w` | 0.4625 | 0.2475 | tool/2 = 0.0325 |
| payload `m_p` | **0.495** | **0.280** | **0.065** |

Design A levers: 0.100 / 0.20 / 0.30 / 0.40 / 0.4315 / 0.463 (m).
Design C levers: 0.110 / 0.22 / 0.33 / 0.44 / 0.475 / 0.510 (m).

## 6. Static torque results

| | A | **B** | C |
|---|---|---|---|
| τ_J2 arm-only | 6.24 Nm | **6.70 Nm** | 6.86 Nm |
| τ_J4 arm-only | 1.80 | **1.94** | 1.98 |
| τ_J6 arm-only | 0.062 | **0.064** | 0.069 |
| per kg payload at J2 | 4.54 Nm | **4.86 Nm** | 5.00 Nm |
| per kg at J4 | 2.58 | **2.75** | 2.85 |
| per kg at J6 | 0.618 | **0.638** | 0.687 |
| **payload @ SF 1.5 on rated (binding joint J2)** | **1.27 kg** | **1.09 kg** | **1.03 kg** |
| J4-limited payload @ SF 1.5 (8.3 Nm AK70) | 1.45 | 1.31 | 1.25 |
| payload at rated, no SF | 2.59 | 2.33 | 2.23 |
| payload @ SF 1.5 on peak (48 Nm) | 5.67 | 5.21 | 5.03 |

- **J2 is the binding joint in all three designs**; J4 has ~20 % headroom
  vs J2; J6 is not binding even with a 4 Nm placeholder.
- The A→C payload spread is only ~18 %: distal actuator mass dominates the
  shoulder load and is nearly scale-invariant, so reach is a cheap variable
  here — the structural terms move only a few percent across the window.
- B's 1.09 kg continuous claim uses the *known* J2 actuator only; it does
  not depend on the open J1/J3/J5/J6/J7 choices (except via the fill-in
  masses, which are ~30 % of the arm-only term).

## 7. Dynamics check (B, 1 kg payload)

- I_J2 ≈ Σ m·r² = 0.40·0.1075² + 0.821·0.215² + 0.35·0.3225² +
  0.60·0.430² + 0.20·0.4625² + m_p·0.495² = 0.233 + 0.245·m_p
  → **0.478 kg·m²** with 1 kg payload.
- α = 3 rad/s² → τ_dyn ≈ 1.4 Nm ≈ 8 % of rated (before SF) — covered by the
  SF 1.5 quasi-static budget; re-check for α > ~8 rad/s².
- I_J4 ≪ (≤ ~0.06 kg·m² incl. payload) → < 0.2 Nm at 3 rad/s² — not binding.
- Speed: no-load 16.75 rad/s @ 24 V (AK10) vs POC limits 2.17–2.61 rad/s →
  ~6× headroom; velocity limits can be raised later for free performance.

## 8. Mechanical envelope (Design B)

- **J2 (AK10, Ø98 × 61.7):** shoulder housing Ø~110, ~65 mm deep;
  L1 = 180 mm leaves ~115 mm of visible base column above the housing.
- **J4 (AK70, Ø89 × 50.25):** elbow housing Ø~95; clear tube between
  housings = 215 − (61.7 + 50.25)/2 ≈ **153 mm** — no housing clash.
- **Forearm 215 mm:** J6/J7 (≤ Ø70 placeholder) + 65 mm tool → ~95 mm
  wrist cluster — fits with margin.
- **Links:** 50–60 mm structural cross-section, *narrower* than the
  motors; housings protrude (per the brief). Do not size links to motor
  diameter.
- **Collision risks:** (a) upper arm vs base at deep J2 flexion — keep
  ≥ 15° clearance or a physical stop; (b) elbow fold set by ~55 + 45 mm
  housing radii vs the 215 mm span — allow ~±135° at J4, verify in CAD;
  (c) tool vs forearm at negative J6 — 65 mm tool is short, low risk.
- **Cables:** base raceway at J1, articulating clevis/spiral at the J3/J5
  rolls, loop at the wrist; the +0.1–0.2 kg/link allowance in §2.3 covers
  this.
- **Base:** 2.5–3.0 kg, footprint ≥ 0.35 m square recommended; tip-over
  estimate: (1 kg × 9.81 × 0.5 m) ≈ 4.9 Nm overturning vs a ~17 Nm
  restoring moment at the 0.175 m half-footprint — ample, confirm at CAD.

## 9. Workspace (B, qualitative)

- J1 ±π → 360° yaw.
- Horizontal: ~0.50 m when flat, ~0.65 m at an angle.
- Vertical: to ~0.65 m above the desk; below the shoulder to ~0.1–0.15 m
  (base-limited).
- Exact numbers: run a coverage sweep (grid of `/solve` targets) against
  the rescaled FK after implementation.

## 10. Open items / verify before implementation

1. **AK70-10 revision** — verify exact spec (search was down this session);
   values above are from the brief.
2. **xArm7 URDF** — re-extract joint origins if a public model appears
   (class-level reference only so far).
3. **J1/J3/J5/J6/J7 selection** — fill the parameter table (§2.2); J6 must
   cover ~1.3 Nm at 1 kg payload + SF (any small joint does).
4. **Structure masses** — replace the m_u/m_f/m_w estimates with
   CAD/measurements; §6 re-runs as a one-line-per-joint sum.
5. **Physical joint stops** — define in CAD; then tighten IK limits
   (currently ±π / ±2.09) to stops − 5–10° clearance.
6. **Base tip-over check** at CAD stage (estimate §8 is favorable).
7. **Optional:** slide roll-joint origins along their axes in the URDF to
   match housings (kinematically neutral, §4).

## 11. Implementation checklist (post-approval, Design B numbers)

Per the triple-port discipline (HANDOVER §5):

1. `docs/arm7-kinematic-spec.md` — L1 0.180, L2/L3 0.215, tool 0.065;
   recompute §5 anchor poses; add the actuator-envelope note.
2. C++ test ports (`tests/arm7_fk.hpp`, `examples/arm7_cross_check`) —
   constants; rebuild + ctest + cross-check against the Python port.
3. `service/arm7.py` — JOINTS origin-z: J2 0.180, J4/J6 0.215;
   TOOL_OFFSET 0.065. Limits/velocities/axes unchanged.
4. `robot_description/arm7.urdf` — rescale each segment's visuals by its
   own ratio (≈ 0.53, not perfectly uniform: 180/340 = 0.529,
   215/400 = 0.538, 65/126 = 0.516); **recommended: rebuild the visuals at
   the new dimensions** (first step toward the motor-housing visuals the
   brief's "coherent mechanical design" implies); joint origins follow the
   spec; mesh `scale` attributes as needed.
5. pytest — anchor expectations; new cross-check targets inside the B
   workspace: proposed **200/100/300** and **300/150/300 mm**
   (norms 380 / 440 mm < 495 mm ✓).
6. Web demo — target sliders x/y −550…550, z 0…650; defaults 300/150/300;
   camera: view.dist 3.3 → 1.8 m, wheel clamp 0.3–3.0 m, ground grid
   ±0.6 m; "CCD cross-check" text updated.
7. Docs — mark this study approved; HANDOVER bullet; no HANDOVER model
   changes beyond that.
8. No `pickik` binding changes — the binding is model-agnostic.
