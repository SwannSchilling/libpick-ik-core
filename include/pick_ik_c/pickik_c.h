/* ============================================================================
 * pick_ik_c — thin C ABI over the PickIK core (pick_ik::IkSolver contract)
 * ============================================================================
 *
 * A stable, compiler-agnostic C interface for native hosts that cannot link
 * C++ (Blender add-ons via ctypes, Unity via P/Invoke, C hosts, WASM,
 * ...). This layer contains NO solver code: it adapts C callbacks and POD
 * structs to the existing C++ `IkSolver` contract (see pick_ik/solver.hpp).
 *
 * Memory ownership: every handle is created by a `*_create` call and freed
 * by the matching `*_free`. Nothing else crosses the boundary — results come
 * back into caller-provided buffers.
 *
 * Thread safety: handles are immutable after creation and safe to use from
 * multiple threads concurrently; `pickik_solve` is re-entrant (no global
 * state). Call it from a worker thread if the solve may exceed your UI
 * frame budget.
 *
 * Conventions: meters / radians; poses are 16 doubles, ROW-major 4x4 in the
 * standard homogeneous layout [R | t; 0 0 0 1] — the translation is in column
 * 3 (elements 3, 7, 11). Quaternion order where used: [x,y,z,w].
 */

#ifndef PICKIK_C_H
#define PICKIK_C_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported-symbol visibility (dllexport/dllimport on Windows). */
#if defined(_WIN32) && defined(_MSC_VER)
  #if defined(PICK_IK_C_BUILDING)
    #define PICKIK_C_API __declspec(dllexport)
  #else
    #define PICKIK_C_API __declspec(dllimport)
  #endif
#else
  #define PICKIK_C_API
#endif

/* Result codes. */
#define PICKIK_OK            0
#define PICKIK_E_BADARG     (-1)  /* null pointer / inconsistent size */
#define PICKIK_E_NOMEM      (-2)  /* allocation failure */
#define PICKIK_E_FK         (-3)  /* host FK callback returned non-zero */
#define PICKIK_E_INTERNAL   (-4)  /* unexpected C++ exception */

/* Opaque handles. */
typedef struct pickik_robot_s pickik_robot;
typedef struct pickik_solver_s pickik_solver;

/**
 * @brief FK callback: writes `n_joints + 1` frames (row-major 4x4 doubles,
 *        base frame): joint 0..n-1 child frames (pivots) then the tip frame.
 * @return PICKIK_OK on success, non-zero on failure (abort the solve).
 *        Must be re-entrant (the solver may call it concurrently).
 */
typedef int (*pickik_link_fk_fn)(void* user, int n_joints, double const* q,
                                 double* frames /* (n_joints + 1) * 16 */);

/**
 * @brief Create a robot (joint limits / velocities / local axes) for solving.
 * @param n_joints      number of variables (e.g. 7).
 * @param lower         n_joints lower limits [rad]; all joints bounded.
 * @param upper         n_joints upper limits [rad].
 * @param max_velocity  n_joints max velocities [rad/s]; 0.0 = no velocity
 *                      weighting for that joint.
 * @param local_axes    3 * n_joints: joint i's rotation axis in joint i's
 *                      frame (unit vector), i.e. the URDF joint axis.
 */
PICKIK_C_API int pickik_robot_create(int n_joints, double const* lower,
                                     double const* upper, double const* max_velocity,
                                     double const* local_axes, pickik_robot** out);

/** @brief Is q (n doubles) inside all joint limits? Writes 0/1 to out_valid. */
PICKIK_C_API int pickik_robot_is_valid(pickik_robot* robot, double const* q, int* out_valid);

PICKIK_C_API int pickik_robot_free(pickik_robot* robot);

/* ---------------------------------------------------------------------------
 * Solvers — each create returns a fresh handle; free when done.
 * ------------------------------------------------------------------------ */

PICKIK_C_API int pickik_ccd_create(int max_passes, double damping, double epsilon,
                                   pickik_solver** out);

PICKIK_C_API int pickik_gradient_create(double step_size, double min_cost_delta,
                                        double max_time, int max_iterations,
                                        int stop_on_valid_solution, pickik_solver** out);

PICKIK_C_API int pickik_memetic_create(int elite_size, int population_size,
                                       double wipeout_fitness_tol, int max_generations,
                                       double max_time, int num_threads,
                                       int stop_on_valid_solution, int stop_on_first_soln,
                                       pickik_solver** out);

PICKIK_C_API int pickik_solver_free(pickik_solver* solver);

/* ---------------------------------------------------------------------------
 * Solve options (POD). Fill from pickik_options_default() and override.
 * ------------------------------------------------------------------------ */
typedef struct {
    double position_threshold;    /* [m]; 1e-3 */
    double orientation_threshold; /* [rad]; < 0 => position-only goal */
    double cost_threshold;        /* [cost units]; 1e-3 */
    double position_scale;        /* pose-cost weight; 1.0 */
    double rotation_scale;        /* pose-cost weight; 0.0 in the default (position-only
                                   * goals carry no orientation cost — the stack-wide
                                   * convention, see the service's /solve). Set ~0.5 for
                                   * full-pose goals. */
    double minimal_displacement_weight; /* >0: stay near the seed; 0 = off */
    double joint_target_weight;   /* >0: pull named joints to targets; 0 = off */
    double const* joint_target_values;  /* n_joints entries (rad) */
    int const* joint_target_has;        /* n_joints entries; 0 = no target for joint */
    int has_look_at;                  /* 1 = enable look-at secondary objective */
    double look_at_point[3];          /* base frame, meters */
    double look_at_axis[3];           /* tip frame axis to point at the point */
    double look_at_weight;            /* 0 = off even when has_look_at */
} pickik_options;

PICKIK_C_API void pickik_options_default(pickik_options* options);

typedef struct {
    int success;            /* 1 = solved within thresholds */
    double position_error;  /* [m]; -1 if not evaluated */
    double orientation_error; /* [rad]; -1 if not evaluated (position-only) */
    double time_ms;          /* wall-clock solve time */
} pickik_result;

/**
 * @brief Solve one inverse-kinematics problem (exactly one target, v1).
 * @param solver       handle from one of the create functions.
 * @param robot        handle (limits + local axes baked in).
 * @param fk           host FK callback (may be NULL => the solver uses pure
 *                     cost-based search without FK feedback; the standard
 *                     solvers all require an FK).
 * @param fk_user      opaque pointer passed through to fk.
 * @param q_seed       n doubles: starting configuration.
 * @param target       16 doubles: target pose (row-major 4x4, base frame).
 * @param options      see above.
 * @param out_q        caller buffer, n doubles: solved configuration.
 * @param out_result   caller buffer.
 * @return PICKIK_OK on completion (check out_result->success); PICKIK_E_FK
 *         if the FK callback failed; PICKIK_E_BADARG / PICKIK_E_INTERNAL on
 *         misuse or internal error.
 */
PICKIK_C_API int pickik_solve(pickik_solver* solver, pickik_robot* robot,
                              pickik_link_fk_fn fk, void* fk_user,
                              double const* q_seed, double const* target,
                              pickik_options const* options,
                              double* out_q, pickik_result* out_result);

/* ---------------------------------------------------------------------------
 * Built-in arm7 model (the Design B 7-DOF desktop arm, compiled in from the
 * shared model header examples/arm7/arm7.hpp) — so hosts get native
 * millisecond-class FK with no callbacks of their own.
 * ------------------------------------------------------------------------ */

#define PICKIK_ARM7_JOINTS 7

/** @brief arm7 robot handle (limits + velocities + local axes from the model). */
PICKIK_C_API int pickik_arm7_robot_create(pickik_robot** out);

/**
 * @brief arm7 link FK: writes 8 row-major 4x4 frames (7 pivots + tool0).
 *        `user` is unused (pass NULL); n_joints must be 7.
 */
PICKIK_C_API int pickik_arm7_link_fk(void* user, int n_joints, double const* q,
                                     double* frames);

/** @brief arm7 local joint axes into out (3 * 7 doubles). */
PICKIK_C_API int pickik_arm7_local_axes(double* out);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PICKIK_C_H */
