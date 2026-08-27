// Python bindings for the libpick_ik_core solver contract (pick_ik/solver.hpp).
//
// Module name: pickik
//
// Exposes:
//   - make_robot / Robot / JointSpec
//   - SolveOptions / IkResult
//   - CcdSolver / PickIkGradientSolver / PickIkMemeticSolver (via the base
//     IkSolver contract: one .solve() for all of them)
//
// The FK callback is a Python callable:
//   fk(q) -> sequence of n+1 4x4 frames (n joint child frames + tip frame),
//   base frame, meters. It may be a plain nested list or a numpy array per
//   frame. It is always evaluated on the calling Python thread, which holds
//   the GIL: even the memetic solver's native gradient-descent threads never
//   touch the interpreter — they post joint vectors to an internal request
//   queue and wait for the result (the "FK pump", see below).
//
// Conventions:
//   - targets: list of 4x4 frames (base frame, meters); v1: exactly one.
//   - local_axes: list of [x, y, z] per joint (joint axis in its joint frame).
//   - quaternion convention everywhere: [x, y, z, w] (ROS / p5.js order).

#include <pick_ik/solvers.hpp>

// Use pybind11's simple GIL management: `gil_scoped_acquire` then maps to the
// standard, re-entrant CPython `PyGILState_Ensure`/`PyGILState_Release` pair.
// The default (custom) implementation instead creates and deletes a
// throw-away PyThreadState on every acquire, which corrupts the GIL state when
// the FK callback is invoked from both native worker threads and the calling
// Python thread (NULL+8 access violation under concurrent solves). The simple
// path is the correct tool for "call Python from a native thread".
#define PYBIND11_SIMPLE_GIL_MANAGEMENT

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace py = pybind11;

namespace {

// ------------------------------------------------------------------ Debug log
// Opt-in crash forensics: set PICK_IK_DEBUG_LOG=/path to append a one-line
// phase trace (thread hash + phase) to that file. Used to localize the
// intermittent memetic AV. Off by default (no writes when unset).
inline unsigned long thread_tag() {
    return static_cast<unsigned long>(
        std::hash<std::thread::id>{}(std::this_thread::get_id()));
}

struct PhaseLog {
    mutable std::mutex mutex;
    mutable int count = 0;
    FILE* f = nullptr;
    PhaseLog() {
        if (char const* p = std::getenv("PICK_IK_DEBUG_LOG")) {
            if (p[0] != '\0') {
                f = std::fopen(p, "a");
            }
        }
    }
    ~PhaseLog() {
        if (f) {
            std::fclose(f);
        }
    }
    void log(char const* phase) const {
        if (!f) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        std::fprintf(f, "%lu %s\n", thread_tag(), phase);
        // Flush periodically (not every call) so the trace survives a crash
        // without turning the hot FK path into a disk-write bottleneck.
        if (++count % 512 == 0) {
            std::fflush(f);
        }
    }
};

PhaseLog const g_phase_log;

// Serializes solves whose solver spawns native worker threads requesting FK
// evaluations (the memetic solver).
//
// Deliberately a plain C++ mutex, NOT a CPython lock:
//   - A C++ global holding a py::object would outlive interpreter
//     finalization, and its refcount decrement at process exit faults inside
//     python312.dll (NULL+8 read).
//   - The C-level PyThread_acquire_lock(lock, WAIT_LOCK) is broken on this
//     MS Store CPython build (3.12.28): it intermittently returns
//     PY_LOCK_FAILURE for an uncontended lock (reproduced deterministically
//     via a self-test at module init), while the very same lock works through
//     Python's threading.Lock. std::mutex (MSVC's own critical-section/
//     semaphore wrapper) avoids the CPython primitive entirely and has no
//     Python lifetime.
//
// The GIL is released while waiting on the lock (see CLevelLockGuard): at
// that moment no native thread is inside the interpreter (the active solve's
// FK pump runs on the calling thread), so the release/re-acquire pair is the
// only GIL traffic a waiting thread ever does.
//
// Serialization also keeps the shared FK request queue (see "FK pump" below)
// single-consumer. A Python FK is GIL-serialized anyway, so serializing these
// solves costs no real throughput.
std::mutex g_fk_worker_mtx;

struct CLevelLockGuard {
    PyThreadState* saved = nullptr;

    CLevelLockGuard() {
        saved = PyEval_SaveThread();  // release the GIL while waiting
        try {
            g_fk_worker_mtx.lock();  // block, GIL-free
        } catch (...) {
            PyEval_RestoreThread(saved);  // GIL back before the exception flies
            throw;
        }
        PyEval_RestoreThread(saved);  // re-acquire the GIL
    }
    ~CLevelLockGuard() {
        g_fk_worker_mtx.unlock();
    }
    CLevelLockGuard(const CLevelLockGuard&) = delete;
    CLevelLockGuard& operator=(const CLevelLockGuard&) = delete;
};

/// 4x4 frame (nested list/tuple or numpy array) -> Isometry3d.
Eigen::Isometry3d frame_to_isometry(py::object const& obj) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
    if (py::isinstance<py::array>(obj)) {
        py::array_t<double> const arr = obj.cast<py::array_t<double>>();
        if (arr.ndim() != 2 || arr.shape(0) != 4 || arr.shape(1) != 4) {
            throw std::invalid_argument("frame must be 4x4");
        }
        auto const buf = arr.unchecked<2>();
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                m(i, j) = buf(i, j);
    } else {
        std::vector<std::vector<double>> const nested =
            obj.cast<std::vector<std::vector<double>>>();
        if (nested.size() != 4) {
            throw std::invalid_argument("frame must be 4x4");
        }
        for (int i = 0; i < 4; ++i) {
            if (nested[i].size() != 4) {
                throw std::invalid_argument("frame must be 4x4");
            }
            for (int j = 0; j < 4; ++j) {
                m(i, j) = nested[i][j];
            }
        }
    }
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = m.topLeftCorner<3, 3>();
    t.translation() = m.topRightCorner<3, 1>();
    return t;
}

// ------------------------------------------------------------------ FK pump
//
// The FK callback is a Python callable that may only run on a thread holding
// the GIL. The memetic solver evaluates it from native threads: its solver
// thread (solution test + final error evaluation) and the per-generation
// gradient-descent threads.
//
// Naive approach — each native thread wraps its FK call in
// PyGILState_Ensure/Release — corrupts CPython under repeated solves: the
// thread-state create/teardown churn on short-lived native threads, combined
// with the calling Python thread's repeated PyEval_SaveThread/RestoreThread
// around each solve, intermittently leaves the interpreter's GIL-owner thread
// state NULL (observed as NULL+8/NULL+16 access violations inside
// _PyEval_EvalFrameDefault / _PyObject_New on a native thread mid-FK-call).
//
// Instead, the FK is always evaluated on the *calling* Python thread ("the
// pump"), which holds the GIL for the whole solve:
//   - native threads call NO CPython API: they post the joint vector to the
//     queue below and wait on a plain C++ condition variable;
//   - the pump (calling thread, GIL held) pops requests and evaluates the
//     Python FK;
//   - the native threads also never touch Python refcounts: the FK lambda
//     carries a borrowed PyObject* only (see make_link_fk), because the
//     upstream solver copies it into per-generation / per-species thread
//     lambdas whose refcount churn on native threads would race outside the
//     GIL;
//   - the only GIL release in the whole flow is the brief
//     PyEval_SaveThread/RestoreThread in CLevelLockGuard while waiting for
//     the solve-serialization mutex, when no native thread is inside the
//     interpreter.
//
// Single-threaded solvers (gradient, CCD) spawn no native threads; their FK
// calls run directly on the calling thread with the GIL held.
struct FkRequest {
    std::vector<double> q;  // set by the producer before it posts
    // Result: written by the pump under mtx, then signaled via cv.
    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    std::exception_ptr exc;
    std::vector<Eigen::Isometry3d> frames;
};

std::mutex g_fk_queue_mtx;
std::condition_variable g_fk_queue_cv;
std::deque<std::shared_ptr<FkRequest>> g_fk_queue;

/// Evaluate the Python FK callable. The GIL must be held on this thread.
/// `fn` is a BORROWED pointer (py::handle): the owning reference lives on the
/// calling Python thread for the whole solve (see make_link_fk).
std::vector<Eigen::Isometry3d> call_fk(py::handle const& fn,
                                       std::vector<double> const& q) {
    g_phase_log.log("fk:enter");
    py::object const qlist = py::cast(q);
    g_phase_log.log("fk:cast-q");
    py::object const result = fn(qlist);
    g_phase_log.log("fk:called");
    py::sequence const out = py::cast<py::sequence>(result);
    g_phase_log.log("fk:seq");
    std::vector<Eigen::Isometry3d> frames;
    frames.reserve(out.size());
    for (size_t i = 0; i < out.size(); ++i) {
        frames.push_back(frame_to_isometry(out[static_cast<int>(i)]));
    }
    g_phase_log.log("fk:parsed");
    if (frames.empty()) {
        throw std::runtime_error("FK callback returned no frames");
    }
    return frames;
}

/// Wrap a Python FK callable as a pick_ik::LinkFkFn.
/// On the calling Python thread (GIL held) the FK is evaluated directly; on
/// native solver threads it is posted to the FK pump (see above) and the
/// caller waits for the result without touching the GIL.
///
/// CRITICAL: the lambda captures only a BORROWED PyObject* — never an owning
/// py::object. The upstream memetic solver copies this std::function into
/// per-generation gradient-descent thread lambdas (ik_memetic.cpp, one copy
/// per elite per generation) and into per-species thread lambdas
/// (num_threads > 1). Those copies are constructed and destroyed concurrently
/// on native threads, i.e. OUTSIDE the GIL. An owning py::object in the
/// capture would therefore perform plain (non-atomic-checked)
/// Py_INCREF/Py_DECREF on the callable from several native threads at once;
/// with num_threads > 1 that race loses refcount updates and deallocates the
/// callable on a native thread while the GIL owner is mid-bytecode (observed
/// as _PyObject_New/NULL+16 and _PyEval_EvalFrameDefault/NULL+8 AVs). With a
/// borrowed pointer no refcount operation ever happens on a native thread.
///
/// Lifetime: the owning reference is the caller's `fk` argument to solve(),
/// which stays alive for the whole solve; every native thread the solve
/// spawns is joined before solve() returns, so the borrowed pointer is never
/// used after the owner dies.
pick_ik::LinkFkFn make_link_fk(py::handle fk) {
    PyObject* const raw = fk.ptr();
    std::thread::id const calling_tid = std::this_thread::get_id();
    return [raw, calling_tid](std::vector<double> const& q) -> std::vector<Eigen::Isometry3d> {
        if (std::this_thread::get_id() == calling_tid) {
            // Calling thread: the GIL is held for the whole single-threaded
            // solve, so evaluate directly.
            return call_fk(raw, q);
        }
        // Native solver thread: post to the pump and wait (no CPython API).
        g_phase_log.log("fk:post");
        auto const req = std::make_shared<FkRequest>();
        req->q = q;
        {
            std::lock_guard lk(g_fk_queue_mtx);
            g_fk_queue.push_back(req);
        }
        g_fk_queue_cv.notify_one();
        std::unique_lock lk(req->mtx);
        req->cv.wait(lk, [&req] { return req->done; });
        g_phase_log.log("fk:got-result");
        if (req->exc) {
            std::rethrow_exception(req->exc);
        }
        return std::move(req->frames);
    };
}

}  // namespace

PYBIND11_MODULE(pickik, m) {
    m.doc() = "pickik - Python bindings for the libpick_ik_core solver contract";

    // ------------------------------------------------------------------ Robot
    py::class_<pick_ik::Robot::JointSpec>(m, "JointSpec")
        .def(py::init([](double min, double max, double max_velocity, bool bounded) {
                 pick_ik::Robot::JointSpec s;
                 s.min = min;
                 s.max = max;
                 s.max_velocity = max_velocity;
                 s.bounded = bounded;
                 return s;
             }),
             py::arg("min"), py::arg("max"),
             py::arg("max_velocity") = 0.0, py::arg("bounded") = true,
             "Joint position/velocity limits (radians, rad/s).")
        .def_readonly("min", &pick_ik::Robot::JointSpec::min)
        .def_readonly("max", &pick_ik::Robot::JointSpec::max)
        .def_readonly("max_velocity", &pick_ik::Robot::JointSpec::max_velocity)
        .def_readonly("bounded", &pick_ik::Robot::JointSpec::bounded);

    py::class_<pick_ik::Robot>(m, "Robot")
        .def("is_valid_configuration", &pick_ik::Robot::is_valid_configuration,
             py::arg("q"), "True if every joint value is within its limits.")
        .def_property_readonly("num_joints",
                               [](pick_ik::Robot const& r) {
                                   return static_cast<int>(r.variables.size());
                               });

    m.def(
        "make_robot",
        [](std::vector<pick_ik::Robot::JointSpec> const& joints) {
            return pick_ik::Robot::make(joints);
        },
        py::arg("joints"), "Create a Robot from plain joint specifications.");

    // ------------------------------------------------------------- Options
    py::class_<pick_ik::SolveOptions>(m, "SolveOptions")
        .def(py::init<>())
        .def_readwrite("position_threshold",
                       &pick_ik::SolveOptions::position_threshold,
                       "Position error threshold [m] for success.")
        .def_readwrite("orientation_threshold",
                       &pick_ik::SolveOptions::orientation_threshold,
                       "Orientation error threshold [rad]; None solves position-only.")
        .def_readwrite("cost_threshold", &pick_ik::SolveOptions::cost_threshold,
                       "PickIK solution-test cost threshold.")
        .def_readwrite("position_scale", &pick_ik::SolveOptions::position_scale,
                       "Position weight of the pose cost function.")
        .def_readwrite("rotation_scale", &pick_ik::SolveOptions::rotation_scale,
                       "Orientation weight of the pose cost function "
                       "(use 0.0 for position-only goals).")
        .def_readwrite("minimal_displacement_weight",
                       &pick_ik::SolveOptions::minimal_displacement_weight,
                       "Secondary objective: pull the solution toward the seed "
                       "(upstream 'minimal_displacement_weight'). 0.0 disables "
                       "it. Applies to the gradient/memetic solvers; CCD ignores "
                       "it.");

    // ------------------------------------------------------------------ Result
    py::class_<pick_ik::IkResult>(m, "IkResult")
        .def_readonly("success", &pick_ik::IkResult::success,
                      "True if the thresholds were met.")
        .def_readonly("q", &pick_ik::IkResult::q,
                      "Solution; on failure: the seed (PickIK solvers) or the "
                      "final configuration (CCD).")
        .def_readonly("position_error", &pick_ik::IkResult::position_error,
                      "Max position error vs. target [m].")
        .def_readonly("orientation_error", &pick_ik::IkResult::orientation_error,
                      "Max orientation error vs. target [rad]; -1 when not "
                      "evaluated (position-only).");

    // ------------------------------------------------------------------ Solvers
    // NOTE: all Python conversions happen while the GIL is held (default for
    // pybind11 calls), and the GIL stays held for the whole solve:
    //   - solvers without native FK workers evaluate the FK directly on the
    //     calling thread;
    //   - solvers that spawn native FK worker threads (memetic) run the solve
    //     on a dedicated native thread while the calling thread — holding the
    //     GIL — pumps the workers' FK requests through the queue (see the
    //     "FK pump" section). No native thread ever calls a CPython GIL API.
    auto solve_fn = [](pick_ik::IkSolver const& solver,
                       pick_ik::Robot const& robot,
                       py::object const& fk,
                       std::vector<std::array<double, 3>> const& local_axes,
                       std::vector<double> const& q_seed,
                       std::vector<py::object> const& targets,
                       pick_ik::SolveOptions const& options) {
        g_phase_log.log("solve:enter");
        std::vector<Eigen::Vector3d> axes;
        axes.reserve(local_axes.size());
        for (auto const& a : local_axes) {
            axes.emplace_back(a[0], a[1], a[2]);
        }
        std::vector<Eigen::Isometry3d> target_poses;
        target_poses.reserve(targets.size());
        for (auto const& t : targets) {
            target_poses.push_back(frame_to_isometry(t));
        }
        g_phase_log.log("solve:converted");
        auto const link_fk = make_link_fk(fk);
        if (solver.spawns_fk_worker_threads()) {
            // Serialize these solves (see g_fk_worker_mtx), then run the solve
            // on a dedicated native thread while this calling thread — holding
            // the GIL the whole time — pumps the FK requests its native threads
            // post (see the "FK pump" section above). No native thread calls
            // any CPython GIL API; that mix of PyGILState_Ensure/Release on
            // short-lived native threads with PyEval_SaveThread/RestoreThread
            // on the calling thread intermittently corrupts CPython's GIL-owner
            // thread state under repeated solves (NULL+8/NULL+16 access
            // violations inside _PyEval_EvalFrameDefault / _PyObject_New).
            CLevelLockGuard const lock_guard;
            g_phase_log.log("solve:pump");
            std::optional<pick_ik::IkResult> result;
            std::exception_ptr exc = nullptr;
            bool solve_done = false;  // guarded by g_fk_queue_mtx
            std::thread solver_thread([&] {
                try {
                    result = solver.solve(robot, link_fk, axes, q_seed,
                                          target_poses, options);
                } catch (...) {
                    exc = std::current_exception();
                }
                {
                    std::lock_guard lk(g_fk_queue_mtx);
                    solve_done = true;
                }
                g_fk_queue_cv.notify_all();
            });
            // Pump: evaluate the FK requests posted by the solver's native
            // threads. The GIL stays held on this thread throughout.
            for (;;) {
                std::unique_lock lk(g_fk_queue_mtx);
                g_fk_queue_cv.wait(lk, [&] {
                    return !g_fk_queue.empty() || solve_done;
                });
                auto batch = std::move(g_fk_queue);
                g_fk_queue.clear();
                bool const done_now = solve_done;
                lk.unlock();
                for (auto const& req : batch) {
                    std::exception_ptr req_exc = nullptr;
                    std::vector<Eigen::Isometry3d> req_frames;
                    try {
                        req_frames = call_fk(fk, req->q);
                    } catch (...) {
                        req_exc = std::current_exception();
                    }
                    {
                        std::lock_guard rlk(req->mtx);
                        req->frames = std::move(req_frames);
                        req->exc = req_exc;
                        req->done = true;
                    }
                    req->cv.notify_one();
                }
                if (done_now && batch.empty()) {
                    break;
                }
            }
            solver_thread.join();  // happens-before: result/exc are final now
            if (exc) {
                std::rethrow_exception(exc);
            }
            g_phase_log.log("solve:return");
            return result.value();
        }
        g_phase_log.log("solve:keep-gil");
        auto const result =
            solver.solve(robot, link_fk, axes, q_seed, target_poses, options);
        g_phase_log.log("solve:return");
        return result;
    };

    py::class_<pick_ik::IkSolver>(m, "IkSolver",
                                  "Generic solver interface (all solvers share .solve()).")
        .def("name", &pick_ik::IkSolver::name)
        .def("solve", solve_fn, py::arg("robot"), py::arg("fk"), py::arg("local_axes"),
             py::arg("seed"), py::arg("targets"), py::arg("options"),
             R"doc(Solve IK for `targets` starting from `seed`.

fk: Python callable q -> n+1 4x4 frames (n joint child frames + tip frame),
    base frame, meters (plain lists or numpy arrays).
local_axes: joint i's axis in its joint frame, e.g. [[0,0,1]]*7.
targets: list of 4x4 target frames (v1: exactly one).
options: SolveOptions; set orientation_threshold=None (+ rotation_scale=0.0)
    for position-only solves.

Returns an IkResult.
)doc");

    py::class_<pick_ik::CcdSolver, pick_ik::IkSolver>(m, "CcdSolver",
        "Cyclic Coordinate Descent - faithful port of the p5.js POC's solveCCD. "
        "Position-only: chases targets[0]'s position (orientation_error = -1).")
        .def(py::init<int, double, double>(), py::arg("max_passes") = 300,
             py::arg("damping") = 0.1, py::arg("epsilon") = 1e-8,
             "max_passes = full J7->J1 sweeps; the POC runs 5 per frame at ~60 fps, "
             "so 600 ~= 2 seconds of POC runtime.");

    py::class_<pick_ik::PickIkGradientSolver, pick_ik::IkSolver>(
        m, "PickIkGradientSolver",
        "Upstream PickIK local gradient-descent solver (deterministic).")
        .def(py::init([](double step_size, double min_cost_delta, double max_time,
                         int max_iterations, bool stop_opt) {
                  pick_ik::GradientIkParams p;
                  p.step_size = step_size;
                  p.min_cost_delta = min_cost_delta;
                  p.max_time = max_time;
                  p.max_iterations = max_iterations;
                  p.stop_optimization_on_valid_solution = stop_opt;
                  return pick_ik::PickIkGradientSolver(p);
              }),
             py::arg("step_size") = 0.0001, py::arg("min_cost_delta") = 1e-12,
             py::arg("max_time") = 0.05, py::arg("max_iterations") = 100,
             py::arg("stop_optimization_on_valid_solution") = true,
             "Wall-clock budget in seconds; the search is deterministic but "
             "time-bounded, so give hard targets a wide budget.");

    py::class_<pick_ik::PickIkMemeticSolver, pick_ik::IkSolver>(
        m, "PickIkMemeticSolver",
        "Upstream PickIK global memetic/evolutionary solver with gradient "
        "exploitation. Population is seeded randomly: results are not "
        "bit-reproducible run to run.")
        .def(py::init([](size_t elite_size, size_t population_size,
                         double wipeout_fitness_tol, int max_generations,
                         double max_time, size_t num_threads, bool stop_opt,
                         bool stop_first) {
                  pick_ik::MemeticIkParams p;
                  p.elite_size = elite_size;
                  p.population_size = population_size;
                  p.wipeout_fitness_tol = wipeout_fitness_tol;
                  p.max_generations = max_generations;
                  p.max_time = max_time;
                  // num_threads > 1 is safe here: every native thread
                  // (species + gradient-descent elites) routes its FK
                  // evaluations through the calling thread's FK pump (see
                  // "FK pump" above) and never touches the interpreter
                  // itself. Each extra thread is a full independent memetic
                  // run, bounded by CPU cores.
                  p.num_threads = num_threads;
                  p.stop_optimization_on_valid_solution = stop_opt;
                  p.stop_on_first_soln = stop_first;
                  return pick_ik::PickIkMemeticSolver(p);
              }),
             py::arg("elite_size") = 4, py::arg("population_size") = 16,
             py::arg("wipeout_fitness_tol") = 0.00001,
             py::arg("max_generations") = 100, py::arg("max_time") = 1.0,
             py::arg("num_threads") = 1,
             py::arg("stop_optimization_on_valid_solution") = true,
             py::arg("stop_on_first_soln") = true,
             "elite_size/population_size: evolution population. num_threads "
             "runs that many independent parallel memetic species (each with "
             "its own elite gradient-descent threads); all FK callbacks are "
             "evaluated on the calling Python thread via the binding's FK "
             "pump, so any value is safe — bounded by your CPU core count. "
             "Gradient-descent exploitation uses default GradientIkParams.");
}
