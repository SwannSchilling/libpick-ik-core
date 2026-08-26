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
//   frame. The GIL is acquired for every call, so the callback is thread
//   safe, but a Python FK is slowest under the memetic solver's parallel
//   evaluation — for Python FKs consider num_threads=1..2 there.
//
// Conventions:
//   - targets: list of 4x4 frames (base frame, meters); v1: exactly one.
//   - local_axes: list of [x, y, z] per joint (joint axis in its joint frame).
//   - quaternion convention everywhere: [x, y, z, w] (ROS / p5.js order).

#include <pick_ik/solvers.hpp>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace py = pybind11;

namespace {

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

/// Wrap a Python FK callable as a pick_ik::LinkFkFn (thread-safe; GIL per call).
pick_ik::LinkFkFn make_link_fk(py::object const& fk) {
    auto const fn = fk;  // hold a reference; std::function copy keeps it alive
    return [fn](std::vector<double> const& q) -> std::vector<Eigen::Isometry3d> {
        py::gil_scoped_acquire gil;
        py::sequence const out = py::cast<py::sequence>(fn(py::cast(q)));
        std::vector<Eigen::Isometry3d> frames;
        frames.reserve(out.size());
        for (size_t i = 0; i < out.size(); ++i) {
            frames.push_back(frame_to_isometry(out[static_cast<int>(i)]));
        }
        if (frames.empty()) {
            throw std::runtime_error("FK callback returned no frames");
        }
        return frames;
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
                       "(use 0.0 for position-only goals).");

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
    // pybind11 calls), and the GIL is released for the actual solve. This is
    // required for the memetic solver: its worker threads call the (Python)
    // FK callback, which re-acquires the GIL per call — if the calling thread
    // kept the GIL for the whole solve, the workers would block on it forever.
    auto solve_fn = [](pick_ik::IkSolver const& solver,
                       pick_ik::Robot const& robot,
                       py::object const& fk,
                       std::vector<std::array<double, 3>> const& local_axes,
                       std::vector<double> const& q_seed,
                       std::vector<py::object> const& targets,
                       pick_ik::SolveOptions const& options) {
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
        auto const link_fk = make_link_fk(fk);
        py::gil_scoped_release release;
        return solver.solve(robot, link_fk, axes, q_seed, target_poses, options);
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
             "elite_size/population_size: evolution population. num_threads: "
             "parallel species (with a Python FK callback, GIL-serialized). "
             "Gradient-descent exploitation uses default GradientIkParams.");
}
