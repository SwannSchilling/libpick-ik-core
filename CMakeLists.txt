# libpick_ik_core
#
# Standalone, ROS/MoveIt-free core of the PickIK inverse-kinematics solver.
# Plain CMake project (no ament): buildable and testable outside any ROS 2
# environment.
#
# Provenance: solver sources are extracted from PickNikRobotics/pick_ik
# (BSD-3-Clause, see LICENSE). `ik_gradient.cpp` and `ik_memetic.cpp` are
# byte-identical to upstream; `robot.cpp`/`goal.cpp` are the pure parts of the
# upstream files (MoveIt factories removed, pure `Robot::make` added).
# RSL `random`/`queue` are vendored byte-identical from PickNikRobotics/RSL
# (BSD-3-Clause, see extern/rsl/LICENSE) because upstream RSL's build itself
# depends on rclcpp.

cmake_minimum_required(VERSION 3.22)
project(pick_ik_core LANGUAGES CXX)

option(PICK_IK_CORE_BUILD_TESTS "Build pick_ik_core tests" ON)
option(PICK_IK_CORE_BUILD_EXAMPLES "Build pick_ik_core examples" ON)
option(PICK_IK_CORE_BUILD_PYTHON "Build the Python (pybind11) binding 'pickik'" OFF)
option(PICK_IK_CORE_BUILD_C_ABI "Build the pick_ik_c C ABI shared library" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# MSVC does not define M_PI & co. from <cmath> without this (no-op on other
# toolchains; upstream PickIK is developed against GCC/Clang).
add_compile_definitions(_USE_MATH_DEFINES)

# ---------------------------------------------------------------------------
# Dependencies (non-ROS; fetched when not found on the system)
# ---------------------------------------------------------------------------
# Keep CTest from registering the tests that fetched subprojects may add
# (e.g. Eigen's ~900-test suite). pick_ik_core's own tests are enabled
# explicitly in tests/CMakeLists.txt.
set(BUILD_TESTING OFF CACHE BOOL "Subproject tests are disabled for pick_ik_core" FORCE)

include(FetchContent)

find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found; fetching Eigen 3.4.0")
    FetchContent_Declare(
        eigen
        # Canonical Eigen repository (github.com/eigen/eigen is a stale mirror)
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(eigen)
endif()

find_package(fmt QUIET)
if(NOT fmt_FOUND)
    message(STATUS "fmt not found; fetching fmt 10.2.1")
    FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG 10.2.1
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(fmt)
endif()

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(pick_ik_core STATIC
    src/robot.cpp
    src/goal.cpp
    src/ik_gradient.cpp
    src/ik_memetic.cpp
    src/solver.cpp
    extern/rsl/src/random.cpp)
add_library(pick_ik::core ALIAS pick_ik_core)

target_compile_features(pick_ik_core PUBLIC cxx_std_17)
target_include_directories(pick_ik_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern/rsl/include>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:rsl>)
    # ik_memetic.hpp includes <rsl/random.hpp> in its public interface
    # (kept as in upstream); consumers therefore need the vendored RSL
    # headers: from the source tree via the BUILD_INTERFACE entry, from an
    # install tree via the INSTALL_INTERFACE entry (install() below puts the
    # headers at <prefix>/include/rsl). Raw source paths must stay out of the
    # install interface or CMake's export generation fails out-of-source.
target_link_libraries(pick_ik_core
    PUBLIC Eigen3::Eigen
    PRIVATE fmt::fmt)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# Vendored RSL headers (needed by the public interface of ik_memetic.hpp)
install(
    DIRECTORY extern/rsl/include/rsl/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rsl)
install(
    TARGETS pick_ik_core
    EXPORT pick_ik_core-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(
    EXPORT pick_ik_core-targets
    NAMESPACE pick_ik::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pick_ik_core)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(PICK_IK_CORE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Examples (standalone, no ROS/MoveIt)
# ---------------------------------------------------------------------------
if(PICK_IK_CORE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ---------------------------------------------------------------------------
# C ABI (shared library for native hosts: Blender ctypes, Unity, C, WASM)
# ---------------------------------------------------------------------------
# Thin adapter over the IkSolver contract — no solver code (roadmap §3.0a).
# Ships the built-in arm7 model (native FK) so hosts need no callbacks.
if(PICK_IK_CORE_BUILD_C_ABI)
    add_library(pick_ik_c SHARED src/pickik_c.cpp)
    target_include_directories(pick_ik_c
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples)
    target_compile_definitions(pick_ik_c PRIVATE PICK_IK_C_BUILDING=1)
    target_link_libraries(pick_ik_c PRIVATE pick_ik_core)
    set_target_properties(pick_ik_c PROPERTIES OUTPUT_NAME pick_ik_c)
    install(FILES include/pick_ik_c/pickik_c.h
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/pick_ik_c)
    install(TARGETS pick_ik_c
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# ---------------------------------------------------------------------------
# Python binding (optional; off by default so normal builds stay dependency-free)
# ---------------------------------------------------------------------------
if(PICK_IK_CORE_BUILD_PYTHON)
    add_subdirectory(bindings/python)
endif()
