# Python binding (pybind11) for the pick_ik_core solver contract.
#
# Optional: built only when PICK_IK_CORE_BUILD_PYTHON=ON (default OFF), so the
# core's normal builds stay dependency-free. If pybind11 is not found on the
# system (pip: python -m pybind11 --cmake-dir -> -Dpybind11_DIR=...), it is
# fetched.

find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
    message(STATUS "pybind11 not found; fetching pybind11 v2.13.6")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.13.6
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(pybind11)
endif()

pybind11_add_module(pickik MODULE src/pickik_module.cpp)
target_link_libraries(pickik PRIVATE pick_ik_core)
target_compile_features(pickik PRIVATE cxx_std_17)
