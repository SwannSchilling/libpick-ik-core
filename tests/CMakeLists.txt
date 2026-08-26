# Tests for libpick_ik_core.
#
# No MoveIt, no ROS: kinematic models are built by hand (see arm7_fk.hpp and
# the RR helpers in ik_tests_core.cpp).

find_package(Catch2 3.4 QUIET)
if(NOT Catch2_FOUND)
    include(FetchContent)
    message(STATUS "Catch2 not found; fetching Catch2 v3.7.1")
    FetchContent_Declare(
        catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.7.1
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(catch2)
    # When used as a subproject, the catch_discover_tests()/catch_add_tests()
    # helpers live in Catch2's extras directory and must be included manually.
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
    include(Catch)
endif()

add_executable(test-pick_ik_core
    goal_tests.cpp
    robot_tests_core.cpp
    ik_tests_core.cpp
    solver_tests.cpp)
target_link_libraries(test-pick_ik_core
    PRIVATE
        pick_ik_core
        Catch2::Catch2WithMain)

# The top level disables CTest for subprojects (BUILD_TESTING=OFF);
# re-enable it for this directory and below.
enable_testing()
include(CTest)
catch_discover_tests(test-pick_ik_core)
