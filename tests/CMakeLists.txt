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
    solver_tests.cpp
    c_abi_tests.cpp)
# The shared arm7 model header lives under examples/ (it is an integration
# building block, not a public API of the core library).
target_include_directories(test-pick_ik_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../examples)
# The C ABI suite exercises pick_ik_c.dll the way a native host would.
target_link_libraries(test-pick_ik_core PRIVATE pick_ik_c)
# Put the DLL next to the test executable so ctest/discovery can load it
# (MSVC searches the exe directory first).
add_custom_command(TARGET test-pick_ik_core POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:pick_ik_c> $<TARGET_FILE_DIR:test-pick_ik_core>)
target_link_libraries(test-pick_ik_core
    PRIVATE
        pick_ik_core
        Catch2::Catch2WithMain)

# The top level disables CTest for subprojects (BUILD_TESTING=OFF);
# re-enable it for this directory and below.
enable_testing()
include(CTest)
catch_discover_tests(test-pick_ik_core)
