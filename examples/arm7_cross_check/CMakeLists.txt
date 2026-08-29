# arm7_cross_check: standalone arm7 FK cross-check + PickIK solve demo.
# The arm7 model itself is the shared header ../arm7/arm7.hpp.
add_executable(arm7_cross_check main.cpp)
target_include_directories(arm7_cross_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(arm7_cross_check PRIVATE pick_ik_core)
