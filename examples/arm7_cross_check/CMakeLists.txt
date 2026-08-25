# arm7_cross_check: standalone p5.js-POC FK cross-check + PickIK solve demo.
add_executable(arm7_cross_check main.cpp)
target_link_libraries(arm7_cross_check PRIVATE pick_ik_core)
