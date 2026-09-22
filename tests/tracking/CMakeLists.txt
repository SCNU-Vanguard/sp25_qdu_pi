cmake_minimum_required(VERSION 3.16)
project(sp25_tracking_tests LANGUAGES CXX)

# [9.9-7][9.9-8] 使用真实 Armor/Tracker/Solver/Target/EKF；无 SDK、相机、NPU 或控制输出。
find_package(OpenCV REQUIRED COMPONENTS core calib3d)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
add_library(tracking_test_support STATIC
    ../../tasks/auto_aim/armor.cpp
    ../../tasks/auto_aim/tracker.cpp
    ../../tasks/auto_aim/solver.cpp
    ../../tasks/auto_aim/target.cpp
    ../../tasks/auto_aim/aimer.cpp # [9.9-10] 显式延迟测试使用真实 Aimer/弹道。
    ../../tools/trajectory.cpp
    ../../tools/extended_kalman_filter.cpp
    ../../tools/math_tools.cpp
    ../../tools/logger.cpp)
target_include_directories(tracking_test_support PUBLIC ../.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tracking_test_support PUBLIC ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp spdlog::spdlog)
target_compile_features(tracking_test_support PUBLIC cxx_std_17)

# 只写入测试构建目录；此合成标定仅用于让实际 Solver 参与回归，不可部署到机器人。
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/tracker_test.yaml" "
enemy_color: red
min_detect_count: 2
max_temp_lost_count: 3
outpost_max_temp_lost_count: 5
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]
R_camera2gimbal: [0, 0, 1, -1, 0, 0, 0, -1, 0]
t_camera2gimbal: [0, 0, 0]
camera_matrix: [1000, 0, 720, 0, 1000, 540, 0, 0, 1]
distort_coeffs: [0, 0, 0, 0, 0]
# [9.9-10] 合成 Aimer 配置，绝不用于机器人部署。
yaw_offset: 0
pitch_offset: 0
comming_angle: 55
leaving_angle: 20
decision_speed: 2
high_speed_delay_time: 0
low_speed_delay_time: 0
max_host_frame_age_ms: 50
left_yaw_offset: 3
right_yaw_offset: -3
")

enable_testing()
add_executable(tracker_selection_test ../tracker_selection_test.cpp)
target_link_libraries(tracker_selection_test PRIVATE tracking_test_support)
add_test(NAME tracker_resolution_selection COMMAND tracker_selection_test
         "${CMAKE_CURRENT_BINARY_DIR}/tracker_test.yaml")
add_executable(target_matching_test ../target_matching_test.cpp)
target_link_libraries(target_matching_test PRIVATE tracking_test_support)
add_test(NAME target_candidate_bounds COMMAND target_matching_test)
set_tests_properties(tracker_resolution_selection target_candidate_bounds PROPERTIES TIMEOUT 30)

# [9.9-9] 真实 PnP/跟踪回归与 API 故障注入分为两个进程，替身不会链接进产品。
foreach(test_name IN ITEMS solver_validity_test solver_failure_test aimer_delay_test)
  add_executable(${test_name} ../${test_name}.cpp)
  target_link_libraries(${test_name} PRIVATE tracking_test_support)
  add_test(NAME ${test_name} COMMAND ${test_name} "${CMAKE_CURRENT_BINARY_DIR}/tracker_test.yaml")
  set_tests_properties(${test_name} PROPERTIES TIMEOUT 30)
endforeach()
