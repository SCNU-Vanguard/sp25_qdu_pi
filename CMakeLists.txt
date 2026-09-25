cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
# [9.9-2][Pi-4.24] HailoRT 的版本约束、查找与链接集中在 tasks/auto_aim/hailo。

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
# [9.9-1] 全向感知源码保留，依赖检测入口，待 Hailo 接入后恢复构建。

####################src###################
# [9.9-1] 删除 mt_standard / mt_auto_aim_debug 旧推理链。
# standard、standard_mpc、auto_aim_debug_mpc、uav、uav_debug 及打符入口源码保留。
# [9.9-4][Pi接入] YOLO 接通后恢复纯自瞄主循环；打符入口仍缺对应 HEF，保持不构建。
# 实际运行还需本车相机/IMU 标定、通信与配置验收，首次上板先使用 hailo_detector_test。
add_executable(standard src/standard.cpp)
target_link_libraries(standard PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
# [9.21-QDU-READONLY] Read-only full-chain acceptance entry: the same production chain as standard,
# but the QDU link is constructed with force_read_only=true so no command can ever be transmitted.
# Use it with qdu_board_simulator to exercise Tracker/Solver/Aimer on live attitude before any
# gimbal or launcher is connected.  It is not auto-aim acceptance and proves nothing about real
# C-board RX/TX.
add_executable(standard_readonly src/standard_readonly.cpp)
target_link_libraries(standard_readonly PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
add_executable(auto_aim_test tests/auto_aim_test.cpp)
target_link_libraries(auto_aim_test PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################tests##################
# [9.9-2] HEF + 图片运行时验证只链接 Hailo 和 OpenCV，不初始化相机、云台或开火链。
add_executable(hailo_runtime_test tests/hailo_runtime_test.cpp)
target_link_libraries(hailo_runtime_test PRIVATE hailo_runtime)
add_executable(hailo_detector_test tests/hailo_detector_test.cpp) # [Pi接入] 完整检测入口图片验收。
target_link_libraries(hailo_detector_test PRIVATE hailo_detector)
# [9.9-1] 保留单路检测测试源码，待新后端接入；删除旧多线程推理专用测试。
add_executable(camera_test tests/camera_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)

target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
# [9.9-1] 只有在找到 ROS 环境时才会添加通信测试。
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    # [9.9-1] 哨兵入口依赖检测，暂不构建；ROS 通信测试继续保留。
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()
