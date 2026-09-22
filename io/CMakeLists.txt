cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED) # [9.9-5] 显式声明采集/同步线程依赖。

add_subdirectory(serial)

# [9.21-PI-HIK-ONLY] The deployed robot has one HikRobot camera.  Keep exactly that backend in the
# production IO target so unrelated vendor SDKs cannot initialize inside the same process.
add_library(io STATIC 
    hikrobot/hikrobot.cpp
    camera.cpp
    camera_frame.cpp # [9.9-5] 固定槽位池和最新帧同步，不改通用队列。
    cboard.cpp
    qdu_shared_topic_protocol.cpp # [9.21-QDU] QDU libxr SharedTopic 字节协议。
    qdu_shared_topic.cpp # [9.21-QDU] DevC-USB 链路、失效门控与热插拔重连。
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(SP_HIKROBOT_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/amd64/libMvCameraControl.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(SP_HIKROBOT_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64/libMvCameraControl.so")
  # [9.21-PI-HIK-SDK] MvCameraControl loads these USB/image modules with dlopen at runtime.
  # Fail during CMake configuration instead of reaching MV_E_LOAD_LIBRARY (0x8000000c) on the robot.
  set(SP_HIKROBOT_RUNTIME_FILES
      "${SP_HIKROBOT_LIBRARY}"
      "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64/libMvUsb3vTL.so"
      "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64/libFormatConversion.so"
      "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64/libMediaProcess.so"
      "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64/libMVRender.so"
      "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64/libMvSDKVersion.so"
      "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/arm64/CommonParameters.ini")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()
if(NOT DEFINED SP_HIKROBOT_RUNTIME_FILES)
  set(SP_HIKROBOT_RUNTIME_FILES "${SP_HIKROBOT_LIBRARY}")
endif()
foreach(SP_HIKROBOT_RUNTIME_FILE IN LISTS SP_HIKROBOT_RUNTIME_FILES)
  if(NOT EXISTS "${SP_HIKROBOT_RUNTIME_FILE}")
    message(FATAL_ERROR "HikRobot runtime component not found: ${SP_HIKROBOT_RUNTIME_FILE}")
  endif()
endforeach()

# [9.21-PI-HIK-SDK] Link the repository's verified runtime by absolute path.  The former bare
# MvCameraControl name could silently select another installed SDK; camera_test then failed in
# MV_CC_EnumDevices with MV_E_RESOURCE while the focused test using the verified runtime worked.
target_link_libraries(io "${SP_HIKROBOT_LIBRARY}" yaml-cpp serial Threads::Threads)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND OR NOT sp_msgs_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

    # 设置 ROS2 依赖
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs sp_msgs)
endif()
