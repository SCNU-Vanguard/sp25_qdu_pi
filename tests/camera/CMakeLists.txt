cmake_minimum_required(VERSION 3.16)
project(sp25_camera_tests LANGUAGES CXX)

# [9.9-5][9.9-6] 离线测试编译真实生产源码，仅设备 API 由测试文件提供，不链接假驱动到产品。
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
add_library(camera_pool STATIC ../../io/camera_frame.cpp)
target_include_directories(camera_pool PUBLIC ../.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_pool PUBLIC ${OpenCV_LIBS} Threads::Threads)
target_compile_features(camera_pool PUBLIC cxx_std_17)
enable_testing()
add_executable(camera_frame_test ../camera_frame_test.cpp)
target_link_libraries(camera_frame_test PRIVATE camera_pool)
add_test(NAME camera_latest_frame COMMAND camera_frame_test)

add_executable(hikrobot_driver_test ../hikrobot_driver_test.cpp ../../io/hikrobot/hikrobot.cpp)
target_include_directories(hikrobot_driver_test PRIVATE ../../io/hikrobot/include)
target_link_libraries(hikrobot_driver_test PRIVATE camera_pool yaml-cpp spdlog::spdlog)
add_test(NAME hikrobot_driver_lifecycle COMMAND hikrobot_driver_test)
set_tests_properties(camera_latest_frame hikrobot_driver_lifecycle PROPERTIES TIMEOUT 30)

# [9.9-6] Pi 上单独编译相机验收程序；只链接海康，不初始化 NPU/云台或其他相机。
option(SP_BUILD_HIK_DEVICE_TEST "Build HikRobot USB camera device test" OFF)
if(SP_BUILD_HIK_DEVICE_TEST)
  find_package(OpenCV REQUIRED COMPONENTS core highgui)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(hik_arch arm64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
    set(hik_arch amd64)
  else()
    message(FATAL_ERROR "Unsupported HikRobot SDK architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()
  # 可用 -DHIK_SDK_LIBRARY=/实际/MVS/lib/...so 指定 Pi 已安装且与头文件匹配的 SDK。
  set(HIK_SDK_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../io/hikrobot/lib/${hik_arch}/libMvCameraControl.so"
      CACHE FILEPATH "HikRobot MVS library")
  add_executable(hik_camera_test ../camera_test.cpp ../../io/hikrobot/hikrobot.cpp
                 ../../tools/exiter.cpp ../../tools/logger.cpp)
  target_compile_definitions(hik_camera_test PRIVATE SP_HIK_ONLY)
  target_include_directories(hik_camera_test PRIVATE ../../io/hikrobot/include)
  target_link_libraries(hik_camera_test PRIVATE camera_pool yaml-cpp spdlog::spdlog
                        ${OpenCV_LIBS} "${HIK_SDK_LIBRARY}")
endif()
