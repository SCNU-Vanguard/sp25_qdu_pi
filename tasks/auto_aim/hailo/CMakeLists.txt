cmake_minimum_required(VERSION 3.16)

# [9.9-2][Pi-4.24] 按 Pi 当前开发包、已加载驱动和运行库核对结果使用 4.24.0。
find_package(HailoRT 4.24.0 EXACT REQUIRED)
message(STATUS "SP25 HailoRT build version: ${HailoRT_VERSION}") # [Pi-4.24] 明确显示实际构建版本。
add_library(hailo_runtime STATIC hailo_runtime.cpp hailo_output.cpp)
# [Pi-4.24] 将找到的构建版本传给启动检查，避免 CMake 和 C++ 各手写一份版本号。
target_compile_definitions(hailo_runtime PRIVATE
    SP_HAILORT_BUILD_VERSION="${HailoRT_VERSION}")
# [9.9-2] 路径相对本模块，允许独立测试工程复用同一生产实现。
target_include_directories(hailo_runtime PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hailo_runtime PUBLIC ${OpenCV_LIBS} PRIVATE HailoRT::libhailort)
target_compile_features(hailo_runtime PUBLIC cxx_std_17)

# [9.9-2][9.9-3][Pi接入] 独立检测库供原 SP25 主循环与首次图片验收共用，不依赖 IO。
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(hailo_detector STATIC
    ../yolo.cpp ../armor.cpp hailo_decoder.cpp szu_int16_model.cpp
    ../lightbar_refiner.cpp) # [Pi灯条] 所有现有YOLO调用方共用同一修正实现。
target_link_libraries(hailo_detector PUBLIC hailo_runtime Eigen3::Eigen yaml-cpp)
