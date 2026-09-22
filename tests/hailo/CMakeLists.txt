cmake_minimum_required(VERSION 3.16)
project(sp25_hailo_tests LANGUAGES CXX)

# [9.9-2] 独立测试入口：主机输出合同测试只需 OpenCV，便于在无 NPU 的开发机验证。
find_package(OpenCV REQUIRED COMPONENTS core)
add_executable(hailo_output_test
    ../hailo_output_test.cpp
    ../../tasks/auto_aim/hailo/hailo_output.cpp)
target_include_directories(hailo_output_test PRIVATE ../.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hailo_output_test PRIVATE ${OpenCV_LIBS})
target_compile_features(hailo_output_test PRIVATE cxx_std_17)
enable_testing()
add_test(NAME hailo_output_contract COMMAND hailo_output_test)

# [9.9-3] Decoder/Armor 使用实际生产源码，无需 SDK、模型或设备即可验证后处理。
find_package(Eigen3 REQUIRED)
add_executable(hailo_decoder_test
    ../hailo_decoder_test.cpp
    ../../tasks/auto_aim/hailo/hailo_decoder.cpp
    ../../tasks/auto_aim/armor.cpp
    ../../tasks/auto_aim/hailo/hailo_output.cpp)
target_include_directories(hailo_decoder_test PRIVATE ../.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hailo_decoder_test PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
target_compile_features(hailo_decoder_test PRIVATE cxx_std_17)
add_test(NAME hailo_decoder_contract COMMAND hailo_decoder_test)

# [9.9-4] 单独检验语义构造，确保绕过 Decoder 的调用也不能输入退化几何。
add_executable(armor_constructor_test ../armor_constructor_test.cpp ../../tasks/auto_aim/armor.cpp)
target_include_directories(armor_constructor_test PRIVATE ../.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(armor_constructor_test PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
target_compile_features(armor_constructor_test PRIVATE cxx_std_17)
add_test(NAME armor_constructor_contract COMMAND armor_constructor_test)

# [9.9-3][Pi接入] 模型 tail/语义与完整 YOLO 前后处理分别验证，后者仅替换硬件 I/O。
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(yaml-cpp REQUIRED)
add_library(hailo_model_test_support STATIC
    ../../tasks/auto_aim/lightbar_refiner.cpp # [Pi灯条] 测试真实生产实现，不使用重复实现。
    ../../tasks/auto_aim/hailo/szu_int16_model.cpp
    ../../tasks/auto_aim/hailo/hailo_decoder.cpp
    ../../tasks/auto_aim/hailo/hailo_output.cpp
    ../../tasks/auto_aim/armor.cpp)
target_include_directories(hailo_model_test_support PUBLIC ../.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hailo_model_test_support PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)
target_compile_features(hailo_model_test_support PUBLIC cxx_std_17)
add_executable(szu_int16_model_test ../szu_int16_model_test.cpp)
target_link_libraries(szu_int16_model_test PRIVATE hailo_model_test_support)
add_test(NAME szu_int16_model_contract COMMAND szu_int16_model_test)
add_executable(yolo_pipeline_test ../yolo_pipeline_test.cpp ../../tasks/auto_aim/yolo.cpp)
target_link_libraries(yolo_pipeline_test PRIVATE hailo_model_test_support yaml-cpp)
add_test(NAME yolo_pipeline_contract COMMAND yolo_pipeline_test ${CMAKE_CURRENT_BINARY_DIR})

# [Pi灯条] 原图灯条匹配、角点重建/去重、歧义与错误配置的离线回归。
add_executable(lightbar_refiner_test ../lightbar_refiner_test.cpp)
target_link_libraries(lightbar_refiner_test PRIVATE hailo_model_test_support)
add_test(NAME lightbar_refiner_contract COMMAND lightbar_refiner_test)

# [9.9-2] Pi 可单独构建 HEF/图片验证程序，不依赖完整自瞄的 IO、JSON、标定配置。
# [Pi-4.24] 版本约束集中在生产 Hailo 模块，测试入口不另存版本号。
# [Pi视频A] 在同一独立工程增加无窗口视频验收，仅该硬件测试分支增加 videoio 依赖。
option(SP_BUILD_HAILO_DEVICE_TEST "Build the HailoRT HEF/image/video tests" OFF)
# [Pi主线C] 可选真实相机+Hailo入口；默认关闭，原5项合同测试不新增SDK依赖。
option(SP_BUILD_HAILO_CAMERA_TEST "Build the headless HikRobot + Hailo live test" OFF)
if(SP_BUILD_HAILO_CAMERA_TEST AND NOT SP_BUILD_HAILO_DEVICE_TEST)
    message(FATAL_ERROR "SP_BUILD_HAILO_CAMERA_TEST requires SP_BUILD_HAILO_DEVICE_TEST=ON")
endif()
if(SP_BUILD_HAILO_DEVICE_TEST)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)
    add_subdirectory(../../tasks/auto_aim/hailo runtime)
    add_executable(hailo_runtime_test ../hailo_runtime_test.cpp)
    target_link_libraries(hailo_runtime_test PRIVATE hailo_runtime)
    # [Pi接入] 此目标链接真实 HailoRT，不链接上面的测试替身。
    add_executable(hailo_detector_test ../hailo_detector_test.cpp)
    target_link_libraries(hailo_detector_test PRIVATE hailo_detector)
    # [Pi视频A] 复用真实生产检测库，不引入 Plotter、waitKey 或另一套检测实现。
    add_executable(hailo_detector_video_test ../hailo_detector_video_test.cpp)
    target_link_libraries(hailo_detector_video_test PRIVATE hailo_detector ${OpenCV_LIBS})
    # [Pi主线C] 编译同一份生产相机/YOLO和已有测试入口，只链接海康，不拉入整套IO/通信依赖。
    if(SP_BUILD_HAILO_CAMERA_TEST)
        find_package(Threads REQUIRED)
        find_package(spdlog REQUIRED)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
            set(hik_arch arm64)
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
            set(hik_arch amd64)
        else()
            message(FATAL_ERROR "Unsupported HikRobot SDK architecture: ${CMAKE_SYSTEM_PROCESSOR}")
        endif()
        set(HIK_SDK_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../io/hikrobot/lib/${hik_arch}/libMvCameraControl.so"
            CACHE FILEPATH "HikRobot MVS library used by hailo_camera_test")
        if(NOT EXISTS "${HIK_SDK_LIBRARY}")
            message(FATAL_ERROR "HIK_SDK_LIBRARY does not exist: ${HIK_SDK_LIBRARY}")
        endif()
        add_executable(hailo_camera_test ../camera_detect_test.cpp
            ../../io/hikrobot/hikrobot.cpp ../../io/camera_frame.cpp ../../tools/logger.cpp)
        target_compile_definitions(hailo_camera_test PRIVATE SP_HIK_ONLY)
        target_include_directories(hailo_camera_test PRIVATE ../../io/hikrobot/include)
        target_link_libraries(hailo_camera_test PRIVATE hailo_detector Threads::Threads
            spdlog::spdlog "${HIK_SDK_LIBRARY}")
    endif()
endif()
