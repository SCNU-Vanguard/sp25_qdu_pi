cmake_minimum_required(VERSION 3.16)

# [9.9-1] 全向感知通过 auto_aim 检测入口工作，不再直接依赖 OpenVINO。
# 源码保留，根 CMake 在新检测后端接入前不添加本目录。

add_library(omniperception OBJECT 
    decider.cpp
    perceptron.cpp
)
target_link_libraries(omniperception auto_aim)
