cmake_minimum_required(VERSION 3.16)
# [9.9-1] 删除打符 OpenVINO 实现及依赖；保留解算、预测、瞄准算法。
# buff_detector.cpp 依赖尚未接入的新推理实现，暂不进入构建。
# [9.9-1] 源码也没有 Ceres 调用或头文件引用，一并移除无用的构建依赖。

add_library(auto_buff OBJECT 
    buff_type.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
)

target_link_libraries(auto_buff auto_aim)
