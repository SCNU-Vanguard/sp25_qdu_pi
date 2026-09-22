cmake_minimum_required(VERSION 3.16)

# [9.9-1] 物理移除旧分类器、Detector、YOLO 后端和多线程推理链。
# [Pi接入] 保留 SP25 上层算法；检测入口由 hailo_detector 实现。

add_subdirectory(planner/tinympc)
# [9.9-4][Pi接入] 运行时、模型 tail、Decoder 和 Armor 共用独立检测库。
add_subdirectory(hailo)

add_library(auto_aim OBJECT 
    # [9.9-3][Pi接入] Armor/Decoder 已归入 hailo_detector，避免重复编译/定义。
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    planner/planner.cpp
)

target_link_libraries(auto_aim io tinympcstatic hailo_detector)
