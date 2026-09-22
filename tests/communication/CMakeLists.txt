cmake_minimum_required(VERSION 3.16)
project(sp25_qdu_communication_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# [9.21-QDU] This focused target validates the wire codec without camera/Hailo dependencies.
add_executable(
  qdu_shared_topic_protocol_test
  qdu_shared_topic_protocol_test.cpp
  ../../io/qdu_shared_topic_protocol.cpp)
target_include_directories(qdu_shared_topic_protocol_test PRIVATE ../..)

enable_testing()
add_test(NAME qdu_shared_topic_protocol_test COMMAND qdu_shared_topic_protocol_test)

# [9.21-QDU] The Linux/Pi-only monitor exercises the real serial reconnect worker in RX-only mode.
if(UNIX)
  find_package(Eigen3 REQUIRED)
  find_package(yaml-cpp REQUIRED)
  find_package(spdlog REQUIRED)
  find_package(fmt REQUIRED)
  find_package(Threads REQUIRED)
  add_subdirectory(../../io/serial serial)
  add_executable(
    qdu_link_monitor
    qdu_link_monitor.cpp
    ../../io/cboard.cpp
    ../../io/gimbal/gimbal.cpp
    ../../io/qdu_shared_topic.cpp
    ../../io/qdu_shared_topic_protocol.cpp
    ../../tools/logger.cpp)
  target_include_directories(qdu_link_monitor PRIVATE ../.. ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(qdu_link_monitor PRIVATE yaml-cpp spdlog::spdlog fmt::fmt serial Threads::Threads)

  # [9.21-QDU-SIM] Pseudo-terminal C-board emulator used by the read-only full-chain acceptance
  # entry.  It only needs the production wire codec, so it does not pull in the serial worker,
  # camera or Hailo.
  add_executable(
    qdu_board_simulator
    qdu_board_simulator.cpp
    ../../io/qdu_shared_topic_protocol.cpp
    ../../tools/logger.cpp)
  target_include_directories(qdu_board_simulator PRIVATE ../.. ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(qdu_board_simulator PRIVATE spdlog::spdlog fmt::fmt Threads::Threads)

  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    # [9.21-QDU] PTY simulation verifies disconnect detection, reopen, parser reset, and no TX.
    add_test(
      NAME qdu_hotplug_sim_test
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qdu_hotplug_sim_test.py
              $<TARGET_FILE:qdu_link_monitor>)
    set_tests_properties(qdu_hotplug_sim_test PROPERTIES TIMEOUT 20)
    # [9.21-QDU-NORMAL-TX] Simultaneously feed IMU packets and verify a real nonzero CBoard command
    # plus fire=false on the opposite side of the pseudo-terminal.
    add_test(
      NAME qdu_normal_tx_sim_test
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qdu_normal_tx_sim_test.py
              $<TARGET_FILE:qdu_link_monitor>)
    set_tests_properties(qdu_normal_tx_sim_test PROPERTIES TIMEOUT 15)
    # [9.21-QDU-SIM] Pair the pseudo-terminal C-board emulator with the read-only serial worker and
    # assert live quaternions decode cleanly while the consumer transmits nothing at all.
    add_test(
      NAME qdu_readonly_sim_test
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qdu_readonly_sim_test.py
              $<TARGET_FILE:qdu_board_simulator> $<TARGET_FILE:qdu_link_monitor>)
    set_tests_properties(qdu_readonly_sim_test PROPERTIES TIMEOUT 40)
    # [9.21-QDU-SIM] The emulator stops publishing while keeping the port open; the link must then
    # declare attitude stale on its own rather than silently reusing the last quaternion.
    add_test(
      NAME qdu_stall_sim_test
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qdu_stall_sim_test.py
              $<TARGET_FILE:qdu_board_simulator> $<TARGET_FILE:qdu_link_monitor>)
    set_tests_properties(qdu_stall_sim_test PROPERTIES TIMEOUT 40)
    # [9.21-QDU-SIM] The other half of the staleness story: a port that opens cleanly and then never
    # delivers a single quaternion, which is the state the real board is in.  imu_age_ms() must
    # report -1 rather than a large positive age, and imu_fresh must stay 0.
    add_test(
      NAME qdu_silent_board_sim_test
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qdu_silent_board_sim_test.py
              $<TARGET_FILE:qdu_board_simulator> $<TARGET_FILE:qdu_link_monitor>)
    set_tests_properties(qdu_silent_board_sim_test PROPERTIES TIMEOUT 40)
  endif()
endif()
