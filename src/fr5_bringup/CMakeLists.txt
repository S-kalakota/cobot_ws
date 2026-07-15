cmake_minimum_required(VERSION 3.16)
project(fr5_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/a1_move_and_read.py
  scripts/a2_gripper.py
  scripts/a2_tcp_calibrate.py
  scripts/a3_measure_workspace.py
  scripts/a3_planning_scene.py
  scripts/a3_tcp_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
