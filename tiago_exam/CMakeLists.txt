cmake_minimum_required(VERSION 3.4.0)
project(tiago_exam)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
  PROGRAMS
    scripts/tuck_arm.py
    scripts/task2_navigator.py
    scripts/task3_pick_place.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
