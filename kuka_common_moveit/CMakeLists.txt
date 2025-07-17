cmake_minimum_required(VERSION 3.22)
project(kuka_common_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wait_for_robot_description
  DESTINATION lib/${PROJECT_NAME}
)