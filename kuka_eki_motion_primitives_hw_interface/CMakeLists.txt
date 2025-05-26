cmake_minimum_required(VERSION 3.8)
project(kuka_eki_motion_primitives_hw_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=unused-but-set-variable -Werror=return-type)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(motion_primitives_forward_controller REQUIRED)
find_package(angles REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

add_subdirectory(eki_communication)

add_library(
  kuka_eki_motion_primitives_hw_interface
  SHARED
  src/motion_primitives_kuka_driver.cpp
)

target_include_directories(
  kuka_eki_motion_primitives_hw_interface
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/eki_communication/include
)

target_link_libraries(
  kuka_eki_motion_primitives_hw_interface
  eki_communication
)

ament_target_dependencies(
  kuka_eki_motion_primitives_hw_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  motion_primitives_forward_controller
  TinyXML
  angles
)

# prevent pluginlib from using boost
target_compile_definitions(kuka_eki_motion_primitives_hw_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface kuka_eki_motion_primitives_hw_interface.xml)

install(
  TARGETS
  kuka_eki_motion_primitives_hw_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  kuka_eki_motion_primitives_hw_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Install Python execs
ament_python_install_package(${PROJECT_NAME})

# Install executables
install(PROGRAMS
  examples/send_dummy_motion_primitives.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
