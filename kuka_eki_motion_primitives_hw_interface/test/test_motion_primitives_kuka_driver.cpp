#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>  // Für Node

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestMotionPrimitivesKukaDriver : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Optional: initialize rclcpp if not already done
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Dummy-Node für Logging/Clock Interfaces
    node_ = std::make_shared<rclcpp::Node>("test_motion_primitives_kuka_driver");

    motion_primitives_kuka_driver_2dof_ =
      R"(
        <ros2_control name="MotionPrimitivesKukaDriver2dof" type="system">
          <hardware>
            <plugin>kuka_eki_motion_primitives_hw_interface/MotionPrimitivesKukaDriver</plugin>
          </hardware>
          <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <param name="initial_position">1.57</param>
          </joint>
          <joint name="joint2">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <param name="initial_position">0.7854</param>
          </joint>
        </ros2_control>
    )";
  }

  std::string motion_primitives_kuka_driver_2dof_;
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestMotionPrimitivesKukaDriver, load_motion_primitives_kuka_driver_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + motion_primitives_kuka_driver_2dof_ +
              ros2_control_test_assets::urdf_tail;

  auto clock_iface = node_->get_node_clock_interface();
  auto log_iface = node_->get_node_logging_interface();

  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf, clock_iface, log_iface));
}
