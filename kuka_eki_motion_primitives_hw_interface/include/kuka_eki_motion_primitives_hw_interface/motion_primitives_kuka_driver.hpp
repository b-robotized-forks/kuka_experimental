// Copyright (c) 2025, b»robotized
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Mathias Fuhrer

#ifndef KUKA_EKI_MOTION_PRIMITIVES_HW_INTERFACE__MOTION_PRIMITIVES_KUKA_DRIVER_HPP_
#define KUKA_EKI_MOTION_PRIMITIVES_HW_INTERFACE__MOTION_PRIMITIVES_KUKA_DRIVER_HPP_

#include <string>
#include <vector>

#include "kuka_eki_motion_primitives_hw_interface/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "motion_primitives_forward_controller/execution_state.hpp"

// #include "kuka_eki_hw_interface/eki_helper.hpp"
#include "aip/rbt/Robot.h"


namespace kuka_eki_motion_primitives_hw_interface
{
class MotionPrimitivesKukaDriver : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_joint_states_;
  std::vector<double> hw_mo_prim_states_;
  std::vector<double> hw_mo_prim_commands_;

  std::atomic<int8_t> current_execution_status_{ExecutionState::IDLE};
  std::atomic_bool ready_for_new_primitive_{false}; // Flag to indicate if the hw-interface is ready for a new motion primitive

  std::string robot_ip_;
  int eki_robot_port_;
  int eki_robot_meta_port_;

  // int eki_cmd_buff_len_;
  // int eki_max_cmd_buff_len_ = 5;  // by default, limit command buffer to 5 (size of advance run in KRL)

  // EKI socket read/write
  // int eki_read_state_timeout_ = 100;  // [s]; settable by parameter (default = 5)
  // boost::asio::io_service ios_;
  // std::unique_ptr<boost::asio::deadline_timer> deadline_;
  // boost::asio::ip::udp::endpoint eki_server_endpoint_;
  // std::shared_ptr<boost::asio::ip::udp::socket> eki_server_socket_;

  // bool eki_write_command(const std::vector<double> &joint_position);

  rbt::Robot robot_;
};

}  // namespace kuka_eki_motion_primitives_hw_interface

#endif  // KUKA_EKI_MOTION_PRIMITIVES_HW_INTERFACE__MOTION_PRIMITIVES_KUKA_DRIVER_HPP_
