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
#include <queue>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "motion_primitives_forward_controller/motion_primitives_forward_controller.hpp"
#include "eki_communication/Robot.h"
#include "control_msgs/msg/motion_primitive.hpp"

using MoprimExecutionState = motion_primitives_forward_controller::ExecutionState;
using MoprimMotionType = control_msgs::msg::MotionPrimitive;
using MoprimMotionHelperType = motion_primitives_forward_controller::MotionHelperType;

namespace kuka_eki_motion_primitives_hw_interface
{
class MotionPrimitivesKukaDriver : public hardware_interface::SystemInterface
{
public:
  virtual ~MotionPrimitivesKukaDriver();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_joint_pos_states_;
  std::vector<double> hw_joint_vel_states_;
  std::vector<double> hw_joint_eff_states_;
  std::vector<double> hw_mo_prim_states_;
  std::vector<double> hw_mo_prim_commands_;

  MoprimExecutionState current_execution_status_{MoprimExecutionState::IDLE};
  std::atomic_bool ready_for_new_primitive_{false}; // Flag to indicate if the hw-interface is ready for a new motion primitive

  std::queue<int> checkCommandIdDoneQueue;  // Queue to check if a command with a specific ID is done to return SUCCESS
  
  std::string robot_ip_;
  int eki_robot_port_;
  int eki_robot_meta_port_;

  rbt::Robot robot_;
  bool robot_stopped_{false};
  bool robot_error_{false};

  // Async thread handling
  std::unique_ptr<std::thread> async_execute_motion_thread_;
  std::shared_ptr<std::thread> async_stop_motion_thread_;
  std::atomic_bool async_thread_shutdown_{ false };
  std::mutex execution_mutex_;
  std::mutex stop_mutex_;

  std::atomic_bool new_execution_available_{ false };
  std::atomic_bool new_stop_available_{ false };
  std::atomic_bool new_reset_available_{ false };
  void asyncExecuteMotionThread();
  void asyncStopMotionThread();

  std::atomic_bool build_motion_sequence_{false};   // flag to put all following primitives into a motion sequence instead of sending single primitives
  bool add_linear_joint_cmd();
  bool add_linear_cartesian_cmd();
  bool add_circular_cartesian_cmd();
  void add_vel_and_acc_to_command(rbt::MoveCommand &command);
  void add_blending_to_command(rbt::MoveCommand &command);
  void reset_command_interfaces();
  void quaternionToEuler(double qx, double qy, double qz, double qw, double& rx, double& ry, double& rz);
};

}  // namespace kuka_eki_motion_primitives_hw_interface

#endif  // KUKA_EKI_MOTION_PRIMITIVES_HW_INTERFACE__MOTION_PRIMITIVES_KUKA_DRIVER_HPP_
