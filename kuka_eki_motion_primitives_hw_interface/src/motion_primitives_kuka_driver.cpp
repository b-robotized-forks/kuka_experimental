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

#include <limits>
#include <vector>

#include "kuka_eki_motion_primitives_hw_interface/motion_primitives_kuka_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "motion_primitives_forward_controller/motion_type.hpp"

namespace kuka_eki_motion_primitives_hw_interface
{
hardware_interface::CallbackReturn MotionPrimitivesKukaDriver::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Initializing Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Failed to initialize SystemInterface");
    return CallbackReturn::ERROR;
  }

  info_ = info;

  // Joint states for RViz, ...
  hw_joint_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // State interfaces for the motion_primitive_forward_controller
  hw_mo_prim_states_.resize(2, std::numeric_limits<double>::quiet_NaN());     // execution_status, ready_for_new_primitive
  hw_mo_prim_commands_.resize(25, std::numeric_limits<double>::quiet_NaN());  // motion_type + 6 joints + 2*7 positions + blend_radius + velocity + acceleration + move_time

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionPrimitivesKukaDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Configuring Hardware Interface");

  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotionPrimitivesKukaDriver::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Exporting State Interfaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  // State interfaces with joint states for RViz, ...
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_states_[i]));
  }

  // State interfaces for the motion_primitive_forward_controller
  state_interfaces.emplace_back(hardware_interface::StateInterface("motion_primitive", "execution_status", &hw_mo_prim_states_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("motion_primitive", "ready_for_new_primitive", &hw_mo_prim_states_[1]));

  for (const auto &state_interface : state_interfaces)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("MotionPrimitivesKukaDriver"),
      "State Interface: Name = %s",
      state_interface.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotionPrimitivesKukaDriver::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Exporting Command Interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Command for motion type (motion_type)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "motion_type", &hw_mo_prim_commands_[0]));
  // Joint position commands (q1, q2, ..., q6)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q1", &hw_mo_prim_commands_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q2", &hw_mo_prim_commands_[2]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q3", &hw_mo_prim_commands_[3]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q4", &hw_mo_prim_commands_[4]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q5", &hw_mo_prim_commands_[5]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q6", &hw_mo_prim_commands_[6]));
  // Position commands (pos_x, pos_y, pos_z, pos_qx, pos_qy, pos_qz, pos_qz)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_x", &hw_mo_prim_commands_[7]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_y", &hw_mo_prim_commands_[8]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_z", &hw_mo_prim_commands_[9]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qx", &hw_mo_prim_commands_[10]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qy", &hw_mo_prim_commands_[11]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qz", &hw_mo_prim_commands_[12]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qw", &hw_mo_prim_commands_[13]));
  // Via Position commands for circula motion
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_x", &hw_mo_prim_commands_[14]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_y", &hw_mo_prim_commands_[15]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_z", &hw_mo_prim_commands_[16]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qx", &hw_mo_prim_commands_[17]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qy", &hw_mo_prim_commands_[18]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qz", &hw_mo_prim_commands_[19]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qw", &hw_mo_prim_commands_[20]));
  // Other command parameters (blend_radius, velocity, acceleration, move_time)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "blend_radius", &hw_mo_prim_commands_[21]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "velocity", &hw_mo_prim_commands_[22]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "acceleration", &hw_mo_prim_commands_[23]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "move_time", &hw_mo_prim_commands_[24]));

  // Print all command interfaces with ROS info
  for (const auto &command_interface : command_interfaces)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("MotionPrimitivesKukaDriver"),
      "Command Interface: Name = %s",
      command_interface.get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MotionPrimitivesKukaDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Activating Hardware Interface");
  ready_for_new_primitive_ = true; // set to true to allow sending new commands

  robot_ip_ = info_.hardware_parameters["robot_ip"];
  eki_robot_port_ = std::stoi(info_.hardware_parameters["eki_robot_port"]);
  // eki_robot_meta_port_ = std::stoi(info_.hardware_parameters["eki_robot_meta_port"]);
  eki_robot_meta_port_ = 0; // TODO(mathias31415): Read parameter instead of hardcodeing
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Trying to connect to the host: [%s], port: [%d], meta_port: [%d]", robot_ip_.c_str(), eki_robot_port_, eki_robot_meta_port_);
  if (robot_ip_.empty() || eki_robot_port_ == 0)
  {
      RCLCPP_ERROR(
          rclcpp::get_logger("MotionPrimitivesKukaDriver"), "robot_ip and eki_robot_port cannot be empty");
      return CallbackReturn::ERROR;
  }

  robot_.connect_async(robot_ip_, eki_robot_port_, eki_robot_meta_port_);

  // TODO(mathias31415): Check this code block
  // robot_.listener = [this](rbt::RobotEvent event, rbt::Robot *robot) -> void {
  //   switch (event)
  //   {
  //   case rbt::RobotEvent::CONNECT:
  //       RCLCPP_INFO(get_logger(), "Robot connected"); 
  //       break;
  //   case rbt::RobotEvent::RUN:
  //       RCLCPP_INFO(get_logger(), "Robot running"); 
  //       break;
  //   case rbt::RobotEvent::STATE:
  //       break;
  //   }
  // };



  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "System Successfully activated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionPrimitivesKukaDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Deactivating Hardware Interface");
  robot_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "System Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotionPrimitivesKukaDriver::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rbt::RobotState robot_state = robot_.get_state();
  const rbt::PoseJoints& joints = robot_state.position_joints;

  hw_joint_states_[0] = joints.a1;
  hw_joint_states_[1] = joints.a2;
  hw_joint_states_[2] = joints.a3;
  hw_joint_states_[3] = joints.a4;
  hw_joint_states_[4] = joints.a5;
  hw_joint_states_[5] = joints.a6;

  // TODO(mathias31415): Fill execution status and ready_for_new_primitive_ with real data
  hw_mo_prim_states_[0] = current_execution_status_;    // 0=idle, 1=executing, 2=success, 3=error
  hw_mo_prim_states_[1] = static_cast<double>(ready_for_new_primitive_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotionPrimitivesKukaDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check if we have a new command
  if (!std::isnan(hw_mo_prim_commands_[0])) {
    RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Command of type: %f recived", hw_mo_prim_commands_[0]);
    ready_for_new_primitive_ = false; // set to false to indicate that the driver is busy handeling a command
    double motion_type = hw_mo_prim_commands_[0];
    // TODO(mathias31415): Handle new commands --> extra thread needed?
    switch (static_cast<uint8_t>(motion_type)) 
    {
      case MotionType::STOP_MOTION: {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "STOP_MOTION command received (handeling not implemented yet)");
        // TODO(mathias31415): Handle STOP_MOTION command

        current_execution_status_ = ExecutionState::IDLE;
        // Reset command interfaces
        std::fill(hw_mo_prim_commands_.begin(), hw_mo_prim_commands_.end(), std::numeric_limits<double>::quiet_NaN());
        ready_for_new_primitive_ = true; // TODO(mathias31415): Only for testing --> adjust later
        break;
      }
      case MotionType::LINEAR_JOINT: { // MoveJ/ PTP
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "LINEAR_JOINT command received");

        // Check if joint positions are valid
        for (int i = 1; i <= 6; ++i) {
          if (std::isnan(hw_mo_prim_commands_[i])) {
              RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: joint positions contain NaN values");
              current_execution_status_ = ExecutionState::ERROR;
              return hardware_interface::return_type::OK;  // TODO(mathias31415): OK or ERROR?
          }
        }
        current_execution_status_ = ExecutionState::EXECUTING;
        // std::vector<double> joint_positions = {hw_mo_prim_commands_[1], hw_mo_prim_commands_[2], hw_mo_prim_commands_[3], hw_mo_prim_commands_[4], hw_mo_prim_commands_[5], hw_mo_prim_commands_[6]};
        constexpr double rad_to_deg = 180.0 / M_PI;
        std::vector<double> joint_positions = {     // get joint positions in degrees
            hw_mo_prim_commands_[1] * rad_to_deg,
            hw_mo_prim_commands_[2] * rad_to_deg,
            hw_mo_prim_commands_[3] * rad_to_deg,
            hw_mo_prim_commands_[4] * rad_to_deg,
            hw_mo_prim_commands_[5] * rad_to_deg,
            hw_mo_prim_commands_[6] * rad_to_deg};
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), 
              "Executing moveJ with joint positions: [%f, %f, %f, %f, %f, %f]", 
              joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5]);
        
        rbt::MoveCommand command;
        command = rbt::MoveCommand(rbt::PoseJoints(joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5], 0.0));   // last 0.0 for not used A7
        robot_.perform(command);
        bool success = robot_.run(); // TODO(mathias31415): Check if single primitive or sequence
        current_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;  // TODO(mathias31415): Its not the execution status, but the send status?
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "After executing moveJ: current_execution_status_ = %d", current_execution_status_.load());
        if(success){
          ready_for_new_primitive_ = true; // set to true to allow sending new commands
        }
        // Reset command interfaces
        std::fill(hw_mo_prim_commands_.begin(), hw_mo_prim_commands_.end(), std::numeric_limits<double>::quiet_NaN());
        break;
      }
      case MotionType::LINEAR_CARTESIAN: { // MoveL/ LIN
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "LINEAR_CARTESIAN command received (handeling not implemented yet)");
        // TODO(mathias31415): Handle LINEAR_CARTESIAN command

        current_execution_status_ = ExecutionState::IDLE;
        // Reset command interfaces
        std::fill(hw_mo_prim_commands_.begin(), hw_mo_prim_commands_.end(), std::numeric_limits<double>::quiet_NaN());
        ready_for_new_primitive_ = true; // TODO(mathias31415): Only for testing --> adjust later
        break;
      }
      case MotionType::CIRCULAR_CARTESIAN: {  // MoveC/ CIRC
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "CIRCULAR_CARTESIAN command received (handeling not implemented yet)");
        // TODO(mathias31415): Handle CIRCULAR_CARTESIAN command

        // Reset command interfaces
        std::fill(hw_mo_prim_commands_.begin(), hw_mo_prim_commands_.end(), std::numeric_limits<double>::quiet_NaN());
        ready_for_new_primitive_ = true; // TODO(mathias31415): Only for testing --> adjust later
        break;
      }
      default: {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: motion type %f is not supported", motion_type);
        current_execution_status_ = ExecutionState::ERROR;
        return hardware_interface::return_type::ERROR;  // TODO(mathias31415): OK or ERROR?
      }
    }
  } 

  // TODO(mathias31415): Keep alive cmd with current joint state only neccesary for eki simulator, causes problem with real robot --> fix simulator instead

  // else {
  //   // Send "keep alive" msg to the robot if no new command is received
  //   // TODO(mathias31415): Check if this is needed? Keep alive without data? No keep alive at all? (eki simulator needs new data to prevent timeout)
  //   // RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "No new command received, sending keep alive msg to robot");
  //   if (eki_cmd_buff_len_ < eki_max_cmd_buff_len_)
  //   {
      // eki_write_command(hw_joint_states_);  // send current joint positions back to the robot
  //     // TODO(mathias31415): Check return value of eki_write_command
  //   }
  // }  
  return hardware_interface::return_type::OK;
}

// bool MotionPrimitivesKukaDriver::eki_write_command(const std::vector<double> &joint_position_command)
// {
//   TiXmlDocument xml_out;
//   TiXmlElement* robot_command = new TiXmlElement("RobotCommand");
//   TiXmlElement* pos = new TiXmlElement("Pos");
//   TiXmlText* empty_text = new TiXmlText("");
//   robot_command->LinkEndChild(pos);
//   pos->LinkEndChild(empty_text);   // force <Pos></Pos> format (vs <Pos />)
//   char axis_name[] = "A1";
//   for (long unsigned int i = 0; i < hw_joint_states_.size(); ++i)
//   {
//       pos->SetAttribute(axis_name, std::to_string(angles::to_degrees(joint_position_command[i])).c_str());
//       axis_name[1]++;
//   }
//   xml_out.LinkEndChild(robot_command);
//   TiXmlPrinter xml_printer;
//   xml_printer.SetStreamPrinting();  // no linebreaks
//   xml_out.Accept(&xml_printer);

//   eki_server_socket_->send_to(boost::asio::buffer(xml_printer.CStr(), xml_printer.Size()),
//                                         eki_server_endpoint_);
//   RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Sent command: %s", xml_printer.CStr());
//   return true;
// }

}  // namespace kuka_eki_motion_primitives_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kuka_eki_motion_primitives_hw_interface::MotionPrimitivesKukaDriver, hardware_interface::SystemInterface)
