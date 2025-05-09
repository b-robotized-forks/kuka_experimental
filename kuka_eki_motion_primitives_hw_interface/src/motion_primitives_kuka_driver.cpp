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
MotionPrimitivesKukaDriver::~MotionPrimitivesKukaDriver()
{
  if (async_execute_motion_thread_ && async_execute_motion_thread_->joinable()) 
  {
    async_execute_motion_thread_->join();
    async_execute_motion_thread_.reset();
  }
}
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

  async_thread_shutdown_ = false;

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

  async_execute_motion_thread_ = std::make_unique<std::thread>(&MotionPrimitivesKukaDriver::asyncExecuteMotionThread, this);

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
  eki_robot_meta_port_ = std::stoi(info_.hardware_parameters["eki_robot_meta_port"]);
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Trying to connect to the host: [%s], port: [%d], meta_port: [%d]", robot_ip_.c_str(), eki_robot_port_, eki_robot_meta_port_);
  if (robot_ip_.empty() || eki_robot_port_ == 0)
  {
      RCLCPP_ERROR(
          rclcpp::get_logger("MotionPrimitivesKukaDriver"), "robot_ip and eki_robot_port cannot be empty");
      return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Connecting to the robot ...");
  robot_.connect_async(robot_ip_, eki_robot_port_, eki_robot_meta_port_);
  robot_.await_connection(); // TODO(mathias31415): Seems to not work with async connect? --> always returns true?
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Connected to the robot.");
  

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
  constexpr double deg_to_rad = M_PI / 180.0;
  hw_joint_states_[0] = joints.a1 * deg_to_rad;
  hw_joint_states_[1] = joints.a2 * deg_to_rad;
  hw_joint_states_[2] = joints.a3 * deg_to_rad;
  hw_joint_states_[3] = joints.a4 * deg_to_rad;
  hw_joint_states_[4] = joints.a5 * deg_to_rad;
  hw_joint_states_[5] = joints.a6 * deg_to_rad;

  // Check execution status
  bool is_executing = robot_.is_active();
  current_execution_status_ = is_executing ? ExecutionState::EXECUTING : ExecutionState::IDLE;
  hw_mo_prim_states_[0] = current_execution_status_;    // 0=idle, 1=executing, 2=success, 3=error
  hw_mo_prim_states_[1] = static_cast<double>(ready_for_new_primitive_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotionPrimitivesKukaDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check if we have a new command
  if (!std::isnan(hw_mo_prim_commands_[0])) {
    // RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Command of type: %f recived", hw_mo_prim_commands_[0]);
    ready_for_new_primitive_ = false; // set to false to indicate that the driver is busy handeling a command
    double motion_type = hw_mo_prim_commands_[0];
    switch (static_cast<uint8_t>(motion_type)) 
    {
      case MotionType::STOP_MOTION: {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "STOP_MOTION command received");
        current_execution_status_ = ExecutionState::STOPPED;
        robot_.abort_commands();
        reset_command_interfaces();
        break;
      }
      case MotionType::RESET_STOP: {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "RESET_STOP command received");
        current_execution_status_ = ExecutionState::IDLE;
        reset_command_interfaces();
        robot_.reset_abort_commands();
        ready_for_new_primitive_ = true;
        break;
      }
      case MotionType::MOTION_SEQUENCE_START: {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Received MOTION_SEQUENCE_START: add all following commands to the motion sequence.");
        build_motion_sequence_ = true;  // set flag to put all following commands into the motion sequence
        reset_command_interfaces();
        ready_for_new_primitive_ = true; // set to true to allow sending new commands
        break;
      }
      case MotionType::MOTION_SEQUENCE_END: {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Received MOTION_SEQUENCE_END: executing motion sequence ...");
        build_motion_sequence_ = false;
        std::lock_guard<std::mutex> guard(execution_mutex_);
        if (!new_execution_available_) {
          new_execution_available_ = true;  // set flag for async thread to send command to robot
        }
        reset_command_interfaces();
        break;
      }
      case MotionType::LINEAR_JOINT: { // MoveJ/ PTP
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "LINEAR_JOINT command received");
        if(!add_linear_joint_cmd()) {
          RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Failed to add LINEAR_JOINT command");
          current_execution_status_ = ExecutionState::ERROR;
          return hardware_interface::return_type::ERROR;
        }
        reset_command_interfaces();
        if(!build_motion_sequence_) { // send single command imimediately
          std::lock_guard<std::mutex> guard(execution_mutex_);
          if (!new_execution_available_) {
            new_execution_available_ = true;  // set flag for async thread to send command to robot
          }
        } else {
          ready_for_new_primitive_ = true; // set to true to allow sending new commands
        }
        break;
      }
      case MotionType::LINEAR_CARTESIAN: { // MoveL/ LIN
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "LINEAR_CARTESIAN command received");
        if(!add_linear_cartesian_cmd()) {
          RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Failed to add LINEAR_CARTESIAN command");
          current_execution_status_ = ExecutionState::ERROR;
          return hardware_interface::return_type::ERROR;
        }
        reset_command_interfaces();
        if(!build_motion_sequence_) { // send single command imimediately
          std::lock_guard<std::mutex> guard(execution_mutex_);
          if (!new_execution_available_) {
            new_execution_available_ = true;  // set flag for async thread to send command to robot
          }
        } else {
          ready_for_new_primitive_ = true; // set to true to allow sending new commands
        }
        break;
      }
      case MotionType::CIRCULAR_CARTESIAN: {  // MoveC/ CIRC
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "CIRCULAR_CARTESIAN command received");
        if(!add_circular_cartesian_cmd()) {
          RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Failed to add CIRCULAR_CARTESIAN command");
          current_execution_status_ = ExecutionState::ERROR;
          return hardware_interface::return_type::ERROR;
        }
        reset_command_interfaces();
        if(!build_motion_sequence_) { // send single command imimediately
          std::lock_guard<std::mutex> guard(execution_mutex_);
          if (!new_execution_available_) {
            new_execution_available_ = true;  // set flag for async thread to send command to robot
          }
        } else {
          ready_for_new_primitive_ = true; // set to true to allow sending new commands
        }
        break;
      }
      default: {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: motion type %f is not supported", motion_type);
        current_execution_status_ = ExecutionState::ERROR;
        return hardware_interface::return_type::ERROR;  // TODO(mathias31415): OK or ERROR?
      }
    }
  } 
  return hardware_interface::return_type::OK;
}

bool MotionPrimitivesKukaDriver::add_linear_joint_cmd()
{
  // Check if joint positions are valid
  for (int i = 1; i <= 6; ++i) {
    if (std::isnan(hw_mo_prim_commands_[i])) {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: joint positions contain NaN values");
        return false;
    }
  }
  constexpr double rad_to_deg = 180.0 / M_PI;
  std::vector<double> joints = {     // get joint positions in degrees
      hw_mo_prim_commands_[1] * rad_to_deg,
      hw_mo_prim_commands_[2] * rad_to_deg,
      hw_mo_prim_commands_[3] * rad_to_deg,
      hw_mo_prim_commands_[4] * rad_to_deg,
      hw_mo_prim_commands_[5] * rad_to_deg,
      hw_mo_prim_commands_[6] * rad_to_deg};
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), 
        "Adding LINEAR_JOINT with joint positions: [%f, %f, %f, %f, %f, %f]", 
        joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
  rbt::MoveCommand command;
  command = rbt::MoveCommand(rbt::PoseJoints(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], 0.0));   // last 0.0 for not used A7
  robot_.perform(command);
  return true;
}

bool MotionPrimitivesKukaDriver::add_linear_cartesian_cmd()
{
  // Check if pose values (position and quaternion) are valid
  for (int i = 7; i <= 13; ++i) {
    if (std::isnan(hw_mo_prim_commands_[i])) {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: pose contains NaN values");
        return false;
    }
  }
  double rx, ry, rz;
  quaternionToEuler(hw_mo_prim_commands_[10], hw_mo_prim_commands_[11], hw_mo_prim_commands_[12], hw_mo_prim_commands_[13], rx, ry, rz);

  constexpr double rad_to_deg = 180.0 / M_PI;
  std::vector<double> pose = {
    hw_mo_prim_commands_[7] * 1000.0, // from m to mm
    hw_mo_prim_commands_[8] * 1000.0,
    hw_mo_prim_commands_[9] * 1000.0,
    rx * rad_to_deg,  // from rad to deg
    ry * rad_to_deg,
    rz * rad_to_deg};

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), 
        "Adding LINEAR_CARTESIAN with pose: [%f, %f, %f, %f, %f, %f]", 
        pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  rbt::MoveCommand command;
  command = rbt::MoveCommand(rbt::PoseCartesian(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]), true);
  robot_.perform(command);
  return true;
}

bool MotionPrimitivesKukaDriver::add_circular_cartesian_cmd()
{
  // Check if pose values (position and quaternion) are valid
  for (int i = 7; i <= 20; ++i) {
    if (std::isnan(hw_mo_prim_commands_[i])) {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: pose contains NaN values");
        return false;
    }
  }
  double goal_rx, goal_ry, goal_rz;
  quaternionToEuler(hw_mo_prim_commands_[10], hw_mo_prim_commands_[11], hw_mo_prim_commands_[12], hw_mo_prim_commands_[13], goal_rx, goal_ry, goal_rz);

  double via_rx, via_ry, via_rz;
  quaternionToEuler(hw_mo_prim_commands_[17], hw_mo_prim_commands_[18], hw_mo_prim_commands_[19], hw_mo_prim_commands_[20], via_rx, via_ry, via_rz);

  constexpr double rad_to_deg = 180.0 / M_PI;
  std::vector<double> goal_pose = {
    hw_mo_prim_commands_[7] * 1000.0, // from m to mm
    hw_mo_prim_commands_[8] * 1000.0,
    hw_mo_prim_commands_[9] * 1000.0,
    goal_rx * rad_to_deg,  // from rad to deg
    goal_ry * rad_to_deg,
    goal_rz * rad_to_deg};
  std::vector<double> via_pose = {
    hw_mo_prim_commands_[14] * 1000.0, // from m to mm
    hw_mo_prim_commands_[15] * 1000.0,
    hw_mo_prim_commands_[16] * 1000.0,
    via_rx * rad_to_deg,  // from rad to deg
    via_ry * rad_to_deg,
    via_rz * rad_to_deg};

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), 
        "Adding CIRCULAR_CARTESIAN with goal_pose: [%f, %f, %f, %f, %f, %f] and via_pose: [%f, %f, %f, %f, %f, %f]", 
        goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3], goal_pose[4], goal_pose[5],
        via_pose[0], via_pose[1], via_pose[2], via_pose[3], via_pose[4], via_pose[5]);
  rbt::MoveCommand command;
  command = rbt::MoveCommand(rbt::PoseCartesian(via_pose[0], via_pose[1], via_pose[2], via_pose[3], via_pose[4], via_pose[5]),
                             rbt::PoseCartesian(goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3], goal_pose[4], goal_pose[5]));
  robot_.perform(command);

  return true;
}

void MotionPrimitivesKukaDriver::reset_command_interfaces()
{
  std::fill(hw_mo_prim_commands_.begin(), hw_mo_prim_commands_.end(), std::numeric_limits<double>::quiet_NaN());
}

void MotionPrimitivesKukaDriver::asyncExecuteMotionThread()
{
  while (!async_thread_shutdown_) 
  {
    if (new_execution_available_) 
    {
      std::lock_guard<std::mutex> guard(execution_mutex_);
      new_execution_available_ = false;
      RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Sending command to robot ...");
      while(!robot_.run()){}
      RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Sending command to robot ... done");
      ready_for_new_primitive_ = true;
    }
    // Small sleep to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "[asyncExecuteMotionThread] Exiting");
}

// Convert quaternion to Euler angles (roll, pitch, yaw) 
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void MotionPrimitivesKukaDriver::quaternionToEuler(double qx, double qy, double qz, double qw, double& rx, double& ry, double& rz) {
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (qw * qx + qy * qz);
  double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  rx = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = std::sqrt(1 + 2 * (qw * qy - qx * qz));
  double cosp = std::sqrt(1 - 2 * (qw * qy - qx * qz));
  ry = 2 * std::atan2(sinp, cosp) - M_PI / 2;

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (qw * qz + qx * qy);
  double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  rz = std::atan2(siny_cosp, cosy_cosp);

  // RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Converted quaternion [%f, %f, %f, %f] to Euler angles: [%f, %f, %f]",qx, qy, qz, qw, rx, ry, rz);
}

}  // namespace kuka_eki_motion_primitives_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kuka_eki_motion_primitives_hw_interface::MotionPrimitivesKukaDriver, hardware_interface::SystemInterface)
