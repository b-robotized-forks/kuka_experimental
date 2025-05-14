// Copyright (c) 2025, H-KA Hochschule Karlsruhe - University of Applied Sciences
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
// Authors: Students of the Insitute for Robotics and Autonomous Systems (IRAS) 
//          - (Supervisor: Prof. Dr.-Ing. Christian Wurll), 
//          Moritz Weisenböhler, Mathias Fuhrer

#include <eki_communication/MoveCommand.h>

rbt::MoveCommand::MoveCommand(const rbt::MoveCommand *base)
{
    if (base != nullptr)
    {
        *this = *base;
    }
}

rbt::MoveCommand::MoveCommand(PoseJoints target, const rbt::MoveCommand *base) : MoveCommand(base)
{
    mode = MoveMode::JOINT;
    target_joints = target;
}

rbt::MoveCommand::MoveCommand(PoseCartesian target, bool lin, const rbt::MoveCommand *base) : MoveCommand(base)
{
    mode = lin ? MoveMode::CARTESIAN_LIN : MoveMode::CARTESIAN_PTP;
    target_cartesian = target;
}

rbt::MoveCommand::MoveCommand(PoseCartesian via, PoseCartesian target, const rbt::MoveCommand *base) : MoveCommand(base)
{
    mode = MoveMode::CARTESIAN_CIRC;
    target_cartesian = target;
    via_cartesian = via;
}

rbt::MoveCommand::MoveCommand(int target, const rbt::MoveCommand *base) : MoveCommand(base)
{
    mode = MoveMode::TEACHED;
    target_teached = target;
}

void rbt::MoveCommand::to_xml(XmlWriter &writer) const
{
    writer.open_element("Move", {{"Mode", std::to_string(static_cast<int>(mode))},
                                 {"BaseIndex", std::to_string(base_index)},
                                 {"ToolIndex", std::to_string(tool_index)},
                                 {"Velocity", std::to_string(velocity)},
                                 {"Acceleration", std::to_string(acceleration)},
                                 {"WaitForGripper", std::to_string(wait_for_gripper)}});

    writer.add_element("Joint", {{"A1", std::to_string(target_joints.a1)},
                                 {"A2", std::to_string(target_joints.a2)},
                                 {"A3", std::to_string(target_joints.a3)},
                                 {"A4", std::to_string(target_joints.a4)},
                                 {"A5", std::to_string(target_joints.a5)},
                                 {"A6", std::to_string(target_joints.a6)},
                                 {"A7", std::to_string(target_joints.a7)}});

    writer.add_element("Cartesian", {{"X", std::to_string(target_cartesian.x)},
                                     {"Y", std::to_string(target_cartesian.y)},
                                     {"Z", std::to_string(target_cartesian.z)},
                                     {"A", std::to_string(target_cartesian.a)},
                                     {"B", std::to_string(target_cartesian.b)},
                                     {"C", std::to_string(target_cartesian.c)}});

    writer.add_element("Cartesian_Aux", {{"X", std::to_string(via_cartesian.x)},
                                        {"Y", std::to_string(via_cartesian.y)},
                                        {"Z", std::to_string(via_cartesian.z)},
                                        {"A", std::to_string(via_cartesian.a)},
                                        {"B", std::to_string(via_cartesian.b)},
                                        {"C", std::to_string(via_cartesian.c)}});

    writer.add_element("Teached", {{"PositionIndex", std::to_string(target_teached)}});

    writer.close_element();
}