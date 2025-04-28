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
// Authors: Moritz Weisenböhler

#include <eki_communication/RobotState.h>

int rbt::RobotState::max_id_ = 0;

void rbt::RobotState::from_xml(XmlReader &reader)
{
    auto element = reader.get_element("Command");
    command_id = element->IntAttribute("Id");

    element = reader.get_element("Position/Joint");
    position_joints = PoseJoints{element->FloatAttribute("A1"),
                                 element->FloatAttribute("A2"),
                                 element->FloatAttribute("A3"),
                                 element->FloatAttribute("A4"),
                                 element->FloatAttribute("A5"),
                                 element->FloatAttribute("A6"),
                                 element->FloatAttribute("A7")};

    element = reader.get_element("Position/Cartesian");
    position_cartesian = PoseCartesian{element->FloatAttribute("X"),
                                       element->FloatAttribute("Y"),
                                       element->FloatAttribute("Z"),
                                       element->FloatAttribute("A"),
                                       element->FloatAttribute("B"),
                                       element->FloatAttribute("C")};

    element = reader.get_element("Velocity");
    velocity = PoseJoints{element->FloatAttribute("A1"),
                          element->FloatAttribute("A2"),
                          element->FloatAttribute("A3"),
                          element->FloatAttribute("A4"),
                          element->FloatAttribute("A5"),
                          element->FloatAttribute("A6")};

    element = reader.get_element("Torque");
    torque = PoseJoints{element->FloatAttribute("A1"),
                        element->FloatAttribute("A2"),
                        element->FloatAttribute("A3"),
                        element->FloatAttribute("A4"),
                        element->FloatAttribute("A5"),
                        element->FloatAttribute("A6")};

    element = reader.get_element("Gripper/Jaw");
    gripper_jaw_position = element->FloatAttribute("Position");
    gripper_jaw_status = element->FloatAttribute("Status");

    element = reader.get_element("Gripper/Vacuum");
    gripper_vacuum_suction = element->FloatAttribute("Suction");
    gripper_vacuum_force1 = element->FloatAttribute("Force1");
    gripper_vacuum_force2 = element->FloatAttribute("Force2");
    gripper_vacuum_cylinder = element->FloatAttribute("Cylinder");

    element = reader.get_element("Info");
    info_code = element->IntAttribute("Code");
    info_message = element->Attribute("Message");

    element = reader.get_element("Error");
    error_code = element->IntAttribute("Code");
    error_message = element->Attribute("Message");
}