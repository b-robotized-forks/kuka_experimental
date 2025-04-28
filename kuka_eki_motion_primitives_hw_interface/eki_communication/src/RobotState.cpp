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