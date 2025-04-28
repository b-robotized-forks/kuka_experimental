#include <aip/rbt/MoveCommand.h>

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

    writer.add_element("Teached", {{"PositionIndex", std::to_string(target_teached)}});

    writer.close_element();
}