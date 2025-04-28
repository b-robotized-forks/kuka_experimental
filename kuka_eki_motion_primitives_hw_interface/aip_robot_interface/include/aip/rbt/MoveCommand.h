#pragma once

#include <aip/rbt/xml/XmlWriter.h>
#include <aip/rbt/core/Types.h>

namespace rbt
{
class MoveCommand
{
private:
public:
    MoveCommand(const rbt::MoveCommand *base = nullptr);
    MoveCommand(PoseJoints target, const rbt::MoveCommand *base = nullptr);
    MoveCommand(PoseJoints target, const rbt::MoveCommand &base) : MoveCommand(target, &base) {}
    MoveCommand(PoseCartesian target, bool lin, const rbt::MoveCommand *base = nullptr);
    MoveCommand(PoseCartesian target, bool lin, const rbt::MoveCommand &base) : MoveCommand(target, lin, &base) {}
    MoveCommand(PoseCartesian target, const rbt::MoveCommand *base = nullptr) : MoveCommand(target, false, base) {}
    MoveCommand(PoseCartesian target, const rbt::MoveCommand &base) : MoveCommand(target, &base) {}
    MoveCommand(int target, const rbt::MoveCommand *base = nullptr);
    MoveCommand(int target, const rbt::MoveCommand &base) : MoveCommand(target, &base) {}
    ~MoveCommand() {}

    MoveMode mode = MoveMode::NONE;
    int base_index = 0;
    int tool_index = 0;
    float velocity = 0.f;
    bool wait_for_gripper = false;

    PoseJoints target_joints;
    PoseCartesian target_cartesian;
    int target_teached = 0;

    void to_xml(XmlWriter &writer) const;
};
} // namespace rbt