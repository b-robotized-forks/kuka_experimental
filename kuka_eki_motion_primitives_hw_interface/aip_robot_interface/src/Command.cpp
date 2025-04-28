#include <aip/rbt/Command.h>

int rbt::Command::max_id_ = 0;

rbt::Command::Command(const MoveCommand &move) : Command()
{
    type_ = CommandType::MOVE;
    move_ = move;
}

rbt::Command::Command(const GripCommand &grip) : Command()
{
    type_ = CommandType::GRIP;
    grip_ = grip;
}

rbt::Command::Command(const MoveCommand &move, const GripCommand &grip) : Command()
{
    type_ = CommandType::COMBINED;
    move_ = move;
    grip_ = grip;
}

rbt::Command rbt::Command::copy() const
{
    return Command(move_, grip_);
}

void rbt::Command::to_xml(XmlWriter &writer) const
{
    writer.open_element("RobotCommand", {{"Id", std::to_string(id_)},
                                         {"Type", std::to_string(static_cast<int>(type_))}});

    move_.to_xml(writer);
    grip_.to_xml(writer);

    writer.close_element();
}