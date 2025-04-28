#include <aip/rbt/core/CommandSequence.h>
#include <iostream>

void rbt::CommandSequence::add(const rbt::Command &command)
{
    commands_.push_back(command.copy());
}

void rbt::CommandSequence::add(const rbt::CommandSequence &sequence)
{
    for (const Command &command : sequence.commands_)
    {
        add(command);
    }
}

void rbt::CommandSequence::update(const rbt::RobotState &state)
{
    int index = 0;

    while (index < commands_.size())
    {
        if (commands_[index].id() == state.command_id)
        {
            break;
        }

        ++index;
    }

    position_ = index;
}

void rbt::CommandSequence::reset()
{
    position_ = 0;
}

void rbt::CommandSequence::clear()
{
    commands_.clear();

    reset();
}

void rbt::CommandSequence::to_xml(XmlWriter &writer)
{
    for (rbt::Command &command : remaining_commands())
    {
        command.to_xml(writer);
    }
}

std::vector<rbt::Command> rbt::CommandSequence::all_commands()
{
    return commands_;
}

std::vector<rbt::Command> rbt::CommandSequence::remaining_commands()
{
    return std::vector<rbt::Command>(commands_.begin() + position_, commands_.end());
}