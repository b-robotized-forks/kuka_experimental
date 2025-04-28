#pragma once

#include <aip/common/default.h>
#include <aip/rbt/Command.h>
#include <aip/rbt/RobotState.h>

namespace rbt
{
    class CommandSequence
    {
    private:
        std::vector<Command> commands_;

        int position_ = 0;

    public:
        CommandSequence() {}
        CommandSequence(const std::vector<Command> &commands) : commands_(commands) {}
        ~CommandSequence() {}

        void add(const Command &command);
        void add(const CommandSequence &sequence);

        int position() { return position_; }

        void update(const RobotState &state);

        void reset();
        void clear();

        void to_xml(XmlWriter &writer);

        int size() { return commands_.size(); }
        bool is_finished() { return position_ == size(); }

        std::vector<Command> all_commands();
        std::vector<Command> remaining_commands();
    };
} // namespace rbt
