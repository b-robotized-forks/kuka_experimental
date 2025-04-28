#pragma once

#include <aip/rbt/MoveCommand.h>
#include <aip/rbt/GripCommand.h>

namespace rbt
{
    class Command
    {
    private:
        static int max_id_;
        int id_ = 0;

        CommandType type_ = CommandType::NONE;
        MoveCommand move_;
        GripCommand grip_;

    public:
        Command() : id_(++max_id_) {}
        Command(const MoveCommand &move);
        Command(const GripCommand &grip);
        Command(const MoveCommand &move, const GripCommand &grip);
        ~Command() {}

        Command copy() const;

        int id() const { return id_; }

        void to_xml(XmlWriter &writer) const;
    };
} // namespace rbt