#pragma once

#include <aip/rbt/xml/XmlWriter.h>
#include <aip/rbt/core/Types.h>

namespace rbt
{
class MetaCommand
{
private:
public:
    MetaCommand(const rbt::MetaCommand *base = nullptr);
    MetaCommand(float velocity, bool abort, const rbt::MetaCommand *base = nullptr);
    MetaCommand(float velocity, bool abort, const rbt::MetaCommand &base) : MetaCommand(velocity, abort, &base){};
    ~MetaCommand() {}

    float velocity_override = 1.f;
    bool abort_commands = false;

    void to_xml(XmlWriter &writer) const;
};
} // namespace rbt