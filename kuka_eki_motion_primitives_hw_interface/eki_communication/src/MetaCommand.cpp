#include <eki_communication/MetaCommand.h>

rbt::MetaCommand::MetaCommand(const rbt::MetaCommand *base)
{
    if (base != nullptr)
    {
        *this = *base;
    }
}

rbt::MetaCommand::MetaCommand(float velocity, bool abort, const rbt::MetaCommand *base) : MetaCommand(base)
{
    velocity_override = velocity;
    abort_commands = abort;
}

void rbt::MetaCommand::to_xml(XmlWriter &writer) const
{

    writer.open_element("MetaCommand", {{"VelocityOverride", std::to_string(velocity_override)},
                                        {"AbortCommands", std::to_string(abort_commands)}});

    writer.close_all();
}