#include <eki_communication/RobotMetaState.h>

int rbt::RobotMetaState::max_id_ = 0;

void rbt::RobotMetaState::from_xml(XmlReader &reader)
{
    auto element = reader.get_element("MetaState");
    velocity_override = element->FloatAttribute("VelocityOverride");
    commands_empty = element->IntAttribute("CommandsEmpty");
}