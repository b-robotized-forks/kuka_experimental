#pragma once

#include <eki_communication/xml/XmlReader.h>

#include <eki_communication/core/Types.h>

namespace rbt
{
class RobotMetaState
{
private:
    static int max_id_;
    int id_ = 0;

public:
    RobotMetaState() : id_(++max_id_) {}
    ~RobotMetaState() {}

    int id() const { return id_; }

    float velocity_override = -1.f;
    bool commands_empty = false;

    void from_xml(XmlReader &reader);
};
} // namespace rbt