#pragma once

#include <eki_communication/xml/XmlWriter.h>
#include <eki_communication/core/Types.h>

namespace rbt
{
class GripCommand
{
private:
public:
    GripCommand(const rbt::GripCommand *base = nullptr);
    GripCommand(float item_size, bool close, const rbt::GripCommand *base = nullptr);
    GripCommand(float item_size, bool close, const rbt::GripCommand &base) : GripCommand(item_size, close, &base) {}
    GripCommand(bool suction_active, float cylinder_position, const rbt::GripCommand *base = nullptr);
    GripCommand(bool suction_active, float cylinder_position, const rbt::GripCommand &base) : GripCommand(suction_active, cylinder_position, &base) {}
    GripCommand(float item_size, bool close, bool suction_active, float cylinder_position, const rbt::GripCommand *base = nullptr);
    GripCommand(float item_size, bool close, bool suction_active, float cylinder_position, const rbt::GripCommand &base) : GripCommand(item_size, close, suction_active, cylinder_position, &base) {}
    ~GripCommand() {}

    GripMode mode = GripMode::NONE;

    float tolerance = 0.f;
    float velocity = 1.f;
    float force = 0.f;
    GripPositions positions;
    bool direction_mode = false;

    bool suction = false;
    float cylinder = 0.f;

    GripCommand copy() const;
    void from(const rbt::GripCommand &base);

    void to_xml(XmlWriter &writer) const;
};
} // namespace rbt