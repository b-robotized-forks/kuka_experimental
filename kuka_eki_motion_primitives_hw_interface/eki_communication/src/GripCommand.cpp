// Copyright (c) 2025, H-KA Hochschule Karlsruhe - University of Applied Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Moritz Weisenböhler

#include <eki_communication/GripCommand.h>

rbt::GripCommand::GripCommand(const rbt::GripCommand *base)
{
    if (base != nullptr)
    {
        *this = *base;
    }
}

rbt::GripCommand::GripCommand(float item_size, bool close, const rbt::GripCommand *base) : GripCommand(base)
{
    mode = GripMode::JAW;
    positions = {item_size};
    direction_mode = close;
}

rbt::GripCommand::GripCommand(bool suction_active, float cylinder_position, const rbt::GripCommand *base) : GripCommand(base)
{
    mode = GripMode::VACUUM;
    suction = suction_active;
    cylinder = cylinder_position;
}

rbt::GripCommand::GripCommand(float item_size, bool close, bool suction_active, float cylinder_position, const rbt::GripCommand *base) : GripCommand(base)
{
    mode = GripMode::COMBINED;
    positions = {item_size};
    direction_mode = close;
    suction = suction_active;
    cylinder = cylinder_position;
}

void rbt::GripCommand::to_xml(XmlWriter &writer) const
{
    writer.open_element("Grip", {{"Mode", std::to_string(static_cast<int>(mode))}});

    writer.add_element("Jaw", {{"Tolerance", std::to_string(tolerance)},
                               {"Velocity", std::to_string(velocity * 100)},
                               {"Force", std::to_string(force)},
                               {"BasePosition", std::to_string(positions.base)},
                               {"WorkPosition", std::to_string(positions.work)},
                               {"TeachPosition", std::to_string(positions.teach)},
                               {"ShiftPosition", std::to_string(positions.shift)},
                               {"DirectionMode", std::to_string(direction_mode)}});

    writer.add_element("Vacuum", {{"Suction", std::to_string(suction)},
                                  {"Cylinder", std::to_string(cylinder)}});

    writer.close_element();
}