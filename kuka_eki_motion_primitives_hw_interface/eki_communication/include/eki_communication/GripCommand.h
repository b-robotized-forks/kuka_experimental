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
// Authors: Students of the Insitute for Robotics and Autonomous Systems (IRAS) 
//          - (Supervisor: Prof. Dr.-Ing. Christian Wurll), 
//          Moritz Weisenböhler,
//          Mathias Fuhrer

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

    void from(const rbt::GripCommand &base);

    void to_xml(XmlWriter &writer) const;
};
} // namespace rbt