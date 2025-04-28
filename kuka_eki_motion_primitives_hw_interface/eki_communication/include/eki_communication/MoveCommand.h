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

#pragma once

#include <eki_communication/xml/XmlWriter.h>
#include <eki_communication/core/Types.h>

namespace rbt
{
class MoveCommand
{
private:
public:
    MoveCommand(const rbt::MoveCommand *base = nullptr);
    MoveCommand(PoseJoints target, const rbt::MoveCommand *base = nullptr);
    MoveCommand(PoseJoints target, const rbt::MoveCommand &base) : MoveCommand(target, &base) {}
    MoveCommand(PoseCartesian target, bool lin, const rbt::MoveCommand *base = nullptr);
    MoveCommand(PoseCartesian target, bool lin, const rbt::MoveCommand &base) : MoveCommand(target, lin, &base) {}
    MoveCommand(PoseCartesian target, const rbt::MoveCommand *base = nullptr) : MoveCommand(target, false, base) {}
    MoveCommand(PoseCartesian target, const rbt::MoveCommand &base) : MoveCommand(target, &base) {}
    MoveCommand(int target, const rbt::MoveCommand *base = nullptr);
    MoveCommand(int target, const rbt::MoveCommand &base) : MoveCommand(target, &base) {}
    ~MoveCommand() {}

    MoveMode mode = MoveMode::NONE;
    int base_index = 0;
    int tool_index = 0;
    float velocity = 0.f;
    bool wait_for_gripper = false;

    PoseJoints target_joints;
    PoseCartesian target_cartesian;
    int target_teached = 0;

    void to_xml(XmlWriter &writer) const;
};
} // namespace rbt