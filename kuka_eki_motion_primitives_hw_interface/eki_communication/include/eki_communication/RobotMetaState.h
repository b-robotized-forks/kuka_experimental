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