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

#include <eki_communication/MoveCommand.h>
#include <eki_communication/GripCommand.h>

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