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

#include <eki_common/default.h>
#include <eki_communication/Command.h>
#include <eki_communication/RobotState.h>

namespace rbt
{
    class CommandSequence
    {
    private:
        std::vector<Command> commands_;

        int position_ = 0;

    public:
        CommandSequence() {}
        CommandSequence(const std::vector<Command> &commands) : commands_(commands) {}
        ~CommandSequence() {}

        void add(const Command &command);
        void add(CommandSequence &sequence);

        int position() { return position_; }

        void update(const RobotState &state);

        void reset();
        void clear();

        void to_xml(XmlWriter &writer);

        int size() { return commands_.size(); }
        bool is_finished() { return position_ == size(); }
    };
} // namespace rbt
