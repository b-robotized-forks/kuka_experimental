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

#include <eki_communication/core/CommandSequence.h>
#include <iostream>

void rbt::CommandSequence::add(const rbt::Command &command)
{
    commands_.emplace_back(command);
}

void rbt::CommandSequence::add(rbt::CommandSequence &sequence)
{
    for (Command &command : sequence.commands_)
    {
        commands_.emplace_back(command);
    }
}

void rbt::CommandSequence::update(const rbt::RobotState &state)
{
    int index = 0;

    while (index < commands_.size())
    {
        if (commands_[index].id() == state.command_id)
        {
            break;
        }

        ++index;
    }

    position_ = index;
}

void rbt::CommandSequence::reset()
{
    position_ = 0;
}

void rbt::CommandSequence::clear()
{
    commands_.clear();

    reset();
}

void rbt::CommandSequence::to_xml(XmlWriter &writer)
{
    for (auto cmd_it = commands_.begin() + position_; cmd_it != commands_.end(); ++cmd_it)
    {
        cmd_it->to_xml(writer);
    }
}