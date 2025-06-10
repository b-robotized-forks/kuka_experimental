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

#include <eki_communication/Command.h>

int rbt::Command::max_id_ = 0;

rbt::Command::Command(const MoveCommand &move) : Command()
{
    type_ = CommandType::MOVE;
    move_ = move;
}

rbt::Command::Command(const GripCommand &grip) : Command()
{
    type_ = CommandType::GRIP;
    grip_ = grip;
}

rbt::Command::Command(const MoveCommand &move, const GripCommand &grip) : Command()
{
    type_ = CommandType::COMBINED;
    move_ = move;
    grip_ = grip;
}

void rbt::Command::to_xml(XmlWriter &writer) const
{
    writer.open_element("RobotCommand", {{"Id", std::to_string(id_)},
                                         {"Type", std::to_string(static_cast<int>(type_))}});

    move_.to_xml(writer);
    grip_.to_xml(writer);

    writer.close_element();
}