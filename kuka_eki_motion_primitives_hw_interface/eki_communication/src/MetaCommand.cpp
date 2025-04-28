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

#include <eki_communication/MetaCommand.h>

rbt::MetaCommand::MetaCommand(const rbt::MetaCommand *base)
{
    if (base != nullptr)
    {
        *this = *base;
    }
}

rbt::MetaCommand::MetaCommand(float velocity, bool abort, const rbt::MetaCommand *base) : MetaCommand(base)
{
    velocity_override = velocity;
    abort_commands = abort;
}

void rbt::MetaCommand::to_xml(XmlWriter &writer) const
{

    writer.open_element("MetaCommand", {{"VelocityOverride", std::to_string(velocity_override)},
                                        {"AbortCommands", std::to_string(abort_commands)}});

    writer.close_all();
}