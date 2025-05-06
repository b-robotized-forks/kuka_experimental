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
//          Moritz Weisenböhler

#include <eki_common/core/Logger.h>

#include <eki_common/tools/IO.h>

Logger Logger::global_instance = Logger();

void Logger::log(const std::string &message, unsigned int hierarchy, LogLevel level)
{
    LogEntry entry = LogEntry();

    entry.message = message;
    entry.hierarchy = hierarchy;
    entry.level = level;

    entries.push_back(entry);

    write_log_(entry);
}

void Logger::write_log_(const LogEntry &entry) const
{
    if (entry.level >= this->level)
    {
        std::string message = entry.to_string();

        if (file_path != "")
        {
            IO::write_file(file_path, message + "\n", true);
        }

        for (size_t i = 0; i < entry.hierarchy; ++i)
        {
            std::cout << HIERARCHY_PREFIX;
        }

        std::cout << message << std::endl;
    }
}
