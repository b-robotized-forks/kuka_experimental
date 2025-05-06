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

#pragma once

#include <cstdint> // for uint8_t
#include <eki_common/default.h>

class LogLevel
{
public:
    enum Value : uint8_t
    {
        Debug = 0,
        Info = 1,
        Warn = 2,
        Error = 3
    };

    LogLevel(Value value) : value_(value) {}
    LogLevel() : LogLevel(Info) {}

    operator Value() const { return value_; }
    explicit operator bool() = delete;

    std::string to_string() const;
    
private:
    Value value_;
};