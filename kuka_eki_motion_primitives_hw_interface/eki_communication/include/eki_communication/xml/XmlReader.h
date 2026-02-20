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

#include <sstream>

#include <tinyxml2.h>

#include <eki_common/default.h>

class XmlReader
{
private:
    tinyxml2::XMLDocument document_;

    std::vector<std::string> split(const std::string &s, char delimiter);

public:
    XmlReader() {}
    XmlReader(const std::string &xml);
    ~XmlReader() {}

    void parse(const std::string &xml);
    tinyxml2::XMLElement *get_root() { return document_.RootElement(); }
    tinyxml2::XMLElement *get_element(const std::string &path);

    bool has_error();
    tinyxml2::XMLError error_id();
    std::string error();
};