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

#include <eki_common/default.h>

class XmlWriter
{
private:
    std::string prolog_;
    std::stringstream stream_;

    std::vector<std::string> opened_tags_;

public:
    XmlWriter() {}
    ~XmlWriter() {}

    std::string line_break = "";

    void add_prolog();
    void add_prolog(const std::string &version, const std::string &encoding);

    void add_element(const std::string &tag);
    void add_element(const std::string &tag, const std::map<std::string, std::string> &attributes);
    void open_element(const std::string &tag);
    void open_element(const std::string &tag, const std::map<std::string, std::string> &attributes);
    void add_content(const std::string &content);
    void close_element();
    void close_all();

    std::string get_string();
    void clear();
};