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

class XmlReaderOld
{
private:
    std::stringstream stream_;

    bool is_string_;
    bool is_escaped_;

    std::string read_until(char sign);

public:
    XmlReaderOld();
    XmlReaderOld(const std::string &xml);
    ~XmlReaderOld();

    void set_string(const std::string &xml);

    std::string get_prolog();

    std::string get_tag();
    std::map<std::string, std::string> get_attributes();
    std::vector<std::string> get_content();
};

XmlReaderOld::XmlReaderOld()
{
}

XmlReaderOld::XmlReaderOld(const std::string &xml)
{
    parse(xml);
}

XmlReaderOld::~XmlReaderOld()
{
}

void XmlReaderOld::parse(const std::string &xml)
{
    stream_ = std::stringstream(xml);
}

std::string XmlReaderOld::get_prolog()
{
    char *s;
    stream_.read(s, 2);

    if (s == "<?")
    {
        return "<?" + read_until('>');
    }

    return "";
}

std::string XmlReaderOld::get_tag()
{
    read_until('<');
    return "";
}

std::map<std::string, std::string> XmlReaderOld::get_attributes()
{
    std::map<std::string, std::string> attributes;

    return attributes;
}

std::vector<std::string> XmlReaderOld::get_content()
{
    std::vector<std::string> content;

    return content;
}

std::string XmlReaderOld::read_until(char sign)
{
    std::string text;

    char *s;

    while (stream_.good())
    {
        stream_.read(s, 1);

        if (*s == sign)
        {
            break;
        }

        text += s;
    }

    return text;
}