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

#include <eki_communication/xml/XmlReader.h>

XmlReader::XmlReader(const std::string &xml)
{
    parse(xml);
}

void XmlReader::parse(const std::string &xml)
{
    document_.Parse(xml.c_str(), xml.size());
}

tinyxml2::XMLElement *XmlReader::get_element(const std::string &path)
{
    std::vector<std::string> names = split(path, '/');

    tinyxml2::XMLElement *element = get_root();

    int index = 0;

    if (names.size() > 0 && names[index] == element->Name())
    {
        ++index;
    }

    for (; index < names.size(); ++index)
    {
        element = element->FirstChildElement(names[index].c_str());
    }

    return element;
}

std::vector<std::string> XmlReader::split(const std::string &s, char delimiter)
{
    std::vector<std::string> splits;

    int start = 0;
    int end = 0;

    while (true)
    {
        end = s.find(delimiter, end + 1);

        if (end == -1)
        {
            splits.push_back(s.substr(start));
            break;
        }

        splits.push_back(s.substr(start, end - start));

        start = end + 1;
    }

    return splits;
}

bool XmlReader::has_error()
{
    return document_.Error();
}

tinyxml2::XMLError XmlReader::error_id()
{
    return document_.ErrorID();
}

std::string XmlReader::error()
{
    std::string error = "[" + std::to_string(document_.ErrorID()) + "] ";
    error += document_.ErrorName();
    error += " at line " + std::to_string(document_.ErrorLineNum());

    return error;
}