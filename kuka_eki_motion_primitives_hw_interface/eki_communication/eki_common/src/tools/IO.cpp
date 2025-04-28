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

#include <eki_common/tools/IO.h>

#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <sstream>

bool IO::file_exists(const std::string &path)
{
    struct stat buffer;

    return path.size() > 0 && stat(path.c_str(), &buffer) == 0;
}

std::string IO::read_file(const std::string &path)
{
    std::stringstream content;

    std::ifstream stream(path);

    if (stream.is_open())
    {
        std::string line;

        while (std::getline(stream, line))
        {
            content << line << '\n';
        }

        stream.close();
    }

    return content.str();
}

bool IO::write_file(const std::string &path, const std::string &content, bool append)
{
    std::ofstream stream(path, append ? std::ios::app : std::ios::out);

    if (stream.is_open())
    {
        stream << content;

        return true;
    }

    return false;
}