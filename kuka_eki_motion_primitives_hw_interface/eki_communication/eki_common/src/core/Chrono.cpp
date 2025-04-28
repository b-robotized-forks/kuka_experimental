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

#include <eki_common/core/Chrono.h>

Chrono Chrono::global_instance = Chrono();

std::string Chrono::start()
{
    std::string key;

    for (size_t i = 1;; ++i)
    {
        key = "entry_" + std::to_string(i);

        if (entries.count(key) == 0)
        {
            break;
        }
    }

    return start(key);
}

std::string Chrono::start(std::string key)
{
    entries[key] = ChronoEntry();

    entries[key].start();

    return key;
}

int64_t Chrono::stop(std::string key)
{
    if (entries.count(key) == 1)
    {
        entries[key].stop();

        return entries[key].duration();
    }

    return -1;
}
