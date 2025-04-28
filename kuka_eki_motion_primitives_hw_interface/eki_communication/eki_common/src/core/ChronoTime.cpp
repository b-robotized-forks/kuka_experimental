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

#include <eki_common/core/ChronoTime.h>

#include <iomanip>
#include <sstream>

int64_t ChronoTime::duration(const ChronoTime &until) const
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time_ - until.time_).count();
}

std::string ChronoTime::to_string(const std::string& format) const
{
    std::time_t tt = std::chrono::system_clock::to_time_t(time_);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&tt), format.c_str());

    return ss.str();
}