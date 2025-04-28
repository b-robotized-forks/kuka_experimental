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

#include <eki_common/types/Pose.h>

#include <math.h>

float Pose::distance_to(const Pose &other) const
{
    float dx = x - other.x;
    float dy = y - other.y;
    float dz = z - other.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string Pose::to_string() const
{
    return "Pose: (" + std::to_string(x) + ", " + std::to_string(y) + +", " + std::to_string(z) + +", " + std::to_string(a) + +", " + std::to_string(b) + +", " + std::to_string(c) + ")";
}