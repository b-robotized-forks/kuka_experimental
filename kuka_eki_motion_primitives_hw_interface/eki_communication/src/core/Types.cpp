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

#include <eki_communication/core/Types.h>

#include <math.h>

rbt::GripPositions::GripPositions(float item_size, float force_distance)
{
    item_size *= 100 / 2;
    force_distance *= 100;

    float min_position = 75;   // 1.5 mm
    float max_position = 7575; // 151.5 mm

    float target_position = max_position - item_size + 4750;

    base = min_position;

    shift = std::fmax(target_position - force_distance / 2, base + 1);
    teach = std::fmax(target_position, shift + 1);
    work = std::fmax(target_position + force_distance / 2, teach + 1);

    work = std::fmin(work, max_position);
    teach = std::fmin(teach, work - 1);
    shift = std::fmin(shift, teach - 1);
}
