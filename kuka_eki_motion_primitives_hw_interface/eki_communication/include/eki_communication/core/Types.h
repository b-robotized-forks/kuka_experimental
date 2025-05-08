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
//          Moritz Weisenböhler, Mathias Fuhrer

#pragma once
#include <limits>

namespace rbt
{
enum class CommandType
{
    NONE = 0,
    COMBINED = 1,
    MOVE = 2,
    GRIP = 3
};

enum class MoveMode
{
    NONE = 0,
    JOINT = 1,
    CARTESIAN_PTP = 2,
    CARTESIAN_LIN = 3,
    TEACHED = 4,
    CARTESIAN_CIRC =6
};

enum class GripMode
{
    NONE = 0,
    COMBINED = 1,
    JAW = 2,
    VACUUM = 3
};

struct PoseJoints
{
    float a1 = std::numeric_limits<float>::quiet_NaN();
    float a2 = std::numeric_limits<float>::quiet_NaN();
    float a3 = std::numeric_limits<float>::quiet_NaN();
    float a4 = std::numeric_limits<float>::quiet_NaN();
    float a5 = std::numeric_limits<float>::quiet_NaN();
    float a6 = std::numeric_limits<float>::quiet_NaN();
    float a7 = std::numeric_limits<float>::quiet_NaN();

    PoseJoints() {}
    PoseJoints(float a1, float a2, float a3, float a4, float a5, float a6) : a1(a1), a2(a2), a3(a3), a4(a4), a5(a5), a6(a6) {}
    PoseJoints(float a1, float a2, float a3, float a4, float a5, float a6, float a7) : a1(a1), a2(a2), a3(a3), a4(a4), a5(a5), a6(a6), a7(a7) {}
};

struct PoseCartesian
{
    float x = std::numeric_limits<float>::quiet_NaN();
    float y = std::numeric_limits<float>::quiet_NaN();
    float z = std::numeric_limits<float>::quiet_NaN();
    float a = std::numeric_limits<float>::quiet_NaN();
    float b = std::numeric_limits<float>::quiet_NaN();
    float c = std::numeric_limits<float>::quiet_NaN();

    PoseCartesian() {}
    PoseCartesian(float x, float y, float z, float a) : x(x), y(y), z(z), a(a) {}
    PoseCartesian(float x, float y, float z, float a, float b, float c) : x(x), y(y), z(z), a(a), b(b), c(c) {}
};

struct GripPositions
{
    float base = 0.f;
    float work = 0.f;
    float teach = 0.f;
    float shift = 0.f;

    GripPositions() {}
    GripPositions(float base, float work, float teach, float shift) : base(base), work(work), teach(teach), shift(shift) {}
    GripPositions(float item_size, float force_distance = 0);
};

enum RobotEvent
{
    CONNECT = 0,
    STATE = 1,
    RUN = 2
};
} // namespace rbt
