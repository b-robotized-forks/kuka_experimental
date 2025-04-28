#pragma once

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
    TEACHED = 4
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
    float a1 = 0.f;
    float a2 = 0.f;
    float a3 = 0.f;
    float a4 = 0.f;
    float a5 = 0.f;
    float a6 = 0.f;
    float a7 = 0.f;

    PoseJoints() {}
    PoseJoints(float a1, float a2, float a3, float a4, float a5, float a6) : a1(a1), a2(a2), a3(a3), a4(a4), a5(a5), a6(a6) {}
    PoseJoints(float a1, float a2, float a3, float a4, float a5, float a6, float a7) : a1(a1), a2(a2), a3(a3), a4(a4), a5(a5), a6(a6), a7(a7) {}
};

struct PoseCartesian
{
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float a = 0.f;
    float b = 0.f;
    float c = 0.f;

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
