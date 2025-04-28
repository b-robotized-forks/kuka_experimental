#pragma once

#include <aip/common/default.h>

class Pose
{
public:
    Pose() {}
    Pose(float x, float y, float z, float a, float b, float c) : x(x), y(y), z(z), a(a), b(b), c(c) {}
    
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float a = 0.f;
    float b = 0.f;
    float c = 0.f;

    float distance_to(const Pose &other) const;

    std::string to_string() const;
};