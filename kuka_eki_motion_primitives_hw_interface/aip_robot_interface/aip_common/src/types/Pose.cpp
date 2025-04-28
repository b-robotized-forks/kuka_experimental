#include <aip/common/types/Pose.h>

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