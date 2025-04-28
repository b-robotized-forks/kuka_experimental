#pragma once

#include <chrono>

#include <aip/common/default.h>

class ChronoTime
{
public:
    ChronoTime() : time_(now_()) {}
    ~ChronoTime() {}

    int64_t duration() const { return duration(ChronoTime()); }
    int64_t duration(const ChronoTime &until) const;

    std::string to_string(const std::string &format) const;

private:
    std::chrono::system_clock::time_point time_;

    std::chrono::system_clock::time_point now_() { return std::chrono::system_clock::now(); } 
};
