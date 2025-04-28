#pragma once

#include <eki_common/default.h>
#include <eki_common/core/LogLevel.h>
#include <eki_common/core/ChronoTime.h>

struct LogEntry
{
    std::string message;
    unsigned int hierarchy;
    LogLevel level;
    ChronoTime time;

    std::string to_string() const { return "[" + level.to_string() + " | " + time.to_string("%H:%M:%S") + "]: " + message; }
};
