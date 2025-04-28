#include <aip/common/core/LogLevel.h>

std::string LogLevel::to_string() const
{
    switch (value_)
    {
    case LogLevel::Debug:
        return "Debug";
    case LogLevel::Info:
        return "Info";
    case LogLevel::Warn:
        return "Warn";
    case LogLevel::Error:
        return "Error";
    
    default:
        return "";
    }
}