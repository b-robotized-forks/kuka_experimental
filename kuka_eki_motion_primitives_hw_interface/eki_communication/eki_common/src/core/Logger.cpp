#include <eki_common/core/Logger.h>

#include <eki_common/tools/IO.h>

Logger Logger::global_instance = Logger();

void Logger::log(const std::string &message, unsigned int hierarchy, LogLevel level)
{
    LogEntry entry = LogEntry();

    entry.message = message;
    entry.hierarchy = hierarchy;
    entry.level = level;

    entries.push_back(entry);

    write_log_(entry);
}

void Logger::write_log_(const LogEntry &entry) const
{
    if (entry.level >= this->level)
    {
        std::string message = entry.to_string();

        if (file_path != "")
        {
            IO::write_file(file_path, message + "\n", true);
        }

        for (size_t i = 0; i < entry.hierarchy; ++i)
        {
            std::cout << HIERARCHY_PREFIX;
        }

        std::cout << message << std::endl;
    }
}
