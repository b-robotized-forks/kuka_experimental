#pragma once

#include <iostream>

#include <aip/common/default.h>
#include <aip/common/core/LogEntry.h>

class Logger
{
public:
  const std::string HIERARCHY_PREFIX = "  ";
  static Logger global_instance;

  Logger() : Logger(LogLevel::Info) {}
  Logger(LogLevel level) : level(level) {}
  ~Logger() {}

  LogLevel level;
  std::string file_path = "";

  void log(const std::string &message, unsigned int hierarchy = 0, LogLevel level = LogLevel::Info);

private:
  std::vector<LogEntry> entries;

  void write_log_(const LogEntry &entry) const;
};
