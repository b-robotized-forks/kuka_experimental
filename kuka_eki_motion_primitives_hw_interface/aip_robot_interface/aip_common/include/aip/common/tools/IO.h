#pragma once

#include <aip/common/default.h>

class IO
{
public:
    static bool file_exists(const std::string &path);
    static std::string read_file(const std::string &path);
    static bool write_file(const std::string &path, const std::string &content, bool append = false);
};