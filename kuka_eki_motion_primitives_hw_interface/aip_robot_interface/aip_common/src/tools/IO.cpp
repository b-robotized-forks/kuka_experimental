#include <aip/common/tools/IO.h>

#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <sstream>

bool IO::file_exists(const std::string &path)
{
    struct stat buffer;

    return path.size() > 0 && stat(path.c_str(), &buffer) == 0;
}

std::string IO::read_file(const std::string &path)
{
    std::stringstream content;

    std::ifstream stream(path);

    if (stream.is_open())
    {
        std::string line;

        while (std::getline(stream, line))
        {
            content << line << '\n';
        }

        stream.close();
    }

    return content.str();
}

bool IO::write_file(const std::string &path, const std::string &content, bool append)
{
    std::ofstream stream(path, append ? std::ios::app : std::ios::out);

    if (stream.is_open())
    {
        stream << content;

        return true;
    }

    return false;
}