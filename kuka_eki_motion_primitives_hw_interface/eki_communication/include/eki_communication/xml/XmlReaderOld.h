#pragma once

#include <sstream>

#include <eki_common/default.h>

class XmlReaderOld
{
private:
    std::stringstream stream_;

    bool is_string_;
    bool is_escaped_;

    std::string read_until(char sign);

public:
    XmlReaderOld();
    XmlReaderOld(const std::string &xml);
    ~XmlReaderOld();

    void set_string(const std::string &xml);

    std::string get_prolog();

    std::string get_tag();
    std::map<std::string, std::string> get_attributes();
    std::vector<std::string> get_content();
};

XmlReaderOld::XmlReaderOld()
{
}

XmlReaderOld::XmlReaderOld(const std::string &xml)
{
    parse(xml);
}

XmlReaderOld::~XmlReaderOld()
{
}

void XmlReaderOld::parse(const std::string &xml)
{
    stream_ = std::stringstream(xml);
}

std::string XmlReaderOld::get_prolog()
{
    char *s;
    stream_.read(s, 2);

    if (s == "<?")
    {
        return "<?" + read_until('>');
    }

    return "";
}

std::string XmlReaderOld::get_tag()
{
    read_until('<');
    return "";
}

std::map<std::string, std::string> XmlReaderOld::get_attributes()
{
    std::map<std::string, std::string> attributes;

    return attributes;
}

std::vector<std::string> XmlReaderOld::get_content()
{
    std::vector<std::string> content;

    return content;
}

std::string XmlReaderOld::read_until(char sign)
{
    std::string text;

    char *s;

    while (stream_.good())
    {
        stream_.read(s, 1);

        if (*s == sign)
        {
            break;
        }

        text += s;
    }

    return text;
}