#pragma once

#include <sstream>

#include <tinyxml2.h>

#include <eki_common/default.h>

class XmlReader
{
private:
    tinyxml2::XMLDocument document_;

    std::vector<std::string> split(const std::string &s, char delimiter);

public:
    XmlReader() {}
    XmlReader(const std::string &xml);
    ~XmlReader() {}

    void parse(const std::string &xml);
    tinyxml2::XMLElement *get_root() { return document_.RootElement(); }
    tinyxml2::XMLElement *get_element(const std::string &path);

    bool has_error();
    tinyxml2::XMLError error_id();
    std::string error();
};