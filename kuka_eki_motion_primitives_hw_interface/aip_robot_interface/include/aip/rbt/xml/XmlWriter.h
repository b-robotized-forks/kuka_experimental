#pragma once

#include <sstream>

#include <aip/common/default.h>

class XmlWriter
{
private:
    std::string prolog_;
    std::stringstream stream_;

    std::vector<std::string> opened_tags_;

public:
    XmlWriter() {}
    ~XmlWriter() {}

    std::string line_break = "";

    void add_prolog();
    void add_prolog(const std::string &version, const std::string &encoding);

    void add_element(const std::string &tag);
    void add_element(const std::string &tag, const std::map<std::string, std::string> &attributes);
    void open_element(const std::string &tag);
    void open_element(const std::string &tag, const std::map<std::string, std::string> &attributes);
    void add_content(const std::string &content);
    void close_element();
    void close_all();

    std::string get_string();
    void clear();
};