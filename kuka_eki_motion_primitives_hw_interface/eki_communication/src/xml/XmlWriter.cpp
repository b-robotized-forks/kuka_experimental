#include <eki_communication/xml/XmlWriter.h>

void XmlWriter::add_prolog()
{
    add_prolog("1.0", "UTF-8");
}

void XmlWriter::add_prolog(const std::string &version, const std::string &encoding)
{
    std::stringstream stream;
    stream << "<?xml version=\"" << version << "\" encoding=\"" << encoding << "\"?>" << line_break;

    prolog_ = stream.str();
}

void XmlWriter::add_element(const std::string &tag)
{
    add_element(tag, {});
}

void XmlWriter::add_element(const std::string &tag, const std::map<std::string, std::string> &attributes)
{
    open_element(tag, attributes);
    close_element();
}

void XmlWriter::open_element(const std::string &tag)
{
    open_element(tag, {});
}

void XmlWriter::open_element(const std::string &tag, const std::map<std::string, std::string> &attributes)
{
    opened_tags_.push_back(tag);

    stream_ << "<" << tag;

    for (const auto &attribute : attributes)
    {
        stream_ << " " << attribute.first << "=\"" << attribute.second << "\"";
    }

    stream_ << ">" << line_break;
}

void XmlWriter::add_content(const std::string &content)
{
    stream_ << content << line_break;
}

void XmlWriter::close_element()
{

    stream_ << "</" << opened_tags_.back() << ">" << line_break;

    opened_tags_.pop_back();
}

void XmlWriter::close_all()
{
    while (opened_tags_.size() > 0)
    {
        close_element();
    }
}

std::string XmlWriter::get_string()
{
    return prolog_ + stream_.str();
}

void XmlWriter::clear()
{
    stream_.str("");
}