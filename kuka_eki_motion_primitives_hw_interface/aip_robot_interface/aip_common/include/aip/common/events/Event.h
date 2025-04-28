#pragma once

#include <aip/common/default.h>

class Event
{
public:
  std::string type;
  std::string message;

  Event(const std::string &type) : Event(type, "") {}
  Event(const std::string &type, const std::string &message) : id_(++max_id_), type(type), message(message) {}

  int id() { return id_; }

private:
  static int max_id_;

  int id_;
};