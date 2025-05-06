// Copyright (c) 2025, H-KA Hochschule Karlsruhe - University of Applied Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Students of the Insitute for Robotics and Autonomous Systems (IRAS) 
//          - (Supervisor: Prof. Dr.-Ing. Christian Wurll), 
//          Moritz Weisenböhler

#pragma once

#include <eki_common/default.h>

#include <eki_common/core/Chrono.h>
#include <eki_common/core/Logger.h>

class Component
{
public:
  Component() : Component("Unknown") {}
  Component(const std::string &name) : Component(name, "", 0) {}
  Component(const std::string &name, const std::string &description) : Component(name, description, 0) {}
  Component(const std::string &name, unsigned int hierarchy) : Component(name, "", hierarchy) {}
  Component(const std::string &name, const std::string &description, unsigned int hierarchy) : name_(name), description_(description), hierarchy_(hierarchy) {}

  void log(const std::string &message, LogLevel level) { Logger::global_instance.log(prefixed_(message), hierarchy_, level); }
  void log(const std::string &message) { log(message, LogLevel::Info); }

  void start_chrono(const std::string &key) { Chrono::global_instance.start(prefixed_(key)); }
  int64_t stop_chrono(const std::string &key) { return Chrono::global_instance.stop(prefixed_(key)); }
  void log_chrono(const std::string &key) { log(key + ": " + std::to_string(Chrono::global_instance.stop(prefixed_(key)) / 1e6) + " ms"); }

protected:
  std::string name_;
  std::string description_;

private:
  unsigned int hierarchy_;

  std::string prefixed_(const std::string &value) { return "[" + name_ + "] " + value; }
};