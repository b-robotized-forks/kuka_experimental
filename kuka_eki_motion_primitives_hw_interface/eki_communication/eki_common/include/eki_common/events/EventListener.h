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

#include <functional>

#include <eki_common/default.h>
#include <eki_common/events/Event.h>

template <class T>
class EventListener
{
public:
    EventListener(const std::string &type, const std::function<void(T)> &callback) : id_(++max_id_), type_(type), callback_(callback) {}

    int id() const { return id_; }

    bool is_suitable(T event) const { return event.type == type_; }

    void invoke(T event) const
    {
        callback_(event);
    }

private:
    static int max_id_;

    int id_;
    std::string type_;
    std::function<void(T)> callback_ = nullptr;
};

template <class T>
int EventListener<T>::max_id_ = 0;