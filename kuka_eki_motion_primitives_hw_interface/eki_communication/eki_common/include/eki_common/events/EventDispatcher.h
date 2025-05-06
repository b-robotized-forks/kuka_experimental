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
#include <eki_common/events/EventListener.h>

template <class T>
class EventDispatcher
{
public:
    EventDispatcher() {}

    int addEventListener(const std::string &type, const std::function<void(T)> &callback)
    {
        EventListener<T> listener{type, callback};

        listeners_.insert({listener.id(), listener});

        return listener.id();
    }

    bool removeEventListener(int id) { return listeners_.erase(id) > 0; }

    void dispatchEvent(const T &event) const
    {
        std::vector<EventListener<T>> listeners;

        // Copy suitable listeners to prevent conflict with removeEventListener calls in callback
        for (std::pair<int, EventListener<T>> listener_entry : listeners_)
        {
            if (listener_entry.second.is_suitable(event))
            {
                listeners.push_back(listener_entry.second);
            }
        }

        for (EventListener<T> listener : listeners)
        {
            listener.invoke(event);
        }
    }

private:
    std::map<int, EventListener<T>> listeners_;
};