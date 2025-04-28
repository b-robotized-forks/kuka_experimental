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