#pragma once

#include <functional>

#include <aip/common/default.h>
#include <aip/common/events/Event.h>

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