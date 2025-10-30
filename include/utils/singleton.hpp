#pragma once

namespace roboctrl::utils{

template <typename T>
class singleton {
public:
    singleton(const singleton&) = delete;
    singleton& operator=(const singleton&) = delete;
    static T& instance() {
        static T instance;
        return instance;
    }

protected:
    singleton() = default;
    virtual ~singleton() = default;
};

}