#include "device/base.hpp"

roboctrl::device::device_base::device_base(const std::chrono::nanoseconds offline_timeout) : offline_timeout_ { offline_timeout }
{
}