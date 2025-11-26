#include "ctrl/gimbal.h"
#include "core/async.hpp"

using namespace roboctrl::ctrl;
using namespace roboctrl;
using namespace roboctrl::device;

roboctrl::awaitable<void> gimbal::task(){
    while(true){
        
        co_await wait_for(1ms);
    }
}

bool gimbal::init(const info_type& info){
    log_info("Gimbal initiated");
    roboctrl::spawn(task());
    return true;
}