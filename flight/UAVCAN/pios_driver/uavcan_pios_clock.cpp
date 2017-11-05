#include "uavcan_pios_clock.hpp"

extern "C" {
    #include "jlink_rtt.h"
    uint64_t PIOS_UAVCAN_TIM_GetMonoTime();
    uint32_t PIOS_DELAY_GetuS();
}

using namespace uavcan;
using namespace pios_uavcan;

// static uint64_t mono_time = 0; 
// static uint64_t overflow_count = 0;

uavcan::MonotonicTime PIOSUAVCANClock::getMonotonic() const 
{ 
    // JLinkRTTPrintf(0, "getMonotonic: %ld\n", PIOS_UAVCAN_TIM_GetMonoTime());
    return uavcan::MonotonicTime::fromUSec(PIOS_UAVCAN_TIM_GetMonoTime()); 
    // return uavcan::MonotonicTime::fromUSec(PIOS_DELAY_GetuS()); 

    // static uint32_t prev_time = 0;
    // uint32_t current_time = PIOS_DELAY_GetuS();
    // if(current_time < prev_time)
    // {
    //     ++overflow_count;
    // }   
    // mono_time = current_time + overflow_count * 0xffffffff;
    // prev_time = current_time;

    // return uavcan::MonotonicTime::fromUSec(mono_time); 
}

uavcan::UtcTime PIOSUAVCANClock::getUtc() const 
{ 
    return uavcan::UtcTime::fromUSec(0); 
}
void PIOSUAVCANClock::adjustUtc(uavcan::UtcDuration adjustment) 
{ 
    // DO NOTHING ON MCU
}