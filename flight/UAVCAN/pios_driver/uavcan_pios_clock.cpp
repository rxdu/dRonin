#include "uavcan_pios_clock.hpp"

extern "C" {
    #include "jlink_rtt.h"
    uint64_t PIOS_UAVCAN_TIM_GetMonoTime();
}

using namespace uavcan;
using namespace pios_uavcan;

uavcan::MonotonicTime PIOSUAVCANClock::getMonotonic() const 
{ 
    // JLinkRTTPrintf(0, "getMonotonic: %ld\n", PIOS_UAVCAN_TIM_GetMonoTime());
    return uavcan::MonotonicTime::fromUSec(PIOS_UAVCAN_TIM_GetMonoTime()); 
}

uavcan::UtcTime PIOSUAVCANClock::getUtc() const 
{ 
    return uavcan::UtcTime::fromUSec(0); 
}
void PIOSUAVCANClock::adjustUtc(uavcan::UtcDuration adjustment) 
{ 
    // DO NOTHING ON MCU
}