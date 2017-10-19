#include "uavcan_pios_clock.hpp"

extern "C" {
    #include "jlink_rtt.h"
    uint32_t PIOS_RTC_GetSystemTime();        
}

using namespace uavcan;
using namespace pios_uavcan;

uavcan::MonotonicTime PIOSUAVCANClock::getMonotonic() const 
{ 
    JLinkRTTPrintf(0, "RTC Counter: %ld\n", PIOS_RTC_GetSystemTime());
    return uavcan::MonotonicTime::fromUSec(PIOS_RTC_GetSystemTime()); 
}

uavcan::UtcTime PIOSUAVCANClock::getUtc() const 
{ 
    return uavcan::UtcTime::fromUSec(0); 
}
void PIOSUAVCANClock::adjustUtc(uavcan::UtcDuration adjustment) 
{ 
    // DO NOTHING ON MCU
}