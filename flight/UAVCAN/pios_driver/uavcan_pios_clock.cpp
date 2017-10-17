#include "uavcan_pios_clock.hpp"

extern "C" {
    uint32_t PIOS_Thread_Systime(void);
}

using namespace uavcan;
using namespace pios_uavcan;

uavcan::MonotonicTime PIOSUAVCANClock::getMonotonic() const 
{ 
    return uavcan::MonotonicTime::fromUSec(PIOS_Thread_Systime()); 
}

uavcan::UtcTime PIOSUAVCANClock::getUtc() const 
{ 
    return uavcan::UtcTime::fromUSec(0); 
}
void PIOSUAVCANClock::adjustUtc(uavcan::UtcDuration adjustment) 
{ 
    // DO NOTHING ON MCU
}