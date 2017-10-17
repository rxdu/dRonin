#ifndef UAVCAN_PIOS_CLOCK_HPP
#define UAVCAN_PIOS_CLOCK_HPP

#include "uavcan/driver/system_clock.hpp"

namespace pios_uavcan
{

/**
 * Adapter for uavcan::ISystemClock.
 * Reference: uavcan_lpc11c24, from libuavcan
 */
class PIOSUAVCANClock : public uavcan::ISystemClock, uavcan::Noncopyable
{
    PIOSUAVCANClock() = default;

    uavcan::MonotonicTime getMonotonic()     const override;
    uavcan::UtcTime getUtc()                 const override;
    void adjustUtc(uavcan::UtcDuration adjustment) override;

public:
    /**
     * Calls clock::init() as needed.
     */
    static PIOSUAVCANClock& instance()
    {
        static PIOSUAVCANClock pios_uavcan_clock;
        return pios_uavcan_clock;
    }
};

}

#endif /* UAVCAN_PIOS_CLOCK_HPP */