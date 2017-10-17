#ifndef UAVCAN_PIOS_DRIVER_HPP
#define UAVCAN_PIOS_DRIVER_HPP

#include "uavcan/driver/can.hpp"

// PIOS only supports 1 CAN bus 
#define UAVCAN_PIOS_NUM_IFACES 1

namespace pios_uavcan
{

/**
 * This class implements CAN driver interface for libuavcan.
 * No configuration needed other than CAN baudrate.
 * This class is a singleton.
 * Reference: uavcan_lpc11c24, from libuavcan
 */
class PIOSUAVCANDriver: public uavcan::ICanDriver, public uavcan::ICanIface, uavcan::Noncopyable
{
    PIOSUAVCANDriver() = default;

public:
    /**
     * Returns the singleton reference.
     * No other copies can be created.
     */
    static PIOSUAVCANDriver& instance() {
        static PIOSUAVCANDriver pios_can_driver;
        return pios_can_driver; 
    }

    uavcan::int16_t send(const uavcan::CanFrame& frame,
        uavcan::MonotonicTime tx_deadline,
        uavcan::CanIOFlags flags) override;

    uavcan::int16_t receive(uavcan::CanFrame& out_frame,
            uavcan::MonotonicTime& out_ts_monotonic,
            uavcan::UtcTime& out_ts_utc,
            uavcan::CanIOFlags& out_flags) override;

    uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks,
            const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
            uavcan::MonotonicTime blocking_deadline) override;

    uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
                        uavcan::uint16_t num_configs) override;

    uavcan::uint64_t getErrorCount() const override;

    uavcan::uint16_t getNumFilters() const override;
    
    uavcan::ICanIface* getIface(uavcan::uint8_t iface_index) override;
    
    uavcan::uint8_t getNumIfaces() const override;
};

}

#endif /* UAVCAN_PIOS_DRIVER_HPP */