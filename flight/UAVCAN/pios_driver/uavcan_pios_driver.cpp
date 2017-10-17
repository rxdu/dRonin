#include "uavcan_pios_driver.hpp"

using namespace uavcan;
using namespace pios_uavcan;

uavcan::int16_t PIOSUAVCANDriver::send(const uavcan::CanFrame& frame,
    uavcan::MonotonicTime tx_deadline,
    uavcan::CanIOFlags flags)
{
    return 0;
}

uavcan::int16_t PIOSUAVCANDriver::receive(uavcan::CanFrame& out_frame,
        uavcan::MonotonicTime& out_ts_monotonic,
        uavcan::UtcTime& out_ts_utc,
        uavcan::CanIOFlags& out_flags)
{
    return 0;
}

uavcan::int16_t PIOSUAVCANDriver::select(uavcan::CanSelectMasks& inout_masks,
        const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
        uavcan::MonotonicTime blocking_deadline)
{
    return 0;
}

uavcan::int16_t PIOSUAVCANDriver::configureFilters(const uavcan::CanFilterConfig* filter_configs,
                    uavcan::uint16_t num_configs) 
{
    return 0;
}

uavcan::uint64_t PIOSUAVCANDriver::getErrorCount() const 
{
    return 0;
}

uavcan::uint16_t PIOSUAVCANDriver::getNumFilters() const 
{
    return 0;
}

uavcan::ICanIface* PIOSUAVCANDriver::getIface(uavcan::uint8_t iface_index) 
{
    return (iface_index == 0) ? this : nullptr;
}

uavcan::uint8_t PIOSUAVCANDriver::getNumIfaces() const
{
    return UAVCAN_PIOS_NUM_IFACES;
}