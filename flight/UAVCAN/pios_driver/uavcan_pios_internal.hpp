#ifndef UAVCAN_PIOS_INTERNAL_HPP
#define UAVCAN_PIOS_INTERNAL_HPP

#include "uavcan/driver/can.hpp"

namespace pios_uavcan
{

namespace pios_uavcan_internal
{
   
/**
 * RX queue item.
 * The application shall not use this directly.
 */
struct CanRxItem
{
    uavcan::uint64_t utc_usec;
    uavcan::CanFrame frame;
    uavcan::CanIOFlags flags;

    CanRxItem():utc_usec(0),flags(0){}
};

struct TxItem
{
    uavcan::MonotonicTime deadline;
    uavcan::CanFrame frame;
    bool pending;
    bool loopback;
    bool abort_on_error;

    TxItem()
        : pending(false)
        , loopback(false)
        , abort_on_error(false)
    { }
};

class RxQueue
{
    CanRxItem* const buf_;
    const uavcan::uint8_t capacity_;
    uavcan::uint8_t in_;
    uavcan::uint8_t out_;
    uavcan::uint8_t len_;
    uavcan::uint32_t overflow_cnt_;

    void registerOverflow();

public:
    RxQueue(CanRxItem* buf, uavcan::uint8_t capacity)
        : buf_(buf)
        , capacity_(capacity)
        , in_(0)
        , out_(0)
        , len_(0)
        , overflow_cnt_(0)
    { }

    void push(const uavcan::CanFrame& frame, const uint64_t& utc_usec, uavcan::CanIOFlags flags);
    void pop(uavcan::CanFrame& out_frame, uavcan::uint64_t& out_utc_usec, uavcan::CanIOFlags& out_flags);

    void reset();

    unsigned getLength() const { return len_; }

    uavcan::uint32_t getOverflowCount() const { return overflow_cnt_; }
};

}
}

#endif /* UAVCAN_PIOS_INTERNAL_HPP */