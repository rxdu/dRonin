#include "uavcan_pios_internal.hpp"

using namespace pios_uavcan::pios_uavcan_internal;

/*
 * CanIface::RxQueue
 */
void RxQueue::registerOverflow()
{
    if (overflow_cnt_ < 0xFFFFFFFF)
    {
        overflow_cnt_++;
    }
}

void RxQueue::push(const uavcan::CanFrame& frame, const uint64_t& utc_usec, uavcan::CanIOFlags flags)
{
    buf_[in_].frame    = frame;
    buf_[in_].utc_usec = utc_usec;
    buf_[in_].flags    = flags;
    in_++;
    if (in_ >= capacity_)
    {
        in_ = 0;
    }
    len_++;
    if (len_ > capacity_)
    {
        len_ = capacity_;
        registerOverflow();
        out_++;
        if (out_ >= capacity_)
        {
            out_ = 0;
        }
    }
}

void RxQueue::pop(uavcan::CanFrame& out_frame, uavcan::uint64_t& out_utc_usec, uavcan::CanIOFlags& out_flags)
{
    if (len_ > 0)
    {
        out_frame    = buf_[out_].frame;
        out_utc_usec = buf_[out_].utc_usec;
        out_flags    = buf_[out_].flags;
        out_++;
        if (out_ >= capacity_)
        {
            out_ = 0;
        }
        len_--;
    }
    else { UAVCAN_ASSERT(0); }
}

void RxQueue::reset()
{
    in_ = 0;
    out_ = 0;
    len_ = 0;
    overflow_cnt_ = 0;
}