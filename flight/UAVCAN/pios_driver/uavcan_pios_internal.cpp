#include "uavcan_pios_internal.hpp"

using namespace pios_uavcan::pios_uavcan_internal;

SEMAPHORE_DECL(bus_sem, 0);

bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
    static const uavcan::int64_t MaxDelayMSec = 0x000FFFFF;
    
    const uavcan::int64_t msec = duration.toMSec();
    msg_t ret = msg_t();
    
    if (msec <= 0)
    {
# if (CH_KERNEL_MAJOR == 2)
        ret = chSemWaitTimeout(&bus_sem,TIME_IMMEDIATE);
# else // ChibiOS 3+
        ret = chSemWaitTimeout(&bus_sem,TIME_IMMEDIATE);
# endif
    }
    else
    {
# if (CH_KERNEL_MAJOR == 2)
        ret = chSemWaitTimeout(&bus_sem, (msec > MaxDelayMSec) ? MS2ST(MaxDelayMSec) : MS2ST(msec));
# else // ChibiOS 3+
        ret = chSemWaitTimeout(&bus_sem, (msec > MaxDelayMSec) ? MS2ST(MaxDelayMSec) : MS2ST(msec));
# endif
    }
# if (CH_KERNEL_MAJOR == 2)
    return ret == RDY_OK;
# else // ChibiOS 3+
    return ret == MSG_OK;
# endif
}

void BusEvent::signal()
{
    chSemSignal(&bus_sem);
}

void BusEvent::signalFromInterrupt()
{
# if (CH_KERNEL_MAJOR == 2)
    chSysLockFromIsr();
    chSemSignalI(&bus_sem);
    chSysUnlockFromIsr();
# else // ChibiOS 3+
    chSysLockFromISR();
    chSemSignalI(&bus_sem);
    chSysUnlockFromISR();
# endif
}

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