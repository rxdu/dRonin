#include "uavcan_pios_driver.hpp"
#include "uavcan_pios_clock.hpp"

extern "C" {
    #include "pios.h"
    #include "jlink_rtt.h"
}

using namespace uavcan;
using namespace pios_uavcan;
using namespace pios_uavcan_internal;

/**************************** C Interface ****************************/
// Defined in C++ source files
// Note: must be declared as 
//  extern "C" void PIOSUAVCAN__FuncName();

extern "C" void PIOSUAVCAN_ResetDriverState()
{
    PIOSUAVCANDriver::instance().resetDriverState();
}

extern "C" void PIOSUAVCAN_TxISR_Callback(uavcan::uint8_t mailbox_index, bool txok)
{   
    PIOSUAVCANDriver::instance().handleTxInterrupt(mailbox_index, txok);
}

extern "C" void PIOSUAVCAN_RxISR_Callback(uint32_t msg_id, bool is_ext, uint8_t dlc, uint8_t data[8], bool overrun)
{
    // JLinkRTTPrintf(0, "id: %x, dlc: %x, data: %2x, %2x, %2x, %2x, %2x, %2x, %2x, %2x\n", 
    //     msg_id, dlc, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );   
    PIOSUAVCANDriver::instance().handleRxInterrupt(msg_id, is_ext, dlc, data, overrun);
}


// Defined in C source files
extern "C" {
    uint8_t PIOS_CAN_TxUAVCANData(uint32_t msg_id, uint8_t is_ext, uint8_t *data); 
    void PIOS_CAN_CancelTransmit(uint8_t Mailbox);
    uint8_t PIOS_CAN_GetLastErrorCode();
    void PIOS_CAN_ClearLEC();
    uint8_t PIOS_CAN_AllMailboxesBusy();
}

/***************************** C++ Class *****************************/

uavcan::int16_t PIOSUAVCANDriver::send(const uavcan::CanFrame& frame,
    uavcan::MonotonicTime tx_deadline,
    uavcan::CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8)
    {
        return -ErrUnsupportedFrame;
    }

    CriticalSectionLocker lock;

    uint8_t ext_flag = 0;
    if(frame.isExtended())
        ext_flag = 1;
    uint8_t txmailbox = PIOS_CAN_TxUAVCANData(frame.id, ext_flag, (uint8_t*)frame.data);

    // No empty mailbox available
    if(txmailbox == CAN_TxStatus_NoMailBox)
        return 0;

    peak_tx_mailbox_index_ = uavcan::max(peak_tx_mailbox_index_, txmailbox);    // Statistics
        
    /*
     * Registering the pending transmission so we can track its deadline and loopback it as needed
     */
    TxItem& txi = tx_pending_[txmailbox];
    txi.deadline       = tx_deadline;
    txi.frame          = frame;
    txi.loopback       = (flags & uavcan::CanIOFlagLoopback) != 0;
    txi.abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
    txi.pending        = true;

    return 1;
}

uavcan::int16_t PIOSUAVCANDriver::receive(uavcan::CanFrame& out_frame,
        uavcan::MonotonicTime& out_ts_monotonic,
        uavcan::UtcTime& out_ts_utc,
        uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = PIOSUAVCANClock::instance().getMonotonic();  // High precision is not required for monotonic timestamps

    uavcan::uint64_t utc_usec = 0;
    {
        CriticalSectionLocker lock;
        if (rx_queue_.getLength() == 0)
        {
            return 0;
        }
        rx_queue_.pop(out_frame, utc_usec, out_flags);
    }
    out_ts_utc = uavcan::UtcTime::fromUSec(utc_usec);

    return 1;
}

uavcan::int16_t PIOSUAVCANDriver::select(uavcan::CanSelectMasks& inout_masks,
        const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces],
        uavcan::MonotonicTime blocking_deadline)
{
    const uavcan::CanSelectMasks in_masks = inout_masks;
    const uavcan::MonotonicTime time = PIOSUAVCANClock::instance().getMonotonic(); 

    discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
    {
        CriticalSectionLocker cs_locker;
        pollErrorFlagsFromISR();
    }

    inout_masks = makeSelectMasks(pending_tx);          // Check if we already have some of the requested events
    if ((inout_masks.read  & in_masks.read)  != 0 ||
        (inout_masks.write & in_masks.write) != 0)
    {
        return 1;
    }

    BusEvent::instance().wait(blocking_deadline - time); // Block until timeout expires or any iface updates
    inout_masks = makeSelectMasks(pending_tx);  // Return what we got even if none of the requested events are set

    return 1;                                   // Return value doesn't matter as long as it is non-negative
}

uavcan::int16_t PIOSUAVCANDriver::configureFilters(const uavcan::CanFilterConfig* filter_configs,
                    uavcan::uint16_t num_configs) 
{
    // filters should be configured in PIOS_CAN_Init() in "pios_can.c"

    return 0;
}

uavcan::uint64_t PIOSUAVCANDriver::getErrorCount() const 
{
    CriticalSectionLocker lock;
    return error_cnt_ + rx_queue_.getOverflowCount();
}

uavcan::uint16_t PIOSUAVCANDriver::getNumFilters() const 
{
    return NumFilters;
}

uavcan::ICanIface* PIOSUAVCANDriver::getIface(uavcan::uint8_t iface_index) 
{
    return (iface_index == 0) ? this : nullptr;
}

uavcan::uint8_t PIOSUAVCANDriver::getNumIfaces() const
{
    return UAVCAN_PIOS_NUM_IFACES;
}

/////////////////////////////////////////////

void PIOSUAVCANDriver::pollErrorFlagsFromISR()
{
    const uavcan::uint8_t lec = PIOS_CAN_GetLastErrorCode();
    
    if (lec != 0)
    {
        PIOS_CAN_ClearLEC();
        error_cnt_++;

        // Serving abort requests
        for (int i = 0; i < NumTxMailboxes; i++)    // Dear compiler, may I suggest you to unroll this loop please.
        {
            TxItem& txi = tx_pending_[i];
            if (txi.pending && txi.abort_on_error)
            {
                PIOS_CAN_CancelTransmit(i);
                txi.pending = false;
                served_aborts_cnt_++;
            }
        }
    }
}

void PIOSUAVCANDriver::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++)
    {
        TxItem& txi = tx_pending_[i];
        if (txi.pending && txi.deadline < current_time)
        {
            // cancel transmission at mailbox i
            PIOS_CAN_CancelTransmit(i);
            txi.pending = false;
            error_cnt_++;
        }
    }
}

bool PIOSUAVCANDriver::canAcceptNewTxFrame(const uavcan::CanFrame& frame) const
{
    /*
     * We can accept more frames only if the following conditions are satisfied:
     *  - There is at least one TX mailbox free (obvious enough);
     *  - The priority of the new frame is higher than priority of all TX mailboxes.
     */
    if(PIOS_CAN_AllMailboxesBusy())
        return false;   // All TX mailboxes are busy transmitting.
    else
        return true;    // At least one TX mailbox is free.

    // The second condition requires a critical section.
    CriticalSectionLocker lock;

    for (int mbx = 0; mbx < NumTxMailboxes; mbx++)
    {
        if (tx_pending_[mbx].pending && !frame.priorityHigherThan(tx_pending_[mbx].frame))
        {
            return false;       // There's a mailbox whose priority is higher or equal the priority of the new frame.
        }
    }

    return true;                // This new frame will be added to a free TX mailbox in the next @ref send().
}

uavcan::CanSelectMasks PIOSUAVCANDriver::makeSelectMasks(const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces]) const
{
    uavcan::CanSelectMasks msk;

    msk.read  = isRxBufferEmpty() ? 0 : 1;

    if (pending_tx[0] != UAVCAN_NULLPTR)
    {
        msk.write = canAcceptNewTxFrame(*pending_tx[0]) ? 1 : 0;
    }

    return msk;
}

bool PIOSUAVCANDriver::isRxBufferEmpty() const
{
    CriticalSectionLocker lock;
    return rx_queue_.getLength() == 0;
}

unsigned PIOSUAVCANDriver::getRxQueueLength() const
{
    CriticalSectionLocker lock;
    return rx_queue_.getLength();
}

bool PIOSUAVCANDriver::hadActivity()
{
    CriticalSectionLocker lock;
    const bool ret = had_activity_;
    had_activity_ = false;
    return ret;
}

void PIOSUAVCANDriver::resetDriverState()
{
    rx_queue_.reset();
    error_cnt_ = 0;
    served_aborts_cnt_ = 0;
    uavcan::fill_n(tx_pending_, NumTxMailboxes, pios_uavcan_internal::TxItem());
    peak_tx_mailbox_index_ = 0;
    had_activity_ = false;
}

/////////////////////////////////////////////

void PIOSUAVCANDriver::handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, const uavcan::uint64_t utc_usec)
{
    UAVCAN_ASSERT(mailbox_index < NumTxMailboxes);

    had_activity_ = had_activity_ || txok;

    TxItem& txi = tx_pending_[mailbox_index];

    if (txi.loopback && txok && txi.pending)
    {
        rx_queue_.push(txi.frame, utc_usec, uavcan::CanIOFlagLoopback);
    }

    txi.pending = false;
}

void PIOSUAVCANDriver::handleTxInterrupt(uavcan::uint8_t mailbox_index, bool txok)
{
    // (mailbox_index ＞＝ NumTxMailboxes) means all mailboxes are full
    if(mailbox_index < NumTxMailboxes)
        handleTxMailboxInterrupt(mailbox_index, txok, 0);
    BusEvent::instance().signalFromInterrupt();
    
    pollErrorFlagsFromISR();
}

void PIOSUAVCANDriver::handleRxInterrupt(uint32_t msg_id, bool is_ext, uint8_t dlc, uint8_t data[8], bool overrun)
{
    // process message from CAN bus
    if (overrun)
        error_cnt_++;

    /*
     * Read the frame contents
     */
    uavcan::CanFrame frame;

    frame.id = msg_id;
    if(is_ext)
        frame.id |= uavcan::CanFrame::FlagEFF;

    frame.dlc = dlc;

    frame.data[0] = data[0];
    frame.data[1] = data[1];
    frame.data[2] = data[2];
    frame.data[3] = data[3];
    frame.data[4] = data[4];
    frame.data[5] = data[5];
    frame.data[6] = data[6];
    frame.data[7] = data[7];

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    rx_queue_.push(frame, 0, 0);
    had_activity_ = true;
    BusEvent::instance().signalFromInterrupt();
    
    pollErrorFlagsFromISR();
}