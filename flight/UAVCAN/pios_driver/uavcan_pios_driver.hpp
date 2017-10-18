#ifndef UAVCAN_PIOS_DRIVER_HPP
#define UAVCAN_PIOS_DRIVER_HPP

#include "uavcan/driver/can.hpp"
#include "uavcan_pios_internal.hpp"

// This driver only supports 1 CAN bus @ 1Mbps
#define UAVCAN_PIOS_NUM_IFACES      1
#define UAVCAN_PIOS_RX_BUF_SIZE     128

#define CAN_TxStatus_NoMailBox      ((uint8_t)0x04) 

namespace pios_uavcan
{

enum { NumTxMailboxes = 3 };
enum { NumFilters = 14 };

/**
 * Driver error codes.
 * These values can be returned from driver functions negated.
 */
//static const uavcan::int16_t ErrUnknown               = 1000; ///< Reserved for future use
static const uavcan::int16_t ErrNotImplemented          = 1001; ///< Feature not implemented
static const uavcan::int16_t ErrInvalidBitRate          = 1002; ///< Bit rate not supported
static const uavcan::int16_t ErrLogic                   = 1003; ///< Internal logic error
static const uavcan::int16_t ErrUnsupportedFrame        = 1004; ///< Frame not supported (e.g. RTR, CAN FD, etc)
static const uavcan::int16_t ErrMsrInakNotSet           = 1005; ///< INAK bit of the MSR register is not 1
static const uavcan::int16_t ErrMsrInakNotCleared       = 1006; ///< INAK bit of the MSR register is not 0
static const uavcan::int16_t ErrBitRateNotDetected      = 1007; ///< Auto bit rate detection could not be finished
static const uavcan::int16_t ErrFilterNumConfigs        = 1008; ///< Number of filters is more than supported

/**
 * This class implements CAN driver interface for libuavcan.
 * No configuration needed other than CAN baudrate.
 * This class is a singleton.
 * Reference: uavcan_lpc11c24, from libuavcan
 */
class PIOSUAVCANDriver: public uavcan::ICanDriver, public uavcan::ICanIface, uavcan::Noncopyable
{
    PIOSUAVCANDriver()
    : rx_queue_(rx_buffer_, UAVCAN_PIOS_RX_BUF_SIZE)
    {};

    // RX Queue, which manages RX buffer
    pios_uavcan_internal::RxQueue rx_queue_;

    // Memory allocated for RX&TX items
    pios_uavcan_internal::CanRxItem rx_buffer_[UAVCAN_PIOS_RX_BUF_SIZE];
    pios_uavcan_internal::TxItem tx_pending_[NumTxMailboxes];

    //BusEvent& update_event_;   
    uavcan::uint64_t error_cnt_;
    uavcan::uint32_t served_aborts_cnt_; 
    uavcan::uint8_t peak_tx_mailbox_index_;
    bool had_activity_;

    // Helper functions
    void pollErrorFlagsFromISR();
    void discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time);
    bool canAcceptNewTxFrame(const uavcan::CanFrame& frame) const;
    // This function returns select masks indicating which interfaces are available for read/write.
    uavcan::CanSelectMasks makeSelectMasks(const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces]) const;    

    bool isRxBufferEmpty() const;
    unsigned getRxQueueLength() const;    
    bool hadActivity();

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
            const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces],
            uavcan::MonotonicTime blocking_deadline) override;

    uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
                        uavcan::uint16_t num_configs) override;

    uavcan::uint64_t getErrorCount() const override;

    uavcan::uint16_t getNumFilters() const override;
    
    uavcan::ICanIface* getIface(uavcan::uint8_t iface_index) override;
    
    uavcan::uint8_t getNumIfaces() const override;

    // Connection with PIOS
    void handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, const uavcan::uint64_t utc_usec);
    void resetDriverState();
    void handleTxInterrupt(uavcan::uint8_t mailbox_index, bool txok);
    void handleRxInterrupt(uint32_t msg_id, bool is_ext, uint8_t dlc, uint8_t data[8], bool overrun);
};

}

#endif /* UAVCAN_PIOS_DRIVER_HPP */