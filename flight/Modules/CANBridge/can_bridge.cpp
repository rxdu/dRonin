#include "can_bridge.hpp"

extern "C" {
    #include "jlink_rtt.h"
}

extern "C" void CANBridge_UpdateComm()
{
    // SEGGER_RTT_WriteString(0, "Update can bridge\n");
    CANBridge::instance().updateComm();
}

CANBridge::CANBridge():
    can_node_(pios_uavcan::PIOSUAVCANDriver::instance(),
        pios_uavcan::PIOSUAVCANClock::instance()),
    kv_pub_(can_node_),
    started_(false)
{
    can_node_.setNodeID(16);
    can_node_.setName("CANBridge");
    uavcan::protocol::SoftwareVersion swver;
    
    swver.vcs_commit = 0xef;
    swver.optional_field_flags = swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
    
    can_node_.setSoftwareVersion(swver);

    const int node_init_res = can_node_.start();
    if (node_init_res < 0)
        started_ = false;
    else
        started_ = true;
    
    const int kv_pub_init_res = kv_pub_.init();
    if (kv_pub_init_res < 0)
    {
        SEGGER_RTT_WriteString(0, "Failed to init publisher\n");
    }
    kv_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
    kv_pub_.setPriority(uavcan::TransferPriority::MiddleLower);    

    can_node_.setModeOperational();    
}

void CANBridge::updateComm()
{
    const int spin_res = can_node_.spin(uavcan::MonotonicDuration::fromMSec(1000));
    if (spin_res < 0)
    {
        SEGGER_RTT_WriteString(0, "Transient failure \n");
    }

    /*
     * Publishing a random value using the publisher created above.
     * All message types have zero-initializing default constructors.
     * Relevant usage info for every data type is provided in its DSDL definition.
     */
    uavcan::protocol::debug::KeyValue kv_msg;  // Always zero initialized
    kv_msg.value = std::rand() / float(RAND_MAX);

    /*
     * Arrays in DSDL types are quite extensive in the sense that they can be static,
     * or dynamic (no heap needed - all memory is pre-allocated), or they can emulate std::string.
     * The last one is called string-like arrays.
     * ASCII strings can be directly assigned or appended to string-like arrays.
     * For more info, please read the documentation for the class uavcan::Array<>.
     */
    kv_msg.key = "a";   // "a"
    kv_msg.key += "b";  // "ab"
    kv_msg.key += "c";  // "abc"

    /*
     * Publishing the message.
     */
    const int pub_res = kv_pub_.broadcast(kv_msg);
    if (pub_res < 0)
    {
        SEGGER_RTT_WriteString(0, "KV publication failure\n");
    }    
    else
    {
        SEGGER_RTT_WriteString(0, "CAN msg sent successfully\n");
    }
}