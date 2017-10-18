#include "can_bridge.hpp"

extern "C" {
    #include "jlink_rtt.h"
}

extern "C" void CANBridge_UpdateComm()
{
    SEGGER_RTT_WriteString(0, "Update can bridge\n");
}

CANBridge::CANBridge():
    can_node_(pios_uavcan::PIOSUAVCANDriver::instance(),
        pios_uavcan::PIOSUAVCANClock::instance())
{
    can_node_.setNodeID(16);
    can_node_.setName("CANBridge");
    uavcan::protocol::SoftwareVersion swver;
    
    swver.vcs_commit = 0xef;
    swver.optional_field_flags = swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
    
    can_node_.setSoftwareVersion(swver);
}

void CANBridge::updateComm()
{

}