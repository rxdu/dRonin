#include "can_bridge.hpp"

extern "C" {
    #include "jlink_rtt.h"
    #include "can_bridge_task.h"
}

extern "C" void CANBridge_UpdateComm(bool sensor_updated, struct CANIMURawData *gyro, struct CANIMURawData *accel)
{
    // SEGGER_RTT_WriteString(0, "Update can bridge\n");
    CANBridge::instance().updateComm(sensor_updated, gyro, accel);
}

CANBridge::CANBridge():
    can_node_(pios_uavcan::PIOSUAVCANDriver::instance(),
        pios_uavcan::PIOSUAVCANClock::instance()),
    kv_pub_(can_node_),
    kv_sub_(can_node_),
    imu_pub_(can_node_),
    started_(false)
{
    can_node_.setNodeID(16);
    can_node_.setName("pixcar.CANBridge");

    const int node_init_res = can_node_.start();
    if (node_init_res < 0)
        started_ = false;
    else
        started_ = true;
    
    const int kv_pub_init_res = kv_pub_.init();
    if (kv_pub_init_res < 0)
        SEGGER_RTT_WriteString(0, "Failed to init publisher\n");
    else
        SEGGER_RTT_WriteString(0, "Publisher init successfully\n");
    kv_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
    kv_pub_.setPriority(uavcan::TransferPriority::MiddleLower);    

    // const int kv_sub_start_res =
    // kv_sub_.start([&](const uavcan::protocol::debug::KeyValue& msg)
    //     {
    //         SEGGER_RTT_WriteString(0, "Msg received\n");
    //         JLinkRTTPrintf(0, "msg key: %d, %d, %d\n",msg.key[0],msg.key[1],msg.key[2]);
    //     }
    // );
    // if (kv_sub_start_res < 0)
    //     SEGGER_RTT_WriteString(0, "Failed to init subscriber\n");
    // else
    //     SEGGER_RTT_WriteString(0, "Subscriber init successfully\n");

    can_node_.setModeOperational();    
}

void CANBridge::updateComm(bool sensor_updated, struct CANIMURawData *gyro, struct CANIMURawData *accel)
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
    // kv_msg.key = "r";   // "a"
    // kv_msg.key += "u";  // "ab"
    // kv_msg.key += "i";  // "abc"
    // kv_msg.key += "x";  // "abc"
    // kv_msg.key += "i";  // "abc"
    // kv_msg.key += "a";  // "abc"
    // kv_msg.key += "n";  // "abc"
    // kv_msg.key += "g";  // "abc"
    // kv_msg.key += "d";  // "abc"
    // kv_msg.key += "u";  // "abc"
    kv_msg.key = "ruixiang";

    // JLinkRTTPrintf(0, "cap: %ld, used: %ld, peak: %ld\n", 
    //     can_node_.getAllocator().getBlockCapacity(), 
    //     can_node_.getAllocator().getNumUsedBlocks(),
    //     can_node_.getAllocator().getPeakNumUsedBlocks());

    /*
     * Publishing the message.
     */
    // const int pub_res = kv_pub_.broadcast(kv_msg);
    // if (pub_res < 0)
    // {
    //     SEGGER_RTT_WriteString(0, "KV publication failure\n");
    // }    
    // else
    // {
    //     SEGGER_RTT_WriteString(0, "CAN msg sent successfully\n");
    // }

    if(sensor_updated)
    {
        uavcan::equipment::ahrs::RawIMU imu_msg;  // Always zero initialized
        imu_msg.rate_gyro_latest[0] = gyro->x;
        imu_msg.rate_gyro_latest[1] = gyro->y;
        imu_msg.rate_gyro_latest[2] = gyro->z;

        imu_msg.accelerometer_latest[0] = accel->x;
        imu_msg.accelerometer_latest[1] = accel->y;
        imu_msg.accelerometer_latest[2] = accel->z;

        const int pub_res = imu_pub_.broadcast(imu_msg);
        if (pub_res < 0)
        {
            SEGGER_RTT_WriteString(0, "IMU msg publication failure\n");
        }    
        else
        {
            SEGGER_RTT_WriteString(0, "IMU msg sent successfully\n");
        }
    }
}