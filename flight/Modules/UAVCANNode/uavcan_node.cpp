#include "uavcan_node.hpp"

extern "C" {
    #include "jlink_rtt.h"
}

extern "C" void updateCmdFromCAN(float servo_cmd, float motor_cmd);

extern "C" bool UAVCANNode_InitComm()
{
    return UAVCANNode::instance().started_;
}

extern "C" void UAVCANNode_SpinNode(int32_t timeout)
{
    UAVCANNode::instance().spinNode(timeout);
}

extern "C" void UAVCANNode_PublishIMUData(struct CANIMURawData *imu_data)
{
    UAVCANNode::instance().publishIMUData(imu_data);
}

extern "C" void UAVCANNode_PublishMagData(struct CANMagRawData *mag_data)
{
    UAVCANNode::instance().publishMagData(mag_data);
}

extern "C" void UAVCANNode_PublishSpeedData(struct CANSpeedRawData *spd_data)
{
    UAVCANNode::instance().publishSpeedData(spd_data);
}

/*****************************************************************************/

UAVCANNode::UAVCANNode() : can_node_(pios_uavcan::PIOSUAVCANDriver::instance(),
                                     pios_uavcan::PIOSUAVCANClock::instance()),
                           imu_pub_(can_node_),
                           mag_pub_(can_node_),
                           spd_pub_(can_node_),
                           cmd_sub_(can_node_),
                           started_(false)
{
    can_node_.setNodeID(16);
    can_node_.setName("pixcar.UAVCANNode");

    const int node_init_res = can_node_.start();
    if (node_init_res < 0)
        started_ = false;
    else
        started_ = true;

    const int imu_pub_init_res = imu_pub_.init();
    if (imu_pub_init_res < 0)
        SEGGER_RTT_WriteString(0, "Failed to init IMU publisher\n");
    else
        SEGGER_RTT_WriteString(0, "Publisher IMU init successfully\n");
    imu_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(5));
    imu_pub_.setPriority(uavcan::TransferPriority(IMU_MSG_PRIORITY));

    const int mag_pub_init_res = mag_pub_.init();
    if (mag_pub_init_res < 0)
        SEGGER_RTT_WriteString(0, "Failed to init MAG publisher\n");
    else
        SEGGER_RTT_WriteString(0, "Publisher MAG init successfully\n");
    mag_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(13));
    mag_pub_.setPriority(uavcan::TransferPriority(MAG_MSG_PRIORITY));

    const int spd_pub_init_res = spd_pub_.init();
    if (spd_pub_init_res < 0)
        SEGGER_RTT_WriteString(0, "Failed to init Speed publisher\n");
    else
        SEGGER_RTT_WriteString(0, "Publisher Speed init successfully\n");
    spd_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(10));
    spd_pub_.setPriority(uavcan::TransferPriority(SPD_MSG_PRIORITY));

    const int cmd_sub_start_res =
        cmd_sub_.start([&](const pixcar::CarCommand &msg) {
            JLinkRTTPrintf(0, "Msg received: %d, %d\n", (int32_t)(msg.servo_cmd * 100), (int32_t)(msg.motor_cmd * 100));

            struct pios_can_cmd_data cmd;
            cmd.steering = msg.servo_cmd;
            cmd.throttle = msg.motor_cmd;
            PIXCAR_SetNavigationDesired(&cmd);
        });
    if (cmd_sub_start_res < 0)
        SEGGER_RTT_WriteString(0, "Failed to init CMD subscriber\n");
    else
        SEGGER_RTT_WriteString(0, "Subscriber CMD init successfully\n");

    can_node_.setModeOperational();
}

void UAVCANNode::spinNode(int32_t spin_timeout)
{
    const int spin_res = can_node_.spin(uavcan::MonotonicDuration::fromMSec(spin_timeout));
    if (spin_res < 0)
    {
        SEGGER_RTT_WriteString(0, "UAVCAN node transient failure \n");
    }
    // else
    //      SEGGER_RTT_WriteString(0, "UAVCAN node spin \n");
    // JLinkRTTPrintf(0, "UAVCAN memory usage (used-free-peak): %ld, %ld, %ld\n", can_node_.getAllocator().getNumUsedBlocks(), 
    //                 can_node_.getAllocator().getNumFreeBlocks(),
    //                 can_node_.getAllocator().getPeakNumUsedBlocks());
}

void UAVCANNode::publishIMUData(struct CANIMURawData *imu_data)
{
    pixcar::CarRawIMU imu_msg; // Always zero initialized

    imu_msg.time_stamp = imu_data->time_stamp;

    imu_msg.gyro[0] = imu_data->gyro.x;
    imu_msg.gyro[1] = imu_data->gyro.y;
    imu_msg.gyro[2] = imu_data->gyro.z;

    imu_msg.accel[0] = imu_data->accel.x;
    imu_msg.accel[1] = imu_data->accel.y;
    imu_msg.accel[2] = imu_data->accel.z;

    const int imu_pub_res = imu_pub_.broadcast(imu_msg);
    (void)imu_pub_res;
    // if (imu_pub_res < 0)
    // {
    //     SEGGER_RTT_WriteString(0, "IMU msg publication failure\n");
    // }
    // else
    // {
    //     SEGGER_RTT_WriteString(0, "IMU msg sent successfully\n");
    // }
}

void UAVCANNode::publishMagData(struct CANMagRawData *mag_data)
{
    pixcar::CarRawMag mag_msg;

    mag_msg.time_stamp = mag_data->time_stamp;
    mag_msg.mag[0] = mag_data->mag.x;
    mag_msg.mag[1] = mag_data->mag.y;
    mag_msg.mag[2] = mag_data->mag.z;

    const int mag_pub_res = mag_pub_.broadcast(mag_msg);
    (void)mag_pub_res;
}

void UAVCANNode::publishSpeedData(struct CANSpeedRawData *spd_data)
{
    pixcar::CarRawSpeed spd_msg;

    spd_msg.time_stamp = spd_data->time_stamp;
    spd_msg.hallsensor_count = spd_data->hallsensor_count;
    spd_msg.speed_estimate = spd_data->speed_estimate;

    const int spd_pub_res = spd_pub_.broadcast(spd_msg);
    (void)spd_pub_res;
    // if (spd_pub_res < 0)
    // {
    //     SEGGER_RTT_WriteString(0, "Speed msg publication failure\n");
    // }
    // else
    // {
    //     SEGGER_RTT_WriteString(0, "Speed msg sent successfully\n");
    // }
}
