#include "can_bridge.hpp"

extern "C" {
    #include "jlink_rtt.h"
    #include "can_bridge_task.h"
}

extern "C" void updateCmdFromCAN(float servo_cmd, float motor_cmd);

extern "C" bool CANBridge_InitComm()
{
    return CANBridge::instance().started_;
}

extern "C" void CANBridge_UpdateComm(bool sensor_updated, struct CANIMURawData *gyro, struct CANIMURawData *accel, float * speed, int32_t spin_timeout)
{
    // SEGGER_RTT_WriteString(0, "Update can bridge\n");
    CANBridge::instance().updateComm(sensor_updated, gyro, accel, speed, spin_timeout);
}

CANBridge::CANBridge():
    can_node_(pios_uavcan::PIOSUAVCANDriver::instance(),
        pios_uavcan::PIOSUAVCANClock::instance()),
    imu_pub_(can_node_),
    spd_pub_(can_node_),
    cmd_sub_(can_node_),
    started_(false)
{
    can_node_.setNodeID(16);
    can_node_.setName("pixcar.CANBridge");

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
    imu_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
    imu_pub_.setPriority(uavcan::TransferPriority::OneLowerThanHighest);    

    const int spd_pub_init_res = spd_pub_.init();
    if (spd_pub_init_res < 0)
        SEGGER_RTT_WriteString(0, "Failed to init Speed publisher\n");
    else
        SEGGER_RTT_WriteString(0, "Publisher Speed init successfully\n");
    spd_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
    spd_pub_.setPriority(uavcan::TransferPriority::OneLowerThanHighest);    

    const int cmd_sub_start_res =
    cmd_sub_.start([&](const pixcar::CarCommand& msg)
        {
            JLinkRTTPrintf(0, "Msg received: %d, %d\n",(int32_t)(msg.servo_cmd*100), (int32_t)(msg.motor_cmd*100));

            updateCmdFromCAN(msg.servo_cmd, msg.motor_cmd);
        }
    );
    if (cmd_sub_start_res < 0)
        SEGGER_RTT_WriteString(0, "Failed to init CMD subscriber\n");
    else
        SEGGER_RTT_WriteString(0, "Subscriber CMD init successfully\n");

    can_node_.setModeOperational();    
}

void CANBridge::updateComm(bool sensor_updated, struct CANIMURawData *gyro, struct CANIMURawData *accel, float * speed, int32_t spin_timeout)
{
    const int spin_res = can_node_.spin(uavcan::MonotonicDuration::fromMSec(spin_timeout));
    if (spin_res < 0)
    {
        SEGGER_RTT_WriteString(0, "Transient failure \n");
    }
    // can_node_.spinOnce();

    if(sensor_updated)
    {
        pixcar::CarRawIMU imu_msg;  // Always zero initialized
        imu_msg.gyro[0] = gyro->x;
        imu_msg.gyro[1] = gyro->y;
        imu_msg.gyro[2] = gyro->z;

        imu_msg.accel[0] = accel->x;
        imu_msg.accel[1] = accel->y;
        imu_msg.accel[2] = accel->z;

        // imu_msg.gyro[0] = (float)0.1;
        // imu_msg.gyro[1] = (float)0.2;
        // imu_msg.gyro[2] = (float)0.3;

        // imu_msg.accel[0] = (float)0.4;
        // imu_msg.accel[1] = (float)0.5;
        // imu_msg.accel[2] = (float)0.6;

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

    pixcar::CarSpeed spd_msg;
    spd_msg.speed = *speed;
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