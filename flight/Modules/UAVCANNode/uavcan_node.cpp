#include "can_bridge.hpp"

extern "C" {
    #include "jlink_rtt.h"
    #include "can_bridge_task.h"
    uint32_t PIOS_Thread_Systime(void);    
}

extern "C" void updateCmdFromCAN(float servo_cmd, float motor_cmd);

extern "C" bool UAVCANNode_InitComm()
{
    return UAVCANNode::instance().started_;
}

extern "C" void UAVCANNode_UpdateComm(struct CANIMURawData *imu_data, float *speed, int32_t spin_timeout)
{
    // SEGGER_RTT_WriteString(0, "Update can bridge\n");
    UAVCANNode::instance().updateComm(imu_data, speed, spin_timeout);
}

UAVCANNode::UAVCANNode():
    can_node_(pios_uavcan::PIOSUAVCANDriver::instance(),
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
    imu_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
    imu_pub_.setPriority(uavcan::TransferPriority::OneLowerThanHighest);    

    const int mag_pub_init_res = mag_pub_.init();
    if (mag_pub_init_res < 0)
        SEGGER_RTT_WriteString(0, "Failed to init MAG publisher\n");
    else
        SEGGER_RTT_WriteString(0, "Publisher MAG init successfully\n");
    mag_pub_.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
    mag_pub_.setPriority(uavcan::TransferPriority::OneLowerThanHighest);    

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

void spinNode(int32_t spin_timeout)
{
    const int spin_res = can_node_.spin(uavcan::MonotonicDuration::fromMSec(spin_timeout));
    if (spin_res < 0)
    {
        SEGGER_RTT_WriteString(0, "UAVCAN node transient failure \n");
    }
}

void UAVCANNode::updateComm(struct CANIMURawData *imu_data, float *speed, int32_t spin_timeout)
{
    uint32_t time_stamp = PIOS_Thread_Systime();

    const int spin_res = can_node_.spin(uavcan::MonotonicDuration::fromMSec(spin_timeout));
    if (spin_res < 0)
    {
        SEGGER_RTT_WriteString(0, "Transient failure \n");
    }
    // can_node_.spinOnce();

    if(imu_data->gyro_accel_updated)
    {
        pixcar::CarRawIMU imu_msg;  // Always zero initialized
        
        imu_msg.time_stamp = time_stamp;
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

    if(imu_data->mag_updated)
    {
        pixcar::CarRawMag mag_msg;  

        mag_msg.time_stamp = time_stamp;
        mag_msg.mag[0] = imu_data->mag.x;
        mag_msg.mag[1] = imu_data->mag.y;
        mag_msg.mag[2] = imu_data->mag.z;

        const int mag_pub_res = mag_pub_.broadcast(mag_msg);
        (void)mag_pub_res;
    }

    pixcar::CarRawSpeed spd_msg;
    spd_msg.time_stamp = time_stamp;
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