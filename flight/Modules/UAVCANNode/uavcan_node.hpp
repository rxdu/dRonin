#ifndef MODULES_CAN_BRIDGE_HPP
#define MODULES_CAN_BRIDGE_HPP

extern "C" {
    #include "can_bridge_task.h"
}

#include "pios_uavcan.hpp"
#include <pixcar/CarRawIMU.hpp>
#include <pixcar/CarRawMag.hpp>
#include <pixcar/CarRawSpeed.hpp>
#include <pixcar/CarCommand.hpp>

static constexpr unsigned NodeMemoryPoolSize = 2800;

class UAVCANNode
{
    UAVCANNode();

    uavcan::Node<NodeMemoryPoolSize> can_node_;
    
    uavcan::Publisher<pixcar::CarRawIMU> imu_pub_;
    uavcan::Publisher<pixcar::CarRawMag> mag_pub_;
    uavcan::Publisher<pixcar::CarRawSpeed> spd_pub_;
    uavcan::Subscriber<pixcar::CarCommand> cmd_sub_;

public:
    bool started_;

public:
    static UAVCANNode& instance()
    {
        static UAVCANNode can_bridge;
        return can_bridge;
    };

    void spinNode(int32_t spin_timeout);
    void updateComm(struct CANIMURawData *imu_data, float *speed, int32_t spin_timeout);
};

#endif /* MODULES_CAN_BRIDGE_HPP */