/* 
 * uavcan_node.hpp
 * 
 * Created on: Nov 04, 2017 17:08
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef UAVCAN_NODE_HPP
#define UAVCAN_NODE_HPP

extern "C" {
    #include "uavcan_interface.h"
}

#include "pios_uavcan.hpp"

#include <pixcar/CarRawIMU.hpp>
#include <pixcar/CarRawMag.hpp>
#include <pixcar/CarRawSpeed.hpp>
#include <pixcar/CarCommand.hpp>

static constexpr unsigned NodeMemoryPoolSize = 4096;

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

    void publishIMUData(struct CANIMURawData *imu_data);
    void publishMagData(struct CANMagRawData *mag_data);
    void publishSpeedData(struct CANSpeedRawData *spd_data);
};

#endif /* UAVCAN_NODE_HPP */
