#ifndef MODULES_CAN_BRIDGE_HPP
#define MODULES_CAN_BRIDGE_HPP

extern "C" {
    #include "can_bridge_task.h"
}

#include "pios_uavcan.hpp"
#include <uavcan/protocol/debug/KeyValue.hpp> // uavcan.protocol.debug.KeyValue
#include <uavcan/equipment/ahrs/RawIMU.hpp>

static constexpr unsigned NodeMemoryPoolSize = 2800;

class CANBridge
{
    CANBridge();

    uavcan::Node<NodeMemoryPoolSize> can_node_;

    uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub_;
    uavcan::Subscriber<uavcan::protocol::debug::KeyValue> kv_sub_;
    
    uavcan::Publisher<uavcan::equipment::ahrs::RawIMU> imu_pub_;

public:
    bool started_;

public:
    static CANBridge& instance()
    {
        static CANBridge can_bridge;
        return can_bridge;
    };

    void updateComm(bool sensor_updated, struct CANIMURawData *gyro, struct CANIMURawData *accel);
};

#endif /* MODULES_CAN_BRIDGE_HPP */