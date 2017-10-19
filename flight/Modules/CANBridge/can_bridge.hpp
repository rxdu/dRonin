#ifndef MODULES_CAN_BRIDGE_HPP
#define MODULES_CAN_BRIDGE_HPP

#include "pios_uavcan.hpp"
#include <uavcan/protocol/debug/KeyValue.hpp> // uavcan.protocol.debug.KeyValue

static constexpr unsigned NodeMemoryPoolSize = 2800;

class CANBridge
{
    CANBridge();

    uavcan::Node<NodeMemoryPoolSize> can_node_;
    uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub_;
    uavcan::Subscriber<uavcan::protocol::debug::KeyValue> kv_sub_;

public:
    bool started_;

public:
    static CANBridge& instance()
    {
        static CANBridge can_bridge;
        return can_bridge;
    };

    void updateComm();
};

#endif /* MODULES_CAN_BRIDGE_HPP */