#ifndef MODULES_CAN_BRIDGE_HPP
#define MODULES_CAN_BRIDGE_HPP

#include "pios_uavcan.hpp"

static constexpr unsigned NodeMemoryPoolSize = 2800;

class CANBridge
{
    CANBridge();

    uavcan::Node<NodeMemoryPoolSize> can_node_;

public:
    static CANBridge& instance()
    {
        static CANBridge can_bridge;
        return can_bridge;
    };

    void updateComm();
};

#endif /* MODULES_CAN_BRIDGE_HPP */