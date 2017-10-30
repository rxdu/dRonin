#ifndef CAN_BRIDGE_TASK_H
#define CAN_BRIDGE_TASK_H

struct CANIMURawData
{
    float x;
    float y;
    float z;
};

struct CANCmdData
{
    float servo;
    float motor;
};

void getCmdFromCAN(float* servo_cmd, float* motor_cmd);
void resetCmdFromCAN(void);

#endif /* CAN_BRIDGE_TASK_H */