#ifndef CAN_BRIDGE_TASK_H
#define CAN_BRIDGE_TASK_H

struct CANIMUSensorData
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

struct CANIMURawData
{
    bool gyro_accel_updated;
    struct CANIMUSensorData gyro;
    struct CANIMUSensorData accel;

    bool mag_updated;
    struct CANIMUSensorData mag;
};

void getCmdFromCAN(float* servo_cmd, float* motor_cmd);
void resetCmdFromCAN(void);

#endif /* CAN_BRIDGE_TASK_H */