#ifndef CANBUS_CONTROL_H
#define CANBUS_CONTROL_H

#include "stdint.h"
#include "manualcontrolcommand.h"
#include "manualcontrolsettings.h"
#include "stabilizationdesired.h"
#include "stabilizationsettings.h"

//! Initialize the canbus control mode
int32_t canbus_control_initialize();

//! Process inputs and arming
int32_t canbus_control_update();

//! Select and use transmitter control
int32_t canbus_control_select(bool reset_controller);

//! Query what the flight mode _would_ be if this is selected
uint8_t canbus_control_get_flight_mode();

//! Get any control events
enum control_events canbus_control_get_events();

#endif /* CANBUS_CONTROL_H */