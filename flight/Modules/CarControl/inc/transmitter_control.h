/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @addtogroup Control Control Module
 * @{
 *
 * @file       transmitter_control.h
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2015-2016
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2015
 * @brief      Process transmitter inputs and use as control source
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#ifndef TRANSMITTER_CONTROL_H
#define TRANSMITTER_CONTROL_H

#include "stdint.h"
#include "manualcontrolcommand.h"
#include "manualcontrolsettings.h"
#include "stabilizationdesired.h"
#include "stabilizationsettings.h"

//! Initialize the transmitter control mode
int32_t transmitter_control_initialize();

//! Process inputs and arming
int32_t transmitter_control_update();

//! Select and use transmitter control
int32_t transmitter_control_select(bool reset_controller);

//! Query what the flight mode _would_ be if this is selected
uint8_t transmitter_control_get_flight_mode();

//! Get any control events
enum control_events transmitter_control_get_events();

#endif /* TRANSMITTER_CONTROL_H */

/**
 * @}
 * @}
 */
