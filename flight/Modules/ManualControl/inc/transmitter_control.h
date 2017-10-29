/**
 ******************************************************************************
 * @addtogroup Modules Modules
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
#include "drivingstatus.h"
#include "carmanualcontrolcommand.h"
#include "carmanualcontrolsettings.h"
#include "navigationdesired.h"
#include "navigationsettings.h"

/*
 * These are assumptions we make in the driving code about the order of settings and their consistency between
 * objects.  Please keep this synchronized to the UAVObjects
 */
#define assumptions1 ( \
((int)CARMANUALCONTROLSETTINGS_NAVIGATIONSETTINGS_DISABLED      == (int)NAVIGATIONDESIRED_NAVIGATIONMODE_DISABLED)       && \
((int)CARMANUALCONTROLSETTINGS_NAVIGATIONSETTINGS_MANUAL      == (int)NAVIGATIONDESIRED_NAVIGATIONMODE_MANUAL)       && \
((int)CARMANUALCONTROLSETTINGS_NAVIGATIONSETTINGS_LINEFOLLOWING      == (int)NAVIGATIONDESIRED_NAVIGATIONMODE_LINEFOLLOWING)  		\
)

#define assumptions_drivingmode ( \
( (int)CARMANUALCONTROLSETTINGS_DRIVINGMODEPOSITION_MANUAL == (int) DRIVINGSTATUS_DRIVINGMODE_MANUAL) && \
( (int)CARMANUALCONTROLSETTINGS_DRIVINGMODEPOSITION_NAVIGATION == (int) DRIVINGSTATUS_DRIVINGMODE_NAVIGATION) \
)

#define assumptions_channelcount ( \
( (int)CARMANUALCONTROLCOMMAND_CHANNEL_NUMELEM == (int)CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NUMELEM ) && \
( (int)CARMANUALCONTROLCOMMAND_CHANNEL_NUMELEM == (int)CARMANUALCONTROLSETTINGS_CHANNELNUMBER_NUMELEM ) && \
( (int)CARMANUALCONTROLCOMMAND_CHANNEL_NUMELEM == (int)CARMANUALCONTROLSETTINGS_CHANNELMIN_NUMELEM ) && \
( (int)CARMANUALCONTROLCOMMAND_CHANNEL_NUMELEM == (int)CARMANUALCONTROLSETTINGS_CHANNELMAX_NUMELEM ) && \
( (int)CARMANUALCONTROLCOMMAND_CHANNEL_NUMELEM == (int)CARMANUALCONTROLSETTINGS_CHANNELNEUTRAL_NUMELEM ) )

DONT_BUILD_IF(!(assumptions1), TransmitterControlAssumptions1);
DONT_BUILD_IF(!(assumptions_drivingmode), TransmitterControlAssumptions_fm);
DONT_BUILD_IF(!(assumptions_channelcount), TransmitterControlAssumptions_cc);

#define NEQ(a,b) (((int) (a)) != ((int) (b)))

DONT_BUILD_IF(
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_ROLL, CARMANUALCONTROLSETTINGS_NAVIGATIONSETTINGS_ROLL) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_ROLL, NAVIGATIONSETTINGS_MANUALRATE_ROLL) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_ROLL, NAVIGATIONSETTINGS_RATEEXPO_ROLL) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_ROLL, NAVIGATIONSETTINGS_HORIZONEXPO_ROLL) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_ROLL, NAVIGATIONSETTINGS_DEADBANDWIDTH_ROLL) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_ROLL, NAVIGATIONSETTINGS_DEADBANDSLOPE_ROLL),
		ROLLIndicesMismatch);

DONT_BUILD_IF(
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_PITCH, CARMANUALCONTROLSETTINGS_NAVIGATIONSETTINGS_PITCH) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_PITCH, NAVIGATIONSETTINGS_MANUALRATE_PITCH) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_PITCH, NAVIGATIONSETTINGS_RATEEXPO_PITCH) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_PITCH, NAVIGATIONSETTINGS_HORIZONEXPO_PITCH) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_PITCH, NAVIGATIONSETTINGS_DEADBANDWIDTH_PITCH) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_PITCH, NAVIGATIONSETTINGS_DEADBANDSLOPE_PITCH),
		PITCHIndicesMismatch);

DONT_BUILD_IF(
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_YAW, CARMANUALCONTROLSETTINGS_NAVIGATIONSETTINGS_YAW) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_YAW, NAVIGATIONSETTINGS_MANUALRATE_YAW) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_YAW, NAVIGATIONSETTINGS_RATEEXPO_YAW) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_YAW, NAVIGATIONSETTINGS_HORIZONEXPO_YAW) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_YAW, NAVIGATIONSETTINGS_DEADBANDWIDTH_YAW) ||
		NEQ(NAVIGATIONDESIRED_NAVIGATIONMODE_YAW, NAVIGATIONSETTINGS_DEADBANDSLOPE_YAW),
		YAWIndicesMismatch);

//! Initialize the transmitter control mode
int32_t transmitter_control_initialize();

//! Process inputs and arming
int32_t transmitter_control_update();

//! Select and use transmitter control
int32_t transmitter_control_select(bool reset_controller);

//! Query what the flight mode _would_ be if this is selected
uint8_t transmitter_control_get_flight_mode();

//! Get any control events
enum control_status transmitter_control_get_status();

#endif /* TRANSMITTER_CONTROL_H */

/**
 * @}
 * @}
 */
