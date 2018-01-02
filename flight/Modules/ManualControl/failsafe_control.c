/**
 ******************************************************************************
 * @addtogroup Modules Modules
 * @{
 * @addtogroup Control Control Module
 * @{
 *
 * @file       failsafe_control.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2015-2016
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2014
 * @brief      Failsafe controller when transmitter control is lost
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

#include "openpilot.h"
#include "control.h"
#include "failsafe_control.h"
#include "transmitter_control.h"

#include "drivingstatus.h"
#include "caractuatordesired.h"
#include "systemsettings.h"

//! Initialize the failsafe controller
int32_t failsafe_control_initialize()
{
	return 0;
}

//! Perform any updates to the failsafe controller
int32_t failsafe_control_update()
{
	return 0;
}

static bool armed_when_enabled;
/**
 * Select and use failsafe control
 * @param [in] reset_controller True if previously another controller was used
 */
int32_t failsafe_control_select(bool reset_controller)
{
	if (reset_controller) {
		DrivingStatusArmedOptions armed; 
		DrivingStatusArmedGet(&armed);
		armed_when_enabled = (armed == DRIVINGSTATUS_ARMED_ARMED);
	}

	uint8_t driving_status;
	DrivingStatusDrivingModeGet(&driving_status);
	if (driving_status != DRIVINGSTATUS_DRIVINGMODE_EMERGENCY|| reset_controller) {
		driving_status = DRIVINGSTATUS_DRIVINGMODE_EMERGENCY;
		DrivingStatusDrivingModeSet(&driving_status);
	}

	CarActuatorDesiredData actuator;
	CarActuatorDesiredGet(&actuator);
	actuator.Roll = 0;
	actuator.Pitch = 0;
	actuator.Yaw = 0;
	actuator.Steering = 0;
	actuator.Throttle = 0;
	CarActuatorDesiredSet(&actuator);

	return 0;
}

/**
 * @}
 * @}
 */

