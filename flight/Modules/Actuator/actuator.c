/**
 ******************************************************************************
 * @addtogroup Modules Modules
 * @{
 * @addtogroup ActuatorModule Actuator Module
 * @{
 *
 * @file       actuator.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2015-2016
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2016
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Actuator module. Drives the actuators (servos, motors etc).
 * @brief      Take the values in @ref ActuatorDesired and mix to set the outputs
 *
 * This module ultimately controls the outputs.  The values from @ref ActuatorDesired
 * are combined based on the values in @ref MixerSettings and then scaled by the
 * values in @ref ActuatorSettings to create the output PWM times.
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

#include <math.h>

#include "openpilot.h"
#include "pios_thread.h"
#include "pios_queue.h"
#include "misc_math.h"

#include "systemsettings.h"
#include "drivingstatus.h"
#include "cameradesired.h"
#include "caractuatorsettings.h"
#include "caractuatordesired.h"
#include "caractuatorcommand.h"
#include "carmixersettings.h"
#include "carmanualcontrolcommand.h"

#include "jlink_rtt.h"

// Private constants
#define MAX_QUEUE_SIZE 2

#if defined(PIOS_ACTUATOR_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_ACTUATOR_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1336
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGHEST
#define FAILSAFE_TIMEOUT_MS 100

#define MIXER_SCALE 128

#define MAX_MIX_ACTUATORS 2

#ifndef MAX_MIX_ACTUATORS
#define MAX_MIX_ACTUATORS CARACTUATORCOMMAND_CHANNEL_NUMELEM
#endif

DONT_BUILD_IF(CARACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM > PIOS_SERVO_MAX_BANKS, TooManyServoBanks);
DONT_BUILD_IF(MAX_MIX_ACTUATORS > CARACTUATORCOMMAND_CHANNEL_NUMELEM, TooManyMixers);

// Private types

// Private variables
static struct pios_queue *queue;
static struct pios_thread *taskHandle;

// used to inform the actuator thread that actuator / mixer settings are updated
// set true to ensure they're fetched on first run
static volatile bool driving_status_updated = true;
static volatile bool manual_control_cmd_updated = true;
static volatile bool actuator_settings_updated = true;
static volatile bool mixer_settings_updated = true;

/* These are various settings objects used throughout the actuator code */
static CarActuatorSettingsData actuatorSettings;

// The actual mixer settings data, pulled at the top of the actuator thread
static CarMixerSettingsData mixerSettings;
static CarMixerSettingsMixer1TypeOptions types_mixer[MAX_MIX_ACTUATORS];
/* In the mixer, a row consists of values for one output actuator.
 * A column consists of values for scaling one axis's desired command.
 */
static float motor_mixer[MAX_MIX_ACTUATORS * CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM];
static CarMixerSettingsCurve2SourceOptions curve2_src;
static float curve1[CARMIXERSETTINGS_THROTTLECURVE1_NUMELEM];
static float curve2[CARMIXERSETTINGS_THROTTLECURVE2_NUMELEM];

// Private functions
static void actuator_task(void* parameters);

static float scale_channel(float value, int idx);
static void set_failsafe();

static CarMixerSettingsMixer1TypeOptions get_mixer_type(int idx);
static float throt_curve(const float input, const float *curve,
		uint8_t num_points);
static float collective_curve(const float input, const float *curve,
		uint8_t num_points);

volatile enum actuator_interlock actuator_interlock = ACTUATOR_INTERLOCK_OK;

/**
 * @brief Module initialization
 * @return 0
 */
int32_t ActuatorStart()
{
	// Watchdog must be registered before starting task
	PIOS_WDG_RegisterFlag(PIOS_WDG_ACTUATOR);

	// Start main task
	taskHandle = PIOS_Thread_Create(actuator_task, "Actuator", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	TaskMonitorAdd(TASKINFO_RUNNING_ACTUATOR, taskHandle);

	return 0;
}

/**
 * @brief Module initialization
 * @return 0
 */
int32_t ActuatorInitialize()
{
	// Register for notification of changes to ActuatorSettings
	if (CarActuatorSettingsInitialize()  == -1) {
		return -1;
	}
	CarActuatorSettingsConnectCallbackCtx(UAVObjCbSetFlag, &actuator_settings_updated);

	// Register for notification of changes to MixerSettings
	if (CarMixerSettingsInitialize()  == -1) {
		return -1;
	}
	CarMixerSettingsConnectCallbackCtx(UAVObjCbSetFlag, &mixer_settings_updated);

	// Listen for ActuatorDesired updates (Primary input to this module)
	if (CarActuatorDesiredInitialize()  == -1) {
		return -1;
	}

	queue = PIOS_Queue_Create(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
	CarActuatorDesiredConnectQueue(queue);

	// Primary output of this module
	if (CarActuatorCommandInitialize() == -1) {
		return -1;
	}

	return 0;
}

MODULE_HIPRI_INITCALL(ActuatorInitialize, ActuatorStart);


static float get_curve2_source(CarActuatorDesiredData *desired, CarMixerSettingsCurve2SourceOptions source)
{
	float tmp;

	switch (source) {
	case CARMIXERSETTINGS_CURVE2SOURCE_THROTTLE:
		return desired->Throttle;
		break;
	case CARMIXERSETTINGS_CURVE2SOURCE_ROLL:
		return desired->Roll;
		break;
	case CARMIXERSETTINGS_CURVE2SOURCE_PITCH:
		return desired->Pitch;
		break;
	case CARMIXERSETTINGS_CURVE2SOURCE_YAW:
		return desired->Yaw;
		break;
	case CARMIXERSETTINGS_CURVE2SOURCE_COLLECTIVE:
		CarManualControlCommandCollectiveGet(&tmp);
		return tmp;
		break;
	case CARMIXERSETTINGS_CURVE2SOURCE_ACCESSORY0:
	case CARMIXERSETTINGS_CURVE2SOURCE_ACCESSORY1:
	case CARMIXERSETTINGS_CURVE2SOURCE_ACCESSORY2:
		(void) 0;

		int idx = source - CARMIXERSETTINGS_CURVE2SOURCE_ACCESSORY0;

		if (idx < 0) {
			return 0;
		}

		if (idx >= CARMANUALCONTROLCOMMAND_ACCESSORY_NUMELEM) {
			return 0;
		}

		float accessories[CARMANUALCONTROLCOMMAND_ACCESSORY_NUMELEM];

		CarManualControlCommandAccessoryGet(accessories);

		return accessories[idx];
		break;
	}

	/* Can't get here */
	return 0;
}

static void compute_one_mixer(int mixnum,
		int16_t (*vals)[CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM],
		CarMixerSettingsMixer1TypeOptions type)
{
	types_mixer[mixnum] = type;

	mixnum *= CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM;

	if ((type != CARMIXERSETTINGS_MIXER1TYPE_SERVO) &&
			(type != CARMIXERSETTINGS_MIXER1TYPE_MOTOR)) {
		for (int i = 0; i < CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM; i++) {
			// Ensure unused types are zero-filled
			motor_mixer[mixnum+i] = 0;
		}
	} else {
		for (int i = 0; i < CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM; i++) {
			motor_mixer[mixnum+i] = (*vals)[i] * (1.0f / MIXER_SCALE);
		}
	}
}

/* Here be dragons */
#define compute_one_token_paste(b) compute_one_mixer(b-1, &mixerSettings.Mixer ## b ## Vector, mixerSettings.Mixer ## b ## Type)

static void compute_mixer()
{
	CarMixerSettingsData mixerSettings;

	CarMixerSettingsGet(&mixerSettings);

#if MAX_MIX_ACTUATORS > 0
	compute_one_token_paste(1);
#endif
#if MAX_MIX_ACTUATORS > 1
	compute_one_token_paste(2);
#endif
#if MAX_MIX_ACTUATORS > 2
	compute_one_token_paste(3);
#endif
#if MAX_MIX_ACTUATORS > 3
	compute_one_token_paste(4);
#endif
#if MAX_MIX_ACTUATORS > 4
	compute_one_token_paste(5);
#endif
#if MAX_MIX_ACTUATORS > 5
	compute_one_token_paste(6);
#endif
}

static void fill_desired_vector(
		CarActuatorDesiredData *desired,
		float val1, float val2,
		float (*cmd_vector)[CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM])
{
	(*cmd_vector)[CARMIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE1] = val1;
	(*cmd_vector)[CARMIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE2] = val2;
	(*cmd_vector)[CARMIXERSETTINGS_MIXER1VECTOR_ROLL] = desired->Roll;
	(*cmd_vector)[CARMIXERSETTINGS_MIXER1VECTOR_PITCH] = desired->Pitch;
	(*cmd_vector)[CARMIXERSETTINGS_MIXER1VECTOR_YAW] = desired->Yaw;

	/* Accessory0..Accessory2 are filled in when ManualControl changes
	 * in normalize_input_data
	 */
}

static void post_process_scale_and_commit(float *motor_vect,
		float *desired_vect, float dT,
		bool armed, bool spin_while_armed, bool stabilize_now,
		float *maxpoweradd_bucket)
{
	float min_chan = INFINITY;
	float max_chan = -INFINITY;
	float neg_clip = 0;
	int num_motors = 0;
	CarActuatorCommandData command;

	const float hangtime_leakybucket_timeconstant = 0.3f;

	/* Hangtime maximum power add is now a "leaky bucket" system, ensuring
	 * that the average added power in the long term is the configured value
	 * but allowing higher values briefly.
	 *
	 * The intention is to allow more aggressive maneuvers than hangtime
	 * previously did, while still providing similar safety properties.
	 * A secondary motivation is to prevent tumbling when throttle is
	 * chopped during fast-forward-driving and more than hangtime power
	 * levels are needed for stabilization (because of aerodynamic forces).
	 *
	 * The "maximum" stored is based on recent throttle history-- it decays
	 * with time; at high throttle it corresponds to 300ms of the current
	 * power; at lower throttle it corresponds to 300ms of double the
	 * configured value.
	 */
	float maxpoweradd_softlimit = MAX(
			2 * actuatorSettings.LowPowerStabilizationMaxPowerAdd,
			desired_vect[CARMIXERSETTINGS_MIXER1VECTOR_THROTTLECURVE1])
		* hangtime_leakybucket_timeconstant;

	/* If we're under the limit, add this tick's hangtime power allotment */
	if (*maxpoweradd_bucket < maxpoweradd_softlimit) {
		*maxpoweradd_bucket += actuatorSettings.LowPowerStabilizationMaxPowerAdd * dT;
	} else {
		/* Otherwise, decay towards the current limit on a 300ms
		 * time constant.
		 */
		float alpha = dT / (dT + hangtime_leakybucket_timeconstant);

		*maxpoweradd_bucket = alpha * maxpoweradd_softlimit +
			(1-alpha) * (*maxpoweradd_bucket);
	}

	/* The maximum power add is what would spend the current allotment in
	 * 300ms.  In other words, in the absence of recent high-throttle,
	 * start from double the hangtime configured percentage and decay on
	 * a 300ms time constant IF IT IS ACTUALLY USED.
	 *
	 * This is separate from the above decay, so we could actually be
	 * decaying twice as fast if both are in play.
	 */
	float maxpoweradd = (*maxpoweradd_bucket) / hangtime_leakybucket_timeconstant;

	for (int ct = 0; ct < MAX_MIX_ACTUATORS; ct++) {
		switch (get_mixer_type(ct)) {
			case CARMIXERSETTINGS_MIXER1TYPE_DISABLED:
				// Set to minimum if disabled.
				// This is not the same as saying
				// PWM pulse = 0 us
				motor_vect[ct] = -1;
				break;

			case CARMIXERSETTINGS_MIXER1TYPE_SERVO:
				break;

			case CARMIXERSETTINGS_MIXER1TYPE_MOTOR:
				min_chan = fminf(min_chan, motor_vect[ct]);
				max_chan = fmaxf(max_chan, motor_vect[ct]);

				if (motor_vect[ct] < 0.0f) {
					neg_clip += motor_vect[ct];
				}

				num_motors++;
				break;
			case CARMIXERSETTINGS_MIXER1TYPE_CAMERAPITCH:
				if (CameraDesiredHandle()) {
					CameraDesiredPitchGet(
							&motor_vect[ct]);
				} else {
					motor_vect[ct] = -1;
				}
				break;
			case CARMIXERSETTINGS_MIXER1TYPE_CAMERAROLL:
				if (CameraDesiredHandle()) {
					CameraDesiredRollGet(
							&motor_vect[ct]);
				} else {
					motor_vect[ct] = -1;
				}
				break;
			case CARMIXERSETTINGS_MIXER1TYPE_CAMERAYAW:
				if (CameraDesiredHandle()) {
					CameraDesiredRollGet(
							&motor_vect[ct]);
				} else {
					motor_vect[ct] = -1;
				}
				break;
			default:
				set_failsafe();
				PIOS_Assert(0);
		}
	}

	float gain = 1.0f;
	float offset = 0.0f;

	/* This is a little dubious.  Scale down command ranges to
	 * fit.  It may cause some cross-axis coupling, though
	 * generally less than if we were to actually let it clip.
	 */
	if ((max_chan - min_chan) > 1.0f) {
		gain = 1.0f / (max_chan - min_chan);

		max_chan *= gain;
		min_chan *= gain;
	}

	/* Sacrifice throttle because of clipping */
	if (max_chan > 1.0f) {
		offset = 1.0f - max_chan;
	} else if (min_chan < 0.0f) {
		/* Low-side clip management-- how much power are we
		 * willing to add??? */

		neg_clip /= num_motors;

		/* neg_clip is now the amount of throttle "already added." by
		 * clipping...
		 *
		 * Find the "highest possible value" of offset.
		 * if neg_clip is -15%, and maxpoweradd is 10%, we need to add
		 * -5% to all motors.
		 * if neg_clip is 5%, and maxpoweradd is 10%, we can add up to
		 * 5% to all motors to further fix clipping.
		 */
		offset = neg_clip + maxpoweradd;

		/* Add the lesser of--
		 * A) the amount the lowest channel is out of range.
		 * B) the above calculated offset.
		 */
		offset = MIN(-min_chan, offset);

		/* The amount actually added is the above offset, plus the
		 * amount that came from negative clipping.  (It's negative
		 * though, so subtract instead of add).  Spend this from
		 * the leaky bucket. 
		 */
		*maxpoweradd_bucket -= (offset - neg_clip) * dT;
	}

	for (int ct = 0; ct < MAX_MIX_ACTUATORS; ct++) {
		// Motors have additional protection for when to be on
		if (get_mixer_type(ct) == CARMIXERSETTINGS_MIXER1TYPE_MOTOR) {
			if (!armed) {
				motor_vect[ct] = -1;  //force min throttle
			} else if (!stabilize_now) {
				if (!spin_while_armed) {
					motor_vect[ct] = -1;
				} else {
					motor_vect[ct] = 0;
				}
			} else {
				motor_vect[ct] = motor_vect[ct] * gain + offset;

				if (motor_vect[ct] > 0) {
					// Apply curve fitting, mapping the input to the propeller output.
					motor_vect[ct] = powapprox(motor_vect[ct], actuatorSettings.MotorInputOutputCurveFit);
				} else {
					motor_vect[ct] = 0;
				}
			}
		}

		command.Channel[ct] = scale_channel(motor_vect[ct], ct);
	}

	// Store update time
	command.UpdateTime = 1000.0f*dT;

	CarActuatorCommandMaxUpdateTimeGet(&command.MaxUpdateTime);

	if (command.UpdateTime > command.MaxUpdateTime)
		command.MaxUpdateTime = 1000.0f*dT;

	// Update output object
	if (!CarActuatorCommandReadOnly()) {
		CarActuatorCommandSet(&command);
	} else {
		// it's read only during servo configuration--
		// so GCS takes precedence.
		CarActuatorCommandGet(&command);
	}

	for (int n = 0; n < MAX_MIX_ACTUATORS; ++n) {
		PIOS_Servo_Set(n, command.Channel[n]);
	}

	PIOS_Servo_Update();
}

static void normalize_input_data(uint32_t this_systime,
		float (*desired_vect)[CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM],
		bool *armed, bool *spin_while_armed, bool *stabilize_now)
{
	static float manual_throt = -1;
	float throttle_val = -1;
	CarActuatorDesiredData desired;

	static DrivingStatusData drivingStatus;

	CarActuatorDesiredGet(&desired);

	if (driving_status_updated) {
		DrivingStatusGet(&drivingStatus);
		driving_status_updated = false;
	}

	if (manual_control_cmd_updated) {
		// just pull out the throttle_val... and accessory0-2 and
		// fill direct into the vect
		CarManualControlCommandThrottleGet(&manual_throt);
		manual_control_cmd_updated = false;
		CarManualControlCommandAccessoryGet(
			&(*desired_vect)[CARMIXERSETTINGS_MIXER1VECTOR_ACCESSORY0]);
	}

	*armed = drivingStatus.Armed == DRIVINGSTATUS_ARMED_ARMED;
	*spin_while_armed = actuatorSettings.MotorsSpinWhileArmed == CARACTUATORSETTINGS_MOTORSSPINWHILEARMED_TRUE;

	throttle_val = desired.Throttle;

	if (!*armed) {
		throttle_val = -1;
	}

	*stabilize_now = throttle_val > 0.0f;

	float val1 = throt_curve(throttle_val, curve1,
			CARMIXERSETTINGS_THROTTLECURVE1_NUMELEM);

	//The source for the secondary curve is selectable
	float val2 = collective_curve(
			get_curve2_source(&desired, curve2_src),
			curve2, CARMIXERSETTINGS_THROTTLECURVE2_NUMELEM);

	fill_desired_vector(&desired, val1, val2, desired_vect);
}

/**
 * @brief Main Actuator module task
 *
 * Universal matrix based mixer for VTOL, helis and fixed wing.
 * Converts desired roll,pitch,yaw and throttle to servo/ESC outputs.
 *
 * Because of how the Throttle ranges from 0 to 1, the motors should too!
 *
 * Note this code depends on the UAVObjects for the mixers being all being the same
 * and in sequence. If you change the object definition, make sure you check the code!
 *
 * @return -1 if error, 0 if success
 */
static void actuator_task(void* parameters)
{
	// Connect update callbacks
	DrivingStatusConnectCallbackCtx(UAVObjCbSetFlag, &driving_status_updated);
	CarManualControlCommandConnectCallbackCtx(UAVObjCbSetFlag, &manual_control_cmd_updated);

	// Ensure the initial state of actuators is safe.
	actuator_settings_updated = false;
	CarActuatorSettingsGet(&actuatorSettings);

	PIOS_Servo_SetMode(actuatorSettings.TimerUpdateFreq,
			CARACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM,
			actuatorSettings.ChannelMax,
			actuatorSettings.ChannelMin);
	set_failsafe();

	/* This is out here because not everything may change each time */
	uint32_t last_systime = PIOS_Thread_Systime();
	float desired_vect[CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM] = { 0 };
	float dT = 0.0f;
	
	float maxpoweradd_bucket = 0.0f;

	// Main task loop
	while (1) {
		/* If settings objects have changed, update our internal
		 * state appropriately.
		 */
		if (actuator_settings_updated) {
			actuator_settings_updated = false;
			CarActuatorSettingsGet(&actuatorSettings);

			PIOS_Servo_SetMode(actuatorSettings.TimerUpdateFreq,
					CARACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM,
					actuatorSettings.ChannelMax,
					actuatorSettings.ChannelMin);
		}

		if (mixer_settings_updated) {
			mixer_settings_updated = false;

			compute_mixer();
			// XXX compute_inverse_mixer();

			CarMixerSettingsThrottleCurve1Get(curve1);
			CarMixerSettingsThrottleCurve2Get(curve2);
			CarMixerSettingsCurve2SourceGet(&curve2_src);
		}

		PIOS_WDG_UpdateFlag(PIOS_WDG_ACTUATOR);

		UAVObjEvent ev;

		// Wait until the ActuatorDesired object is updated
		if (!PIOS_Queue_Receive(queue, &ev, FAILSAFE_TIMEOUT_MS)) {
			// If we hit a timeout, set the actuator failsafe and
			// try again.
			set_failsafe();
			continue;
		}

		uint32_t this_systime = PIOS_Thread_Systime();

		/* Check how long since last update; this is stored into the
		 * UAVO to allow analysis of actuation jitter.
		 */
		if (this_systime > last_systime) {
			dT = (this_systime - last_systime) / 1000.0f;
			/* (Otherwise, the timer has wrapped [rare] and we should
			 * just reuse dT)
			 */
		}

		last_systime = this_systime;

		if (actuator_interlock != ACTUATOR_INTERLOCK_OK) {
			/* Chosen because: 50Hz does 4-6 updates in 100ms */
			uint32_t exp_time = this_systime + 100;

			while (actuator_interlock != ACTUATOR_INTERLOCK_OK) {
				/* Simple state machine.  If someone has asked us to
				 * stop, set actuator failsafe for a short while.
				 * Then, set the flag to STOPPED.
				 *
				 * Setting to STOPPED isn't atomic, so we rely on
				 * anyone who has stopped us to waitfor STOPPED
				 * before putting us back to OK.
				 */
				if (actuator_interlock == ACTUATOR_INTERLOCK_STOPREQUEST) {
					set_failsafe();

					this_systime = PIOS_Thread_Systime();

					if ((exp_time - this_systime) > 100) {
						actuator_interlock = ACTUATOR_INTERLOCK_STOPPED;
					}
				}

				PIOS_Thread_Sleep(3);
				PIOS_WDG_UpdateFlag(PIOS_WDG_ACTUATOR);
			}

			PIOS_Servo_SetMode(actuatorSettings.TimerUpdateFreq,
					CARACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM,
					actuatorSettings.ChannelMax,
					actuatorSettings.ChannelMin);
			continue;
		}


		float motor_vect[MAX_MIX_ACTUATORS];

		bool armed, spin_while_armed, stabilize_now;

		/* Receive manual control and desired UAV objects.  Perform
		 * arming / hangtime checks; form a vector with desired
		 * axis actions.
		 */
		normalize_input_data(this_systime, &desired_vect, &armed,
				&spin_while_armed, &stabilize_now);

		/* Multiply the actuators x desired matrix by the
		 * desired x 1 column vector. */
		matrix_mul_check(motor_mixer, desired_vect, motor_vect,
				MAX_MIX_ACTUATORS,
				CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM,
				1);

		/* Perform clipping adjustments on the outputs, along with
		 * state-related corrections (spin while armed, disarmed, etc).
		 *
		 * Program the actual values to the timer subsystem.
		 */
		post_process_scale_and_commit(motor_vect, desired_vect,
				dT, armed, spin_while_armed, stabilize_now,
				&maxpoweradd_bucket);

		/* If we got this far, everything is OK. */
		AlarmsClear(SYSTEMALARMS_ALARM_ACTUATOR);
	}
}

/**
 * Interpolate a throttle curve
 *
 * throttle curve assumes input is [0,1]
 * this means that the throttle channel neutral value is nearly the same as its min value
 * this is convenient for throttle, since the neutral value is used as a failsafe and would thus shut off the motor
 *
 * @param input the input value, in [0,1]
 * @param curve the array of points in the curve
 * @param num_points the number of points in the curve
 * @return the output value, in [0,1]
 */
static float throt_curve(float const input, float const * curve, uint8_t num_points)
{
	return linear_interpolate(input, curve, num_points, 0.0f, 1.0f);
}

/**
 * Interpolate a collective curve
 *
 * we need to accept input in [-1,1] so that the neutral point may be set arbitrarily within the typical channel input range, which is [-1,1]
 *
 * @param input The input value, in [-1,1]
 * @param curve Array of points in the curve
 * @param num_points Number of points in the curve
 * @return the output value, in [-1,1]
 */
static float collective_curve(float const input, float const * curve, uint8_t num_points)
{
	return linear_interpolate(input, curve, num_points, -1.0f, 1.0f);
}

/**
 * Convert channel from -1/+1 to servo pulse duration in microseconds
 */
static float scale_channel(float value, int idx)
{
	float max = actuatorSettings.ChannelMax[idx];
	float min = actuatorSettings.ChannelMin[idx];
	float neutral = actuatorSettings.ChannelNeutral[idx];

	float valueScaled;
	// Scale
	if (value >= 0.0f) {
		valueScaled = value*(max-neutral) + neutral;
	} else {
		valueScaled = value*(neutral-min) + neutral;
	}

	if (max>min) {
		if (valueScaled > max) valueScaled = max;
		if (valueScaled < min) valueScaled = min;
	} else {
		if (valueScaled < max) valueScaled = max;
		if (valueScaled > min) valueScaled = min;
	}

	return valueScaled;
}

static CarMixerSettingsMixer1TypeOptions get_mixer_type(int idx)
{
	switch (idx) {
	case 0:
		return mixerSettings.Mixer1Type;
		break;
	case 1:
		return mixerSettings.Mixer2Type;
		break;
	case 2:
		return mixerSettings.Mixer3Type;
		break;
	case 3:
		return mixerSettings.Mixer4Type;
		break;
	case 4:
		return mixerSettings.Mixer5Type;
		break;
	case 5:
		return mixerSettings.Mixer6Type;
		break;
	default:
		// We can never get here unless there are mixer channels not handled in the above. Fail out.
		PIOS_Assert(0);
	}

	/* Can't get here. Just make compiler happy */
	return mixerSettings.Mixer1Type;
}

static float channel_failsafe_value(int idx)
{
	switch (get_mixer_type(idx)) {
	case CARMIXERSETTINGS_MIXER1TYPE_MOTOR:
		return actuatorSettings.ChannelMin[idx];
	case CARMIXERSETTINGS_MIXER1TYPE_SERVO:
		return actuatorSettings.ChannelNeutral[idx];
	case CARMIXERSETTINGS_MIXER1TYPE_DISABLED:
		return -1;
	default:
		// TODO: is this actually right/safe?
		return 0;
	}

	/* Can't get here. */
	return -1;
}

/**
 * Set actuator output to the neutral values (failsafe)
 */
static void set_failsafe()
{
	float Channel[CARACTUATORCOMMAND_CHANNEL_NUMELEM] = {0};

	// Set alarm
	AlarmsSet(SYSTEMALARMS_ALARM_ACTUATOR, SYSTEMALARMS_ALARM_CRITICAL);

	// Update servo outputs
	for (int n = 0; n < MAX_MIX_ACTUATORS; ++n) {
		float fs_val = channel_failsafe_value(n);

		Channel[n] = fs_val;

		PIOS_Servo_Set(n, fs_val);
	}

	PIOS_Servo_Update();

	// Update output object's parts that we changed
	CarActuatorCommandChannelSet(Channel);
}

/**
 * @}
 * @}
 */
