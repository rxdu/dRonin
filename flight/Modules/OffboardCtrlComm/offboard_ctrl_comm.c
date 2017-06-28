#include "openpilot.h"
#include <eventdispatcher.h>
#include "misc_math.h"
#include "physical_constants.h"
#include "pios_thread.h"
#include "pios_can.h"

#include "modulesettings.h"
#include "misc_math.h"
#include "offboard_ctrl_types.h"

//
// Configuration
//
#define SAMPLE_PERIOD_MS     10
#define LOAD_DELAY           7000

// Private types
enum {ROLL,PITCH,YAW,MAX_AXES};

typedef struct
{
    bool initialized;
} OffboardCtrlCommStatus;

// Private variables
static OffboardCtrlCommStatus *comm_status;

// Private functions

// Private variables

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t OffboardCtrlCommInitialize(void)
{
	bool module_enabled = false;

	module_enabled = true;

	if (module_enabled) {
		// allocate and initialize the static data storage only if module is enabled
		comm_status = (OffboardCtrlCommStatus *) PIOS_malloc(sizeof(OffboardCtrlCommStatus));
		if (comm_status == NULL) {
			module_enabled = false;
			return -1;
		}

		// make sure that all inputs[] are zeroed
		memset(comm_status, 0, sizeof(OffboardCtrlCommStatus));

		return 0;
	}

	return -1;
}

/* stub: module has no module thread */
int32_t OffboardCtrlCommStart(void)
{
	// if (csd == NULL) {
	// 	return -1;
	// }
  //
	// // Schedule periodic task to process attitude
	// UAVObjEvent ev = {
	// 	.obj = AttitudeActualHandle(),
	// 	.instId = 0,
	// 	.event = 0,
	// };
	// EventPeriodicCallbackCreate(&ev, attitudeUpdated, SAMPLE_PERIOD_MS);

	return 0;
}

MODULE_INITCALL(OffboardCtrlCommInitialize, OffboardCtrlCommStart)

/**
 * Periodic callback that processes changes in the attitude
 * and recalculates the desied gimbal angle.
 */
// static void attitudeUpdated(UAVObjEvent* ev, void *ctx, void *obj, int len)
// {
	// (void) ev; (void) ctx; (void) obj; (void) len;
	// if (ev->obj != AttitudeActualHandle())
	// 	return;
  //
	// float accessories[MANUALCONTROLCOMMAND_ACCESSORY_NUMELEM];
  //
	// ManualControlCommandAccessoryGet(accessories);
  //
	// CameraStabSettingsData *settings = &csd->settings;
  //
	// // Check how long since last update, time delta between calls in ms
	// uint32_t thisSysTime = PIOS_Thread_Systime();
	// float dT_ms = thisSysTime - csd->lastSysTime;
	// csd->lastSysTime = thisSysTime;
  //
	// if (dT_ms <= 0)
	// 	return;
  //
	// float attitude;
	// float output;

// 	for (uint8_t i = 0; i < MAX_AXES; i++) {
//
// 		// Get the current attitude from the selected axis
// 		switch (i) {
// 		case ROLL:
// 			AttitudeActualRollGet(&attitude);
// 			break;
// 		case PITCH:
// 			AttitudeActualPitchGet(&attitude);
// 			break;
// 		case YAW:
// 			AttitudeActualYawGet(&attitude);
// 			break;
// 		}
// 		float rt_ms = (float)settings->AttitudeFilter;
// 		csd->attitude_filtered[i] = (rt_ms / (rt_ms + dT_ms)) * csd->attitude_filtered[i] + (dT_ms / (rt_ms + dT_ms)) * attitude;
// 		attitude = csd->attitude_filtered[i];
//
// 		// Compute new smoothed setpoint
// 		if (settings->Input[i] != CAMERASTABSETTINGS_INPUT_NONE && settings->Input[i] != CAMERASTABSETTINGS_INPUT_POI) {
// 			int idx = settings->Input[i] - CAMERASTABSETTINGS_INPUT_ACCESSORY0;
//
// 			if ((idx >= 0) && (idx < MANUALCONTROLCOMMAND_ACCESSORY_NUMELEM)) {
// 				float input;
// 				float input_rate;
// 				rt_ms = (float) settings->InputFilter;
// 				switch (settings->StabilizationMode[i]) {
// 				case CAMERASTABSETTINGS_STABILIZATIONMODE_ATTITUDE:
// 					input = accessories[idx] * settings->InputRange[i];
// 					csd->inputs[i] = (rt_ms / (rt_ms + dT_ms)) * csd->inputs[i] + (dT_ms / (rt_ms + dT_ms)) * input;
// 					break;
// 				case CAMERASTABSETTINGS_STABILIZATIONMODE_AXISLOCK:
// 					input_rate = accessories[idx] * settings->InputRate[i];
// 					if (fabsf(input_rate) > settings->MaxAxisLockRate)
// 						csd->inputs[i] = bound_sym(csd->inputs[i] + input_rate * dT_ms / 1000.0f, settings->InputRange[i]);
// 					break;
// 				default:
// 					input = 0;
// 				}
// 			}
// 			switch(i) {
// 			case PITCH:
// 				CameraDesiredDeclinationSet(&csd->inputs[i]);
// 				break;
// 			case YAW:
// 				CameraDesiredBearingSet(&csd->inputs[i]);
// 				break;
// 			default:
// 				break;
// 			}
// 		}
// #if defined(CAMERASTAB_POI_MODE)
// 		else if (settings->Input[i] == CAMERASTABSETTINGS_INPUT_POI) {
// 			// Process any updates of the tablet location if it wants to
// 			// be the POI
// 			tablet_info_process();
//
// 			PositionActualData positionActual;
// 			PositionActualGet(&positionActual);
// 			PoiLocationData poi;
// 			PoiLocationGet(&poi);
//
// 			float dLoc[3];
//
// 			dLoc[0] = poi.North - positionActual.North;
// 			dLoc[1] = poi.East - positionActual.East;
// 			dLoc[2] = poi.Down - positionActual.Down;
//
// 			// Compute the pitch and yaw to the POI location, assuming UAVO is level facing north
// 			float distance = sqrtf(powf(dLoc[0], 2) + powf(dLoc[1], 2));
// 			float pitch = atan2f(-dLoc[2], distance) * RAD2DEG;
// 			float yaw = atan2f(dLoc[1], dLoc[0]) * RAD2DEG;
// 			if (yaw < 0.0f)
// 				yaw += 360.0f;
//
// 			// Only try and track objects more than 2 m away
// 			if (distance > 2) {
// 				switch (i) {
// 				case CAMERASTABSETTINGS_INPUT_ROLL:
// 					// Does not make sense to use position to control yaw
// 					break;
// 				case CAMERASTABSETTINGS_INPUT_PITCH:
// 					// Store the absolute declination relative to UAV
// 					CameraDesiredDeclinationSet(&pitch);
// 					csd->inputs[CAMERASTABSETTINGS_INPUT_PITCH] = -pitch;
// 					break;
// 				case CAMERASTABSETTINGS_INPUT_YAW:
// 					CameraDesiredBearingSet(&yaw);
// 					csd->inputs[CAMERASTABSETTINGS_INPUT_YAW] = yaw;
// 					break;
// 				}
// 			}
// 		}
// #endif /* CAMERASTAB_POI_MODE */
//
// 		// Add Servo FeedForward
// 		applyFF(i, dT_ms, &attitude, settings);
//
// 		Set output channels
// 		output = bound_sym((attitude + csd->inputs[i]) / settings->OutputRange[i], 1.0f);
// 		if (thisSysTime > LOAD_DELAY) {
// 			switch (i) {
// 			case ROLL:
// 				CameraDesiredRollSet(&output);
// 				break;
// 			case PITCH:
// 				CameraDesiredPitchSet(&output);
// 				break;
// 			case YAW:
// 				CameraDesiredYawSet(&output);
// 				break;
// 			}
// 		}
// 	}

	// Send a message over CAN, if include
	//gimbal_can_message();
// }

#if defined(PIOS_INCLUDE_CAN)
extern uintptr_t pios_can_id;
#endif /* PIOS_INCLUDE_CAN */

/**
 * Relay control messages to an external gimbal
 */
// static void gimbal_can_message()
// {
// #if defined(PIOS_INCLUDE_CAN)
// 	struct pios_can_gimbal_message bgc_message = {
// 		.fc_roll = attitude.Roll,
// 		.fc_pitch = attitude.Pitch,
// 		.fc_yaw = attitude.Yaw,
// 		.setpoint_roll = 0,
// 		.setpoint_pitch = cameraDesired.Declination,
// 		.setpoint_yaw = cameraDesired.Bearing
// 	};
//
// 	PIOS_CAN_TxData(pios_can_id, PIOS_CAN_GIMBAL, (uint8_t *) &bgc_message);
// #endif /* PIOS_INCLUDE_CAN */
// }
