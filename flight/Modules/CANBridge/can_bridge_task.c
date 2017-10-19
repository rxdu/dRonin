#include "openpilot.h"
#include "pios_queue.h"
#include "pios_thread.h"
    
#include "jlink_rtt.h"

void CANBridge_UpdateComm();

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1200
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGH
#define UPDATE_PERIOD_MS 500

#define CAN_RX_TIMEOUT_MS 10

// Private variables
// static struct pios_queue *queue;
static struct pios_thread *taskHandle;

// Private functions
static void canBridgeTask(void *parameters);

/**
 * Module starting
 */
int32_t CANBridgeStart()
{
	// Watchdog must be registered before starting task
	//PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	/* Delay system */
	PIOS_DELAY_Init();

	// Start main task
	taskHandle = PIOS_Thread_Create(canBridgeTask, "CANBridge", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	//TaskMonitorAdd(TASKINFO_RUNNING_MANUALCONTROL, taskHandle);

	return 0;
}

/**
 * Module initialization
 */
int32_t CANBridgeInitialize()
{

	// queue = PIOS_CAN_RegisterMessageQueue(pios_can_id, PIOS_CAN_GIMBAL);

	return 0;
}

MODULE_HIPRI_INITCALL(CANBridgeInitialize, CANBridgeStart);

/**
 * Module task
 */
static void canBridgeTask(void *parameters)
{
	PIOS_DELAY_WaitmS(1000);

    while (1)
	{
		CANBridge_UpdateComm();
		// JLinkRTTPrintf(0, "RTC System Time: %ld \n", PIOS_RTC_GetSystemTime());
        PIOS_DELAY_WaitmS(UPDATE_PERIOD_MS);
    }
}
