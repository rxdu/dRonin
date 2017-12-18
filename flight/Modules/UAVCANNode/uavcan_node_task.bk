#include "openpilot.h"
#include "pios_queue.h"
#include "pios_thread.h"
#include "uavcan_interface.h"

// UAVOs
#include "gyros.h"
#include "accels.h"
#include "magnetometer.h"
    
#include "jlink_rtt.h"

bool UAVCANNode_InitComm();
void UAVCANNode_SpinNode(int32_t timeout);

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1024*8
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGH

#define UAVCAN_SPIN_TIMEOUT_MS 2
#define UPDATE_PERIOD_MS 3

// Private variables
static struct pios_thread *taskHandle;

// Private functions
static void uavcanNodeTask(void *parameters);

/**
 * Module starting
 */
int32_t UAVCANNodeStart()
{
	// Watchdog must be registered before starting task
	// PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	/* Delay system */
	PIOS_DELAY_Init();
	UAVCANNode_InitComm();

	// Start main task
	taskHandle = PIOS_Thread_Create(uavcanNodeTask, "UAVCANNode", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	//TaskMonitorAdd(TASKINFO_RUNNING_MANUALCONTROL, taskHandle);

	return 0;
}

/**
 * Module initialization
 */
int32_t UAVCANNodeInitialize()
{
	// canCmdQueue = PIOS_Queue_Create(CAN_CMD_QUEUE_LEN, sizeof(struct CANCmdData));
	// if (canCmdQueue == NULL)
	// 	return -1;

	return 0;
}

MODULE_HIPRI_INITCALL(UAVCANNodeInitialize, UAVCANNodeStart);

/**
 * Module task
 */
static void uavcanNodeTask(void *parameters)
{	
    while (1)
	{
		// static uint32_t time_label = 0;
		// uint32_t prev_time_label = time_label;
		// time_label = PIOS_DELAY_GetuS();
		// uint32_t error = time_label - prev_time_label;
		//if(error > 5200)
		// JLinkRTTPrintf(0, "uavcan node update period: %ld\n",error);
		UAVCANNode_SpinNode(UAVCAN_SPIN_TIMEOUT_MS);
		PIOS_DELAY_WaitmS(UPDATE_PERIOD_MS);
    }
}
