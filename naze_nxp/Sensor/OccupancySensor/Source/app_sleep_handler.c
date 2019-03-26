/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_sleep_handler.c
 *
 * DESCRIPTION:        ZLO Demo : Manages sleep configuration.
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2016. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <string.h>
#include "dbg.h"
#include "ZTimer.h"
#include "app_main.h"
#include "pwrm.h"
#include "AppHardwareApi.h"
#include "pdum_gen.h"
#include "pdm.h"
#include "pdum_gen.h"
#include "app_common.h"
#include "PDM_IDs.h"
#include "zcl_options.h"
#include "app_zbp_utilities.h"
#include "zcl_common.h"
#include "app_sleep_handler.h"
#include "app_zcl_tick_handler.h"
#include "app_zcl_sensor_task.h"
#include "app_zlo_sensor_node.h"
#include "App_OccupancySensor.h"
#include "app_occupancy_buttons.h"
#include "app_blink_led.h"
#ifdef APP_NTAG
#include "app_ntag.h"
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_SLEEP_HANDLER
    #define TRACE_SLEEP_HANDLER   FALSE
#else
    #define TRACE_SLEEP_HANDLER   TRUE
#endif

#if ZLO_MAX_REPORT_INTERVAL == 0
    #define MAXIMUM_TIME_TO_SLEEP APP_OCCUPANCY_SENSOR_OCCUPIED_TO_UNOCCUPIED_DELAY + 1
#else
    #define MAXIMUM_TIME_TO_SLEEP ZLO_SYSTEM_MAX_REPORT_INTERVAL
#endif
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE uint8 u8NumberOfTimersTaskTimers(void);
PRIVATE void vScheduleSleep(bool_t bDeepSleep);
PRIVATE void vWakeCallBack(void);
PRIVATE void vStopNonSleepPreventingTimers(void);
PRIVATE void vStartNonSleepPreventingTimers(void);
PRIVATE uint8 u8NumberOfNonSleepPreventingTimers(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE pwrm_tsWakeTimerEvent    sWake;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME:        vAttemptToSleep
 *
 * DESCRIPTION: Checks to see if any software timers are running that may
 * prevent us from going to sleep. If there is none, if wake timer 0 is
 * running, schedule none deep sleep, if there is schedule deep sleep
 * which is checked if its enabled in vScheduleSleep.
 *
 ****************************************************************************/
PUBLIC void vAttemptToSleep(void)
{
	DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: Activity Count = %d", PWRM_u16GetActivityCount());
	DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: Task Timers = %d", u8NumberOfTimersTaskTimers());

    /* Only enter here if the activity count is equal to the number of non sleep preventing timers (in other words, the activity count
     * will become zero when we stop them) */
    if ((PWRM_u16GetActivityCount() == u8NumberOfNonSleepPreventingTimers()) &&
        (0 == u8NumberOfTimersTaskTimers()))
    {
        /* Stop any background timers that are non sleep preventing*/
        vStopNonSleepPreventingTimers();

        /* Check if Wake timer 0 is running.*/
        if (u8AHI_WakeTimerStatus() & E_AHI_WAKE_TIMER_MASK_0)
        {
            vScheduleSleep(FALSE);
        }
        else
        {
            vScheduleSleep(TRUE);
        }
    }
}


/****************************************************************************/
/***        Local Function                                                     ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME:        u8NumberOfTimersTaskTimers
 *
 * DESCRIPTION: Checks to see if any timers are running that shouldn't be
 * interrupted by sleeping.
 *
 ****************************************************************************/
PRIVATE uint8 u8NumberOfTimersTaskTimers(void)
{
    uint8 u8NumberOfRunningTimers = 0;

    if (ZTIMER_eGetState(u8TimerButtonScan) == E_ZTIMER_STATE_RUNNING)
    {
		DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: ButtonScaning");
		u8NumberOfRunningTimers++;
    }
//	u8NumberOfRunningTimers++;	//Ricky Test To No Sleep

    if (ZTIMER_eGetState(u8TimerPoll) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: u8TimerPoll");
        u8NumberOfRunningTimers++;
    }

/*
	if (ZTIMER_eGetState(u8TimerBdbRejoin) == E_ZTIMER_STATE_RUNNING)
	{
		DBG_vPrintf(TRACE_SLEEP_HANDLER, "\n APP Sleep Handler: u8TimerBdbRejoin ");
		u8NumberOfRunningTimers++;
	}
*/

#ifdef APP_NTAG
    if (ZTIMER_eGetState(u8TimerNtag) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: APP_TimerNtag");
        u8NumberOfRunningTimers++;
    }
#endif

	DBG_vPrintf(TRACE_SLEEP_HANDLER, "\n APP Sleep Handler: NumberOfRunningTimers: %d ",u8NumberOfRunningTimers);

    return u8NumberOfRunningTimers;
}


/****************************************************************************
 *
 * NAME:        vScheduleSleep
 *
 * DESCRIPTION: If we have deep sleep enabled and we attempting to deep sleep
 * then re-initialise the power manager for deep sleep
 *
 ****************************************************************************/
PRIVATE void vScheduleSleep(bool_t bDeepSleep)
{
#ifdef DEEP_SLEEP_ENABLE
    if (bDeepSleep)
    {
        PWRM_vInit(E_AHI_SLEEP_DEEP);
        DBG_vPrintf(TRACE_SLEEP_HANDLER , "\nAPP Sleep Handler: Deep Sleep");
    }
    else
    {
        PWRM_eScheduleActivity(&sWake, (MAXIMUM_TIME_TO_SLEEP - u32GetNumberOfZCLTicksSinceReport())*APP_TICKS_PER_SECOND , vWakeCallBack);
        DBG_vPrintf(TRACE_SLEEP_HANDLER , "\nAPP Sleep Handler: Osc on, sleeping for %d ticks", (MAXIMUM_TIME_TO_SLEEP - u32GetNumberOfZCLTicksSinceReport())*APP_TICKS_PER_SECOND);
    }
#else
    PWRM_eScheduleActivity(&sWake, (MAXIMUM_TIME_TO_SLEEP - u32GetNumberOfZCLTicksSinceReport())*APP_TICKS_PER_SECOND , vWakeCallBack);
    DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: Osc on, sleeping for %d ticks", (MAXIMUM_TIME_TO_SLEEP - u32GetNumberOfZCLTicksSinceReport())*APP_TICKS_PER_SECOND);
#endif
}

/****************************************************************************
 *
 * NAME:        vStopNonSleepPreventingTimers
 *
 * DESCRIPTION: The timers in this function should not stop us from sleep.
 * Stop the timers to reduce the activity count which will prevent sleep.
 *
 ****************************************************************************/
PRIVATE void vStopNonSleepPreventingTimers()
{
	DBG_vPrintf(TRACE_SLEEP_HANDLER, "\n vStopNonSleepPreventingTimers \n");
    if (ZTIMER_eGetState(u8TimerTick) != E_ZTIMER_STATE_STOPPED)
        ZTIMER_eStop(u8TimerTick);

}

/****************************************************************************
 *
 * NAME:        vStartNonSleepPreventingTimers
 *
 * DESCRIPTION: Start the timers that wont stop us in vAttemptToSleep
 *
 ****************************************************************************/
PRIVATE void vStartNonSleepPreventingTimers(void)
{
    if (ZTIMER_eGetState(u8TimerTick) != E_ZTIMER_STATE_RUNNING)
    {
		DBG_vPrintf(TRACE_SLEEP_HANDLER, "\n vStartNonSleepPreventingTimers \n");
        ZTIMER_eStart(u8TimerTick, ZCL_TICK_TIME);
	}
}

/****************************************************************************
 *
 * NAME:        u8NumberOfNonSleepPreventingTimers
 *
 * DESCRIPTION: Returns the number of timers that are running that we are
 * prepared to stop before going to sleep.
 *
 ****************************************************************************/
PRIVATE uint8 u8NumberOfNonSleepPreventingTimers(void)
{
    uint8 u8NumberOfRunningTimers = 0;

    if (ZTIMER_eGetState(u8TimerTick) == E_ZTIMER_STATE_RUNNING)
    {
		DBG_vPrintf(TRACE_SLEEP_HANDLER, "\n u8NumberOfNonSleepPreventingTimers");
        u8NumberOfRunningTimers++;
    }

    return u8NumberOfRunningTimers;
}

/****************************************************************************
 *
 * NAME: vWakeCallBack
 *
 * DESCRIPTION:
 * Wake up call back called upon wake up by the schedule activity event.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vWakeCallBack(void)
{
    DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: vWakeCallBack\n");

#ifndef DEEP_SLEEP_ENABLE
    vUpdateZCLTickSinceSleep();
#endif

//    APP_vSetLED( LED1, sSensor.sOccupancySensingServerCluster.u8Occupancy);

    /*Start the u8TimerTick to continue the ZCL tasks */
    vStartNonSleepPreventingTimers();
}

PUBLIC bool u8NumerOfActiveTaskTimer(void)	//Ricky
{
	if ( u8NumberOfTimersTaskTimers())
		return TRUE;
	return FALSE;
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
