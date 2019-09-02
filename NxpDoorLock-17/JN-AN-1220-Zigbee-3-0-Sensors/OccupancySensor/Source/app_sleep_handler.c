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
#include "PDM_IDs.h"
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
#include "app_nwk_event_handler.h"
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
    #define MAXIMUM_TIME_TO_SLEEP ZLO_MAX_REPORT_INTERVAL
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
extern uint8 WhetherAskCacheAllCnt;
extern uint8 u8JoinNetworkFlowPath;
extern bool bPeriodicEventWakeup;
extern uint32 u32SleepTimeTicks;
extern uint8 u8_WakeUpSetBit;//jabin
extern uint8 u8WakeTimerErroCounter;//jabin
extern PUBLIC uint8 u8TimerCheckGetCacheKeepRun;		//moon
extern PUBLIC uint8 u8TimerRetryTimeSynchronization;		//moon
extern uint8 u8Senddatarequest;//jabin
extern bool bButtonEventWakeup;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE pwrm_tsWakeTimerEvent    sWake;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void vStartWakeTimer(uint16 u16Tick);
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
 extern PUBLIC volatile uint8 App_PollRateCnt;
PUBLIC void vAttemptToSleep(void)
{
#if 0
	if( (App_PollRateCnt == 0) && (ZTIMER_eGetState(u8TimerStartJoinNWK) != E_ZTIMER_STATE_RUNNING) && (ZTIMER_eGetState(u8TimerPoll) != E_ZTIMER_STATE_RUNNING) )
	{
		DBG_vPrintf(1, "\nAPP Sleep Handler: Activity Count = %d Task Timers = %d Non Sleep Preventing Timers = %d\n", \
				PWRM_u16GetActivityCount(), u8NumberOfTimersTaskTimers(), u8NumberOfNonSleepPreventingTimers());
		#if 0
		//vStopNonSleepPreventingTimers();//moon
		//if(PWRM_u16GetActivityCount() == 1)
			//PWRM_eFinishActivity(); // Ricky
		if(u8NumberOfNonSleepPreventingTimers() == 1 && PWRM_u16GetActivityCount() == 2)
			PWRM_eFinishActivity(); // Ricky
		#endif
	}
#else
	if( PWRM_u16GetActivityCount() == (u8NumberOfNonSleepPreventingTimers()) )
	{
		DBG_vPrintf(1, "APP Sleep Handler: Activity Count = %d Task Timers = %d Non Sleep Preventing Timers = %d\n", \
				PWRM_u16GetActivityCount(), u8NumberOfTimersTaskTimers(), u8NumberOfNonSleepPreventingTimers());
	}
#endif

    /* Only enter here if the activity count is equal to the number of non sleep preventing timers (in other words, the activity count
     * will become zero when we stop them) */
    if ((PWRM_u16GetActivityCount() == (u8NumberOfNonSleepPreventingTimers())) &&
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
        	bool_t bDeepSleep;
			#ifdef CLD_OTA
        		if(bOTADeepSleepAllowed())
        		{
        			bDeepSleep = TRUE;
        		}
        		else
        		{
        			bDeepSleep = FALSE;
        		}
			#else
        		bDeepSleep = TRUE;
			#endif

        	vScheduleSleep(bDeepSleep);
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
    //uint16 u8NumberRuinningTimers_test = 0;

    if (bButtonDebounceInProgress())
    {
        DBG_vPrintf(1, "\nAPP Sleep Handler: ButtonScaning");
        u8NumberOfRunningTimers++;
        //u8NumberRuinningTimers_test |= 0x0001;
    }
#if 0
    if (ZTIMER_eGetState(u8TimerButtonScan) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(1, "\nAPP Sleep Handler: APP_ButtonsScanTimer");
        u8NumberOfRunningTimers++;
    }
#endif
    if (ZTIMER_eGetState(u8TimerPoll) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerPoll");
        u8NumberOfRunningTimers++;
        //u8NumberRuinningTimers_test |= 0x0002;
    }

    if (ZTIMER_eGetState(u8TimerBlink) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(1, "\nAPP Sleep Handler: APP_JoinBlinkTimer");
        u8NumberOfRunningTimers++;
        //u8NumberRuinningTimers_test |= 0x0004;
    }

#if (defined APP_NTAG_ICODE) || (defined APP_NTAG_AES)
    if (ZTIMER_eGetState(u8TimerNtag) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(1, "\nAPP Sleep Handler: APP_TimerNtag");
        u8NumberOfRunningTimers++;
    }
#endif


if (ZTIMER_eGetState(u8TimerStartResend) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerStartResend");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0008;
}

if (ZTIMER_eGetState(u8TimerRemoteUnlockPemit) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerRemoteUnlockPemit");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0010;
}

if (ZTIMER_eGetState(u8TimerWakeUpTimeOut) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerWakeUpTimeOut");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0020;
}

if (ZTIMER_eGetState(u8TimerLeaveInd) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerLeaveInd");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0040;
}

if (ZTIMER_eGetState(u8TimerSerialTimeout) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerSerialTimeout");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0080;
}

if (ZTIMER_eGetState(u8TimerWriteSerial) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerWriteSerial");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0100;
}

if (ZTIMER_eGetState(u8TimerSerialDioSet) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerSerialDioSet");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0200;
}

if (ZTIMER_eGetState(u8TimerStartJoinNWK) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerStartJoinNWK");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0400;
}
#if 0
if (ZTIMER_eGetState(u8TimerTick) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerTick");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0400;
}

if (ZTIMER_eGetState(u8TimerCheckGetCacheKeepRun) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerCheckGetCacheKeepRun");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0400;
}
if (ZTIMER_eGetState(u8TimerRetryTimeSynchronization) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerRetryTimeSynchronization");
	u8NumberOfRunningTimers++;
	//u8NumberRuinningTimers_test |= 0x0400;
}
#endif
	//DBG_vPrintf(1, "\nAPP Sleep Handler: u8NumberRuinningTimers_test %4x", u8NumberRuinningTimers_test);
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
	//uint32 u32WakeMs;

#if 0//def DEEP_SLEEP_ENABLE
    if (bDeepSleep)
    {
        PWRM_vInit(E_AHI_SLEEP_DEEP);
        DBG_vPrintf(1 , "\nAPP Sleep Handler: Deep Sleep 1");
    }
    else
    {
	   #ifdef CLD_OTA
       if(eOTA_GetState() == OTA_DL_PROGRESS)
    	   u32WakeMs = SENSOR_OTA_SLEEP_IN_SECONDS * APP_TICKS_PER_SECOND;
       else
       #endif
    	   u32WakeMs = (MAXIMUM_TIME_TO_SLEEP - u32GetNumberOfZCLTicksSinceReport())*APP_TICKS_PER_SECOND;

       	   PWRM_eScheduleActivity(&sWake, u32WakeMs, vWakeCallBack);
    	   DBG_vPrintf(1 , "\r\nSLEEP: PWRM_eScheduleActivity(REPORT, %dms) 2", u32WakeMs);
     }
#else
#if 0//def CLD_OTA// moon ota do not keep working in this
    if(eOTA_GetState() == OTA_DL_PROGRESS)
    {
    	DBG_vPrintf(1, "\nAPP Sleep Handler: Osc on, sleeping for %d ticks 3", SENSOR_OTA_SLEEP_IN_SECONDS * APP_TICKS_PER_SECOND);
    	PWRM_eScheduleActivity(&sWake, SENSOR_OTA_SLEEP_IN_SECONDS * APP_TICKS_PER_SECOND , vWakeCallBack);
    }

    else
#endif
    {
    	#if 1
    	//PWRM_eScheduleActivity(&sWake, 1200 * APP_TICKS_PER_SECOND , vWakeCallBack);
    	//PWRM_eScheduleActivity(&sWake, 10 * APP_TICKS_PER_SECOND , vWakeCallBack);//
    	if( TRUE == APP_bNodeIsInRunningState() )
    	{
        	vStartWakeTimer(u32SleepTimeTicks);
        	PWRM_eScheduleActivity(&sWake, (u32SleepTimeTicks) * APP_TICKS_PER_SECOND , vWakeCallBack);
        	DBG_vPrintf(1, "\nAPP Sleep Handler: Osc on, sleeping for %d %d %d ticks 4", u32SleepTimeTicks, u8WakeTimerErroCounter, u8_WakeUpSetBit);
    	}else{
        	vStartWakeTimer(18000);
        	PWRM_eScheduleActivity(&sWake, (18000) * APP_TICKS_PER_SECOND , vWakeCallBack);
        	DBG_vPrintf(1, "\nAPP Sleep Handler: Osc on, sleeping for 18000 %d %d ticks 4", u8WakeTimerErroCounter, u8_WakeUpSetBit);
    	}
		WhetherAskCacheAllCnt = 1;
		u8JoinNetworkFlowPath = 0;
		bPeriodicEventWakeup = FALSE;
        u8_WakeUpSetBit = 0;
        u8WakeTimerErroCounter = 0;
        u8Senddatarequest = 0;
        bButtonEventWakeup = FALSE;
		#else
    	DBG_vPrintf(1, "\nAPP Sleep Handler: Osc on, sleeping for %d ticks 5", (MAXIMUM_TIME_TO_SLEEP - u32GetNumberOfZCLTicksSinceReport())*APP_TICKS_PER_SECOND);
    	PWRM_eScheduleActivity(&sWake, (MAXIMUM_TIME_TO_SLEEP - u32GetNumberOfZCLTicksSinceReport())*APP_TICKS_PER_SECOND , vWakeCallBack);
		#endif
    }
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
	DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nvStopNonSleepPreventingTimers\n");
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
	DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nvStartNonSleepPreventingTimers\n");
    if (ZTIMER_eGetState(u8TimerTick) != E_ZTIMER_STATE_RUNNING)
        ZTIMER_eStart(u8TimerTick, ZCL_TICK_TIME);
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
        //DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: u8TimerTick");
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
#if 0
//#if 0//def CLD_OTA// moon ota do not keep working in this
#if 1//def CLD_OTA// moon ota do not keep working in this
    if (eOTA_GetState() != OTA_IDLE)
    {
    	DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: Start poll timer\n");
    	if(!bOTA_IsWaitToUpgrade())
    	{
    		DBG_vPrintf(1, "\n CLD_OTA Start poll timer\n");
    		//vStartPollTimer(POLL_TIME_FAST);
    		vStartPollTimer(500);
    	}

    }

    if(eOTA_GetState() == OTA_DL_PROGRESS)
    {
        /* Update for 1 second (1000ms) */
        vRunAppOTAStateMachine(SENSOR_OTA_SLEEP_IN_SECONDS*APP_PWRM_TICKS_PER_SECOND);
    }
    else
    {
        /* Update for normal sleep period */
        vRunAppOTAStateMachine((MAXIMUM_TIME_TO_SLEEP - u32GetNumberOfZCLTicksSinceReport())*APP_PWRM_TICKS_PER_SECOND);
    }

    if(bOTA_IsWaitToUpgrade())
    {
        vStopPollTimerTask();
    }
#endif
#endif
    //APP_vSetLED( LED1, sSensor.sOccupancySensingServerCluster.u8Occupancy);//moon

    /*Start the u8TimerTick to continue the ZCL tasks */
    vStartNonSleepPreventingTimers();//moon
}


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
