/*****************************************************************************
 *
 * MODULE:          JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:       app_nwk_event_handler.c
 *
 * DESCRIPTION:     ZLO Demo: Handles all network events like network join/leave
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
#include "dbg.h"
#include "app_events.h"
#include "ZTimer.h"
#include "app_main.h"
#include "App_OccupancySensor.h"
#include "app_sleep_handler.h"
#include "app_common.h"
#include "app_zbp_utilities.h"
#include "app_nwk_event_handler.h"
#include "app_PIR_events.h"
#include "app_blink_led.h"
#include "AppHardwareApi.h"
#include "pwrm.h"
#include "pdum_gen.h"
#include "PDM.h"
#include "pdum_gen.h"
#include "PDM_IDs.h"
#include "app_zlo_sensor_node.h"
#include "app_zcl_sensor_task.h"	//Ricky
#include "app_event_handler.h"
#include "bdb_start.h"
#ifdef CLD_OTA
#include "bdb_ns.h"
//#include "bdb_tl.h"
#endif
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_NWK_EVENT_HANDLER
    #define TRACE_NWK_EVENT_HANDLER   TRUE
#else
    #define TRACE_NWK_EVENT_HANDLER  FALSE
#endif
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern PDM_tsRecordDescriptor sDevicePDDesc;
extern void APP_DoorLockErrCheckAndReport(void);
PUBLIC void App_StopSerialPrepareEnterSleep(void);
extern uint8 u8_WakeUpSetBit;
extern uint8 u8WakeTimerErroCounter;
extern uint8 u8JoinNetworkFlowPath;
extern PUBLIC bool_t bBDBJoinFailed;
extern PUBLIC uint8 u8TimerCheckGetCacheKeepRun;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE uint32 u32PollTime = 0;
PUBLIC volatile uint8 App_PollRateCnt = 0;
uint8 u8Senddatarequest = 0;//jabin 设备入网后打开data request

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC uint8 u8AppNwkNumberOfTimersTaskTimers(void);
extern PUBLIC bool_t bButtonDebounceInProgress(void);
/****************************************************************************
 *
 * NAME: vStartPollTimer
 *
 * DESCRIPTION:
 * Function to start polling.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vStartPollTimer(uint32 u32PollInterval)
{
    u32PollTime = u32PollInterval;
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nAPP Starting poll timer with interval %d", u32PollInterval);
    ZTIMER_eStart(u8TimerPoll, u32PollTime);
}

/****************************************************************************
 *
 * NAME: vStopPollTimerTask
 *
 * DESCRIPTION:
 * Function to stop polling.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vStopPollTimerTask()
{
    u32PollTime = 0;
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nAPP Stopping poll timer");
    ZTIMER_eStop(u8TimerPoll);
}
/****************************************************************************
 *
 * NAME: APP_PollTask
 *
 * DESCRIPTION:
 * Poll Task for timed polling.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_cbTimerPoll( void * pvParam)
{
	if ( App_PollRateCnt > 1 )	//Ricky
	{
		App_PollRateCnt--;
		DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "bBDBJoinFailed = %d %d ", bBDBJoinFailed, u8Senddatarequest);
		if( (TRUE == APP_bNodeIsInRunningState() && bBDBJoinFailed == 0) || (u8Senddatarequest == 1) )//jabin
		{
			ZPS_teStatus state = ZPS_eAplZdoPoll();
			DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "ZPS_eAplZdoPoll = %d\n", state);
			if (139 == state)
			{
				App_PollRateCnt = 1;
				DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "139 APP_cbTimerPoll %d\n",ZPS_u8NwkManagerState());
			}
			else
			{
				if(GateWayCacheAllCnt == 0)
				{

						//if(App_PollRateCnt % 30 == 0)//jabin
						if( App_PollRateCnt == 25 )
						{
							WhetherAskCacheAllCnt = 0;
							DBG_vPrintf(TRACE_NWK_EVENT_HANDLER,"APP_cbTimerPoll 25 ");
							App_AskNumberOfCache();
						}

				}

				if(u8JoinNetworkFlowPath == 1 && App_PollRateCnt == 100)
				{					
					vSendCMEICodeDoorLockReport();//moon
				}
			}
		}
		else
		{
			DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "No In RunningState ");
		}
		
		DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nPoll Sent new poll time %d - %d\n", u32PollTime, App_PollRateCnt);
		ZTIMER_eStart(u8TimerPoll, ZTIMER_TIME_MSEC(u32PollTime));
	}
	else
	{
		DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, " App_PollRateCnt = %d %d %d %d %d %d\n", App_PollRateCnt , u8WakeTimerErroCounter, u8_WakeUpSetBit, WhetherAskCacheAllCnt, u8AppNwkNumberOfTimersTaskTimers(), PWRM_u16GetActivityCount());
		App_PollRateCnt = 0;
		WhetherAskCacheAllCnt = 1;//jabin
		App_StopSerialPrepareEnterSleep();
		ZTIMER_eStop(u8TimerPoll);
		if (ZTIMER_eGetState(u8TimerCheckGetCacheKeepRun) == E_ZTIMER_STATE_RUNNING)
		{
			//DBG_vPrintf(TRACE_APP, "\n u8TimerCheckGetCacheKeepRun ");
			ZTIMER_eStop(u8TimerCheckGetCacheKeepRun);
		}
		vApp_SampleDoorLockStopAllTimer();//jabin
		APP_DoorLockErrCheckAndReport();//异常重启jabin
	}

	if(App_PollRateCnt != 0 && ZTIMER_eGetState(u8TimerPoll) != E_ZTIMER_STATE_RUNNING)//jabin
	{
		DBG_vPrintf(1, "u8TimerPoll Not RUNNING Restart\n");
		ZTIMER_eStart(u8TimerPoll, ZTIMER_TIME_MSEC(500));
	}
}
/****************************************************************************
 *
 * NAME:        u8AppNwkNumberOfTimersTaskTimers
 *
 * DESCRIPTION: Checks to see if any timers are running that shouldn't be
 * interrupted by sleeping.
 *
 ****************************************************************************/
PUBLIC uint8 u8AppNwkNumberOfTimersTaskTimers(void)
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
if (ZTIMER_eGetState(u8TimerCheckGetCacheKeepRun) == E_ZTIMER_STATE_RUNNING)
{
	DBG_vPrintf(1, "\nAPP Sleep Handler: u8TimerCheckGetCacheKeepRun");
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
 * NAME: vHandlePollResponse
 *
 * DESCRIPTION:
 * Processes the poll response, Poll again if we have no ACK to force the Stack
 * to handle the failed poll count or poll to get the data.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandlePollResponse(ZPS_tsAfEvent* psStackEvent)
{
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, " Poll Response u8Status=%x", psStackEvent->uEvent.sNwkPollConfirmEvent.u8Status);

    switch ( psStackEvent->uEvent.sNwkPollConfirmEvent.u8Status)
    {
    case MAC_ENUM_SUCCESS:
    	//DBG_vPrintf(TRACE_NWK_EVENT_HANDLER," MAC_ENUM_SUCCESS");
    case MAC_ENUM_NO_ACK:
    	//DBG_vPrintf(TRACE_NWK_EVENT_HANDLER," MAC_ENUM_NO_ACK");
        //ZPS_eAplZdoPoll();//sometime you can see the data request interving is not 500ms, the reason is here
        break;

    case MAC_ENUM_NO_DATA:
    	//DBG_vPrintf(TRACE_NWK_EVENT_HANDLER," MAC_ENUM_NO_DATA");
    default:
        break;
    }
}

PUBLIC void vAppSampleDoorLockPollChange(uint8 u8PollCnt)
{
	DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n u8JoinNetworkFlowPath %d \n", u8JoinNetworkFlowPath);
	if(u8JoinNetworkFlowPath && App_PollRateCnt > u8PollCnt)
	{
		return;// if still no finish the first time join network flow path, don`t change datarequest! //moon
	}

#if 0
	if (App_SampleDoorLockWakeUpTimeOutDecide())
	{
		App_PollRateCnt = u8PollCnt;
	}
	else
	{
		if (App_PollRateCnt < u8PollCnt)
		{
			App_PollRateCnt = u8PollCnt;
		}
	}
#else
	if (App_PollRateCnt < u8PollCnt)
	{
		App_PollRateCnt = u8PollCnt;
	}
#endif
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n vAppSampleDoorLockPollChange %d \n", App_PollRateCnt);

	if(App_PollRateCnt != 0 && ZTIMER_eGetState(u8TimerPoll) != E_ZTIMER_STATE_RUNNING)
	{	
		if(u32PollTime == 0)
			u32PollTime = 500;
		vStartPollTimer(u32PollTime);
	}
}
/****************************************************************************
 *
 * NAME: vHandleFailedToJoin
 *
 * DESCRIPTION:
 * If we have failed to join/rejoin the network stop the blink timer to
 * go back to sleep
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandleFailedToJoin(void)
{
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nAPP NWK Event Handler: Failed Rejoin");
    /* In case Find And Bind or Keep alive in progress stop it and make sure we go to sleep */
    vEventStopFindAndBind();//moon
    APP_bPersistantPolling &= FALSE;
    bBDBJoinFailed = TRUE;
	
	AppDisableuOtaControler();
}

/****************************************************************************
 *
 * NAME: vHandleFailedRejoin
 *
 * DESCRIPTION:
 * If we have failed to rejoin the network we start the blink timer.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandleFailedRejoin(void)
{
    /* Start Blink Timer to avoid sleeping */
    if(ZTIMER_eGetState(u8TimerBlink) != E_ZTIMER_STATE_RUNNING)
        vStartBlinkTimer(APP_JOINING_BLINK_TIME);

	AppDisableuOtaControler();
}

/****************************************************************************
 *
 * NAME: vHandleNetworkJoinEndDevice
 *
 * DESCRIPTION:
 * If we have joined a new network or rejoined, stop the timer tell the PIR
 * event handler.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandleNetworkJoinEndDevice(void)
{
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nAPP NWK Event Handler: Network Join End Device");

	//DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nCancel vStopPollTimerTask");
	#if 0
    /* Don't turn the timers off if we're in persistent polling mode */
    if(APP_bPersistantPolling != TRUE)
    {
        vStopPollTimerTask();
        vStopBlinkTimer();
    }
	#endif

	if (App_SampleDoorLockWakeUpTimeOutDecide())
		//vAppSampleDoorLockPollChange(20);
		vAppSampleDoorLockPollChange(6);
	else
		vAppSampleDoorLockPollChange(120);
	
	ZTIMER_eStart(u8TimerPoll, 500);
	ZTIMER_eStop(u8TimerBdbRejoin);

    vHandleNewJoinEvent();//moon

	//App_AskNumberOfCache();//moon add - jabin remove

#if (defined APP_NTAG_ICODE) || (defined APP_NTAG_AES)
    sDeviceDesc.eNodeState = E_RUNNING;
    PDM_eSaveRecordData(PDM_ID_APP_SENSOR,
            &sDeviceDesc,
            sizeof(tsDeviceDesc));
    ZPS_vSaveAllZpsRecords();
#endif

}

/****************************************************************************
 *
 * NAME: vHandleNetworkLeave
 *
 * DESCRIPTION:
 * We have left the network so restart as factory new (not sure why we don't
 * restart the joining rather than restarting the whole device).
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandleNetworkLeave(ZPS_tsAfEvent* psStackEvent)
{
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nAPP NWK Event Handler: Network Leave");
    if( psStackEvent->uEvent.sNwkLeaveIndicationEvent.u64ExtAddr == 0 &&
            (psStackEvent->uEvent.sNwkLeaveIndicationEvent.u8Rejoin == FALSE))
    {
        DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nAPP NWK Event Handler: ZDO Leave" );
        APP_vFactoryResetRecords();
        vAHI_SwReset();
    }
}

/****************************************************************************
 *
 * NAME: vHandleNetworkJoinAndRejoin
 *
 * DESCRIPTION:
 * Checks to see if we have joined a network. If we have, stop all the timers,
 * save the network state into PDM and attempt to sleep.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandleNetworkJoinAndRejoin(void)
{

    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n***DEVICE_IN_NETWORK");
    //ZTIMER_eStop(u8TimerPoll);//moon
    //vStopBlinkTimer()//moon
    sDeviceDesc.eNodeState = E_RUNNING;
	
	APP_bNodeIsInRunningState();//moon

    PDM_eSaveRecordData(PDM_ID_APP_SENSOR,
                        &sDeviceDesc,
                        sizeof(tsDeviceDesc));
    ZPS_vSaveAllZpsRecords();


}

/****************************************************************************
 *
 * NAME: vHandleNetworkLeaveConfirm
 *
 * DESCRIPTION:
 * We have left the network so restart as factory new (not sure why we don't
 * restart the joining rather than restarting the whole device).
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandleNetworkLeaveConfirm(ZPS_tsAfEvent* psStackEvent)
{
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nAPP NWK Event Handler: Network Leave Confirm Addr %016llx",
                psStackEvent->uEvent.sNwkLeaveConfirmEvent.u64ExtAddr);
    if ( psStackEvent->uEvent.sNwkLeaveConfirmEvent.u64ExtAddr == 0UL)
    {
        APP_vFactoryResetRecords();
        vAHI_SwReset();
    }
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
