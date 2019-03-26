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
#include <zps_gen.h>
#include "app_events.h"
#include "ZTimer.h"
#include "app_main.h"
#include "App_WL_DoorLock.h"
#include "app_sleep_handler.h"
#include "app_common.h"
#include "app_zbp_utilities.h"
#include "app_nwk_event_handler.h"
#include "app_PIR_events.h"
#include "app_blink_led.h"
#include "AppHardwareApi.h"
#include "pwrm.h"
#include "pdum_gen.h"
#include "pdm.h"
#include "pdum_gen.h"
#include "PDM_IDs.h"
#include "app_zlo_sensor_node.h"
#include "app_zcl_sensor_task.h"	//Ricky
#include "app_event_handler.h"
#include "bdb_start.h"

#include "bdb_ns.h"

#ifdef BDB_SUPPORT_TOUCHLINK
#include "bdb_tl.h"
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
extern uint8 ContrlReAckGetWay;
extern uint8 crtreasknum;//δ�յ�response�ط�
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE uint32 u32PollTime = 1000;

PUBLIC volatile uint8 App_PollRateCnt = 0;
uint8 WhetherAskCacheAllCnt = 1;
uint8 GateWayCacheAllCnt = 0;
uint8 RequestNextCacheNum = 0;
uint8 RecordRequestNextCacheNum = 0xFF;
uint8 RetryRequestNextCacheNum = 0xFF;

//PRIVATE uint8 u8PollNoACKCount = 0;
PUBLIC void vAppSampleDoorLockPollChange(uint8 u8PollCnt);
extern uint8 u8JoinNetworkFlowPath;
extern uint8 GetPollControlResponse;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void App_StopSerialPrepareEnterSleep(void);
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
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nAPP Starting poll timer with interval %d,%d", u32PollInterval,App_PollRateCnt);
    ZTIMER_eStart(u8TimerPoll, ZTIMER_TIME_MSEC(u32PollTime));
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
//	u32PollTime = 0;
	vAppSampleDoorLockPollChange(0);
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n APP Stopping poll timer");
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

		if (ZTIMER_eGetState(u8TimerBdbNs) == E_ZTIMER_STATE_RUNNING)
			DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n u8TimerBdbNs is Running \n ");

		if ((TRUE == APP_bNodeIsInRunningState()) || (ZTIMER_eGetState(u8TimerBdbNs) == E_ZTIMER_STATE_RUNNING))
		{
			if ((ZTIMER_eGetState(u8TimerBdbRejoin) != E_ZTIMER_STATE_RUNNING) || 
				(ZTIMER_eGetState(u8TimerBdbNs) == E_ZTIMER_STATE_RUNNING))
			{
				if (139 == ZPS_eAplZdoPoll())
				{
					App_PollRateCnt = 1;
					DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n APP_cbTimerPoll %d ",ZPS_u8NwkManagerState());
				}
				else
				{		
					#if 0
					//DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n GateWayCacheAllCnt Remain %d \n ", GateWayCacheAllCnt);
					if(GateWayCacheAllCnt == 0)
					{
						if(App_PollRateCnt > 5)
						{
							if(App_PollRateCnt % 20 == 0)
								App_AskNumberOfCache();
						}
						else
						{
							if(App_PollRateCnt % 5 == 0)
								App_AskNumberOfCache();
						}
					}
					else
					{
						if(App_PollRateCnt % 11 == 0 && RecordRequestNextCacheNum <= RequestNextCacheNum)
						{
							//App_AskOneOfCache();
							DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n RequestNextCacheNum No Push Ahead \n ");
						}
						else
						{
							DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n RequestNextCacheNum Keep Ahead \n ");
						}
					}
					#else
					if(App_PollRateCnt % WhetherAskCacheAllCnt == 0)
					{
						DBG_vPrintf(1,"\n WhetherAskCacheAllCnt = 1\n");
						WhetherAskCacheAllCnt = 1;
					}

					if(RetryRequestNextCacheNum == App_PollRateCnt)
					{
						if(GateWayCacheAllCnt != 0 && RecordRequestNextCacheNum == RequestNextCacheNum)
						{
							//App_AskOneOfCache();
							DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n App_AskNumberOfCache No Push Ahead ");
						}
						else
						{

						}
					}
					#endif
				}
			}
			else
			{
				DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n ZTIMER_eGetState(u8TimerBdbRejoin)! ");
				App_PollRateCnt = 0;
			}
		}
		DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n APP Poll Handler: Poll Sent, new poll time %d - %d ", u32PollTime,App_PollRateCnt);
		ZTIMER_eStart(u8TimerPoll, ZTIMER_TIME_MSEC(u32PollTime));

	}
	else
	{

	    if( ((GateWayCacheAllCnt != 0) || (crtreasknum != 0)) && (ContrlReAckGetWay < 2) )
	    {
	    	DBG_vPrintf(1, "\n Try Ask getWay cache num again\n");
	    	ContrlReAckGetWay ++;
	    	ZTIMER_eStart(u8AskNumGetWay,ZTIMER_TIME_MSEC(200));
	    }
		GateWayCacheAllCnt = 0;
		App_PollRateCnt = 0;
		App_StopSerialPrepareEnterSleep();
		vStopPollTimerTask();
	#if 0
		APP_BDBQueueCountPrint();
		DBG_vPrintf(TRUE, "\n APP_msgBdbEvents %d ",ZQ_bQueueIsEmpty(&APP_msgBdbEvents));
		DBG_vPrintf(TRUE, "\n APP_msgZpsEvents %d ",ZQ_bQueueIsEmpty(&APP_msgZpsEvents));
		DBG_vPrintf(TRUE, "\n APP_msgZclEvents %d ",ZQ_bQueueIsEmpty(&APP_msgZclEvents));
		DBG_vPrintf(TRUE, "\n APP_msgAppEvents %d ",ZQ_bQueueIsEmpty(&APP_msgAppEvents));
		DBG_vPrintf(TRUE, "\n zps_msgMlmeDcfmInd %d ",ZQ_bQueueIsEmpty(&zps_msgMlmeDcfmInd));
		DBG_vPrintf(TRUE, "\n zps_TimeEvents %d ",ZQ_bQueueIsEmpty(&zps_TimeEvents));
		DBG_vPrintf(TRUE, "\n APP_msgSerialRx %d ",ZQ_bQueueIsEmpty(&APP_msgSerialRx));
	#endif
		vApp_SampleDoorLockStopAllTimer();
	//	vBDB_SampleDoorLockGetTimerStatus();
	}

	if(App_PollRateCnt != 0 && ZTIMER_eGetState(u8TimerPoll) != E_ZTIMER_STATE_RUNNING)
	{	
		DBG_vPrintf(1, "\n u8TimerPoll Not RUNNING Restart\n");
		ZTIMER_eStart(u8TimerPoll, ZTIMER_TIME_MSEC(500));
	}

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
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\nAPP NWK Event Handler: Poll Response");
    switch ( psStackEvent->uEvent.sNwkPollConfirmEvent.u8Status)
    {
    case MAC_ENUM_SUCCESS:
    case MAC_ENUM_NO_ACK:
		DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n MAC_ENUM_NO_ACK ");	//Ricky Mark
        ZPS_eAplZdoPoll();
        break;

    case MAC_ENUM_NO_DATA:
    default:
        break;
    }

}

PUBLIC void vAppSampleDoorLockPollChange(uint8 u8PollCnt)
{
	DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n u8JoinNetworkFlowPath %d \n",u8JoinNetworkFlowPath);
	if(u8JoinNetworkFlowPath && App_PollRateCnt > u8PollCnt)
	{
		return;// if still no finish the first time join network flow path, don`t change datarequest! //moon
	}

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
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n vAppSampleDoorLockPollChange %d \n",App_PollRateCnt);

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
//    vEventStopFindAndBind();
    APP_bPersistantPolling &= FALSE;
    bBDBJoinFailed = FALSE;	//Ricky Changed TRUE -> FALSE
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
//	if(ZTIMER_eGetState(u8TimerBlink) != E_ZTIMER_STATE_RUNNING) ;	// Ricky
//		vStartBlinkTimer(APP_JOINING_BLINK_TIME);
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
    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n APP NWK Event Handler: Network Join End Device");

	if (App_SampleDoorLockWakeUpTimeOutDecide())
		vAppSampleDoorLockPollChange(20);
	else
		vAppSampleDoorLockPollChange(120);
	vStartPollTimer(500);

    vHandleNewJoinEvent();

#if 1 //def APP_NTAG
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
 *	����//Ricky 
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

    DBG_vPrintf(TRACE_NWK_EVENT_HANDLER, "\n DEVICE_IN_NETWORK");

    sDeviceDesc.eNodeState = E_RUNNING;
#ifdef CLD_OTA
	sZllState.eNodeState = E_RUNNING;	//Ricky
#endif
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
