/*****************************************************************************
 *
 * MODULE:          JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:       app_event_handler.c
 *
 * DESCRIPTION:     ZLO Demo: Handles all the different type of Application events
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
#include "app_zlo_sensor_node.h"
#include "app_occupancy_sensor_state_machine.h"
#include "App_WL_DoorLock.h"
#include "app_sleep_handler.h"
#include "app_event_handler.h"
#include "app_PIR_events.h"
#include "app_reporting.h"
#include "app_blink_led.h"
#include "AppHardwareApi.h"
#include "app_nwk_event_handler.h"
#include "bdb_api.h"
//#include "bdb_fb_api.h"
#include "bdb_start.h"	//Ricky
#include "app_main.h"
#ifdef APP_NTAG
#include "ntag_nwk.h"
#include "app_ntag.h"
#endif

#ifdef CLD_OTA
#include "app_ota_client.h"
#endif

#ifdef BDB_SUPPORT_TOUCHLINK
#include "bdb_tl.h"
#endif

#include "app_SampleDoorLock_Uart.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_EVENT_HANDLER
    #define TRACE_EVENT_HANDLER   TRUE
#else
    #define TRACE_EVENT_HANDLER   FALSE
#endif

static bool u8WakeUpFlg = 0;
PRIVATE ts_SerialCache tsSerialBuffer;

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vDioEventHandler(te_TransitionCode eTransitionCode);
PRIVATE void vEventStartFindAndBind(void);
PRIVATE void vStartPersistantPolling(void);
PRIVATE void vStopPersistantPolling(void);

PRIVATE void vAppProcessSerialCacheShift(void);
PUBLIC bool vAppDecideSerialCacheAreaFull(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern const uint8 u8MyEndpoint;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void vAppSerialCacheArea_Init()
{
	uint8 index = 0;

	tsSerialBuffer.c8Top = -1;

	while(index < SERIAL_CACHE_MAX_ITEM)
	{
		memset(tsSerialBuffer.u8SerialData[index++],0,SERIAL_CACHE_ITEM_LEN);
	}

}

PRIVATE void vAppAddSerialDataToCacheArea(void)
{
	if (tsSerialBuffer.c8Top == -1)
		tsSerialBuffer.c8Top = 0;
	else
		tsSerialBuffer.c8Top += 1;

	memcpy(tsSerialBuffer.u8SerialData[tsSerialBuffer.c8Top],RxSerialMsg,SERIAL_CACHE_ITEM_LEN);
}

PRIVATE void vAppProcessSerialCacheRecevice(void)
{
	if (tsSerialBuffer.c8Top < 0)
		return;

	memset(RxSerialMsg,0,sizeof(RxSerialMsg));
	memcpy(RxSerialMsg,tsSerialBuffer.u8SerialData[0],SERIAL_CACHE_ITEM_LEN);

	if (tsSerialBuffer.c8Top == 0)
		tsSerialBuffer.c8Top = -1;

	//Todo : << 1
	vAppProcessSerialCacheShift();

}

PRIVATE void vAppProcessSerialCacheShift(void)
{
	if (tsSerialBuffer.c8Top <= 0)
		return;

	uint8 i;

	for (i = 0; i < tsSerialBuffer.c8Top; i++)
	{
		memcpy(tsSerialBuffer.u8SerialData[i],tsSerialBuffer.u8SerialData[i+1],SERIAL_CACHE_ITEM_LEN);
	}

	memset(tsSerialBuffer.u8SerialData[tsSerialBuffer.c8Top--],0,SERIAL_CACHE_ITEM_LEN);

}

PUBLIC bool vAppDecideSerialCacheAreaFull(void)
{
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\n vAppDecideSerialCacheAreaFull %d",tsSerialBuffer.c8Top);

	if (tsSerialBuffer.c8Top == -1)
		return FALSE;

	return ( tsSerialBuffer.c8Top < SERIAL_CACHE_MAX_ITEM ? FALSE : TRUE );
}
/****************************************************************************
 *
 * NAME: vDioEventHandler
 *
 * DESCRIPTION:
 * Processes the Dio events like binding and occupancy. Any other events that
 * come through we immediately attempt to go to sleep as we have no process for
 * them.
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vDioEventHandler(te_TransitionCode eTransitionCode )
{
    ZPS_eAplZdoPoll();
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\nAPP Process Buttons: In vSensorStateMachine TransitionCode = %x -> ",eTransitionCode);

    switch(eTransitionCode)
    {

    /* Fall through for the button presses as there will be a delayed action*/
    case COMM_BUTTON_PRESSED:
        break;

    case COMM_BUTTON_RELEASED:
        break;

    case SW1_PRESSED:
        vHandleFallingEdgeEvent();
        break;

    case SW1_RELEASED:
        vHandleRisingEdgeEvent();
        break;

    case SW2_PRESSED:
        vStartPersistantPolling();
        break;

    case SW3_PRESSED:
        vStopPersistantPolling();
        break;

#if defined(APP_NTAG)
    case FD_PRESSED:
    case FD_RELEASED:
        APP_vNtagStart(NFC_NWK_NSC_DEVICE_CLIMATE_SENSOR_DEVICE);
        break;
#endif

    case SW4_PRESSED:
		DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP: Entering DIO WakeUp %d \n",u8RejoinCycles);

		if (u8RejoinCycles) //Ricky 掉网后触发找网
		{
			u8RejoinCycles = 2;			
			ZTIMER_eStart(u8TimerBdbRejoin, ZTIMER_TIME_MSEC(1));
		}

		vStopPollTimerTask();

		if (TRUE == APP_bNodeIsInRunningState())
			App_PollRateCnt = 6;	// For Wonly
		else
			App_PollRateCnt = 6;

		vStartPollTimer(POLL_TIME);
        break;

	case SW5_PRESSED:
		DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP: Entering BDB_vStart()\n");

		// Ricky
	#if 0
		u8JoinNWKCycle = 1;
		vAPP_NWKStartJoinEntry();
	#else
		ZllCommissonCommandScanSend();
//		vStopPollTimerTask();

		App_PollRateCnt = 2;
		vStartPollTimer(ZTIMER_TIME_MSEC(500));
	#endif
		break;

    case SW4_RELEASED:
        vEventStopFindAndBind();
        break;
    case SW2_RELEASED:
    case SW3_RELEASED:

        break;

    default:
        break;

    }
}

/****************************************************************************
 *
 * NAME: vAppHandleAppEvent
 *
 * DESCRIPTION:
 * interprets the button press and calls the state machine.
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vAppHandleAppEvent(APP_tsEvent sButton)
{
	//static uint16 u8OtaCount = 0;
	te_TransitionCode eTransitionCode=NUMBER_OF_TRANSITION_CODE;

	DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: sButton.eType = %d",sButton.eType);

    switch(sButton.eType)
    {

    case APP_E_EVENT_BUTTON_DOWN:
//		DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: Button Number   = %d",sButton.uEvent.sButton.u8Button);
//		DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: DIO State       = %08x",sButton.uEvent.sButton.u32DIOState);

        eTransitionCode = sButton.uEvent.sButton.u8Button;

//		DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP Process Buttons: APP_E_EVENT_BUTTON_DOWN ");
        vDioEventHandler(eTransitionCode);
        break;

    case APP_E_EVENT_BUTTON_UP:
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: Button Number = %d",sButton.uEvent.sButton.u8Button);
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: DIO State     = %08x",sButton.uEvent.sButton.u32DIOState);

        eTransitionCode = BUTTON_RELEASED_OFFSET | sButton.uEvent.sButton.u8Button;

        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: APP_E_EVENT_BUTTON_UP ");
        vDioEventHandler(eTransitionCode);
        break;

    case APP_E_EVENT_WAKE_TIMER:

        DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP_E_EVENT_WAKE_TIMER ");
        vHandleWakeTimeoutEvent();
        break;

    case APP_E_EVENT_SEND_REPORT:
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP_E_EVENT_SEND_REPORT ");
	//	vSendImmediateReport();

        break;

    case APP_E_EVENT_PERIODIC_REPORT:

        DBG_vPrintf(TRACE_EVENT_HANDLER, "\n 30min time on ");
		HeartBeatCount++;

		if (u8RejoinCycles) //Ricky 掉网后触发找网
		{
		
        	DBG_vPrintf(TRACE_EVENT_HANDLER, "\n HeartBeatCount : %d u8RejoinCycles : %d ",HeartBeatCount,u8RejoinCycles);
			if (((HeartBeatCount%8) == 0) || (u8RejoinCycles == 1))
			{
			#if WL_DOORLOCK
				u8RejoinCycles = 2;
				ZTIMER_eStart(u8TimerBdbRejoin, ZTIMER_TIME_MSEC(1));
			#endif
			}
		}
		else
		{

			App_PollRateCnt = 4;
			vStartPollTimer(500);
		}

		if (TRUE == APP_bNodeIsInRunningState())
		{
		#ifndef YYH_CUSTOM
			sSensor.sDoorLockServerCluster.eLockState= 2;
			App_SendReportDoorLockState();
		#else
			vSendReportPowerConfig();
		#endif
		}

        break;

    default:
		
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP Process Buttons: Default ");
        break;
    }

}

PUBLIC void vAppHandleSerialEvent(void)
{
	uint8 u8DecideFlg = 0, u8ret = 0;

	if (RxSerialMsg[0] == 0)
		return;

	if ((RxSerialMsg[1] == 0x0A) && (RxSerialMsg[7] == 0))
	{

		DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent : %0x ",RxSerialMsg[2]);
#if 0
		if (FALSE == APP_bNodeIsInRunningState())
		{
			if (FALSE == App_SampleDoorLockProcessPemit(RxSerialMsg[2]))
				return;
		}
#endif
		switch(RxSerialMsg[2])
		{
			case TransgressAlarm :	// 完成

				u8DecideFlg = 1;
				App_SampleDoorLockSendAlarmCmd(0x33);		
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n TransgressAlarm  ");
				break;

			case TamperAlarm :	// 完成

				App_SampleDoorLockSendAlarmCmd(0x04);
				break;
			
			case PretendLock :	// 完成

				App_SampleDoorLockSendAlarmCmd(0x07);				
				break;

			case UnLockAlarm :	// 完成

				App_SampleDoorLockSendAlarmCmd(0x05);
				break;

			case Coercion :	// 胁迫报警 完成
				
				App_SampleDoorLockSendAlarmCmd(0x06);
				break;

			case AlarmReset :	// 完成

				App_SampleDoorLockResetAlarmCode(RxSerialMsg[8]);
				break;

#ifdef SKYWORTH
			case DoorRing :

				App_SampleDoorLockIASWDStartWaring(RxSerialMsg[8],((uint16)(RxSerialMsg[10] << 8) | (uint16)(RxSerialMsg[9])));

				break;
#endif
			case BatteryAlarm :	// 完成

				sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = 1;
				sSensor.sPowerConfigServerCluster.u8BatteryPercentageRemaining = 50;
				vSendReportPowerConfig();
				break;

			case JoinNWK :	// 完成

				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent start join network ");
	
				if (FALSE == APP_bNodeIsInRunningState())
				{
					u8DecideFlg = 1;
					u8JoinNWKCycle = 0;
					ZTIMER_eStart(u8TimerStartJoinNWK,ZTIMER_TIME_MSEC(1));
//					vAPP_NWKStartJoinEntry();
				}
				else
				{
					u8DecideFlg = 2;
					APP_SerialSendJoinIndication(0,0);				
				}

				break;

			case LeaveNWK :	// 重启需延时

				u8DecideFlg = 2;
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent start leave network ");

				APP_SerialSendJoinIndication(0x01,0x00);
				bLeaveInd = 1;
				ZTIMER_eStart(u8TimerLeaveInd,ZTIMER_TIME_MSEC(3000));

				break;

			case RemoteOperation :	// 待验证

				App_StopSerialPrepareEnterSleep();
				u8ret = App_SerialRemoteOperationHandle(RxSerialMsg[8]);

				if (u8ret & 0x01)
					u8DecideFlg = 1;
				else if (u8ret & 0x02)
				{
					u8WakeUpFlg = 0;
					ZTIMER_eStop(u8TimerWakeUpTimeOut);
					// 上报开锁记录
			//		ZTIMER_eStop(u8TimerRemoteUnlockPemit);
			//		ZTIMER_eStart(u8TimerRemoteUnlockPemit,ZTIMER_TIME_MSEC(1));
					App_SendOperationEventNotification(OPTEVTSOURCE_REMOTE,eSampleDoorLock_LockState,
														0x00,u16PinData,0);
				}

				break;

			case WakeUp :	// 完成

				if ((FALSE == APP_bNodeIsInRunningState()) || (0 != u8RejoinCycles))
				{
					u8DecideFlg = 0;
					App_SerialSendNetworkIndication(0x01);
				}
				else
				{
					u8DecideFlg = 1;
					u8WakeUpFlg = 1;
					ZTIMER_eStop(u8TimerWakeUpTimeOut);
					ZTIMER_eStart(u8TimerWakeUpTimeOut,ZTIMER_TIME_SEC(57));

			//		sSensor.sBasicServerCluster.bDeviceEnabled = 1;
					vSendBasicReport();
					ZTIMER_eStop(u8TimerRemoteUnlockPemit);
					ZTIMER_eStart(u8TimerRemoteUnlockPemit,ZTIMER_TIME_MSEC(1));
					App_SerialSendNetworkIndication(0x00);
				}

				break;

			case LoaclOperation :

				App_SampleDoorLockAnalyzeUnlockRecord();
				App_SampleDoorLockAnalyzeVoltageInformation();

				break;

			case LockReport :

				// 已关锁
				App_SendLoaclOperationLockReport();

				break;

			case GetTime :

//				u8DecideFlg = 1;
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n sSensor.sTimeClientCluster.utctTime %d ",sSensor.sTimeClientCluster.utctTime);
				vSendCurrentTime();

				break;

			default :

				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n Command unsupport ! ");
				break;

		}
	}
	else if((RxSerialMsg[1] == 0x01) && (RxSerialMsg[7] == 1))
	{
	
		DBG_vPrintf(TRACE_EVENT_HANDLER, "\n App_StopSerialPrepareEnterSleep  ");
		// Ricky Stop Serial Operation
		App_StopSerialPrepareEnterSleep();

		if (RxSerialMsg[8] == 0x00)
		{

			switch (RxSerialMsg[2])
			{
				case NwkPrompt :

					// Todo Reboot 1s Daley;
					if (bLeaveInd)
					{
						bLeaveInd = 0;
						ZTIMER_eStop(u8TimerLeaveInd);					
						ZTIMER_eStart(u8TimerLeaveInd,ZTIMER_TIME_MSEC(500));
					}

					break;
				case NwkIndication :

					u8DecideFlg = 1;
					break;

				default:
					break;
			}

		}
		else
		{
			// 60s Enter Sleep
			u8DecideFlg = 1;			
		}
		
	}


	if (u8DecideFlg == 1)
	{

		vStopPollTimerTask();

		if (TRUE == APP_bNodeIsInRunningState())
		{
			App_PollRateCnt = 120;
			vStartPollTimer(ZTIMER_TIME_MSEC(500));
		}
		else
		{
				App_PollRateCnt = 6;
				vStartPollTimer(ZTIMER_TIME_MSEC(500));
		}
	}
	else if (u8DecideFlg == 0)
	{
		if (!u8WakeUpFlg)
		{
			vStopPollTimerTask();
			App_PollRateCnt = 6;
			vStartPollTimer(ZTIMER_TIME_MSEC(500));
		}

	}
	else
	{
		vStopPollTimerTask();
		App_PollRateCnt = 30;
		vStartPollTimer(ZTIMER_TIME_MSEC(500));
	}

//	memset(RxSerialMsg,0,sizeof(RxSerialMsg));

}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: vEventStartFindAndBind
 *
 * DESCRIPTION:
 * Initiates the find and bind procedure, Starts a poll timer and the blink
 * timer.
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vEventStartFindAndBind(void)
{
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\nAPP Process Buttons: eEZ_FindAndBind");
    sBDB.sAttrib.u16bdbCommissioningGroupID = 0xFFFF;
    vAPP_ZCL_DeviceSpecific_SetIdentifyTime(0xFF);
    BDB_eFbTriggerAsInitiator(u8MyEndpoint);

//    vStartBlinkTimer(APP_FIND_AND_BIND_BLINK_TIME);	// Ricky
}

/****************************************************************************
 *
 * NAME: vEventStopFindAndBind
 *
 * DESCRIPTION:
 * Stops the find and bind procedure and attempts to sleep.
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vEventStopFindAndBind(void)
{
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\nAPP Process Buttons: Exit Easy Mode");
//	vAPP_ZCL_DeviceSpecific_IdentifyOff();
//	BDB_vFbExitAsInitiator();
//	vStopBlinkTimer();
//	vStopPollTimerTask();	//Ricky
    vHandleNewJoinEvent();
}

/****************************************************************************
 *
 * NAME: vStartPersistantPolling
 *
 * DESCRIPTION:
 * Starts the Poll timer which will in turn keep the device awake so it can
 * receive data from it's parent.
 *
 ****************************************************************************/
PRIVATE void vStartPersistantPolling(void)
{
    APP_bPersistantPolling |= TRUE;
    vStartPollTimer(POLL_TIME_FAST);
    vStartBlinkTimer(APP_KEEP_AWAKE_TIME);
}

/****************************************************************************
 *
 * NAME: vStopPersistantPolling
 *
 * DESCRIPTION:
 * Stops the poll timer which will allow the device to go back to sleep.
 *
 ****************************************************************************/
PRIVATE void vStopPersistantPolling(void)
{
    APP_bPersistantPolling &= FALSE;
    vStopPollTimerTask();
    vStopBlinkTimer();
}

PUBLIC void vApp_EventHandleSerialCache(bool u8CtrlFlg)
{
	DBG_vPrintf(TRACE_EVENT_HANDLER,"\n vApp_EventHandleSerialCache %d",u8CtrlFlg);

	if (u8CtrlFlg)
	{
		DBG_vPrintf(TRACE_EVENT_HANDLER,"\n Serial add to cache");
		if (!(SERIAL_CACHE_FULL))
		{

			//Ricky : add to cache
			vAppAddSerialDataToCacheArea();
			//Ricky : Delay To Handle
			if ( E_ZTIMER_STATE_RUNNING != ZTIMER_eGetState(u8TimerSerialEvent))
			{
				App_PollRateCnt += 2;
				ZTIMER_eStart(u8TimerSerialEvent, ZTIMER_TIME_MSEC(200));
			}

		}
		else
		{
			DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SERIAL_CACHE_FULL");
		#if 0
			//Ricky : Cas 1
			vAppProcessSerialCacheShift();			
			vAppAddSerialDataToCacheArea();
		#endif
		}
	}
	else
	{
		vAppHandleSerialEvent();
	}


}

PUBLIC void APP_cbStartNWKJoin(void)
{
	u8JoinNWKCycle++;
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\n APP_cbStartNWKJoin %d",u8JoinNWKCycle);

	if (TRUE == APP_bNodeIsInRunningState())
	{
		u8JoinNWKCycle = 4;
	}

	if (u8JoinNWKCycle < 3)
	{
		vAPP_NWKStartJoinEntry();
		ZTIMER_eStop(u8TimerStartJoinNWK);
		ZTIMER_eStart(u8TimerStartJoinNWK,ZTIMER_TIME_SEC(25));
	}
	else
	{
		u8JoinNWKCycle = 4;
		ZTIMER_eStop(u8TimerStartJoinNWK);
	}
}

PRIVATE void vAPP_NWKStartJoinEntry(void)
{

	if (FALSE == APP_bNodeIsInRunningState())
	{
		sDeviceDesc.eNodeState = E_STARTUP;
		BDB_eNsStartNwkSteering();
	}

	BDB_vStart();
}

PUBLIC void App_cbTimerProcessSerialEvent(void)
{
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\n App_cbTimerProcessSerialEvent %d ",tsSerialBuffer.c8Top);

	if (tsSerialBuffer.c8Top >= 0)
	{
		DBG_vPrintf(TRACE_EVENT_HANDLER,"\n Cache Recevice To Process");
		//Todo : Recevice To Process
		vAppProcessSerialCacheRecevice();
		vAppHandleSerialEvent();
	}
	
	if (tsSerialBuffer.c8Top >= 0)
	{
		ZTIMER_eStart(u8TimerSerialEvent, ZTIMER_TIME_MSEC(50));
	}
	else
	{
		DBG_vPrintf(TRACE_EVENT_HANDLER,"\n Cache Clear ");
		ZTIMER_eStop(u8TimerSerialEvent);
		vAppSerialCacheArea_Init();
	}

}

PUBLIC void App_EventStartFindAndBind(void)
{
	vEventStartFindAndBind();

}
PUBLIC void App_cbTimerWakeUpTimeOut(void)
{
	u8WakeUpFlg = 0;
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
