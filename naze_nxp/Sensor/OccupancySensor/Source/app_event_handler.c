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
#include "App_OccupancySensor.h"
#include "app_sleep_handler.h"
#include "app_event_handler.h"
#include "app_PIR_events.h"
#include "app_reporting.h"
#include "app_blink_led.h"
#include "AppHardwareApi.h"
#include "app_nwk_event_handler.h"
#include "bdb_api.h"
#include "bdb_fb_api.h"
#include "bdb_start.h"	//Ricky
#include "app_main.h"
#ifdef APP_NTAG
#include "ntag_nwk.h"
#include "app_ntag.h"
#endif

#ifdef CLD_OTA
#include "app_ota_client.h"
#endif

#ifdef DB_DOORLOCK
#include "app_SampleDoorLock_Uart.h"
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_EVENT_HANDLER
    #define TRACE_EVENT_HANDLER   TRUE
#else
    #define TRACE_EVENT_HANDLER   FALSE
#endif
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

		App_PollRateCnt = 30;
		vStartPollTimer(POLL_TIME);
//		vEventStartFindAndBind();
        break;

	case SW5_PRESSED:
		DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP: Entering BDB_vStart()\n");

		// 需要做个定时器 // Ricky
//		ZTIMER_eStart(u8TimerStartJoinNWK, ZTIMER_TIME_MSEC(100));
		vAPP_NWKStartJoinEntry();

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
	static uint16 u8OtaCount = 0;
	te_TransitionCode eTransitionCode=NUMBER_OF_TRANSITION_CODE;

	DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: sButton.eType = %d",sButton.eType);

    switch(sButton.eType)
    {

    case APP_E_EVENT_BUTTON_DOWN:
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: Button Number   = %d",sButton.uEvent.sButton.u8Button);
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: DIO State       = %08x",sButton.uEvent.sButton.u32DIOState);

        eTransitionCode = sButton.uEvent.sButton.u8Button;

        DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP Process Buttons: APP_E_EVENT_BUTTON_DOWN ");
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

        DBG_vPrintf(TRACE_EVENT_HANDLER, "\n 3min time on ");
		HeartBeatCount++;

		if (u8RejoinCycles) //Ricky 掉网后触发找网
		{
		//	u8RejoinCycles = 2;			
			ZTIMER_eStart(u8TimerBdbRejoin, ZTIMER_TIME_MSEC(1));
		}

		App_PollRateCnt = 10;
		vStartPollTimer(500);
		sSensor.sPowerConfigServerCluster.zbSpecialHeartBeat = 1;	// Ricky cache
		vSendReportPowerConfig();

        break;

    default:
		
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP Process Buttons: Default ");
        break;
    }

}

PUBLIC void vAppHandleSerialEvent(void)
{
	if (RxSerialMsg[0] == 0)
		return;
	
	switch(RxSerialMsg[3])
	{
		case 0x03:

			if (RxSerialMsg[4] == 0x04)
			{
				// 执行加网				
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent start join network ");
				vAPP_NWKStartJoinEntry();
			}
			else if (RxSerialMsg[4] == 0x05)
			{
				// 执行退网
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent start leave network ");
				ZPS_eAplZdoLeaveNetwork(0, FALSE, FALSE);
				/* Leave failed,so just reset everything */
				APP_vFactoryResetRecords();
				vAHI_SwReset();
			}

			break;

		default :

			// 转发上报	0406
			memcpy(sSensor.sBasicServerCluster.au8Transport,RxSerialMsg,RxSerialMsg[2]+2);
			sSensor.sBasicServerCluster.sTransport.u8Length = RxSerialMsg[2]+2;
			sSensor.sBasicServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
			vSendBasicReport();

			break;

	}
	memset(RxSerialMsg,0,sizeof(RxSerialMsg));

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
	// Ricky pollrate 时间需要修改 
	App_PollRateCnt = 30;
    vStartPollTimer(POLL_TIME);
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

PRIVATE void vAPP_NWKStartJoinEntry(void)
{

	if (FALSE == APP_bNodeIsInRunningState())
	{
		sDeviceDesc.eNodeState = E_STARTUP;
		BDB_eNsStartNwkSteering();
	}

	BDB_vStart();
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
