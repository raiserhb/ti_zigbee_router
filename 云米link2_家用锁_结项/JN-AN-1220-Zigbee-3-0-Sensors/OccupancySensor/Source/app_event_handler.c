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
 * Copyright NXP B.V. 2017. All rights reserved
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
#include "app_zcl_sensor_task.h"
#include "app_zcl_tick_handler.h"
#include "bdb_api.h"
#include "bdb_fb_api.h"
#include "app_main.h"
#include "zps_gen.h"
#ifdef APP_NTAG_ICODE
#include "app_ntag_icode.h"
#include "nfc_nwk.h"
#endif
#ifdef APP_NTAG_AES
#include "app_ntag_aes.h"
#include "nfc_nwk.h"
#endif
#include "app_SampleDoorLock_Uart.h"
#include "app_reporting.h"
#include "pwrm.h"
#include "PDM_IDs.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_EVENT_HANDLER
    #define TRACE_EVENT_HANDLER   TRUE
#else
    #define TRACE_EVENT_HANDLER   FALSE
#endif
static bool u8WakeUpFlg = 0;
volatile ts_SerialCache tsSerialBuffer;
extern uint8 u8JoinNetworkFlowPath;
extern PUBLIC volatile uint8 App_PollRateCnt;
bool bPeriodicEventWakeup = FALSE;
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vDioEventHandler(te_TransitionCode eTransitionCode);
//PRIVATE void vEventStartFindAndBind(void);
//PRIVATE void vStartPersistantPolling(void);
//PRIVATE void vStopPersistantPolling(void);
PRIVATE void vAPP_NWKStartJoinEntry(void);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern const uint8 u8MyEndpoint;

extern uint8 u8TimerCheckGetCacheKeepRun;
extern uint8 u8TimerRetryTimeSynchronization;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PUBLIC uint8 WhetherAskCacheAllCnt = 1;
PUBLIC uint8 GateWayCacheAllCnt = 0;
PUBLIC uint8 RequestNextCacheNum = 0;
PUBLIC uint8 RecordRequestNextCacheNum = 0;
PUBLIC uint8 GetPollControlResponse = 0;;
PUBLIC uint8 GateWayCacheNoPushAheadTimes = 0;
PUBLIC uint8 u8RetryTimeSynchronizationTimes = 0;

PUBLIC te_RequestCacheStatus requestCacheStatus = 0;
PUBLIC uint16 HeartBeatCount = 0;
PUBLIC uint8 CMEI[15];  //

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
extern PUBLIC void ZllCommissonCommandScanSend(void);
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
    //ZPS_eAplZdoPoll();//moon
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\nAPP Process Buttons: In vSensorStateMachine TransitionCode = %02x -> ",eTransitionCode);
    switch(eTransitionCode)
    {

    /* Fall through for the button presses as there will be a delayed action*/
    case COMM_BUTTON_PRESSED:
		DBG_vPrintf(TRACE_EVENT_HANDLER,"\n COMM_BUTTON_PRESSED \n");
        break;

    case COMM_BUTTON_RELEASED:		
		DBG_vPrintf(TRACE_EVENT_HANDLER,"\n COMM_BUTTON_RELEASED \n");
        break;

    case SW1_PRESSED:
		DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW1_PRESSED \n");
        //vHandleFallingEdgeEvent();
        break;

    case SW1_RELEASED:
		DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW1_RELEASED \n");
        //vHandleRisingEdgeEvent();
        break;

    case SW2_PRESSED:
		DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW2_PRESSED \n");
        //vStartPersistantPolling();
        break;

    case SW3_PRESSED:
		DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW3_PRESSED \n");
        //vStopPersistantPolling();
        break;

#if (defined APP_NTAG_ICODE) || (defined APP_NTAG_AES)
    case FD_PRESSED:
    case FD_RELEASED:
        #if APP_NTAG_ICODE
            APP_vNtagStart(OCCUPANCYSENSOR_SENSOR_ENDPOINT);
        #endif
        #if APP_NTAG_AES
            APP_vNtagStart(NFC_NWK_NSC_DEVICE_CLIMATE_SENSOR_DEVICE);
        #endif
        break;
#endif

    case SW4_PRESSED:
		{
			static uint16 cnt = 0;
			DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW4_PRESSED %x\n", cnt++);
    	}

		//vStartPollTimer(POLL_TIME);
		//vStartPollTimer(500);
        //vEventStartFindAndBind();
        break;

    case SW4_RELEASED:
		{
			static uint16 cnt = 0;
			DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW4_RELEASED %x\n", cnt++);
    	}
		vAppSampleDoorLockPollChange(16);			
        //vEventStopFindAndBind();
        break;
		
    case SW2_RELEASED:
    case SW3_RELEASED:
        break;

	case SW5_PRESSED:
		{
			static uint16 cnt = 0;
			DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW5_PRESSED %x\n", cnt++);
    	}
	
		ZllCommissonCommandScanSend();
		PWRM_eFinishActivity(); // Ricky
		
		vAppSampleDoorLockPollChange(2);
		vStartPollTimer(ZTIMER_TIME_MSEC(500));
		break;

	case SW12_PRESSED:
		{
			static uint16 cnt = 0;
			DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW12_PRESSED %x\n", cnt++);
    	}

		if (TRUE == APP_bNodeIsInRunningState())
		{
			DBG_vPrintf(1, "\n WhetherAskCacheAllCnt=%d \n", WhetherAskCacheAllCnt);
			if(WhetherAskCacheAllCnt == 1)
			{
				App_AskNumberOfCache();
				WhetherAskCacheAllCnt = 0;
			}
		}

		//vAppSampleDoorLockPollChange(16);
		vAppSampleDoorLockPollChange(6);

		break;

	case SW5_RELEASED:
		{
			static uint16 cnt = 0;
			DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW5_RELEASED %x\n", cnt++);
    	}
		break;

	case SW12_RELEASED:
		{
			static uint16 cnt = 0;
			DBG_vPrintf(TRACE_EVENT_HANDLER,"\n SW12_RELEASED %x\n", cnt++);
    	}
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
    te_TransitionCode eTransitionCode=NUMBER_OF_TRANSITION_CODE;

    switch(sButton.eType)
    {

    case APP_E_EVENT_BUTTON_DOWN:
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: Button Number   = %d",sButton.uEvent.sButton.u8Button);
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: DIO State       = %08x",sButton.uEvent.sButton.u32DIOState);

        eTransitionCode = sButton.uEvent.sButton.u8Button;

        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: Transition Code = %d",eTransitionCode);
        vDioEventHandler(eTransitionCode);
        break;

    case APP_E_EVENT_BUTTON_UP:
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: Button Number = %d",sButton.uEvent.sButton.u8Button);
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: DIO State     = %08x",sButton.uEvent.sButton.u32DIOState);

        eTransitionCode = BUTTON_RELEASED_OFFSET | sButton.uEvent.sButton.u8Button;

        DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: Transition Code = %d",eTransitionCode);
        vDioEventHandler(eTransitionCode);
        break;

    case APP_E_EVENT_WAKE_TIMER:
		{
			static uint16 cnt = 0;
			DBG_vPrintf(TRACE_EVENT_HANDLER,"\n APP_E_EVENT_WAKE_TIMER %x\n", cnt++);
    	}
        //vHandleWakeTimeoutEvent();//moon
        break;

    case APP_E_EVENT_SEND_REPORT:
		{
			static uint16 cnt = 0;
			DBG_vPrintf(TRACE_EVENT_HANDLER,"\n APP_E_EVENT_SEND_REPORT %x\n", cnt++);
    	}
        //vSendImmediateReport();//moon
        break;

    case APP_E_EVENT_PERIODIC_REPORT:
		HeartBeatCount++;
		bPeriodicEventWakeup = TRUE;
        DBG_vPrintf(1, "\n 20min time on HeartBeatCount=%x", HeartBeatCount);
		
		if (TRUE == APP_bNodeIsInRunningState())
		{			
			if(bBDBJoinFailed == 0)
			{
				App_AskNumberOfCache();
				vAppSampleDoorLockPollChange(10);
			}
			else
			{
				vAppSampleDoorLockPollChange(2);// rejoin failed do not send ask cache all count
			}
		}
        break;

    default:
        break;
    }

}

PUBLIC void vAppHandleSerialEvent(void)
{
	uint8 u8DecideFlg = 0, u8ret = 0, u8WhetherChangPollcnt = 1;
	vAppSampleDoorLockPollChange(10);

	if (RxSerialMsg[0] == 0)
		return;
	
	DBG_vPrintf(TRACE_EVENT_HANDLER, "\n RxSerialMsg[2]=%02x, RxSerialMsg[7]=%02x", RxSerialMsg[2], RxSerialMsg[7]);
	
	if (RxSerialMsg[7] == 0)
	{		
		uint8 u8IsNeedAck = 1;
		
		switch(RxSerialMsg[2])
		{
			case TransgressAlarm :	// ���
				u8DecideFlg = 1;
				App_SampleDoorLockSendAlarmCmd(0x33);		
				//DBG_vPrintf(TRACE_EVENT_HANDLER, "\n TransgressAlarm  ");
				break;

			case TamperAlarm :	// ���
				App_SampleDoorLockSendAlarmCmd(0x04);
				break;
			
			case PretendLock :	// ���
				App_SampleDoorLockSendAlarmCmd(0x07);				
				break;

			case UnLockAlarm :	// ���
				App_SampleDoorLockSendAlarmCmd(0x05);
				break;

			case Coercion :	// в�ȱ��� ���
				App_SampleDoorLockSendAlarmCmd(0x06);
				break;
				
			case AlarmReset :	// ���
				App_SampleDoorLockResetAlarmCode(RxSerialMsg[8]);
				break;
				
		#if 0//def SKYWORTH
			case DoorRing :
				App_SampleDoorLockIASWDStartWaring(RxSerialMsg[8], ((uint16)(RxSerialMsg[10] << 8) | (uint16)(RxSerialMsg[9])),TRUE);
				break;
				
			case FingerRequest:
			case FingerTransfer:

				if (((TRUE == APP_bNodeIsInRunningState()) && (0 != u8RejoinCycles)) ||
					(FALSE == APP_bNodeIsInRunningState()) )
				{
					// Break Finger;
					APP_SerialSendFingerBreak();
				}
				else
				{
					memcpy(sSensor.sBasicServerCluster.au8Transport,RxSerialMsg,RxSerialMsg[1]+8);
					sSensor.sBasicServerCluster.sTransport.u8Length = RxSerialMsg[1]+8;
					sSensor.sBasicServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
					vSendBasicReport();
				}
				break;

			case FingerBreak:
			case ManageUsers:

				memcpy(sSensor.sBasicServerCluster.au8Transport,RxSerialMsg,RxSerialMsg[1]+8);
				sSensor.sBasicServerCluster.sTransport.u8Length = RxSerialMsg[1]+8;
				sSensor.sBasicServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
				vSendBasicReport();
				break;
		#endif
		
			case GetModelVersion:
				//APP_AskDoorLockVerInfo();
				APP_SerialSendModelVersion();
				u8IsNeedAck = 0;
				break;
				
		#ifdef SKYWORTH
			case CacheRequest:

				sSensor.sPowerConfigServerCluster.zbSpecialHeartBeat = 1;
				vSendSpecialHeartBeatReport();
				break;
		#endif
		
			case BatteryAlarm :	// ���

				sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = 1;
				sSensor.sPowerConfigServerCluster.u8BatteryPercentageRemaining = 50;
			#ifdef SKYWORTH
				vSendPowerConfigReport();
			#else
				vSendReportPowerConfig();
			#endif
				break;
			
			case JoinNWK :	// ���
				//DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent start join network ");
				if (FALSE == APP_bNodeIsInRunningState())
				{
					u8DecideFlg = 1;
					u8JoinNWKCycle = 0;//moon
					ZTIMER_eStart(u8TimerStartJoinNWK,ZTIMER_TIME_MSEC(1));
				}
				else
				{
					u8DecideFlg = 2;
					APP_SerialSendJoinIndication(0, 0);				
				}
				
				uint8 cnt = 0;

				if(RxSerialMsg[1] >= 0x09 && RxSerialMsg[8] == 0x11 && RxSerialMsg[9] == 0x7F)// 0x117F for youdian, 117F �����ŵ�����ʹ��
				{					
					memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FNB56-DOR27YD1.3", CLD_BAS_MODEL_ID_SIZE);
					
					if(RxSerialMsg[10] == 0x01)// change SNID to FNB56-DOR17YD1.0
					{
						sSensor.sBasicServerCluster.au8ModelIdentifier[9] = '1';
					}
					else if(RxSerialMsg[10] == 0x02)// change SNID to FNB56-DOR27YD1.0
					{
						sSensor.sBasicServerCluster.au8ModelIdentifier[9] = '2';
					}
					else if(RxSerialMsg[10] == 0x03)// change SNID to xxxxxxDOR27YD1.0
					{
						memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, &RxSerialMsg[11], 6);// set user chars to SNID

						uint8 versionChar = 0;
						uint16 u16ByteRead = 0;
						for(cnt = 0; cnt < 6; cnt++)
						{
							PDM_vDeleteDataRecord(PDM_ID_APP_VERSION_CHARS + cnt);
							PDM_eSaveRecordData(PDM_ID_APP_VERSION_CHARS + cnt, &RxSerialMsg[11 + cnt], sizeof(uint8));
						}

						DBG_vPrintf(1, "\npdmVersionChar :");
						for(cnt = 0; cnt < 6; cnt++)
						{
							PDM_eReadDataFromRecord(PDM_ID_APP_VERSION_CHARS + cnt, &versionChar, sizeof(versionChar), &u16ByteRead);
							DBG_vPrintf(1, "%c ", versionChar);
						}
					}

					PDM_vDeleteDataRecord(PDM_ID_APP_YOUDIAN_VERSION_TYPE);
 					PDM_eSaveRecordData(PDM_ID_APP_YOUDIAN_VERSION_TYPE, &RxSerialMsg[10], sizeof(uint8));
				}
				else if(RxSerialMsg[1] >= 0x09 && RxSerialMsg[8] == 0x11 && RxSerialMsg[9] == 0x7E)// just do change SNID to xxxxxxDOR23FBx.x
				{
					memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, &RxSerialMsg[11], 6);// set user chars to SNID
					
					uint8 versionChar = 0;
					uint16 u16ByteRead = 0;
					for(cnt = 0; cnt < 6; cnt++)
					{
						PDM_vDeleteDataRecord(PDM_ID_APP_VERSION_CHARS + cnt);
						PDM_eSaveRecordData(PDM_ID_APP_VERSION_CHARS + cnt, &RxSerialMsg[11 + cnt], sizeof(uint8));
					}
					
					DBG_vPrintf(1, "\npdmVersionChar :");
					for(cnt = 0; cnt < 6; cnt++)
					{
						PDM_eReadDataFromRecord(PDM_ID_APP_VERSION_CHARS + cnt, &versionChar, sizeof(versionChar), &u16ByteRead);
						DBG_vPrintf(1, "%c ", versionChar);
					}

					versionChar = 0x04;
					PDM_vDeleteDataRecord(PDM_ID_APP_YOUDIAN_VERSION_TYPE);
 					PDM_eSaveRecordData(PDM_ID_APP_YOUDIAN_VERSION_TYPE, &versionChar, sizeof(uint8));
				}
				else
				{
					uint8 versionChar = 0xFF;
					PDM_vDeleteDataRecord(PDM_ID_APP_YOUDIAN_VERSION_TYPE);
 					PDM_eSaveRecordData(PDM_ID_APP_YOUDIAN_VERSION_TYPE, &versionChar, sizeof(uint8));
				}

				if(RxSerialMsg[1] >= 0x18)
				{
					DBG_vPrintf(1, "\nPrint  CMEI :");
					for(cnt = 0; cnt < 15; cnt++)// print  CMEI ��
					{
						CMEI[cnt] = RxSerialMsg[17 + cnt];
						DBG_vPrintf(1, "%c ", CMEI[cnt]);
						PDM_vDeleteDataRecord(PDM_ID_APP_CMEI_CODE + cnt);
						PDM_eSaveRecordData(PDM_ID_APP_CMEI_CODE + cnt, &RxSerialMsg[17 + cnt], sizeof(uint8));// save CMEI to flash
					}
				}
				else
				{
					uint8 none = 0xFF;
					for(cnt = 0; cnt < 15; cnt++)// print  CMEI ��
					{
						PDM_vDeleteDataRecord(PDM_ID_APP_CMEI_CODE + cnt);
						PDM_eSaveRecordData(PDM_ID_APP_CMEI_CODE + cnt, &none, sizeof(uint8));// save CMEI to flash
					}

				}
				
				DBG_vPrintf(1, "\nVersionChar :");
				for(cnt = 0; cnt < 16; cnt++)
				{
					DBG_vPrintf(1, "%c ", sSensor.sBasicServerCluster.au8ModelIdentifier[cnt]);
				}
				break;

			case LeaveNWK :	// ��������ʱ
				u8DecideFlg = 2;
				//DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent start leave network ");
				APP_SerialSendJoinIndication(0x01,0x00);
				bLeaveInd = 1;
				ZTIMER_eStart(u8TimerLeaveInd,ZTIMER_TIME_MSEC(3000));
				break;
				
			case RemoteOperation :	// ����֤
				App_StopSerialPrepareEnterSleep();
				u8ret = App_SerialRemoteOperationHandle(RxSerialMsg[8]);

				if (u8ret & 0x01)
					u8DecideFlg = 1;
				else if (u8ret & 0x02)
				{
					u8WakeUpFlg = 0;
					ZTIMER_eStop(u8TimerWakeUpTimeOut);
					// �ϱ�������¼
			//		ZTIMER_eStop(u8TimerRemoteUnlockPemit);
			//		ZTIMER_eStart(u8TimerRemoteUnlockPemit,ZTIMER_TIME_MSEC(1));
					App_SendOperationEventNotification(OPTEVTSOURCE_REMOTE,eSampleDoorLock_LockState,
														0x00,u16PinData,0);
					if(!u8JoinNetworkFlowPath)
						App_PollRateCnt = 4;
				}
				break;
				
			case DoorRing :
				 App_SampleDoorLockIASWDStartWaring(RxSerialMsg[8], ((uint16)(RxSerialMsg[10] << 8) | (uint16)(RxSerialMsg[9])),TRUE);
			case WakeUp :	// ���
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

					//sSensor.sBasicServerCluster.bDeviceEnabled = 1;
					vSendBasicDeviceEnabledReport();
					ZTIMER_eStop(u8TimerRemoteUnlockPemit);
					ZTIMER_eStart(u8TimerRemoteUnlockPemit,ZTIMER_TIME_MSEC(1));
					ZTIMER_eStop(u8TimerStartResend);
					App_SerialSendNetworkIndication(0x00);
					App_Start_Recv_BasicResponse_TimerBit();//moons
				}
				break;
				
			case LoaclOperation :
				App_SampleDoorLockAnalyzeUnlockRecord();
			#ifdef SKYWORTH
				if ((RxSerialMsg[13] & 0x10))
				{
					App_SampleDoorLockVoltageReportProcess();
				}
				else
			#endif
				{
					App_SampleDoorLockAnalyzeVoltageInformation(RxSerialMsg[11]);
				}
				break;
				
			case LockReport :
				// �ѹ���
				App_SendLoaclOperationLockReport();
				break;
				
			case GetTime :
				//u8DecideFlg = 1;
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n sSensor.sTimeClientCluster.utctTime %d ",sSensor.sTimeClientCluster.utctTime);
				vSendCurrentTime();
				u8RetryTimeSynchronizationTimes = 0;
				ZTIMER_eStart(u8TimerRetryTimeSynchronization, ZTIMER_TIME_SEC(3));//moon
				break;
				
			case ReviceIEEE:
				App_SampleDoorLockIEEEReadResponse(RxSerialMsg+8,RxSerialMsg[1]);
				break;
				
			case SleepInTime:
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n Command Sleep In Time ! ");
				u8IsNeedAck = 0;
				if ((FALSE == APP_bNodeIsInRunningState()) || (0 != u8RejoinCycles) || u8JoinNetworkFlowPath)
				{
					App_AckSleepInTime(0x01);// faile
				}
				else
				{
					App_AckSleepInTime(0x00);// success 
					if(!App_SampleDoorLockWakeUpTimeOutDecide())
					{
						u8WakeUpFlg = 0;//App_cbTimerWakeUpTimeOut();// if 0x77 still running, stop it. 
						ZTIMER_eStop(u8TimerWakeUpTimeOut);						
						ZTIMER_eStop(u8TimerRemoteUnlockPemit);
					}
					//vAppSampleDoorLockPollChange(4);
					App_PollRateCnt = 4;
					u8WhetherChangPollcnt = 0;//do not change poll cnt
				}
				break;
				
			case DoorKeyPadLocked:// suning huitailong
				App_SampleDoorLockResetAlarmCode(0x0D);
				break;
				
			case DoorOtherIllgalAlarmReport:// suning huitailong
				App_SampleDoorLockResetAlarmCode(0x0D + RxSerialMsg[8]);// 0x0E 0x0F 0x10
				break;

			case DoorFingerInputLocked:
				App_SampleDoorLockResetAlarmCode(0x11);
				break;

			case DoorBackLocking:
				App_SampleDoorLockResetAlarmCode(0x12);
				break;

			case DoorReleaseBackLocking:
				App_SampleDoorLockResetAlarmCode(0x13);
				break;

			case DoorCardInputLocked:
				App_SampleDoorLockResetAlarmCode(0x14);
				break;
				
			case DoorMechanicalkeyOpenDoor:
				//App_SampleDoorLockResetAlarmCode(0x15);
				App_SendOperationEventNotification(OPTEVTSOURCE_MECHANICALKEY, E_CLD_DOORLOCK_LOCK_STATE_UNLOCKED, 0xFF, 0xFFFF, 0);
				break;
				
			case DoorUserListChangeReport:// suning huitailong
			case DoorSomeModeReport:// suning huitailong
				if(APP_bNodeIsInRunningState())
				{
					DBG_vPrintf(TRACE_EVENT_HANDLER, "\n\n Received UserListChange\n\n");
					sSensor.sDoorLockServerCluster.au8Transport[0] = 0xFF;
					sSensor.sDoorLockServerCluster.au8Transport[1] = 0xFD;// suning huitailong 
					sSensor.sDoorLockServerCluster.au8Transport[2] = 0;// on purpose set to 0, in order to keep the same with KaiDiShi
					sSensor.sDoorLockServerCluster.au8Transport[3] = 0;// on purpose set to 0, in order to keep the same with KaiDiShi
					sSensor.sDoorLockServerCluster.au8Transport[4] = 0;// on purpose set to 0, in order to keep the same with KaiDiShi
					sSensor.sDoorLockServerCluster.au8Transport[5] = RxSerialMsg[1] + 1;// data len, contain "command ID" + "data"
					sSensor.sDoorLockServerCluster.au8Transport[6] = RxSerialMsg[2];// command ID
					
					memcpy(&sSensor.sDoorLockServerCluster.au8Transport[7], &RxSerialMsg[8], RxSerialMsg[1]);// copy data to transport buffer
					sSensor.sDoorLockServerCluster.sTransport.u8Length = RxSerialMsg[1] + 7;// + 7 = "FF FD 00 00 00 len command"
					sSensor.sDoorLockServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);

					vSendDoorLockOtherInfoReport();// report to gateway
				}
				break;
				
			default:				
				u8WhetherChangPollcnt = 0;//do not change poll cnt
				memcpy(sSensor.sBasicServerCluster.au8Transport,RxSerialMsg,RxSerialMsg[1]+8);
				sSensor.sBasicServerCluster.sTransport.u8Length = RxSerialMsg[1]+8;
				sSensor.sBasicServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
				vSendBasicReport(); // clister:Basic, attribute:0x410C
				DBG_vPrintf(1, "\nReport Command ID %02x", RxSerialMsg[2]);
				break;

		}

		if(u8IsNeedAck)
		{
			APP_SerialSendAcknowlegement(RxSerialMsg[2], 0);//moon
		}
	}
	else if(RxSerialMsg[7] == 1)
	{
		// Ricky Stop Serial Operation
		App_StopSerialPrepareEnterSleep();
		
	#ifdef SKYWORTH
		if (RxSerialMsg[2] == BindUnlock)
		{
		//	App_SampleDoorLockSetPinCode(RxSerialMsg[8]);
			if (RxSerialMsg[8] == 0)
			{
				eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_UNLOCKED;
				App_SampleDoorLockLinkageCountIncrement();
				u8WakeUpFlg = 0;
				ZTIMER_eStop(u8TimerWakeUpTimeOut);
			}
			else
			{
				eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_NOT_FULLY_LOCKED;
			}

			App_SendOperationEventNotification(0x60,eSampleDoorLock_LockState,
														0,0,0);

		//	DBG_vPrintf(TRACE_EVENT_HANDLER, "\n App_SampleDoorLockSetPinCode  ");
		}
		#if 0
		if ((RxSerialMsg[2] == BindUnlockCheck) && (RxSerialMsg[8] == 0x00))
		{
		//	ZTIMER_eStop(u8TimerRemoteBindUnlock);
		//	ZTIMER_eStart(u8TimerRemoteBindUnlock,ZTIMER_TIME_MSEC(1));
		}
		#endif
	#endif
	
		switch(RxSerialMsg[2])
		{
			case NwkPrompt :
				if(RxSerialMsg[8] == 0x00)
				{
					// Todo Reboot 1s Daley;
					if (bLeaveInd)
					{
						bLeaveInd = 0;						
						//App_SampleDoorLockDisableInterrupt();
						ZTIMER_eStop(u8TimerLeaveInd);					
						ZTIMER_eStart(u8TimerLeaveInd,ZTIMER_TIME_MSEC(500));
					}
					else
					{												
						uint8 TxSerialMsg_[10] = {0};
						App_SendDataImmediatelyOrSaveIntoCacheBuffer(0, AskDoorLockVerInfo, 10, TxSerialMsg_);
					}
				}
			break;

			case NwkIndication :
				if(RxSerialMsg[8] == 0x00)
				{
					u8DecideFlg = 1;
				}
			break;

			case ServerAskKeyList:
			case ServerOldOpenDoorList:
				if(RxSerialMsg[1] != 1)// no ack faile!!!
				{
					uint8 TxSerialMsg_[13] = {0};
					DBG_vPrintf(TRACE_EVENT_HANDLER, "\n Ack Suing %02x", RxSerialMsg[2]);
					TxSerialMsg_[0] = 0x0D;
					TxSerialMsg_[1] = 0xAA;
					TxSerialMsg_[2] = 0x02;
					TxSerialMsg_[3] = RxSerialMsg[2];

					TxSerialMsg_[4] = RxSerialMsg[3];
					TxSerialMsg_[5]	= RxSerialMsg[4];
					TxSerialMsg_[6]	= RxSerialMsg[5];
					TxSerialMsg_[7]	= RxSerialMsg[6];

					TxSerialMsg_[8] = 0x01;
					TxSerialMsg_[9] = 0xFF;
					TxSerialMsg_[10] = 0xFF;
					TxSerialMsg_[11] = vUART_TxCharCRC(TxSerialMsg_);
					TxSerialMsg_[12] = 0x55;

					vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
				    vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);

					HAL_UART_Write(TxSerialMsg_);
				}
				
			case ServerSetDoorSomeMode:			
				DBG_vPrintf(1, "\nReport Command ID %02x\n\n", RxSerialMsg[2]);
				sSensor.sDoorLockServerCluster.au8Transport[0] = 0xFF;
				sSensor.sDoorLockServerCluster.au8Transport[1] = 0xFD;// suning huitailong 
				sSensor.sDoorLockServerCluster.au8Transport[2] = 0;// on purpose set to 0, in order to keep the same with KaiDiShi
				sSensor.sDoorLockServerCluster.au8Transport[3] = 0;// on purpose set to 0, in order to keep the same with KaiDiShi
				sSensor.sDoorLockServerCluster.au8Transport[4] = 0;// on purpose set to 0, in order to keep the same with KaiDiShi
				sSensor.sDoorLockServerCluster.au8Transport[5] = RxSerialMsg[1] + 1;// data len, contain "command ID" + "data" + "BCC\End"
				sSensor.sDoorLockServerCluster.au8Transport[6] = RxSerialMsg[2];// command ID
				
				memcpy(&sSensor.sDoorLockServerCluster.au8Transport[7], &RxSerialMsg[8], RxSerialMsg[1]);// copy data to transport buffer
				
				sSensor.sDoorLockServerCluster.sTransport.u8Length = RxSerialMsg[1] + 7;// + 7 = "FF FD 00 00 00 len command"
				sSensor.sDoorLockServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
				
				vSendDoorLockOtherInfoReport();// report to gateway, clister:DoorLock, attribute:0x401C
				u8WhetherChangPollcnt = 0;//do not change poll cnt
				break;

			case RemoteUnlock:
				
				break;
				
			case ModelVersionReply:
			case PwdOperation:
			case EnableDoorLock:
			case AskDoorLockVerInfo:
			case AskDoorLockRTC:
			case SetDoorLockTime:
			default:				
				u8WhetherChangPollcnt = 0;//do not change poll cnt
				memcpy(sSensor.sBasicServerCluster.au8Transport,RxSerialMsg,RxSerialMsg[1]+8);
				sSensor.sBasicServerCluster.sTransport.u8Length = RxSerialMsg[1]+8;
				sSensor.sBasicServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
				vSendBasicReport();	// clister:Basic, attribute:0x410C
				DBG_vPrintf(1, "\nReport Command ID %02x", RxSerialMsg[2]);
				break;
		}
	}
	
	if(u8WhetherChangPollcnt)//moon, decide wether need change poll count
	{
		if (u8DecideFlg == 1)
		{

			if (TRUE == APP_bNodeIsInRunningState())
			{
				vAppSampleDoorLockPollChange(120);
				vStartPollTimer(ZTIMER_TIME_MSEC(500));
			}
			else
			{
				vAppSampleDoorLockPollChange(8);
				vStartPollTimer(ZTIMER_TIME_MSEC(500));
			}
		}
		else if (u8DecideFlg == 0)
		{
			if (!u8WakeUpFlg)
			{
				vAppSampleDoorLockPollChange(8);
				vStartPollTimer(ZTIMER_TIME_MSEC(500));
			}

		}
		else
		{
			vAppSampleDoorLockPollChange(30);
			vStartPollTimer(ZTIMER_TIME_MSEC(500));
		}
	}
	
	memset(RxSerialMsg, 0, sizeof(RxSerialMsg));
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
#if 0
PRIVATE void vEventStartFindAndBind(void)
{
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\nAPP Process Buttons: eEZ_FindAndBind");
    sBDB.sAttrib.u16bdbCommissioningGroupID = 0xFFFF;
    vAPP_ZCL_DeviceSpecific_SetIdentifyTime(0xFF);
    BDB_eFbTriggerAsInitiator(u8MyEndpoint);
    vStartPollTimer(POLL_TIME);
    vStartBlinkTimer(APP_FIND_AND_BIND_BLINK_TIME);
}
#endif
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
	#if 1	
    vAPP_ZCL_DeviceSpecific_IdentifyOff();
    BDB_vFbExitAsInitiator();
    vStopBlinkTimer();
    //vStopPollTimerTask();//moon
    vHandleNewJoinEvent();
	#else
    vAPP_ZCL_DeviceSpecific_IdentifyOff();
    BDB_vFbExitAsInitiator();
    vStopBlinkTimer();
    vStopPollTimerTask();
    vHandleNewJoinEvent();
	#endif
}

#if 0
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
	DBG_vPrintf(TRACE_EVENT_HANDLER,"\n vStartPersistantPolling");
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
#endif

PUBLIC void APP_cbStartNWKJoin(void *pvParam)
{
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\n APP_cbStartNWKJoin u8JoinNWKCycle=%d",u8JoinNWKCycle);
	u8JoinNWKCycle++;

	if (TRUE == APP_bNodeIsInRunningState())
	{
		return ;
	}

	vAPP_NWKStartJoinEntry();

}

PRIVATE void vAPP_NWKStartJoinEntry(void)
{
	DBG_vPrintf(TRACE_EVENT_HANDLER,"\n vAPP_NWKStartJoinEntry %d",u8JoinNWKCycle);

	if (FALSE == APP_bNodeIsInRunningState())
	{
		sDeviceDesc.eNodeState = E_STARTUP;
		BDB_eNsStartNwkSteering();
	}

	BDB_vStart();
}
PUBLIC void App_cbTimerWakeUpTimeOut(void *pvParam)
{
	u8WakeUpFlg = 0;
}
PUBLIC bool App_SampleDoorLockWakeUpTimeOutDecide(void)
{
	if (u8WakeUpFlg == 0)
		return TRUE;
	return FALSE;
}
/****************************************************************************
 *
 * NAME: vSend Report Poll Control 0x5000 
 *
 * DESCRIPTION:
 * Method to send a report to all bound nodes.
 *
 ****************************************************************************/
PUBLIC void App_SendReportPollControlSendCacheAck(void)
{
	uint8 u8ret = 0;
    PDUM_thAPduInstance myPDUM_thAPduInstance;
    tsZCL_Address sDestinationAddress;

    DBG_vPrintf(TRUE, "\n APP Report: PollControl ");

    /* get buffer to write the response in*/
    myPDUM_thAPduInstance = hZCL_AllocateAPduInstance();
    /* no buffers - return*/
    if(myPDUM_thAPduInstance == PDUM_INVALID_HANDLE)
    {
        DBG_vPrintf(TRUE, "\n APP Report: PDUM_INVALID_HANDLE");
    }

    /* Set the address mode to send to all bound device and don't wait for an ACK*/
    sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;

    /* Send the report with all attributes.*/
	u8ret = eZCL_ReportAttribute(&sDestinationAddress,GENERAL_CLUSTER_ID_POLL_CONTROL,
								E_CLD_POLL_CONTROL_ATTR_ID_SEND_CACHE_ACK ,1,11,myPDUM_thAPduInstance);
    if (E_ZCL_SUCCESS != u8ret)
    {
    	DBG_vPrintf(TRUE, "\n Poll Control Send Cache Ack %d",u8ret);
    }
	else
	{
    	DBG_vPrintf(TRUE, "\n Poll Control Send Cache Ack Success ");
	}

    /* free buffer and return*/
	PDUM_eAPduFreeAPduInstance(myPDUM_thAPduInstance);
}


/****************************************************************************
 *
 * NAME: vSend Report Poll Control 0x5000 
 *
 * DESCRIPTION:
 * Method to send a report to all bound nodes.
 *
 ****************************************************************************/
PUBLIC void App_SendReportPollControlGetCache(void)
{
	uint8 u8ret = 0;
    PDUM_thAPduInstance myPDUM_thAPduInstance;
    tsZCL_Address sDestinationAddress;

    DBG_vPrintf(TRUE, "\n APP Report: PollControl ");

    /* get buffer to write the response in*/
    myPDUM_thAPduInstance = hZCL_AllocateAPduInstance();
    /* no buffers - return*/
    if(myPDUM_thAPduInstance == PDUM_INVALID_HANDLE)
    {
        DBG_vPrintf(TRUE, "\n APP Report: PDUM_INVALID_HANDLE");
    }

    /* Set the address mode to send to all bound device and don't wait for an ACK*/
    sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;

    /* Send the report with all attributes.*/
	u8ret = eZCL_ReportAttribute(&sDestinationAddress,GENERAL_CLUSTER_ID_POLL_CONTROL,
								E_CLD_POLL_CONTROL_ATTR_ID_GET_CACHE_ITEM ,1,REPORT_EP,myPDUM_thAPduInstance);
    if (E_ZCL_SUCCESS != u8ret)
    {
    	DBG_vPrintf(TRUE, "\n APP Report: Error Sending Report %d",u8ret);
    }

    /* free buffer and return*/
	PDUM_eAPduFreeAPduInstance(myPDUM_thAPduInstance);
    DBG_vPrintf(TRUE, "\n\n APP Report: Report Sent Success \n\n");

}

PUBLIC void App_AskNumberOfCache(void)
{
    DBG_vPrintf(TRUE, "\n\n App Ask Number Of Cache \n");
	#if 0
	if(GateWayCacheAllCnt != 0)
	{		
		DBG_vPrintf(TRUE, " But Return\n");
		return;
	}
	#endif

	if(bBDBJoinFailed == 1)
	{
		DBG_vPrintf(TRUE, " But ReturnbBDBJoinFailed == 1\n");
		return;
	}
	
	GateWayCacheAllCnt = 0;
	GetPollControlResponse = 0;

	sSensor.sPollControlServerCluster.u8GetCacheFromGW = 0;//u8GetCacheFromGW all count
	App_SendReportPollControlGetCache();
	
	requestCacheStatus = E_WaitCacheAllCountResponse;
	ZTIMER_eStart(u8TimerCheckGetCacheKeepRun, ZTIMER_TIME_SEC(5));
}

PUBLIC void App_AskOneOfCache(void)
{
	DBG_vPrintf(TRUE, "\n\n App Ask One Of Cache 1/%d \n\n", GateWayCacheAllCnt);

	sSensor.sPollControlServerCluster.u8GetCacheFromGW = RequestNextCacheNum;
	App_SendReportPollControlGetCache();
}

PUBLIC void APP_CheckGateWayCacheAllCnt(void)//// check wheater still need request cache data from gateway
{
	if(GateWayCacheAllCnt != 0)
		GateWayCacheAllCnt --;// decrease the count of cache datas
	
	DBG_vPrintf(1, "\n\n Gate Way Cache All Cnt = %d \n\n", GateWayCacheAllCnt);
	
	if(GateWayCacheAllCnt != 0)// still have data
	{
		RequestNextCacheNum++;				
		App_AskOneOfCache();// get next
		RecordRequestNextCacheNum = RequestNextCacheNum;

		requestCacheStatus = E_WaitOneCacheDataResponse;//keep request one cache from gateway
		ZTIMER_eStop(u8TimerCheckGetCacheKeepRun);
		ZTIMER_eStart(u8TimerCheckGetCacheKeepRun, ZTIMER_TIME_SEC(5));
	}
	else // no data
	{
		WhetherAskCacheAllCnt = 0;// moon 1->0
		App_PollRateCnt = 6;//moon
		requestCacheStatus = E_NoWaitAnything;// reset get cache statu
		ZTIMER_eStop(u8TimerCheckGetCacheKeepRun);
		ZTIMER_eStart(u8TimerCheckGetCacheKeepRun, ZTIMER_TIME_SEC(3));//moon

		//App_AskNumberOfCache();// ask against, confirm there is no data in gateway, moon
	}

}

void APP_cbTimerCheckGetCacheKeepRun(void * pvParam)
{
	DBG_vPrintf(1, "\n CheckGetCacheKeepRun statue=%d, times=%d \n", requestCacheStatus, GateWayCacheNoPushAheadTimes);
	if(bBDBJoinFailed == 1)
	{
		requestCacheStatus = E_NoWaitAnything;// reset get cache statu
		DBG_vPrintf(1, "\n bBDBJoinFailed=%d", bBDBJoinFailed);
	}

	switch(requestCacheStatus)
	{
		case E_WaitCacheAllCountResponse:
			if(GetPollControlResponse == 0)// still no get cache num!!
			{		
				GateWayCacheNoPushAheadTimes++;
				if(GateWayCacheNoPushAheadTimes > 2)
				{
					DBG_vPrintf(1, "\n No Gate Way Cache All Cnt Time Out=%d \n", GateWayCacheNoPushAheadTimes);
					requestCacheStatus = E_NoWaitAnything;// reset get cache statu
					WhetherAskCacheAllCnt = 1;// wake and enable request get cache cnt
					GetPollControlResponse = 0;
					GateWayCacheNoPushAheadTimes = 0;// reset retry times
					GateWayCacheAllCnt = 0;
					ZTIMER_eStop(u8TimerCheckGetCacheKeepRun);

					ZTIMER_eStart(u8TimerBdbRejoin, ZTIMER_TIME_MSEC(100));// if rejoin success I will App_AskNumberOfCache againt, if not stop and sleep later
				}
				else
				{
					DBG_vPrintf(1, "\n Try Gate Way Cache All Cnt=%d \n", GateWayCacheNoPushAheadTimes);
					App_AskNumberOfCache();// try get cache all cnt againt
					
					ZTIMER_eStart(u8TimerCheckGetCacheKeepRun, ZTIMER_TIME_SEC(5));
					
					vAppSampleDoorLockPollChange(20);
				}
			}
			break;

		case E_WaitOneCacheDataResponse://keep request one cache from gateway
			if(RecordRequestNextCacheNum == RequestNextCacheNum)
			{
				GateWayCacheNoPushAheadTimes++;
				if(GateWayCacheNoPushAheadTimes > 2)
				{
					DBG_vPrintf(1, "\n No Get One Cache Time Out=%d \n", GateWayCacheNoPushAheadTimes);
					requestCacheStatus = E_NoWaitAnything;// reset get cache statu
					WhetherAskCacheAllCnt = 1;
					GateWayCacheNoPushAheadTimes = 0;// reset retry times
					ZTIMER_eStop(u8TimerCheckGetCacheKeepRun);

					if(GateWayCacheAllCnt != 0)
					{
						DBG_vPrintf(1, "But GateWayCacheAllCnt!=0 Ask Laster Time \n");
						App_AskNumberOfCache();// try get cache all cnt againt
						
						vAppSampleDoorLockPollChange(20);
					}
				}
				else
				{
					DBG_vPrintf(1, "\n Try Get One Cache=%d \n", GateWayCacheNoPushAheadTimes);
					App_AskOneOfCache();// try get one cache againt
					
					ZTIMER_eStart(u8TimerCheckGetCacheKeepRun, ZTIMER_TIME_SEC(5));
					
					vAppSampleDoorLockPollChange(20);
				}
			}
			break;

		default:
			requestCacheStatus = E_NoWaitAnything;// reset get cache statu
			WhetherAskCacheAllCnt = 1;
			GateWayCacheAllCnt = 0;
			ZTIMER_eStop(u8TimerCheckGetCacheKeepRun);
			DBG_vPrintf(1, "\n stop Keep Run");
			break;
	}
}

void APP_cbTimerRetryTimeSynchronization(void * pvParam)
{	
	u8RetryTimeSynchronizationTimes++;
	DBG_vPrintf(1, "\n RetryTimeSynchronization=%d", u8RetryTimeSynchronizationTimes);
	if(u8RetryTimeSynchronizationTimes > 3)
	{
		u8RetryTimeSynchronizationTimes = 0;
		DBG_vPrintf(1, "\n RetryTimeSynchronization Timeout");
	}
	else
	{
		vSendCurrentTime();
		ZTIMER_eStart(u8TimerRetryTimeSynchronization, ZTIMER_TIME_SEC(3));//moon
		vAppSampleDoorLockPollChange(15);
	}
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
