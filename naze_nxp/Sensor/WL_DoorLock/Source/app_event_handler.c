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
#include "pwrm.h"
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
ts_SerialCache tsSerialBuffer;
extern uint8 u8JoinNetworkFlowPath;
extern uint8 RequestNextCacheNum;
uint8 HeartBeatWakeUpFlag = 0;//����������1�����ߺ���0
uint8 BatteryAlarmWHeartBeatFlag = 0;//�������Ѳ��ҵ͵�ѹ�����ߺ���0
uint8 crtreasknum = 0;//δ�յ�response�ط�
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vDioEventHandler(te_TransitionCode eTransitionCode);
PRIVATE void vEventStartFindAndBind(void);
PRIVATE void vStartPersistantPolling(void);
#if 0
PRIVATE void vStopPersistantPolling(void);
#endif
PUBLIC void vAppSerialCacheArea_Init(void);

//PRIVATE void vAppProcessSerialCacheShift(void);
PUBLIC bool vAppDecideSerialCacheAreaFull(void);
PUBLIC bool App_SampleDoorLockWakeUpTimeOutDecide(void);

PRIVATE void vAPP_NWKStartJoinEntry(void);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern const uint8 u8MyEndpoint;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
extern uint8 WhetherAskCacheAllCnt;
extern uint8 GateWayCacheAllCnt;
extern uint8 GetPollControlResponse;
extern uint8 ContrlReAckGetWay;
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

		DBG_vPrintf(TRACE_EVENT_HANDLER, "\n SW3_PRESSED %d ",PWRM_u16GetActivityCount());
		PWRM_eFinishActivity();

	//	vStopPersistantPolling();
        break;

#if defined(APP_NTAG)
    case FD_PRESSED:
    case FD_RELEASED:
        APP_vNtagStart(NFC_NWK_NSC_DEVICE_CLIMATE_SENSOR_DEVICE);
        break;
#endif

    case SW4_PRESSED:
	//	DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP: Entering DIO WakeUp %d \n",u8RejoinCycles);

		if (u8RejoinCycles != 0) //Ricky �����󴥷�����
		{
			DBG_vPrintf(1, "\n SW4_PRESSED ZTIMER_eStart(u8TimerBdbRejoin) ");
			u8RejoinCycles = 0;			
			ZTIMER_eStart(u8TimerBdbRejoin, ZTIMER_TIME_MSEC(100));
		}

		if (TRUE == APP_bNodeIsInRunningState())
		{
			vAppSampleDoorLockPollChange(16);
			if(WhetherAskCacheAllCnt == 1)
			{
				ContrlReAckGetWay = 0;
				App_AskNumberOfCache();
				WhetherAskCacheAllCnt = App_PollRateCnt - 5;
				WhetherAskCacheAllCnt = 0;
			}
		}
		else
		{
			vAppSampleDoorLockPollChange(16);
		}

		//vStartPollTimer(POLL_TIME);
		vStartPollTimer(500);
        break;

	case SW5_PRESSED:

		// Ricky
	#if 0
		vAPP_NWKStartJoinEntry();
	#else
		ZllCommissonCommandScanSend();
		PWRM_eFinishActivity();	// Ricky

		vAppSampleDoorLockPollChange(2);
		vStartPollTimer(ZTIMER_TIME_MSEC(500));

	//	DBG_vPrintf(TRACE_EVENT_HANDLER, "\n System Reboot !");
	//	vAHI_SwReset();
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
#ifndef SB
//	#define SB
#endif
PUBLIC void vAppHandleAppEvent(APP_tsEvent sButton)
{
	//static uint16 u8OtaCount = 0;
	te_TransitionCode eTransitionCode=NUMBER_OF_TRANSITION_CODE;

//	DBG_vPrintf(TRACE_EVENT_HANDLER, "\nAPP Process Buttons: sButton.eType = %d",sButton.eType);

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

        eTransitionCode = BUTTON_RELEASED_OFFSET | sButton.uEvent.sButton.u8Button;
        vDioEventHandler(eTransitionCode);
        break;

    case APP_E_EVENT_WAKE_TIMER:

        //DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP_E_EVENT_WAKE_TIMER ");
        vHandleWakeTimeoutEvent();
        break;

    case APP_E_EVENT_SEND_REPORT:
    //    DBG_vPrintf(TRACE_EVENT_HANDLER, "\n APP_E_EVENT_SEND_REPORT ");
	//	vSendImmediateReport();

        break;

    case APP_E_EVENT_PERIODIC_REPORT:
        DBG_vPrintf(TRACE_EVENT_HANDLER, "\n 30min time on ");
        WhetherAskCacheAllCnt = 0;
        HeartBeatWakeUpFlag = 1;
		HeartBeatCount++;

		if (u8RejoinCycles != 0) //Ricky �����󴥷�����
		{
		
        	DBG_vPrintf(TRACE_EVENT_HANDLER, "\n HeartBeatCount : %d u8RejoinCycles : %d ",HeartBeatCount,u8RejoinCycles);
		
		#ifdef SB
			if (((HeartBeatCount%360) == 0) || (u8RejoinCycles == 1))
		#else
			if (((HeartBeatCount%8) == 0) || (u8RejoinCycles == 1))
		#endif
			{
			#if WL_DOORLOCK
				u8RejoinCycles = 0;
				ZTIMER_eStart(u8TimerBdbRejoin, ZTIMER_TIME_MSEC(100));
			#endif
			}
		}
		else
		{

		#if 0 //def SB
			vAppSampleDoorLockPollChange(2);
			vStartPollTimer(100);
		#else
			vAppSampleDoorLockPollChange(10);
			vStartPollTimer(500);
		#endif
		}

		if (TRUE == APP_bNodeIsInRunningState())
		{
		#ifdef YYH_CUSTOM
			vSendReportPowerConfig();
		#else
		
			sSensor.sDoorLockServerCluster.eLockState= 2;
		
		#ifdef SB
			if ((HeartBeatCount%360) == 0)
		#endif
			App_SendReportDoorLockState();

#if 0
			if( GateWayCacheAllCnt <= 0 )
			{
				App_AskNumberOfCache();
			}
#else
			ContrlReAckGetWay = 0;
			App_AskNumberOfCache();
#endif
			vAppSampleDoorLockPollChange(10);
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
	uint8 u8DecideFlg = 0, u8ret = 0, u8WhetherChangPollcnt = 1;

	if (RxSerialMsg[0] == 0)
		return;

	if (RxSerialMsg[7] == 0)
	{
		APP_SerialSendAcknowlegement(RxSerialMsg[2], 0);//moon
		
		DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent : %0x ",RxSerialMsg[2]);
#if 1
		if (TRUE == APP_bNodeIsInRunningState())
		{
			if (TRUE == App_SampleDoorLockProcessResendPemit(RxSerialMsg[2]))
			{
				ZTIMER_eStart(u8TimerStartResend,ZTIMER_TIME_SEC(3));//App_cbTimerStartResend
			}

		}
#endif
		switch(RxSerialMsg[2])
		{
			case TransgressAlarm :	// ���

				u8DecideFlg = 1;
				App_SampleDoorLockSendAlarmCmd(0x33);		
			//	DBG_vPrintf(TRACE_EVENT_HANDLER, "\n TransgressAlarm  ");
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

				App_SampleDoorLockIASWDStartWaring(RxSerialMsg[8],((uint16)(RxSerialMsg[10] << 8) | (uint16)(RxSerialMsg[9])),TRUE);
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
				APP_AskDoorLockVerInfo();
				APP_SerialSendModelVersion();
				break;
				
		#ifdef SKYWORTH
			case CacheRequest:

				sSensor.sPowerConfigServerCluster.zbSpecialHeartBeat = 1;
				vSendSpecialHeartBeatReport();
				break;
		#endif
			case EnableDoorLock:

				break;
			case BatteryAlarm :	// ���
#if 0
				BatteryAlarmWHeartBeatFlag = 1;
#else
				if(HeartBeatWakeUpFlag == 1)
				{
					//HeartBeatWakeUpFlag = 0;
					BatteryAlarmWHeartBeatFlag = 1;
#if 0
					sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = 1;
					sSensor.sPowerConfigServerCluster.u8BatteryPercentageRemaining = 40;
				#ifdef SKYWORTH
					vSendPowerConfigReport();
				#else
					vSendReportPowerConfig();
				#endif
#endif
					App_AskNumberOfCache();
					//APP_CheckGateWayCacheAllCnt();
				}else
				{
#if 1
				sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = 1;
				sSensor.sPowerConfigServerCluster.u8BatteryPercentageRemaining = 40;
			#ifdef SKYWORTH
				vSendPowerConfigReport();
			#else
				vSendReportPowerConfig();
			#endif
#endif
				}
#endif
				break;

			case JoinNWK :	// ���

			//	DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent start join network ");
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

			case LeaveNWK :	// ��������ʱ

				u8DecideFlg = 2;
		//		DBG_vPrintf(TRACE_EVENT_HANDLER, "\n vAppHandleSerialEvent start leave network ");

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
				}

				break;

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

			//		sSensor.sBasicServerCluster.bDeviceEnabled = 1;
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

//				u8DecideFlg = 1;
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n sSensor.sTimeClientCluster.utctTime %d ",sSensor.sTimeClientCluster.utctTime);
				vSendCurrentTime();

				break;

			case ReviceIEEE:

				App_SampleDoorLockIEEEReadResponse(RxSerialMsg+8,RxSerialMsg[1]);
				break;

			case SleepInTime:
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n Command Sleep In Time ! ");
				
				if ((FALSE == APP_bNodeIsInRunningState()) || (0 != u8RejoinCycles) || u8JoinNetworkFlowPath)
				{
					App_AckSleepInTime(0x01);// faile
				}
				else
				{
					App_AckSleepInTime(0x00);// success 
					if(App_SampleDoorLockWakeUpTimeOutDecide)
					{
						u8WakeUpFlg = 0;//App_cbTimerWakeUpTimeOut();// if 0x77 still running, stop it. 
						ZTIMER_eStop(u8TimerWakeUpTimeOut);						
						ZTIMER_eStop(u8TimerRemoteUnlockPemit);
					}
					vAppSampleDoorLockPollChange(4);
					u8WhetherChangPollcnt = 0;//do not change poll cnt
				}
				break;

			case DoorKeyPadLocked:// suning huitailong
				App_SampleDoorLockResetAlarmCode(0x0D);
				break;
				
			case DoorOtherIllgalAlarmReport:// suning huitailong
				App_SampleDoorLockResetAlarmCode(0x0D + RxSerialMsg[8]);
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
		
			default :
				DBG_vPrintf(TRACE_EVENT_HANDLER, "\n Command unsupport ! ");
				break;

		}
	}
	//else if((RxSerialMsg[1] == 0x01) && (RxSerialMsg[7] == 1))
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
						App_SampleDoorLockDisableInterrupt();
						ZTIMER_eStop(u8TimerLeaveInd);					
						ZTIMER_eStart(u8TimerLeaveInd,ZTIMER_TIME_MSEC(500));
					}
				}
			break;

			case NwkIndication :
				if(RxSerialMsg[8] == 0x00)
				{
					u8DecideFlg = 1;
				}
			break;
			case PwdOperation:
				u8WhetherChangPollcnt = 0;//do not change poll cnt
				memcpy(sSensor.sBasicServerCluster.au8Transport,RxSerialMsg,RxSerialMsg[1]+8);
				sSensor.sBasicServerCluster.sTransport.u8Length = RxSerialMsg[1]+8;
				sSensor.sBasicServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
				vSendBasicReport();

				//App_SendReportPollControlSendCacheAck();
				//APP_CheckGateWayCacheAllCnt();
				//WhetherAskCacheAllCnt = 1;
				ZTIMER_eStart(u8DelayAskOneOfNumGetWay,ZTIMER_TIME_MSEC(100));
				DBG_vPrintf(1,"\n come to PwdOperation\n");
				//ZTIMER_eStart(u8AskNumGetWay,ZTIMER_TIME_MSEC(2000));
				break;
			case EnableDoorLock:
				u8WhetherChangPollcnt = 0;//do not change poll cnt
				memcpy(sSensor.sBasicServerCluster.au8Transport,RxSerialMsg,RxSerialMsg[1]+8);
				sSensor.sBasicServerCluster.sTransport.u8Length = RxSerialMsg[1]+8;
				sSensor.sBasicServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
				vSendBasicReport();
				//APP_CheckGateWayCacheAllCnt();
				ZTIMER_eStart(u8DelayAskOneOfNumGetWay,ZTIMER_TIME_MSEC(100));
				DBG_vPrintf(1,"\n come to EnableDoorLock\n");
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
				
				vSendDoorLockOtherInfoReport();// report to gateway
				u8WhetherChangPollcnt = 0;//do not change poll cnt
				break;
				
			case ModelVersionReply:
			default:				
				DBG_vPrintf(1, "\nUnsupport Command ID %02x", RxSerialMsg[2]);
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
//    DBG_vPrintf(TRACE_EVENT_HANDLER,"\nAPP Process Buttons: eEZ_FindAndBind");
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
//    DBG_vPrintf(TRACE_EVENT_HANDLER,"\nAPP Process Buttons: Exit Easy Mode");
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
#if 0
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
	u8JoinNWKCycle++;
    DBG_vPrintf(TRACE_EVENT_HANDLER,"\n APP_cbStartNWKJoin %d",u8JoinNWKCycle);

	if (TRUE == APP_bNodeIsInRunningState())
	{
		return ;
	}

	vAPP_NWKStartJoinEntry();

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

PUBLIC void App_EventStartFindAndBind(void)
{
	vEventStartFindAndBind();

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

    ZPS_eAplZdoPoll();
    DBG_vPrintf(1, "\n APP Report: App_SendReportPollControlGetCache ");

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

    DBG_vPrintf(TRUE, "\n APP Report2: Error Sending Report %d",u8ret);

    /* free buffer and return*/
	PDUM_eAPduFreeAPduInstance(myPDUM_thAPduInstance);
    DBG_vPrintf(TRUE, "\n\n APP Report: Report Sent Success \n\n");

}

PUBLIC void App_AskNumberOfCache(void)
{
    DBG_vPrintf(TRUE, "\n\n App Ask Number Of Cache \n\n");
	#if 0
	if(GateWayCacheAllCnt != 0)
	{		
		DBG_vPrintf(TRUE, " But Return\n");
		return;
	}
	#endif
	
	GateWayCacheAllCnt = 0;
	//ContrlReAckGetWay = 0;
	crtreasknum = 1;//if ask gatway cache total num not response,ask gatway cache total num again

	sSensor.sPollControlServerCluster.u8GetCacheFromGW = 0;//u8GetCacheFromGW all count
	App_SendReportPollControlGetCache();
}


PUBLIC void App_AskOneOfCache(void)
{
	DBG_vPrintf(TRUE, "\n\n App Ask One Of Cache 1/%d \n\n", GateWayCacheAllCnt);
	crtreasknum = 1;//if ask one of gatway cache not response,ask gatway cache total num again
	//sSensor.sPollControlServerCluster.u8GetCacheFromGW = 1;
	sSensor.sPollControlServerCluster.u8GetCacheFromGW = RequestNextCacheNum;
	App_SendReportPollControlGetCache();
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
