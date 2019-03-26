/*****************************************************************************
 *

 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          App_WL_DoorLock.c
 *
 * DESCRIPTION:        ZLO Demo Occupancy Sensor -Implementation
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
#include "zps_gen.h"
#include "app_blink_led.h"
#include "dbg.h"
#include <string.h>

#include "zcl_customcommand.h"
#include "zll_commission.h"

#include "bdb_api.h"
//#include "bdb_fb_api.h"
#include "bdb_start.h"

#include "PDM_IDs.h"
#include "pdm.h"

#include "Alarms.h"
#include "DoorLock.h"
#include "IASWD.h"
#include "app_zlo_sensor_node.h"
#include "app_nwk_event_handler.h"

#include "App_WL_DoorLock.h"
#include "app_SampleDoorLock_Uart.h"
#include "app_occupancy_buttons.h"
#include "app_ota_client.h"
#include "app_event_handler.h"

#ifndef DEBUG_DOORLOCK
    #define TRACE_DOORLOCK FALSE
#else
    #define TRACE_DOORLOCK TRUE
#endif


uint8 GetPollControlResponse = 0;
extern uint8 GateWayCacheAllCnt;
extern uint8 RequestNextCacheNum;
extern uint8 RecordRequestNextCacheNum;
extern uint8 RetryRequestNextCacheNum;
extern uint8 WhetherAskCacheAllCnt;
extern uint8 BatteryAlarmWHeartBeatFlag;
extern uint8 crtreasknum;
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define	BEGYEAR  2000     //  UTC started at 00:00:00 January 1, 2000
#define	DAY      86400UL  // 24 hours * 60 minutes * 60 seconds

#define	IsLeapYear(yr)	(!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))
#define	YearLength(yr)	((uint16)(IsLeapYear(yr) ? 366 : 365))

#define CmdPemitLengh	1

#define RH_DOORLOCKSTATUS		(1)
#define RH_DOORRING				(1 << 1)
#define RH_LOCALOPERATION		(1 << 2)
#define RH_VOLTAGEINFO			(1 << 3)
#define RH_BASIC_RESPON			(1 << 4)

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

const uint8 u8MyEndpoint = OCCUPANCYSENSOR_SENSOR_ENDPOINT;
PUBLIC tsZLO_OccupancySensorDevice sSensor;

PUBLIC bool bLeaveInd = 0;
PUBLIC uint8 u8JoinNWKCycle = 0;
PUBLIC uint16 u16PinData;
PUBLIC UTCTime u32UtcLocalTime;
PUBLIC teCLD_DoorLock_LockState eSampleDoorLock_LockState = 1;
PUBLIC bool bReadWriteAttrFlg = TRUE;

PUBLIC uint16 u16BasicAttrID = 0;

PRIVATE uint8 bAppRemoteUnlockPemit = 0;
PRIVATE uint8 u8AppRemotePinCode[10] = {0};

PRIVATE uint8 SampleDoorLockCmdPemit[CmdPemitLengh] = {0x2A};
#ifdef SKYWORTH
PRIVATE uint32 u32BindUnLockCnt;
#endif
PRIVATE uint32 u32LinkageUnLockCnt = 0x85;

PRIVATE te_SampleDoorLockResendHandle u32SampleDoorLockResend;
PRIVATE uint8 u8ResendCount = 0;
PRIVATE uint8 u8ResendID = 0;

#ifdef SKYWORTH
PRIVATE void App_DB_ConfigCacheNumber(uint8 u8TransportCnt);
#endif
//PRIVATE te_IASWaringDevicePayload eWaringDevicePayload;

/* define the default reports */
tsReports asDefaultReports[ZCL_NUMBER_OF_REPORTS] = \
{\
    {MEASUREMENT_AND_SENSING_CLUSTER_ID_OCCUPANCY_SENSING,{0, E_ZCL_BMAP8, E_CLD_OS_ATTR_ID_OCCUPANCY, ZLO_MIN_REPORT_INTERVAL, ZLO_MAX_REPORT_INTERVAL, 0, {0}}},\
};
#if 1
tsCLD_ZllDeviceTable sDeviceTable = { ZLO_NUMBER_DEVICES,
                                      {
                                          { 0,
                                            HA_PROFILE_ID,
                                            0x000A,// DoorLock,
                                            OCCUPANCYSENSOR_SENSOR_ENDPOINT,
                                            1,//APPLICATION_DEVICE_VERSION,
                                            0,
                                            0}
                                      }
};
#endif
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC bool App_SampleDoorLockSetPinCode(bool bStatus);
PUBLIC void App_SampleDoorLockLinkageCountIncrement(void);
PUBLIC void App_SampleDoorLockDisableInterrupt(void);

PUBLIC void App_SampleDoorLockCommandResend(uint16 u16ClusterId);
PUBLIC void App_SampleDoorLockCommandHandle(uint8 *sPayLoad);

PUBLIC void App_SampleDoorLockLinkageCountInit(void);

PUBLIC void App_SampleDoorLockIASWDStartWaring(uint8 u8Level,uint16 u16Duration,bool u8ResendID);
PUBLIC uint8 App_SampleDoorLockWaringDeviceIDChange(bool u8IDs);
PUBLIC void App_SampleDoorLockIEEEReadResponse(uint8 *sCode,uint8 u8Len);

PUBLIC void App_SampleDoorLockVoltageReportProcess(void);

PUBLIC void App_cbTimerStartResend( void * pvParam);
PUBLIC void APP_cbTimerZCLBasicWriteAttr( void * pvParam);
PUBLIC void APP_cbGetNumGetWay(void * pvParam);
PUBLIC void APP_cbDelayAskOneOfNumGetWay(void * pvParam);

#ifdef SKYWORTH
PUBLIC bool App_SampleDoorLockBindUnlockHandle(const uint8 *sCode,uint8 *sPinCode);
#endif
/****************************************************************************
 *
 * NAME: eApp_HA_RegisterEndpoint
 *
 * DESCRIPTION:
 * Register ZLO endpoints
 *
 * PARAMETER
 * Type                        Name                  Descirption
 * tfpZCL_ZCLCallBackFunction  fptr                  Pointer to ZCL Callback function
 *
 * RETURNS:
 * teZCL_Status
 *
 ****************************************************************************/
PUBLIC teZCL_Status eApp_ZLO_RegisterEndpoint(tfpZCL_ZCLCallBackFunction fptr)
{

	return eZLO_RegisterOccupancySensorEndPoint(OCCUPANCYSENSOR_SENSOR_ENDPOINT,fptr,&sSensor);

//	return eHA_RegisterDoorLockEndPoint(OCCUPANCYSENSOR_SENSOR_ENDPOINT,fptr,&sSensor);
}

/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_Init
 *
 * DESCRIPTION:
 * ZCL Device Specific initialization
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *	锟斤拷写Basic 锟斤拷锟斤拷锟较�/Ricky
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_Init(void)
{

    /* Initialise the strings in Basic */
#ifdef YYH_CUSTOM		// 锟斤拷源锟斤拷
    memcpy(sSensor.sBasicServerCluster.au8ManufacturerName, "FULL Enterprise Corp", CLD_BAS_MANUF_NAME_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FZ-DL5083", CLD_BAS_MODEL_ID_SIZE);
#else
	memcpy(sSensor.sBasicServerCluster.au8ManufacturerName, "FBee", CLD_BAS_MANUF_NAME_SIZE);

	#ifdef SKYWORTH		// 锟斤拷维
		memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FNB56-DOR10CW1.4", CLD_BAS_MODEL_ID_SIZE);
	#else
		#ifdef HUTLON	// 锟斤拷泰锟斤拷
			#ifdef DOOR_HT_FOR_HM // HTL for Haiman 
				memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FNB56-DOR12HM1.0", CLD_BAS_MODEL_ID_SIZE);
			#else      // Hai beishi
				//memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FNB56-DOR11HT1.7", CLD_BAS_MODEL_ID_SIZE);
				//memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FNB56-DOR16HB1.4", CLD_BAS_MODEL_ID_SIZE);
				//memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FNB56-DOR01SN1.1", CLD_BAS_MODEL_ID_SIZE);//
				memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FNB56-DOR21FB1.7", CLD_BAS_MODEL_ID_SIZE);
				//memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FNB56-DOR14KE1.4", CLD_BAS_MODEL_ID_SIZE);
			#endif
		#else			// 锟斤拷锟斤拷
			memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FNB56-DOR07WL2T3", CLD_BAS_MODEL_ID_SIZE);	// V0.5
		#endif
	#endif
#endif

	memcpy(sSensor.sBasicServerCluster.au8DateCode, "20170808", CLD_BAS_DATE_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8SWBuildID, "0000-0004", CLD_BAS_SW_BUILD_SIZE);
	memcpy(sSensor.sBasicServerCluster.au8ProductURL, "feibit.com", CLD_BAS_URL_SIZE);

#if 0
    uint16 u16ByteRead;
    if (PDM_E_STATUS_OK == PDM_eReadDataFromRecord(PDM_ID_APP_PRODUCTCODE,u8ProductCode,16,&u16ByteRead))
    {
		memcpy(sSensor.sBasicServerCluster.au8ProductCode, u8ProductCode, CLD_BAS_PCODE_SIZE);
    }
#endif

    sSensor.sBasicServerCluster.eGenericDeviceType = E_CLD_BAS_GENERIC_DEVICE_TYPE_UNSPECIFIED;

    /* Initialise the strings in Occupancy Cluster */
    sSensor.sOccupancySensingServerCluster.eOccupancySensorType = E_CLD_OS_SENSORT_TYPE_PIR;
    sSensor.sOccupancySensingServerCluster.u8Occupancy = 0;

#ifdef PIR_TYPE_PWM
    sSensor.sOccupancySensingServerCluster.u8PIRUnoccupiedToOccupiedThreshold = APP_OCCUPANCY_SENSOR_TRIGGER_THRESHOLD;
    sSensor.sOccupancySensingServerCluster.u8PIRUnoccupiedToOccupiedDelay = APP_OCCUPANCY_SENSOR_UNOCCUPIED_TO_OCCUPIED_DELAY;
#endif
    sSensor.sOccupancySensingServerCluster.u16PIROccupiedToUnoccupiedDelay = APP_OCCUPANCY_SENSOR_OCCUPIED_TO_UNOCCUPIED_DELAY;
}

/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_SetIdentifyTime
 *
 * DESCRIPTION:
 * ZCL Device Specific setting of identify time
 *
 * PARAMETER:
 * uint16 u16Time Identify time duration
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_SetIdentifyTime(uint16 u16Time)
{
    sSensor.sIdentifyServerCluster.u16IdentifyTime=u16Time;
}
/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_UpdateIdentify
 *
 * DESCRIPTION:
 * ZCL Device Specific Identify Updates
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_UpdateIdentify(void)
{
//	APP_vSetLED(LED2, sSensor.sIdentifyServerCluster.u16IdentifyTime%2);
}
/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_IdentifyOff
 *
 * DESCRIPTION:
 * ZCL Device Specific stop identify
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_IdentifyOff(void)
{
    vAPP_ZCL_DeviceSpecific_SetIdentifyTime(0);
    APP_vSetLED(LED2, 0);
}

/****************************************************************************
 *
 * NAME: app_u8GetDeviceEndpoint
 *
 * DESCRIPTION:
 * Returns the application endpoint
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC uint8 app_u8GetDeviceEndpoint( void)
{
    return OCCUPANCYSENSOR_SENSOR_ENDPOINT;
}

PUBLIC uint8 App_SampleDoorLockWaringDeviceIDChange(bool u8IDs)
{
	if (u8IDs)
	{
		return ++u8ResendID;
	}
	else
		return u8ResendID;
}

PUBLIC void App_SampleDoorLockLinkageCountInit(void)
{
	uint16 u16ByteRead;

	PDM_eReadDataFromRecord(PDM_ID_APP_BIND_UNLOCKCUT,&u32LinkageUnLockCnt,
                            sizeof(uint32),&u16ByteRead);

	if (u32LinkageUnLockCnt < 0x85)
		u32LinkageUnLockCnt |= 0x85;

	u32LinkageUnLockCnt += 512;

}
PUBLIC void App_SampleDoorLockLinkageCountIncrement(void)
{
	u32LinkageUnLockCnt++;
	PDM_eSaveRecordData(PDM_ID_APP_BIND_UNLOCKCUT,&u32LinkageUnLockCnt,sizeof(uint32));
}

PUBLIC void App_SampleDoorLockLinkageCountReset(void)
{
	u32LinkageUnLockCnt = 0;
	PDM_eSaveRecordData(PDM_ID_APP_BIND_UNLOCKCUT,&u32LinkageUnLockCnt,sizeof(uint32));
}

/*********************************************************************
 * @fn      monthLength
 *
 * @param   lpyr - 1 for leap year, 0 if not
 *
 * @param   mon - 0 - 11 (jan - dec)
 *
 * @return  number of days in specified month
 */
static uint8 monthLength( uint8 lpyr, uint8 mon )
{
  uint8 days = 31;

	if ( mon == 1 ) // feb
  {
		days = ( 28 + lpyr );
  }
  else
  {
    if ( mon > 6 ) // aug-dec
    {
      mon--;
    }

    if ( mon & 1 )
    {
      days = 30;
    }
  }

	return ( days );
}

/*********************************************************************
 * @fn      App_ConvertUTCTime
 *
 * @brief   Converts UTCTime to UTCTimeStruct
 *
 * @param   tm - pointer to breakdown struct
 *
 * @param   secTime - number of seconds since 0 hrs, 0 minutes,
 *          0 seconds, on the 1st of January 2000 UTC
 *
 * @return  none
 */
PUBLIC void App_ConvertUTCTime(UTCTimeStruct *tm, UTCTime secTime)
{
//	calculate the time less than a day - hours, minutes, seconds

	uint32 day = secTime % DAY;
	tm->seconds = day % 60UL;
	tm->minutes = (day % 3600UL) / 60UL;
	tm->hour = day / 3600UL;

//	Fill in the calendar - day, month, year
	uint16 numDays = secTime / DAY;
	tm->year = BEGYEAR;

	while ( numDays >= YearLength( tm->year ) )
	{
		numDays -= YearLength( tm->year );
		tm->year++;
	}
	
	tm->month = 0;

	while ( numDays >= monthLength( IsLeapYear( tm->year ), tm->month ) )
	{
		numDays -= monthLength( IsLeapYear( tm->year ), tm->month );
		tm->month++;
	}
	
	tm->day = numDays;

}

PUBLIC uint8 App_SampleDoorLockSendAlarmCmd(uint8 u8AlarmCode)
{
    tsZCL_Address sDestinationAddress;
	uint8 u8Seq = u8GetTransactionSequenceNumber();
	uint8 u8ret = 0;

	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SampleDoorLockSendAlarmCmd \n");
	/* Set the address mode to send to all bound device and don't wait for an ACK*/
	sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;

	u8ret = eCLD_AlarmsSignalAlarm(OCCUPANCYSENSOR_SENSOR_ENDPOINT,REPORT_EP,
						&sDestinationAddress,&u8Seq,u8AlarmCode, CLOSURE_CLUSTER_ID_DOOR_LOCK);

	DBG_vPrintf(1, "\n u8ret %0x ",u8ret);

	return u8ret;
}

PUBLIC uint8 App_SampleDoorLockResetAlarmCode(uint8 u8AlarmCode)
{
	tsZCL_Address sDestinationAddress;
	tsCLD_AlarmsResetAlarmCommandPayload sPayload;
	uint8 u8ret = 0,u8Seq = u8GetTransactionSequenceNumber();

	/* Set the address mode to send to all bound device and don't wait for an ACK*/
	sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;

	sPayload.u8AlarmCode = u8AlarmCode;
	sPayload.u16ClusterId = CLOSURE_CLUSTER_ID_DOOR_LOCK;

	u8ret = eCLD_AlarmsCommandResetAlarmCommandSend(OCCUPANCYSENSOR_SENSOR_ENDPOINT,REPORT_EP,
									&sDestinationAddress,&u8Seq,&sPayload);

	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SampleDoorLockResetAlarmCode u8ret %d \n",u8ret);
	return u8ret;
}

PUBLIC uint8 App_SerialRemoteOperationHandle(uint8 u8OptState)
{
//	teCLD_DoorLock_LockState 
	uint8 u8status = 0;
	tsZCL_Address sDestinationAddress;
	tsCLD_DoorLock_LockUnlockResponsePayload sRemoteOptResult;
	uint8 u8ret = 0,u8Seq = u8GetTransactionSequenceNumber();

	/* Set the address mode to send to all bound device and don't wait for an ACK*/
	sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;


	u16PinData = ((uint16)(RxSerialMsg[10] << 8) | (uint16)(RxSerialMsg[9]));
	
	if (u8OptState == 0x00)
	{
		u8status |= 2;	// 锟较憋拷锟斤拷锟斤拷锟斤拷录 
		eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_UNLOCKED;
		sRemoteOptResult.eStatus = 0;
	}
	else if (u8OptState == 0x01)
	{
		u8status |= 1;	// 锟斤拷睡锟斤拷 
		eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_NOT_FULLY_LOCKED;
		sRemoteOptResult.eStatus = 1;
	}
	else if (u8OptState == 0x02)
	{
		eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_NOT_FULLY_LOCKED;
		sRemoteOptResult.eStatus= 0x7F;
	}

	u8ret = eCLD_DoorLockCommandLockUnlockResponseSend(OCCUPANCYSENSOR_SENSOR_ENDPOINT, REPORT_EP,
									&sDestinationAddress, &u8Seq,E_CLD_DOOR_LOCK_CMD_UNLOCK,// eSampleDoorLock_LockState,// ,
									&sRemoteOptResult );

	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SerialRemoteOperationHandle u8ret %d u8status %d\n",u8ret,u8status);

	return u8status;
}

PUBLIC void App_SendOperationEventNotification(uint8 u8Sourc,uint8 u8LockState,uint8 u8Data,
																	uint16 u16Uid,uint32 u32UtcTime)
{

	bReadWriteAttrFlg = FALSE;

	tsZCL_Address sDestinationAddress;
	zclDoorLockOperationEventNotification_t sDoorLockOptEvtNotifyPayload;
	uint8 u8ret, nBuf[2];

	/* Set the address mode to send to all bound device and don't wait for an ACK*/
	sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;

	nBuf[0]=0x01;
	nBuf[1]=u8Data;

	sDoorLockOptEvtNotifyPayload.operationEventSource = u8Sourc;
	sDoorLockOptEvtNotifyPayload.operationEventCode = u8LockState;
	sDoorLockOptEvtNotifyPayload.userID = u16Uid;
	sDoorLockOptEvtNotifyPayload.pin = 0;
	sDoorLockOptEvtNotifyPayload.zigBeeLocalTime = u32UtcTime;	//todo add Time
	sDoorLockOptEvtNotifyPayload.pData = (void *)nBuf;


	u8ret = eCLD_DoorLockCommandDoorLockOperationEventNotification(
										OCCUPANCYSENSOR_SENSOR_ENDPOINT,
										&sDestinationAddress,
										REPORT_EP,&sDoorLockOptEvtNotifyPayload);

	
	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SendOperationEventNotification u8ret %d \n",u8ret);
	
}

PUBLIC void App_SampleDoorLockAnalyzeUnlockRecord(void)
{

	eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_UNLOCKED;
	u16PinData = ((uint16)(RxSerialMsg[9] << 8) | (uint16)(RxSerialMsg[8]));
	u32UtcLocalTime = (((uint32)RxSerialMsg[17]<<24)|((uint32)RxSerialMsg[16]<<16)|((uint32)RxSerialMsg[15]<<8)|(uint32)RxSerialMsg[14]);

	if (RxSerialMsg[10] == 01)	//锟斤拷锟诫开锟斤拷
	{
		App_SendOperationEventNotification(OPTEVTSOURCE_KEYPAD,eSampleDoorLock_LockState,
											RxSerialMsg[13],u16PinData,u32UtcLocalTime);
	}
	else if (RxSerialMsg[10] == 02)	// 刷锟斤拷
	{
		App_SendOperationEventNotification(OPTEVTSOURCE_RFID,eSampleDoorLock_LockState,
											RxSerialMsg[13],u16PinData,u32UtcLocalTime);
	}
	else if (RxSerialMsg[10] == 03) // 指锟狡匡拷锟斤拷
	{
		App_SendOperationEventNotification(OPTEVTSOURCE_MANUAL,eSampleDoorLock_LockState,
											RxSerialMsg[13],u16PinData,u32UtcLocalTime);
	}
	else if (RxSerialMsg[10] == 04)	// 锟斤拷锟斤拷锟斤拷证
	{
		App_SendOperationEventNotification(OPTEVTSOURCE_MULTIPLEVALIDATION,eSampleDoorLock_LockState,
											RxSerialMsg[13],u16PinData,u32UtcLocalTime);
	}
	else
	{
		eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_LOCKED;
	}


}

PUBLIC void App_SampleDoorLockAnalyzeVoltageInformation(uint8 u8Remaining)
{

	//sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = 0;
	sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = ((u8Remaining < 0x28) ? 1 : 0);
	sSensor.sPowerConfigServerCluster.u8BatteryPercentageRemaining = u8Remaining;

#ifdef SKYWORTH
	vSendPowerConfigReport();
#else
	vSendReportPowerConfig();
#endif

}

PUBLIC void App_SendLoaclOperationLockReport(void)
{
	tsZCL_Address sDestinationAddress;
	tsCLD_DoorLock_LockUnlockResponsePayload sRemoteOptResult;
	uint8 u8ret = 0,u8Seq = u8GetTransactionSequenceNumber();

	/* Set the address mode to send to all bound device and don't wait for an ACK*/
	sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;

	eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_LOCKED;
	sRemoteOptResult.eStatus = 0;

	u8ret = eCLD_DoorLockCommandLockUnlockResponseSend(OCCUPANCYSENSOR_SENSOR_ENDPOINT, REPORT_EP,
									&sDestinationAddress, &u8Seq, E_CLD_DOOR_LOCK_CMD_LOCK,
									&sRemoteOptResult );

	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SendOperationEventNotification u8ret %d \n",u8ret);

}

PUBLIC void App_SampleDoorLockIEEEReadResponse(uint8 *sCode,uint8 u8Len)
{

#if TRACE_DOORLOCK
	uint8 i = 0;
	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SampleDoorLockIEEEReadResponse %d ",u8Len);

	while (i <= u8Len)
		DBG_vPrintf(TRACE_DOORLOCK, " %0x ",sCode[i++]);
	DBG_vPrintf(TRACE_DOORLOCK, "\n ");
#endif

	if ((u8Len < 2) || (u8Len > 29))
		return;

	// sProductCode
	memcpy(sSensor.sBasicServerCluster.au8ProductCode,sCode,u8Len);
	sSensor.sBasicServerCluster.sProductCode.u8Length = u8Len;
//	PDM_eSaveRecordData(PDM_ID_APP_PRODUCTCODE,sCode,u8Len);

	if ((TRUE == APP_bNodeIsInRunningState()) && (0 == u8RejoinCycles))
	{
		vSendBasicProductCodeReport();
	}

}

PUBLIC void App_SampleDoorLockProductCodeReport(void)
{
	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SampleDoorLockProductCodeReport \n");

#if TRACE_DOORLOCK
	uint8 i = 0;

	while (i < 16)
		DBG_vPrintf(TRACE_DOORLOCK, " %0x ",sSensor.sBasicServerCluster.au8ProductCode[i++]);
	DBG_vPrintf(TRACE_DOORLOCK, "\n ");
#endif

	if ((sSensor.sBasicServerCluster.au8ProductCode[0] != 0) && \
		(sSensor.sBasicServerCluster.au8ProductCode[1] != 0) )
	{
		vSendBasicProductCodeReport();
	}

}

PUBLIC void App_SampleDoorLockCommandHandle(uint8 *sPayLoad)
{
#ifdef SKYWORTH
	uint8 sPinCode[9] = {0};
#endif

#if TRACE_DOORLOCK
	uint8 i = 1;
	DBG_vPrintf(TRACE_DOORLOCK, "\n u8CommandId %d Len %d\n",sPayLoad[1],sPayLoad[2]);

	while (i < 11)
		DBG_vPrintf(TRACE_DOORLOCK, " %0x ",sPayLoad[i++]);
	DBG_vPrintf(TRACE_DOORLOCK, "\n ");
#endif

#if 1
	APP_ButtonsStopWakeUp();
#else
	App_StopSerialPrepareEnterSleep();
#endif

	switch(sPayLoad[1])
	{
		case E_CLD_DOOR_LOCK_CMD_LOCK: 
			break;
		case E_CLD_DOOR_LOCK_CMD_UNLOCK:

			if (sSensor.sBasicServerCluster.bDeviceEnabled)
			{
				DBG_vPrintf(TRACE_DOORLOCK, "\n bDeviceEnabled ");
			//	if (bAppRemoteUnlockPemit == 0)
				{
					DBG_vPrintf(TRACE_DOORLOCK, "\n Start 5s TimeOut ");
					ZTIMER_eStop(u8TimerRemoteUnlockPemit);
					ZTIMER_eStart(u8TimerRemoteUnlockPemit,ZTIMER_TIME_SEC(4));
				}
				App_SampleDoorLockRemoteUnlockPemit(TRUE);

				if (0 == memcmp(u8AppRemotePinCode,sPayLoad+2,sPayLoad[2]+1))
				{
					DBG_vPrintf(TRACE_DOORLOCK, "\n 0 == memcmp ");
					return;
				}
				memcpy(u8AppRemotePinCode,sPayLoad+2,sPayLoad[2]+1);
		#if TRACE_DOORLOCK
					i = 0;
					DBG_vPrintf(TRACE_DOORLOCK, "\n u8AppRemotePinCode ");

					while (i < sPayLoad[2]+1)
						DBG_vPrintf(TRACE_DOORLOCK, " %0x ",u8AppRemotePinCode[i++]);
					DBG_vPrintf(TRACE_DOORLOCK, "\n ");
		#endif

				App_SerialSendRemoteUnlock(u8AppRemotePinCode,RemoteUnlock);
			}

			break;

		case E_CLD_DOOR_LOCK_CMD_TOGGLE: 
			break;
		case E_CLD_DOOR_LOCK_CMD_UNLOCK_TIMEOUT:

			App_SerialSendRemoteUnlock(sPayLoad+2,NormallyOpen);
			break;

	#ifdef SKYWORTH
		case E_CLD_DOOR_LOCK_CMD_SET_PINCODE:

			if (App_SampleDoorLockBindUnlockHandle(sPayLoad+6,sPinCode))
		//	if (App_SampleDoorLockFristBindUnlockHandle(sPayLoad+2))
			{
				App_SerialSendRemoteBindUnlock(sPinCode);
			}
		//	else
			{
		//		ZTIMER_eStart(u8TimerRemoteBindUnlock,ZTIMER_TIME_SEC(8));
			}
			break;
	#endif

		case E_CLD_DOOR_LOCK_CMD_SET_YEAR_DAY_SCHEDULE:
			DBG_vPrintf(TRACE_DOORLOCK, "\n SET_YEAR_DAY_SCHEDULE");
			App_SendReportPollControlSendCacheAck();// send ack
			crtreasknum = 0;
			APP_SerialSendPwdOperationData(&sPayLoad[3], sPayLoad[2]);

			#if 1
			//APP_CheckGateWayCacheAllCnt();
			#else
			if(GateWayCacheAllCnt != 0)
				GateWayCacheAllCnt --;
			
			DBG_vPrintf(1, "\n\n Gate Way Cache All Cnt = %d \n\n", GateWayCacheAllCnt);

			if(GateWayCacheAllCnt != 0)
			{
				//RequestNextCacheNum++;
				DBG_vPrintf(1,"\n come to GateWayCacheAllCnt != 0\n");
				//App_AskOneOfCache();// get next
				RecordRequestNextCacheNum = RequestNextCacheNum;
				RetryRequestNextCacheNum = App_PollRateCnt - 16;// 5 second check
			}
			else
			{
				RecordRequestNextCacheNum = 0xFF;
				RetryRequestNextCacheNum = 0xFF;
				DBG_vPrintf(1,"\n come to GateWayCacheAllCnt == 0\n");
				WhetherAskCacheAllCnt = 1;
				vAppSampleDoorLockPollChange(6);
			}
			#endif
			break;
			
		default:
			break;
	}

}

PUBLIC void App_SampleDoorLockCommandResend(uint16 u16ClusterId)
{

	switch( u16ClusterId )
	{

		case 0x0001:

			u32SampleDoorLockResend.u32ResendEvents &= ~RH_VOLTAGEINFO;
			break;

		case 0x0502:
			DBG_vPrintf(TRACE_DOORLOCK, "\n IASWDStartWaring Clear Out \n");
			u32SampleDoorLockResend.u32ResendEvents &= ~RH_DOORRING;
			ZTIMER_eStop(u8TimerBdbRejoin);
			ZTIMER_eStop(u8TimerStartResend);

		case 0xFFFF:
		case 0x0000:
			DBG_vPrintf(TRACE_DOORLOCK, "\n RECV BASIC_DEFAULT_RESPONSE\n");
			u32SampleDoorLockResend.u32ResendEvents &= ~RH_BASIC_RESPON; //moons, recv basic default response, set to 0
			ZTIMER_eStop(u8TimerBdbRejoin);
			ZTIMER_eStop(u8TimerStartResend);
			break;

		default:
			break;
	}

}

#ifdef SKYWORTH
PUBLIC bool App_SampleDoorLockFristBindUnlockHandle(uint8 *sPincode)
{

	if (( sPincode[1] == 0x66) && (sPincode[2] == 0x66))
	{
		App_SerialSendFristBindUnLockNotification(sPincode+4);
		return 0;
	}
	return 1;
}

PUBLIC bool App_SampleDoorLockBindUnlockHandle(const uint8 *sCode,uint8 *sPinCode)
{
	uint8 i;
	uint32 u32Count;

	// 'octe' 6F 63 74 65
	u32Count = ((uint32)((sCode[1]^0x6F)<<24) | (uint32)((sCode[2]^0x63)<<16) | 
				(uint32)((sCode[3]^0x74)<<8) | (uint32)(sCode[4]^0x65));

	if ((u32Count == u32BindUnLockCnt) || (u32Count < 0x85))
		return 0;
	u32BindUnLockCnt = u32Count;

	// 53 6B 79 57 6F 72 74 68
	sPinCode[0] = 8;
	sPinCode[1] = ((uint8)((u32LinkageUnLockCnt<<24)&(0xFF)))^0x53;
	sPinCode[2] = ((uint8)((u32LinkageUnLockCnt<<16)&(0xFF)))^0x6B;
	sPinCode[3] = ((uint8)((u32LinkageUnLockCnt<<8)&(0xFF)))^0x79;
	sPinCode[4] = ((uint8)(u32LinkageUnLockCnt&0xFF))^0x57;
	sPinCode[5] = 0x6F;
	sPinCode[6] = 0x63;
	sPinCode[7] = 0x74;
	sPinCode[8] = 0x65;

	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SampleDoorLockBindUnlockHandle");
	for(i = 1;i < 5;i++)
		DBG_vPrintf(TRACE_DOORLOCK, " %0x ",sPinCode[i]);

	return 1;
}
#endif
PUBLIC void App_SampleDoorLockIASWDStartWaring(uint8 u8Level,uint16 u16Duration,bool u8ResendID)
{
	
	tsZCL_Address sDestinationAddress;
	tsCLD_IASWD_StartWarningReqPayload asStartWaringPayload;
	uint8 u8ret = 0,u8Seq = u8GetTransactionSequenceNumber();

	// Ricky Save for Resend
	u32SampleDoorLockResend.eWaringDevicePayload.u8WarningModeLevel = u8Level;
	u32SampleDoorLockResend.eWaringDevicePayload.u16WarningDuration = u16Duration;
	u32SampleDoorLockResend.u32ResendEvents |= RH_DOORRING;

	/* Set the address mode to send to all bound device and don't wait for an ACK*/
	sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;

	if (u8Level == 1)
		asStartWaringPayload.u8WarningModeStrobeAndSirenLevel = 0x41;
	else
		asStartWaringPayload.u8WarningModeStrobeAndSirenLevel = 0x01;

	if (u16Duration < sSensor.sIASWDServerCluster.u16MaxDuration)
		asStartWaringPayload.u16WarningDuration = u16Duration;
	else
		asStartWaringPayload.u16WarningDuration = 0x3C;

	asStartWaringPayload.eStrobeLevel = 0;

	if (u8ResendID)
	{
	//	asStartWaringPayload.u8StrobeDutyCycle = App_SampleDoorLockWaringDeviceIDChange();
		asStartWaringPayload.u16WarningDuration |= (App_SampleDoorLockWaringDeviceIDChange(TRUE) << 8);
	}
	else
		asStartWaringPayload.u16WarningDuration |= (App_SampleDoorLockWaringDeviceIDChange(FALSE) << 8);


	u8ret = eCLD_IASWDStartWarningReqSend (OCCUPANCYSENSOR_SENSOR_ENDPOINT,	//uint8			u8SourceEndPointId,
									REPORT_EP,								//uint8			u8DestinationEndPointId,
									&sDestinationAddress,			//tsZCL_Address	*psDestinationAddress,
									&u8Seq,							//uint8			*pu8TransactionSequenceNumber,
									&asStartWaringPayload			//tsCLD_IASWD_StartWarningReqPayload	 *psPayload);
									);

	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SampleDoorLockIASWDStartWaring %0x %d ",
					asStartWaringPayload.u8WarningModeStrobeAndSirenLevel,u8ret);

#if 0
	uint8 i;
	uint16 u16NwkAddr = 0;
	for (i = 0; i < BDB_FB_MAX_TARGET_DEVICES; i++)
	{

		u16NwkAddr = vFindBindDestinationAddress(i);
		sDestinationAddress.uAddress.u16DestinationAddress = u16NwkAddr;

		if(u16NwkAddr != 0)
		{
			if (vFindBindDestinationEP(i) == 0xFF)
				continue;

			DBG_vPrintf(TRACE_DOORLOCK, "\n sFindAndBind %04x %0x ",u16NwkAddr,vFindBindDestinationEP(i));

			u8ret = eCLD_IASWDStartWarningReqSend (OCCUPANCYSENSOR_SENSOR_ENDPOINT,		//uint8			u8SourceEndPointId,
									vFindBindDestinationEP(i),					//uint8			u8DestinationEndPointId,
									&sDestinationAddress,						//tsZCL_Address	*psDestinationAddress,
									&u8Seq,										//uint8			*pu8TransactionSequenceNumber,
									&asStartWaringPayload						//tsCLD_IASWD_StartWarningReqPayload	 *psPayload);
									);
			
			DBG_vPrintf(TRACE_DOORLOCK, "\n FindAndBind Send Notification %d ",u8ret);
		}
	}
#endif
}

PUBLIC void App_cbTimerLeaveAndFactoryNew(void *pvParam)
{
	App_SampleDoorLockLinkageCountReset();
	vOTAResetPersist();
	ZPS_eAplZdoLeaveNetwork(0, FALSE, FALSE);
	PDM_vDeleteAllDataRecords();
	APP_vFactoryResetRecords();
	vAHI_SwReset();

}

PUBLIC bool App_SampleDoorLockProcessResendPemit(uint8 u8Cmd)
{
	uint8 i;

	for (i = 0;i < CmdPemitLengh; i++)
	{
		if (SampleDoorLockCmdPemit[i] == u8Cmd)
			return TRUE;
	}

	return FALSE;
}

PUBLIC void App_SampleDoorLockRemoteUnlockPemit(bool bASDflg)
{	

	if (bASDflg)
		bAppRemoteUnlockPemit++;
	else
		bAppRemoteUnlockPemit = bASDflg;

	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SampleDoorLockRemoteUnlockPemit %d ",bAppRemoteUnlockPemit);
}

PUBLIC void App_SampleDoorLockRemoteUnlockCheckIn(void)
{
	memset(u8AppRemotePinCode,0,sizeof(u8AppRemotePinCode));
}

PUBLIC bool App_SampleDoorLockSetPinCode(bool bStatus)
{
	uint8 u8SourceEndPointId = 1,u8DestinationEndPointId = 11;
	tsZCL_Address sDestinationAddress;
	uint8 u8Seq = u8GetTransactionSequenceNumber();


	ZPS_tsAplAib * tsAplAib  = ZPS_psAplAibGetAib();
	uint32 SizeOfBindingTable = tsAplAib->psAplApsmeAibBindingTable->psAplApsmeBindingTable[0].u32SizeOfBindingTable;
	if(SizeOfBindingTable == 0)
	{
		return 1;
	}
	else
	{
		DBG_vPrintf(TRACE_DOORLOCK, "\n moon->SizeOfBindingTable = %d", SizeOfBindingTable);
		uint32 j;
		for(j = 0 ; j < SizeOfBindingTable; j++ )
		{
			DBG_vPrintf(TRACE_DOORLOCK, "\n moon->No %d ClusterId = %d", j, tsAplAib->psAplApsmeAibBindingTable->psAplApsmeBindingTable[0].pvAplApsmeBindingTableEntryForSpSrcAddr[j].u16ClusterId);
			if(CLOSURE_CLUSTER_ID_DOOR_LOCK == tsAplAib->psAplApsmeAibBindingTable->psAplApsmeBindingTable[0].pvAplApsmeBindingTableEntryForSpSrcAddr[j].u16ClusterId)
			{
				DBG_vPrintf(TRACE_DOORLOCK, "\n moon->has CLOSURE_CLUSTER_ID_DOOR_LOCK");		
				u8SourceEndPointId = OCCUPANCYSENSOR_SENSOR_ENDPOINT;
				u8DestinationEndPointId =  tsAplAib->psAplApsmeAibBindingTable->psAplApsmeBindingTable[0].pvAplApsmeBindingTableEntryForSpSrcAddr[j].u8DestinationEndPoint;

				sDestinationAddress.eAddressMode =  tsAplAib->psAplApsmeAibBindingTable->psAplApsmeBindingTable[0].pvAplApsmeBindingTableEntryForSpSrcAddr[j].u8DstAddrMode;
				sDestinationAddress.uAddress.u64DestinationAddress = tsAplAib->psAplApsmeAibBindingTable->psAplApsmeBindingTable[0].pvAplApsmeBindingTableEntryForSpSrcAddr[j].uDstAddress.u64Addr;
			}
		}
	}

	uint16 userID     = 0xFBBF;
	uint8  userStatus = 0xFB;
	uint8  userType   = 0xFB;
	uint8 u8success[4] = {0xFB,0xFB,0xFB,0xFB},u8false[4] = {0xFF,0xFF,0xFF,0xFF};

	tsZCL_OctetString payload;
	payload.u8MaxLength = 0x04;
	payload.u8Length = 0x04;

	if (bStatus)
		payload.pu8Data = u8success;
	else
		payload.pu8Data = u8false;

	tsZCL_TxPayloadItem asPayloadDefinition[] = {
				{1, E_ZCL_UINT16,	&userID},
				{1, E_ZCL_UINT8,	&userStatus},
				{1, E_ZCL_UINT8,	&userType},
				{1, E_ZCL_OSTRING,	&payload}
												  };

    return eZCL_CustomCommandSend(u8SourceEndPointId,
								  u8DestinationEndPointId,
                                  &sDestinationAddress,
                                  CLOSURE_CLUSTER_ID_DOOR_LOCK,
                                  FALSE,
                                  E_CLD_DOOR_LOCK_CMD_SET_PINCODE,
                                  &u8Seq,
                                  asPayloadDefinition,
                                  FALSE,
                                  0,
                                  sizeof(asPayloadDefinition) / sizeof(tsZCL_TxPayloadItem)
                                  );

}

PUBLIC void App_SampleDoorLockDisableInterrupt(void)
{
	
	vAHI_UartDisable(E_AHI_UART_1);
	vAHI_DioInterruptEnable(0,APP_BUTTONS_INPUT_MASK);

}

PUBLIC void App_SampleDoorLockVoltageReportProcess(void)
{

	u32SampleDoorLockResend.u32ResendEvents |= RH_VOLTAGEINFO;
	u32SampleDoorLockResend.eVoltagePayload.u8Remaining = RxSerialMsg[11];
	ZTIMER_eStart(u8TimerStartResend,ZTIMER_TIME_MSEC(750));

}

PUBLIC void App_SampleDoorLockResendProcess(void)
{
//	if (u32SampleDoorLockResend.u32ResendEvents & RH_DOORLOCKSTATUS);

	if (u32SampleDoorLockResend.u32ResendEvents & RH_DOORRING)
	{

		App_SampleDoorLockIASWDStartWaring(u32SampleDoorLockResend.eWaringDevicePayload.u8WarningModeLevel,
											u32SampleDoorLockResend.eWaringDevicePayload.u16WarningDuration,FALSE);

	}

	if (u32SampleDoorLockResend.u32ResendEvents & RH_VOLTAGEINFO)
	{
		u32SampleDoorLockResend.u32ResendEvents &= ~RH_VOLTAGEINFO;
		App_SampleDoorLockAnalyzeVoltageInformation(u32SampleDoorLockResend.eVoltagePayload.u8Remaining);
	}
	if (u32SampleDoorLockResend.u32ResendEvents & RH_BASIC_RESPON)
	{	
		static uint8 BasicResponseResendCnt = 0;
		BasicResponseResendCnt++;
		DBG_vPrintf(TRACE_DOORLOCK, "\n\n BasicResponseResendCnt %d\n\n", BasicResponseResendCnt);
		if(BasicResponseResendCnt > 3)
		{
			BasicResponseResendCnt = 0;
			DBG_vPrintf(TRACE_DOORLOCK, "\n\n Time Out Still No Recv BASIC_RESPON, Start Rejoin\n\n");
			u32SampleDoorLockResend.u32ResendEvents &= ~RH_BASIC_RESPON;//set to 0
			ZTIMER_eStop(u8TimerStartResend);
			ZTIMER_eStart(u8TimerBdbRejoin, ZTIMER_TIME_MSEC(1000));//no basic response, so start rejoin		}		
		}
		else
		{
			vSendBasicDeviceEnabledReport();
			App_Start_Recv_BasicResponse_TimerBit();//moons
		}
	}
}

PUBLIC void App_cbTimerStartResend( void * pvParam)
{
	DBG_vPrintf(TRACE_DOORLOCK, "\n\n\n\n App_cbTimerStartResend %d", u8ResendCount);

	if ((u8ResendCount++ > 3) || (u32SampleDoorLockResend.u32ResendEvents == 0))
	{
		u8ResendCount = 0;
		u32SampleDoorLockResend.u32ResendEvents &= ~RH_DOORRING;
		ZTIMER_eStop(u8TimerStartResend);
	}
	else
	{

		if ((u8ResendCount == 2) && (u32SampleDoorLockResend.u32ResendEvents & RH_DOORRING))
		{
			DBG_vPrintf(TRACE_DOORLOCK, "\n\n\n\n App_cbTimerStartResend RH_DOORRING setup u8TimerBdbRejoin");
			ZTIMER_eStart(u8TimerBdbRejoin, ZTIMER_TIME_MSEC(1));
		}
		else
		{
			// 锟斤拷始锟截凤拷
			App_SampleDoorLockResendProcess();

		}
		ZTIMER_eStart(u8TimerStartResend,ZTIMER_TIME_SEC(3));

	}

}

PUBLIC void App_cbTimerRemoteOperationPermit(void * pvParam)
{
	App_SampleDoorLockRemoteUnlockCheckIn();
	App_SampleDoorLockRemoteUnlockPemit(FALSE);
}


PUBLIC void APP_cbTimerZCLBasicWriteAttr(void * pvParam)
{

	switch (u16BasicAttrID)
	{
		case E_CLD_BAS_ATTR_ID_IR_DATA:
			memcpy(sBasicWriteAttributePValue,sSensor.sBasicServerCluster.au8BasicIRData+2,sSensor.sBasicServerCluster.sBasicIRData.u8Length);
			App_BasicWriteAttributeHandle(1,sSensor.sBasicServerCluster.sBasicIRData.u8Length-2);
			break;
		case E_CLD_BAS_ATTR_ID_TRANSPRT_TRANS_CNT:
			App_BasicWriteAttributeHandle(0,sSensor.sBasicServerCluster.u8TransportCnt);
			break;
		default :
			DBG_vPrintf(TRACE_DOORLOCK, "\n default : No Suitable Attribute Support");
			break;
	}

}

PUBLIC void App_BasicWriteAttributeHandle(uint8 BasicCmd,uint8 Length)
{

	uint8 i = 0;

#if (TRACE_DOORLOCK == TRUE)
	DBG_vPrintf(TRACE_DOORLOCK, "\n App_BasicWriteAttributeHandle %d : ",BasicCmd);
	while(i < Length)
		DBG_vPrintf(TRACE_DOORLOCK, "%0x ",sBasicWriteAttributePValue[i++]);
#endif

#ifdef SKYWORTH
	if (BasicCmd == 1)	// Ricky 锟斤拷锟斤拷锟斤拷锟斤拷锟饺诧拷锟斤拷
	{
		App_DB_ConfigHeartPeriod(Length);
	}
	else if(BasicCmd == 0)	//Ricky 锟斤拷锟斤拷锟铰凤拷锟斤拷锟斤拷
	{
		App_DB_ConfigCacheNumber(Length);
	}
#endif

}
#ifdef SKYWORTH
PRIVATE void App_DB_ConfigCacheNumber(uint8 u8TransportCnt)
{

	if (u8TransportCnt > 120)
	{
		vAppSampleDoorLockPollChange(242);
	}
	else
	{
		vAppSampleDoorLockPollChange(u8TransportCnt*2+2);
	}

	vStartPollTimer(ZTIMER_TIME_MSEC(500));
	DBG_vPrintf(TRACE_DOORLOCK, "\n App_DB_ConfigCacheNumber : %d",u8TransportCnt);
}
#endif
PUBLIC void App_Start_Recv_BasicResponse_TimerBit(void)
{
	DBG_vPrintf(TRACE_DOORLOCK, "\n App_Start_Recv_BasicResponse_TimerBit");
	u32SampleDoorLockResend.u32ResendEvents |= RH_BASIC_RESPON;//set to 1
	ZTIMER_eStart(u8TimerStartResend,ZTIMER_TIME_MSEC(3000));
}

void APP_RecvPollControlResponse(void)
{
	DBG_vPrintf(1, "\n\n Recv Poll Control Response \n\n");

	GetPollControlResponse = 1;
}

void APP_CheckGateWayCacheAllCnt(void)
{
	if(GateWayCacheAllCnt != 0)
		GateWayCacheAllCnt --;
	
	DBG_vPrintf(1, "\n\n Gate Way Cache All Cnt = %d \n\n", GateWayCacheAllCnt);
	
	if(GateWayCacheAllCnt != 0)
	{
		RequestNextCacheNum++;				
		App_AskOneOfCache();// get next
		RecordRequestNextCacheNum = RequestNextCacheNum;
		RetryRequestNextCacheNum = App_PollRateCnt - 16;// 5 second check
	}
	else
	{
#if 1
		if(BatteryAlarmWHeartBeatFlag == 1)
		{
			DBG_vPrintf(1,"\n BatteryAlarmWHeartBeatFlag == 1\n");
			//BatteryAlarmWHeartBeatFlag = 0;
			sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = 1;
			sSensor.sPowerConfigServerCluster.u8BatteryPercentageRemaining = 40;
			vSendReportPowerConfig();
		}
#endif
		RecordRequestNextCacheNum = 0xFF;
		RetryRequestNextCacheNum = 0xFF;
		WhetherAskCacheAllCnt = 0;
		vAppSampleDoorLockPollChange(6);
	}

}
PUBLIC void APP_cbGetNumGetWay(void * pvParam)
{
	WhetherAskCacheAllCnt = 1;
	vAppSampleDoorLockPollChange(10);
	vStartPollTimer(500);

	App_AskNumberOfCache();

	//vAppSampleDoorLockPollChange(10);
}
PUBLIC void APP_cbDelayAskOneOfNumGetWay(void * pvParam)
{
	DBG_vPrintf(1,"ask one of gatway data\n");
	APP_CheckGateWayCacheAllCnt();
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
