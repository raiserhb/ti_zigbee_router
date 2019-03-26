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

#include "Alarms.h"
#include "DoorLock.h"
#include "IASWD.h"

#include "App_WL_DoorLock.h"
#include "app_SampleDoorLock_Uart.h"


#ifndef DEBUG_DOORLOCK
    #define TRACE_DOORLOCK FALSE
#else
    #define TRACE_DOORLOCK TRUE
#endif


/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define	BEGYEAR  2000     //  UTC started at 00:00:00 January 1, 2000
#define	DAY      86400UL  // 24 hours * 60 minutes * 60 seconds

#define	IsLeapYear(yr)	(!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))
#define	YearLength(yr)	((uint16)(IsLeapYear(yr) ? 366 : 365))

#define CmdPemitLengh	3

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

PRIVATE uint8 bAppRemoteUnlockPemit = 0;
PRIVATE uint8 u8AppRemotePinCode[10] = {0};

PRIVATE uint8 SampleDoorLockCmdPemit[CmdPemitLengh] = {0x40,0x41,0x85};


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
 *	填写Basic 相关信息//Ricky
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_Init(void)
{
    /* Initialise the strings in Basic */
#ifdef YYH_CUSTOM		// 永源浩
    memcpy(sSensor.sBasicServerCluster.au8ManufacturerName, "FULL Enterprise Corp", CLD_BAS_MANUF_NAME_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FZ-DL5083", CLD_BAS_MODEL_ID_SIZE);
#else
	memcpy(sSensor.sBasicServerCluster.au8ManufacturerName, "FBee", CLD_BAS_MANUF_NAME_SIZE);

	#ifdef SKYWORTH		// 创维
		memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FB56-DOR10CW0.6", CLD_BAS_MODEL_ID_SIZE);	
	#else
		#ifdef HUTLON	// 汇泰龙
			memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FB56-DOR11HT0.6", CLD_BAS_MODEL_ID_SIZE);
		#else			// 王力
			memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "FB56-DOR07WL2.2", CLD_BAS_MODEL_ID_SIZE);	// V0.5
		#endif
	#endif
#endif

	memcpy(sSensor.sBasicServerCluster.au8DateCode, "20170808", CLD_BAS_DATE_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8SWBuildID, "0000-0004", CLD_BAS_SW_BUILD_SIZE);
	memcpy(sSensor.sBasicServerCluster.au8ProductURL, "feibit.com", CLD_BAS_URL_SIZE);
	memcpy(sSensor.sBasicServerCluster.au8ProductCode, "0x117E", CLD_BAS_PCODE_SIZE);

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
    APP_vSetLED(LED2, sSensor.sIdentifyServerCluster.u16IdentifyTime%2);
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

	u8ret = eCLD_AlarmsSignalAlarm(OCCUPANCYSENSOR_SENSOR_ENDPOINT,11,
						&sDestinationAddress,&u8Seq,u8AlarmCode, CLOSURE_CLUSTER_ID_DOOR_LOCK);

	DBG_vPrintf(TRACE_DOORLOCK, "\n u8ret %0x ",u8ret);

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

	u8ret = eCLD_AlarmsCommandResetAlarmCommandSend(OCCUPANCYSENSOR_SENSOR_ENDPOINT,11,
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
	
	if (RxSerialMsg[8] == 0x00)
	{
		u8status |= 2;	// 上报开锁记录 
		eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_UNLOCKED;
		sRemoteOptResult.eStatus = 0;
	}
	else if (RxSerialMsg[8] == 0x01)
	{
		u8status |= 1;	// 不睡眠 
		eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_NOT_FULLY_LOCKED;
		sRemoteOptResult.eStatus = 1;
	}
	else if (RxSerialMsg[8] == 0x02)
	{
		eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_NOT_FULLY_LOCKED;
		sRemoteOptResult.eStatus= 0x7F;
	}
	u8ret = eCLD_DoorLockCommandLockUnlockResponseSend(OCCUPANCYSENSOR_SENSOR_ENDPOINT, 11,
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
										11,&sDoorLockOptEvtNotifyPayload);

	
	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SendOperationEventNotification u8ret %d \n",u8ret);
	
}

PUBLIC void App_SampleDoorLockAnalyzeUnlockRecord(void)
{

	eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_UNLOCKED;
	u16PinData = ((uint16)(RxSerialMsg[9] << 8) | (uint16)(RxSerialMsg[8]));
	u32UtcLocalTime = (((uint32)RxSerialMsg[17]<<24)|((uint32)RxSerialMsg[16]<<16)|((uint32)RxSerialMsg[15]<<8)|(uint32)RxSerialMsg[14]);

	if (RxSerialMsg[10] == 01)	//密码开锁
	{
		App_SendOperationEventNotification(OPTEVTSOURCE_KEYPAD,eSampleDoorLock_LockState,
											RxSerialMsg[13],u16PinData,u32UtcLocalTime);
	}
	else if (RxSerialMsg[10] == 02)	// 刷卡
	{
		App_SendOperationEventNotification(OPTEVTSOURCE_RFID,eSampleDoorLock_LockState,
											RxSerialMsg[13],u16PinData,u32UtcLocalTime);
	}
	else if (RxSerialMsg[10] == 03) // 指纹开锁
	{
		App_SendOperationEventNotification(OPTEVTSOURCE_MANUAL,eSampleDoorLock_LockState,
											RxSerialMsg[13],u16PinData,u32UtcLocalTime);
	}
	else if (RxSerialMsg[10] == 04)	// 多重验证
	{
		App_SendOperationEventNotification(OPTEVTSOURCE_MULTIPLEVALIDATION,eSampleDoorLock_LockState,
											RxSerialMsg[13],u16PinData,u32UtcLocalTime);
	}
	else
	{
		eSampleDoorLock_LockState = E_CLD_DOORLOCK_LOCK_STATE_LOCKED;
	}


}

PUBLIC void App_SampleDoorLockAnalyzeVoltageInformation(void)
{

	sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = 0;
	sSensor.sPowerConfigServerCluster.u8BatteryPercentageRemaining = 0xc8 -(RxSerialMsg[11]-1)*50; 

	vSendReportPowerConfig();

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

	u8ret = eCLD_DoorLockCommandLockUnlockResponseSend(OCCUPANCYSENSOR_SENSOR_ENDPOINT, 11,
									&sDestinationAddress, &u8Seq, E_CLD_DOOR_LOCK_CMD_LOCK,
									&sRemoteOptResult );

	DBG_vPrintf(TRACE_DOORLOCK, "\n App_SendOperationEventNotification u8ret %d \n",u8ret);

}

PUBLIC void App_SampleDoorLockCommandHandle(uint8 *sPayLoad)
{

#if TRACE_DOORLOCK
	uint8 i = 1;
	DBG_vPrintf(TRACE_DOORLOCK, "\n u8CommandId %d Len %d\n",sPayLoad[1],sPayLoad[2]);

	while (i < 10)
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

			App_SerialSendRemoteUnlock(sPayLoad+3,NormallyOpen);
			break;
		case E_CLD_DOOR_LOCK_CMD_SET_PINCODE:

			App_SerialSendRemoteBindUnlock(sPayLoad+3);
			break;

		default:
			break;
	}

}

PUBLIC void App_SampleDoorLockIASWDStartWaring(uint8 u8Level,uint16 u16Duration)
{
	
	tsZCL_Address sDestinationAddress;
	tsCLD_IASWD_StartWarningReqPayload asStartWaringPayload;
	uint8 u8ret = 0,u8Seq = u8GetTransactionSequenceNumber();

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

	u8ret = eCLD_IASWDStartWarningReqSend (OCCUPANCYSENSOR_SENSOR_ENDPOINT,	//uint8			u8SourceEndPointId,
									11,								//uint8			u8DestinationEndPointId,
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

PUBLIC void App_cbTimerLeaveAndFactoryNew(void)
{

	ZPS_eAplZdoLeaveNetwork(0, FALSE, FALSE);
	APP_vFactoryResetRecords();
	vAHI_SwReset(); //Ricky Todo 3s Delay

}

PUBLIC bool App_SampleDoorLockProcessPemit(uint8 u8Cmd)
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

PUBLIC void App_cbTimerRemoteOperationPermit(void)
{
	App_SampleDoorLockRemoteUnlockCheckIn();
	App_SampleDoorLockRemoteUnlockPemit(FALSE);
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
