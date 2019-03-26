/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          App_WL_DoorLock.h
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

#ifndef APP_WL_DOORLOCK_H_
#define APP_WL_DOORLOCK_H_

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "occupancy_sensor.h"
#include "zcl_options.h"
#include "app_reporting.h"
#include "zll_commission.h"

/****************************************************************************/
/***        Macro definition                                              ***/
/****************************************************************************/

#define APP_OCCUPANCY_SENSOR_UNOCCUPIED_TO_OCCUPIED_DELAY           10
#define APP_OCCUPANCY_SENSOR_TRIGGER_THRESHOLD			            5

#ifdef YYH_CUSTOM
#define APP_OCCUPANCY_SENSOR_OCCUPIED_TO_UNOCCUPIED_DELAY  	        (60*5)
#else
#define APP_OCCUPANCY_SENSOR_OCCUPIED_TO_UNOCCUPIED_DELAY  	        (60*30)		//2 Ricky	30min	
#endif

#define APP_TICKS_PER_SECOND	32000
#define APP_JOINING_BLINK_TIME			(ZTIMER_TIME_MSEC(1000))
#define APP_FIND_AND_BIND_BLINK_TIME 	(ZTIMER_TIME_MSEC(500))
#define APP_KEEP_AWAKE_TIME				(ZTIMER_TIME_MSEC(250))

#if HA_SYSTEM_MAX_REPORT_INTERVAL == 0x0
	#define xDEEP_SLEEP_ENABLE	//Ricky	-> x
#endif

#define FAILED_POLL_COUNT								3
#define FAILED_REJOIN_COUNT								3

/* Only one reportable attribute that is Occupancy attribute */
#define OCCUPANCY_NUMBER_OF_REPORTS									2	// Ricky Basic 、Power Config


// number of seconds since 0 hrs, 0 minutes, 0 seconds, on the
// 1st of January 2000 UTC
typedef uint32 UTCTime;

// To be used with
typedef struct
{
  uint8 seconds;  // 0-59
  uint8 minutes;  // 0-59
  uint8 hour;     // 0-23
  uint8 day;      // 0-30
  uint8 month;    // 0-11
  uint16 year;    // 2000+
} UTCTimeStruct;

typedef enum
{
	OPTEVTSOURCE_KEYPAD	= 0x00,
	OPTEVTSOURCE_RF		= 0x01,	// 用于开锁方式密码
	OPTEVTSOURCE_MANUAL	= 0x02,	// 用于指纹
	OPTEVTSOURCE_RFID	= 0x03,	// 用于刷卡
	OPTEVTSOURCE_REMOTE	= 0x04,	// 区别远程开锁
	OPTEVTSOURCE_MULTIPLEVALIDATION,
	OPTEVTSOURCE_INDETERMINATE
}te_OptEvtSourceValue;

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/
extern PUBLIC bool bLeaveInd;
extern PUBLIC uint8 u8JoinNWKCycle;

extern tsZLO_OccupancySensorDevice sSensor;
extern tsReports asDefaultReports[];

extern uint16 u16PinData;
extern teCLD_DoorLock_LockState eSampleDoorLock_LockState;

extern PUBLIC bool bReadWriteAttrFlg;
extern tsCLD_ZllDeviceTable sDeviceTable;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

void vAPP_ZCL_DeviceSpecific_Init(void);
teZCL_Status eApp_ZLO_RegisterEndpoint(tfpZCL_ZCLCallBackFunction fptr);
PUBLIC void vAPP_ZCL_DeviceSpecific_UpdateIdentify(void);
PUBLIC void vAPP_ZCL_DeviceSpecific_SetIdentifyTime(uint16 u16Time);
PUBLIC void vAPP_ZCL_DeviceSpecific_IdentifyOff(void);
PUBLIC uint8 app_u8GetDeviceEndpoint( void);

PUBLIC void App_ConvertUTCTime(UTCTimeStruct *tm, UTCTime secTime);
PUBLIC uint8 App_SampleDoorLockSendAlarmCmd(uint8 u8AlarmCode);
PUBLIC uint8 App_SampleDoorLockResetAlarmCode(uint8 u8AlarmCode);

PUBLIC uint8 App_SerialRemoteOperationHandle(uint8 u8OptState);
PUBLIC void App_SendOperationEventNotification(uint8 u8Sourc,uint8 u8LockState,uint8 u8Data,
																	uint16 u16Uid,uint32 u32UtcTime);
PUBLIC void App_SampleDoorLockAnalyzeUnlockRecord(void);
PUBLIC void App_SampleDoorLockAnalyzeVoltageInformation(void);
PUBLIC void App_SendLoaclOperationLockReport(void);

PUBLIC void App_cbTimerLeaveAndFactoryNew(void);

PUBLIC bool App_SampleDoorLockProcessPemit(uint8 u8Cmd);
PUBLIC void App_SampleDoorLockRemoteUnlockPemit(bool bASDflg);
PUBLIC void App_SampleDoorLockRemoteUnlockCheckIn(void);
PUBLIC void App_cbTimerRemoteOperationPermit(void);



/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* APP_OCCUPANCY_SENSOR_H_ */
