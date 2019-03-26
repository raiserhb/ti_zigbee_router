
/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "ZTimer.h"
#include "ZQueue.h"
#include "app_main.h"
#include "DBG.h"
#include "AppHardwareApi.h"
//#include "AppQueueApi.h"
#include "app_events.h"
#include "app_reporting.h"
#include "Basic.h"	//Ricky
#include "pwrm.h"
#include "app_nwk_event_handler.h"
#include "app_common.h"

#include "app_SampleDoorLock_Uart.h"
#include "App_DB_DoorLock.h"


/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_APP_DBATTR
	#define TRACE_APP_DBATTR				FALSE
#else
	#define TRACE_APP_DBATTR				TRUE
#endif

PUBLIC uint16 u16BasicAttrID = 0;

PRIVATE uint16 Heart_Period = 180;
PRIVATE uint16 Fixed_Duration = 2;
PRIVATE uint16 Max_Duration = 30;

/****************************************************************************/
/***        Functions 		                                              ***/
/****************************************************************************/
PUBLIC void APP_cbTimerZCLBasicWriteAttr(void)
{
    DBG_vPrintf(TRACE_APP_DBATTR, "\n ---------------------------------------------- ");
    DBG_vPrintf(TRACE_APP_DBATTR, "\n APP_cbTimerZCLBasicWriteAttr ");
    DBG_vPrintf(TRACE_APP_DBATTR, "\n u16BasicAttrID %0x ",u16BasicAttrID);
    DBG_vPrintf(TRACE_APP_DBATTR, "\n ---------------------------------------------- ");

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
			DBG_vPrintf(TRACE_APP_DBATTR, "\n default : No Suitable Attribute Support");
			break;

	}

}

PRIVATE void App_BasicWriteAttributeHandle(uint8 BasicCmd,uint8 Length)
{

	uint8 i = 0;

#if (TRACE_APP_DBATTR == TRUE)
	while(i <= Length)
		DBG_vPrintf(TRACE_APP_DBATTR, "%0x ",sBasicWriteAttributePValue[i++]);
#endif

	if (BasicCmd == 1)	// Ricky 配置心跳等参数
	{
		App_DB_ConfigHeartPeriod(Length);
	}
	else if(BasicCmd == 0)	//Ricky 缓存下发数量
	{
		App_DB_ConfigCacheNumber(Length);
	}

}

PRIVATE void App_DB_ConfigHeartPeriod(uint8 Length)
{

	if((sBasicWriteAttributePValue[0]==0xAA)&&(sBasicWriteAttributePValue[1]==0x55)&&(sBasicWriteAttributePValue[2]==0x08)&&
		(sBasicWriteAttributePValue[3]==0x01)&&(sBasicWriteAttributePValue[4]==0x19))
	{
		Heart_Period =((uint16)(sBasicWriteAttributePValue[6]<<8)|sBasicWriteAttributePValue[5]);
		Fixed_Duration =((uint16)(sBasicWriteAttributePValue[8]<<8)|sBasicWriteAttributePValue[7]);
		Max_Duration =((uint16)(sBasicWriteAttributePValue[10]<<8)|sBasicWriteAttributePValue[9]);

		if ((Heart_Period < 10) || (Heart_Period > 1860))
			Heart_Period = 180;

		if (Max_Duration > 30)
			Max_Duration = 30;

		sSensor.sOccupancySensingServerCluster.u16PIROccupiedToUnoccupiedDelay = Heart_Period;
		DBG_vPrintf(TRACE_APP_DBATTR, "\n Heart_Period:%d, Fixed_Duration:%d,Max_Duration:%d ",Heart_Period, Fixed_Duration,Max_Duration);
	}
	else
	{	// 写入串口
		TxSerialMsg[0] = Length;
		uint8 *pTxSerial = &TxSerialMsg[1];
		memcpy(pTxSerial,sBasicWriteAttributePValue,Length);

		ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 
	}

}

PRIVATE void App_DB_ConfigCacheNumber(uint8 u8TransportCnt)
{
	uint8 u8PollRate = u8TransportCnt*Fixed_Duration;

	if (u8PollRate >= Max_Duration)
	{
		App_PollRateCnt = (Max_Duration+1)*2;
		vStartPollTimer(POLL_TIME);
	}
	else
	{
		App_PollRateCnt = (u8PollRate+1)*2;
		vStartPollTimer(POLL_TIME);
	}
	DBG_vPrintf(TRACE_APP_DBATTR, "\n App_DB_ConfigCacheNumber : %d,Max_Duration:%d ",u8PollRate,Max_Duration);

}

PUBLIC void APP_cbTimerSpecialHeartBeat(void)
{
	APP_tsEvent sButtonEvent;

	/* Post a message to the stack so we aren't handling events
	* in interrupt context*/
	sButtonEvent.eType = APP_E_EVENT_SEND_REPORT;
	ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);

	App_PollRateCnt = 10;
	vStartPollTimer(POLL_TIME);

}


/****************************************************************************/
/***        End Of File 		                                              ***/
/****************************************************************************/

