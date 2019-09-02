
#include <jendefs.h>
#include "ZTimer.h"
#include "ZQueue.h"
#include "app_main.h"
#include "DBG.h"
#include "AppHardwareApi.h"
#include "app_events.h"
#include "PDM_IDs.h"
#include "PDM.h"
#include "app_nwk_event_handler.h"
#include "app_event_handler.h"
#include "app_SampleDoorLock_Uart.h"
#include "app_occupancy_buttons.h"
#include "App_OccupancySensor.h"

#ifndef DEBUG_APP_UART
	#define TRACE_APP_UART				FALSE
#else
	#define TRACE_APP_UART				TRUE
#endif
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define UART_DLM_OFFSET 						0x04		 /**< Offset of UART's DLM register */
#define UART_LCR_OFFSET 						0x0C		 /**< Offset of UART's LCR register */
#define UART_MCR_OFFSET 						0x10		 /**< Offset of UART's MCR register */
#define UART_EFR_OFFSET							0x20		 /**< Offset of UART's EFR register */

/* Define for devices with AFC */
#define UART_AFC_OFFSET							0x2C		 /**< Offset of UART's AFC register */

/* Enable fine grained baud rate selection */
#define UART_ENABLE_ADVANCED_BAUD_SELECTION		TRUE

#define UART_WAKEUP_OUTTIME						250

#define APP_UARTRX_DECIDE_LENGTH(x)				((x == 0x01) || (x == 0x0A) || (x == 60)) ? TRUE : FALSE 

#define IS_EMPTY (0)
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE bool_t bRxEnable;			/**< UART receive enabled */
PRIVATE bool_t bTxEnable;			/**< UART transmit enabled */
PRIVATE bool_t bModemInt;			/**< UART modem status change interrupt enabled */
//PRIVATE bool_t bTxIntServiced;		/**< LAst UART transmit interrupt was serviced, expect another transmit interrupt to follow */
PRIVATE uint8 u8UARTWriteCnt = 0;
PRIVATE uint32 u32CmdCountID = 1;
PRIVATE uint8 RxSerial[80];		//Ricky 用于存放串口接收数据
PUBLIC volatile uint8 RxSerialMsg[80];	//Ricky 用于处理串口数据
PRIVATE uint8 TxSerialMsg[64];	//Ricky 用于存放串口发送数据
PRIVATE uint8 TxSerial[64];		//Ricky 用于串口写入
#define SERVER_WA_BUFFFERSIZE (10)
PUBLIC uint8 u8ServerWriteAttributData[SERVER_WA_BUFFFERSIZE][64];
uint8 u8RestartForOTA = 0;
//[][0] = FF33wirte attribute [commandDataLen]
//[][1] = FF33wirte attribute [commandID]

//*********
//[][2~end] = FF33wirte attribute [data] 
//*********
// OR
//[][2] = FF33wirte attribute [AppCmdId]
//[][3~end] = FF33wirte attribute [data] 
//*********
extern volatile ts_SerialCache tsSerialBuffer;
extern PUBLIC volatile uint8 App_PollRateCnt;
extern uint8 RequestNextCacheNum;
extern uint8 GetPollControlResponse;
extern uint8 u8JoinNetworkFlowPath;
extern uint8 RecordRequestNextCacheNum;
extern uint8 u8TimerCheckGetCacheKeepRun;
extern PUBLIC te_RequestCacheStatus requestCacheStatus;
extern bool bPeriodicEventWakeup;
extern uint16_t reStarSystCount;

uint32 u32SleepTimeTicks = 1200;//1200

extern PUBLIC bool_t bOTA_IsWaitToUpgrade(void);

PRIVATE uint8 vUART_RxCharCRC(uint8 *SerialMsg);
PUBLIC void UART_vSetBaudRate( uint32 u32BaudRate );
PUBLIC bool_t UART_bTxReady();
void General_SendBasic(uint8 AppCmdCounter);
/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
PUBLIC uint8 APP_UartAchieveMessagefromPrivate(uint8 u8RxBytes)
{
	uint8 u8CRC = 0;
	uint8 i = 0;
	
	DBG_vPrintf(TRACE_APP_UART, "\n tsSerialBuffer.u8SerialData[%d][] : \n", u8RxBytes);
	for(i = 0; i < tsSerialBuffer.u8SerialData[u8RxBytes][1] + 10; i++)
	{
		DBG_vPrintf(TRACE_APP_UART, "%02x ", tsSerialBuffer.u8SerialData[u8RxBytes][i]);
	}
	DBG_vPrintf(TRACE_APP_UART, "\n");
		
	i = tsSerialBuffer.u8SerialData[u8RxBytes][1] + 8;// get data crc position

	if(tsSerialBuffer.u8SerialData[u8RxBytes][0] == 0xAA && tsSerialBuffer.u8SerialData[u8RxBytes][i + 1] == 0x55)// check pack head and end chars is correcty
	{
		i = tsSerialBuffer.u8SerialData[u8RxBytes][i];// get crc byte
		
		u8CRC = vUART_RxCharCRC(&tsSerialBuffer.u8SerialData[u8RxBytes][0]);
		//DBG_vPrintf(TRACE_APP_UART, "\n [%d] CRC : %02x", u8RxBytes, i);
		
		if(u8CRC == i)
		{
			DBG_vPrintf(TRACE_APP_UART, " CRC Right\n", u8CRC);
			
			memcpy(RxSerialMsg, tsSerialBuffer.u8SerialData[u8RxBytes], SERIAL_CACHE_ITEM_LEN);

			tsSerialBuffer.u8SerialData[u8RxBytes][0] = 0;
			return ~IS_EMPTY;
		}
		else
		{
			DBG_vPrintf(TRACE_APP_UART, " CRC Error\n", u8CRC);
			
			tsSerialBuffer.u8SerialData[u8RxBytes][0] = 0;
			return IS_EMPTY;
		}
	}
	else
	{
		DBG_vPrintf(TRACE_APP_UART, " Pack Head Or End Char Error\n", u8CRC);
		
		tsSerialBuffer.u8SerialData[u8RxBytes][0] = 0;
		return IS_EMPTY;
	}
}
PUBLIC void APP_UARTInitialise(void)
{
	DBG_vPrintf(TRACE_APP_UART, "\n APP_UARTInitialise");

	/* Start with Receive and transmit enabled */
	bRxEnable = TRUE;
	bTxEnable = TRUE;

	/* Modem status change interrupt is dependent upon flow control mode */
	#ifdef ENABLE_HW_FLOW_CONTROL
		bModemInt = TRUE;
	#endif

	#ifdef ENABLE_HW_SW_FLOW_CONTROL
		bModemInt = FALSE;
	#endif


	// Ricky
 	bAHI_UartEnable(E_AHI_UART_1, TxSerial, (uint16)sizeof(TxSerial), RxSerial, (uint16)sizeof(RxSerial));

    vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
    vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);

	vAHI_UartSetLocation(E_AHI_UART_1,TRUE);	// Ricky

	UART_vSetBaudRate(57600);

#if (ENABLE_HW_FLOW_CONTROL || ENABLE_HW_SW_FLOW_CONTROL)
    /* Turn on RTS */
    vUART_SetRts(bRxEnable);

	/* Is CTS bit set meaning CTS is off ? */
	if (u8AHI_UartReadModemStatus(E_AHI_UART_1) & 0x10)
	{
		/* Disable transmit */
		vUART_SetTxEnable(FALSE);
	}
	/* Is CTS bit is clear meaning CTS is on ? */
	else
	{
		/* Enable transmit */
		vUART_SetTxEnable(TRUE);
    }
#endif

	//DBG_vPrintf(TRACE_APP_UART, "\n vAHI_UartSetInterrupt");
    /* Turn on modem status, tx, rx interrupts */
    vAHI_UartSetInterrupt(E_AHI_UART_1, FALSE, FALSE, FALSE, TRUE, E_AHI_UART_FIFO_LEVEL_1);
	DBG_vPrintf(TRACE_APP_UART, "\n vAHI_UartSetInterrupt");

	//Ricky 清空缓存
	//vAppSerialCacheArea_Init();

}
/****************************************************************************
 *
 * NAME: vUART_SetBuadRate
 *
 * DESCRIPTION:
 *
 * PARAMETERS: Name        RW  Usage
 *
 * RETURNS:
 *
 ****************************************************************************/

PUBLIC void UART_vSetBaudRate ( uint32    u32BaudRate )
{
#if 0
    uint16    u16Divisor      =  0;
    uint32    u32Remainder;
    uint8     u8ClocksPerBit  =  16;
    uint32    u32CalcBaudRate =  0;
    int32     i32BaudError    =  0x7FFFFFFF;

    while (abs(i32BaudError) > (int32)(u32BaudRate >> 4)) /* 6.25% (100/16) error */
    {
        if (--u8ClocksPerBit < 3)
            return;

        /* Calculate Divisor register = 16MHz / (16 x baud rate) */
        u16Divisor = (uint16)(16000000UL / ((u8ClocksPerBit+1) * u32BaudRate));

        /* Correct for rounding errors */
        u32Remainder = (uint32)(16000000UL % ((u8ClocksPerBit+1) * u32BaudRate));

        if (u32Remainder >= (((u8ClocksPerBit+1) * u32BaudRate) / 2))
            u16Divisor += 1;

        u32CalcBaudRate = (16000000UL / ((u8ClocksPerBit+1) * u16Divisor));
		i32BaudError = (int32)u32CalcBaudRate - (int32)u32BaudRate;

		DBG_vPrintf(TRACE_APP_UART, "\n UART_vSetBaudRate %d,%d \n",u8ClocksPerBit,u16Divisor);
#else
    /* Set the calculated clocks per bit */
    vAHI_UartSetClocksPerBit(E_AHI_UART_1, 11);
    /* Set the calculated divisor */
    vAHI_UartSetBaudDivisor(E_AHI_UART_1, 23);
#endif

}
PUBLIC void HAL_UART_Write(const uint8 *WriteBuf)
{
	uint8 i = 0;
	const uint8 cnt = *WriteBuf;

	if(WriteBuf[1] == 0)
		return ;

	DBG_vPrintf(TRACE_APP_UART, "\n HAL_UART_Write");
	/*
	for(i = 0; i <= WriteBuf[0]; i++)
		DBG_vPrintf(TRACE_APP_UART, " %02x", WriteBuf[i]);
	*/
	
	i = 1;
	//while(i < WriteBuf[0])
	while(i < cnt)
	{
		while ( !UART_bTxReady () && ( u8AHI_UartReadLineStatus ( E_AHI_UART_1 ) & E_AHI_UART_LS_THRE ) );
		vAHI_UartWriteData(E_AHI_UART_1, WriteBuf[i]);
		while ( !UART_bTxReady() && !( u8AHI_UartReadLineStatus ( E_AHI_UART_1 ) & E_AHI_UART_LS_TEMT ) );
		
		DBG_vPrintf(TRACE_APP_UART, " %02x", WriteBuf[i]);
		
		i++;
	}
}
/****************************************************************************
 *
 * NAME: vUART_TxReady
 *
 * DESCRIPTION:
 * Set UART RS-232 RTS line low to allow further data
 *
 ****************************************************************************/
PUBLIC bool_t UART_bTxReady()
{
    return u8AHI_UartReadLineStatus ( E_AHI_UART_1 ) & E_AHI_UART_LS_THRE;
}
/****************************************************************************
 *
 * NAME: vUART_HandleUartInterrupt
 *
 * DESCRIPTION  Handles UART interrupt.
 *
 * Adds received character to receive serial queue, if available.
 *
 * Transmits character from transmit serial queue, if available.
 *
 * Reacts to change in CTS status when not using automatic flow control.
 *
 * PARAMETERS       Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
 
PUBLIC void vUART_HandleUartInterrupt(void)
{
	uint8 u8RxData;

	/* Data to receive ? */
	if (u8AHI_UartReadInterruptStatus(E_AHI_UART_1))
    {
//		DBG_vPrintf(TRACE_APP_UART, "\n vUART_HandleUartInterrupt E_AHI_UART_INT_RXDATA");

		#ifdef ENABLE_HW_SW_FLOW_CONTROL
			/* Read line status */
			uint8 u8LineStatus = u8AHI_UartReadLineStatus(E_AHI_UART_1);
		#endif

        /* Receive character from UART */
		u8RxData = u8AHI_UartReadData(E_AHI_UART_1);
		APP_vProcessIncomingSerialCommands(u8RxData);
    }

}
/****************************************************************************/
/***    Implementation                          */
/****************************************************************************/
volatile uint8 u8RxCnt = 0;
PUBLIC void APP_vProcessIncomingSerialCommands ( uint8 u8RxByte )
{
	volatile static uint8 u8CacheNum = 0;//, u8RxCnt = 0;
	//DBG_vPrintf(1, "[%d=%02x] ", u8RxCnt, u8RxByte);
	//DBG_vPrintf(1, "[%02x] ", u8RxByte);
	//RxSerial[u8RxCnt] = u8RxByte;

	if(ZTIMER_eGetState(u8TimerSerialTimeout) != E_ZTIMER_STATE_RUNNING)
	{
		//DBG_vPrintf(TRACE_APP_UART, "\n u8TimerSerialTimeout,ZTIMER_TIME_MSEC(60)");
		ZTIMER_eStart(u8TimerSerialTimeout,ZTIMER_TIME_MSEC(60));// Serial timeout
	}

	if(u8RxCnt == 0)
	{
		if(u8RxByte == 0xAA)
		{
			//DBG_vPrintf(1, "0xAA ");
			tsSerialBuffer.u8SerialData[u8CacheNum][0] = u8RxByte;
			u8RxCnt = 1;
		}
		else
		{
			u8RxCnt = 0;
		}
		return;
	}

	if(u8RxCnt == 1)
	{
		//DBG_vPrintf(1, "Len=%02x ", u8RxByte);
		tsSerialBuffer.u8SerialData[u8CacheNum][1] = u8RxByte;
		u8RxCnt = 2;
	}
	else if(u8RxCnt > 1)
	{
		//DBG_vPrintf(1, "[%d:%02x] ", u8RxCnt, u8RxByte);
		
		tsSerialBuffer.u8SerialData[u8CacheNum][u8RxCnt++] = u8RxByte;
		
		if(u8RxCnt == tsSerialBuffer.u8SerialData[u8CacheNum][1] + 10)
		{
			ZTIMER_eStop(u8TimerSerialTimeout);
			
			ZQ_bQueueSend (&APP_msgSerialRx, &u8CacheNum);
			//DBG_vPrintf(1, "\nZQ_bQueueSend[%d]\n", u8CacheNum);
			
			u8CacheNum++;
			if(u8CacheNum > 5)
				u8CacheNum = 0;
			
			u8RxCnt = 0;
		}
	}
	else
	{
		u8RxCnt = 0;
	}
}
PRIVATE uint8 vUART_RxCharCRC(uint8 *SerialMsg)
{
	uint8 u8CRC = 0;
	uint8 i;

	for (i = 0; i < SerialMsg[1]+8; i++)
	{
		u8CRC = u8CRC^SerialMsg[i];
	}

	DBG_vPrintf(TRACE_APP_UART, " vUART_RxCharCRC : %0x ",u8CRC);
	return u8CRC;
}
PUBLIC uint8 vUART_TxCharCRC(uint8 *SerialMsg)
{
	uint8 u8CRC = 0;
	uint8 i;

	for (i = 1; i <= (SerialMsg[2]+8); i++)
	{
		u8CRC = u8CRC^SerialMsg[i];
	}

//	DBG_vPrintf(TRACE_APP_UART, "\n vUART_TxCharCRC : %0x ",u8CRC);
	return u8CRC;
}
PUBLIC void APP_SerialSendAcknowlegement(uint8 cmdType,bool Ack)
{
#if 1
	uint8 TxSerialMsg_[12] = {0};
	TxSerialMsg_[0] = 0x0C;
	TxSerialMsg_[1] = 0xAA;
	TxSerialMsg_[2] = 0x01;
	TxSerialMsg_[3] = cmdType;

	TxSerialMsg_[4] = RxSerialMsg[3];
	TxSerialMsg_[5]	= RxSerialMsg[4];
	TxSerialMsg_[6]	= RxSerialMsg[5];
	TxSerialMsg_[7]	= RxSerialMsg[6];

	TxSerialMsg_[8] = 0x01;
	TxSerialMsg_[9] = Ack;
	TxSerialMsg_[10] = vUART_TxCharCRC(TxSerialMsg_);
	TxSerialMsg_[11] = 0x55;

    vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
    vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);

	HAL_UART_Write(TxSerialMsg_);

#else
	TxSerialMsg[0] = 0x0C;
	TxSerialMsg[1] = 0xAA;
	TxSerialMsg[2] = 0x01;
	TxSerialMsg[3] = cmdType;

	TxSerialMsg[4]  = RxSerialMsg[3];
	TxSerialMsg[5]	= RxSerialMsg[4];
	TxSerialMsg[6]	= RxSerialMsg[5];
	TxSerialMsg[7]	= RxSerialMsg[6];

	TxSerialMsg[8] = 0x01;
	TxSerialMsg[9] = Ack;
	TxSerialMsg[10] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[11] = 0x55;

	u8UARTWriteCnt = 7;
//	DBG_vPrintf(TRACE_APP_UART, "\n APP_SerialSendAcknowlegement ");
//	DBG_vPrintf(TRACE_APP_UART, "\n cmdType %0x ",cmdType);
#if 0
	// 串口写入只设一个入口	应答包不做唤醒信号
	ZTIMER_eStart(u8TimerWriteSerial, ZTIMER_TIME_MSEC(1));
#else
	memset(TxSerial,0,sizeof(TxSerial));
	memcpy(TxSerial,TxSerialMsg,TxSerialMsg[0]);

    vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
    vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);

	HAL_UART_Write(TxSerial);

	memset(TxSerial,0,sizeof(TxSerial));
#endif
#endif
}
PUBLIC void APP_SerialSendJoinIndication(uint8 Operation, uint8 u8Decide)
{
	// 入网退网提示
	TxSerialMsg[0]	= 0x15;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0x0A;
	TxSerialMsg[3]	= NwkPrompt;
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	TxSerialMsg[8]	= 0x00;

	TxSerialMsg[9]	= Operation;	// Join 00,		Leave 01
	TxSerialMsg[10] = u8Decide;		// Success 00,	failure 01
	TxSerialMsg[11] = 0x00;
	TxSerialMsg[12] = 0x00;
	TxSerialMsg[13] = 0x00;
	TxSerialMsg[14] = 0x00;
	TxSerialMsg[15] = 0x00;
	TxSerialMsg[16] = 0x00;
	TxSerialMsg[17] = 0x00;
	TxSerialMsg[18] = 0x00;

	TxSerialMsg[19] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[20] = 0x55;

	if (Operation)
		u8UARTWriteCnt = 4;
	else
		u8UARTWriteCnt= 0;

	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 

}
PUBLIC void App_SerialSendNetworkIndication(uint8 u8Decide)
{
	TxSerialMsg[0]  = 0x15;
	TxSerialMsg[1]  = 0xAA;
	TxSerialMsg[2]  = 0x0A;
	TxSerialMsg[3]  = NwkIndication;
	TxSerialMsg[4]  = (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]  = (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]  = (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]  = (uint8)u32CmdCountID;
	TxSerialMsg[8]  = 0x00;	

	TxSerialMsg[9]  = u8Decide;
	TxSerialMsg[10] = 0x00;
	TxSerialMsg[11] = 0x00;
	TxSerialMsg[12] = 0x00;
	TxSerialMsg[13] = 0x00;
	TxSerialMsg[14] = 0x00;
	TxSerialMsg[15] = 0x00;
	TxSerialMsg[16] = 0x00;
	TxSerialMsg[17] = 0x00;
	TxSerialMsg[18] = 0x00;

	TxSerialMsg[19] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[20] = 0x55;

	u8UARTWriteCnt= 0;
	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 
}
// RemoteUnlock NormallyOpen
PUBLIC void App_SerialSendRemoteUnlock(uint8 *nPass, uint8 u8Cmd)
{
	if (nPass[0] > 10)
		return;

	uint8 i;

	memset(TxSerialMsg,0,sizeof(TxSerialMsg));

	TxSerialMsg[0]	= 0x15;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0x0A;
	TxSerialMsg[3]	= u8Cmd;
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	TxSerialMsg[8]	= 0x00;

	for (i = 1; i <= nPass[0] ; i++)
		TxSerialMsg[8+i] = nPass[i];

	TxSerialMsg[19] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[20] = 0x55;

	u8UARTWriteCnt = 0;	//Ricky Test
	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 
	DBG_vPrintf(TRACE_APP_UART, "\n App_SerialSendRemoteUnlock ");

}
PUBLIC void APP_SerialSendModelVersion(void)
{
	// 回复模块固件版本
	#if 1
	//uint8 TxSerialMsg_[43];
	TxSerialMsg[0]	= 0x2B;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0x20;
	//TxSerialMsg[3]	= ModelVersionReply;//
	TxSerialMsg[3]	= GetModelVersion;
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	TxSerialMsg[8]	= 0x01;
	uint8 cnt = 0;
	for(cnt = 0; cnt < 16; cnt++)
	{
		TxSerialMsg[9 + cnt] = (uint8)(sSensor.sBasicServerCluster.au8ModelIdentifier[cnt]);
	}
	
	uint64 ieee = ZPS_u64AplZdoGetIeeeAddr();

	for(cnt = 0; cnt < 16; cnt++)
	{
		TxSerialMsg[25 + cnt] = (uint8)((ieee >> (4 * cnt)) & 0x0F);
	}

	TxSerialMsg[41] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[42] = 0x55;
	//u32CmdCountID++;
	//HAL_UART_Write(TxSerialMsg);
	#else
	TxSerialMsg[0]	= 0x15;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0x0A;
	TxSerialMsg[3]	= ModelVersionReply;
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	TxSerialMsg[8]	= 0x00;

	TxSerialMsg[9]	= (uint8)'V';
	TxSerialMsg[10] = (uint8)(sSensor.sBasicServerCluster.au8ModelIdentifier[13]);
	
	TxSerialMsg[11] = (uint8)(sSensor.sBasicServerCluster.au8ModelIdentifier[15]);
	TxSerialMsg[12] = 0x00;
	TxSerialMsg[13] = 0x00;
	TxSerialMsg[14] = 0x00;
	TxSerialMsg[15] = 0x00;
	TxSerialMsg[16] = 0x00;
	TxSerialMsg[17] = 0x00;
	TxSerialMsg[18] = 0x00;

	TxSerialMsg[19] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[20] = 0x55;
	#endif

	u8UARTWriteCnt = 7;

	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 
/*
sSensor.sBasicServerCluster.au8ModelIdentifier[15];
*/
}
PUBLIC void App_SerialSendTimeSynchronizer(UTCTime u32UtcSecs)
{
	UTCTimeStruct UtcTime;

	TxSerialMsg[0]	= 0x15;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0x0A;
	TxSerialMsg[3]	= SetDoorLockTime;
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	TxSerialMsg[8]	= 0x00;

	App_ConvertUTCTime(&UtcTime, u32UtcSecs);

	TxSerialMsg[9]	= (uint8)(UtcTime.year&0xff);
	TxSerialMsg[10] = (uint8)(UtcTime.year>>8);
	TxSerialMsg[11] = UtcTime.month+1;
	TxSerialMsg[12] = UtcTime.day+1;
	TxSerialMsg[13] = UtcTime.hour;
	TxSerialMsg[14] = UtcTime.minutes;
	TxSerialMsg[15] = UtcTime.seconds;
	TxSerialMsg[16] = 0x00;
	TxSerialMsg[17] = 0x00;
	TxSerialMsg[18] = 0x00;
	
	TxSerialMsg[19] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[20] = 0x55;

	u8UARTWriteCnt= 0;
	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 

	DBG_vPrintf(TRACE_APP_UART, "\n App_SerialSendTimeSynchronizer ");
	DBG_vPrintf(TRACE_APP_UART, "\n %0x %d %d %d %d",u32UtcSecs,TxSerialMsg[10],TxSerialMsg[9],TxSerialMsg[11],TxSerialMsg[12]);

}
PUBLIC void APP_cbTimerSerialDioSet(void *pvParam)
{
	APP_ButtonsWakeUpMCU();
	ZTIMER_eStart(u8TimerWriteSerial, ZTIMER_TIME_MSEC(UART_WAKEUP_OUTTIME) );
}
PUBLIC void APP_cbTimerWriteSerial(void *pvParam)
{
//	static uint8 au8WriteBuf[22];
//	DBG_vPrintf(TRACE_APP_UART, "\n APP_cbTimerWriteSerial");	

	vAppSampleDoorLockPollChange(6);

	APP_ButtonsStopWakeUp();

#if 1
    vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
    vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);
#endif

	//memset(TxSerial,0,64);
	memset(TxSerial,0,sizeof(TxSerial));
	memcpy(TxSerial,TxSerialMsg,TxSerialMsg[0]);

	//u32CmdCountID++;
	DBG_vPrintf(TRACE_APP_UART, "\n u8UARTWriteCnt : %0x ",u8UARTWriteCnt);

	HAL_UART_Write(TxSerial);

	if (u8UARTWriteCnt < 7)
	{
		#if 0
		if(TxSerialMsg[3] == PwdOperation)// command id 0x73 is pwdoeration it has appcmdid, do not change this 4 bytes
		{
			u8UARTWriteCnt++;
			ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(800));
		}
		else
		{
			u8UARTWriteCnt++;
			// Ricky Todo Recalculate CRC
			App_SerialPrepareRecalculate();		
			ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(800));
		}
		#else
		u8UARTWriteCnt++;
		ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(800));
		#endif
	}
	else
	{
		u8UARTWriteCnt = 8;
		memset(TxSerialMsg,0,sizeof(TxSerialMsg));		
		
		ZTIMER_eStop(u8TimerWriteSerial);
		ZTIMER_eStop(u8TimerSerialDioSet);
		ZTIMER_eStop(u8TimerWriteSerial);

		App_CheckAtrributeFF33DataBuffer();
	}

}
PUBLIC void APP_cbTimerSerialTimeout(void *pvParam)
{
	u8RxCnt = 0;

	memset(RxSerialMsg,0,sizeof(RxSerialMsg));		
	DBG_vPrintf(TRACE_APP_UART, "\n APP_cbTimerSerialTimeout Set E_STATE_RX_WAIT_START ");
}
PUBLIC void App_StopSerialPrepareEnterSleep(void)
{
	DBG_vPrintf(TRACE_APP_UART, "\n RxSerialMsg[2][3][4][5][6] = %02x %02x %02x %02x %02x", RxSerialMsg[2], RxSerialMsg[3], RxSerialMsg[4], RxSerialMsg[5], RxSerialMsg[6]);
	DBG_vPrintf(TRACE_APP_UART, "\n TxSerial[3] = %02x", TxSerial[3]);
	DBG_vPrintf(TRACE_APP_UART, "\n TxSerialMsg[3][4][5][6][7] = %02x %02x %02x %02x %02x\n", TxSerialMsg[3], TxSerialMsg[4], TxSerialMsg[5], TxSerialMsg[6], TxSerialMsg[7]);
	
#if 0
	u8UARTWriteCnt = 8;
	memset(TxSerialMsg,0,sizeof(TxSerialMsg));		

	APP_ButtonsStopWakeUp();
	ZTIMER_eStop(u8TimerWriteSerial);
	ZTIMER_eStop(u8TimerSerialDioSet);
#else
	#if 0 // just judging Command ID and Serial ID
		if(RxSerialMsg[2] == TxSerialMsg[3] 
			&& RxSerialMsg[3] == TxSerialMsg[4]
				&& RxSerialMsg[4] == TxSerialMsg[5]
					&& RxSerialMsg[5] == TxSerialMsg[6]
						&& RxSerialMsg[6] == TxSerialMsg[7])//if rx commandid == tx commandid, do clear tx buffer
	#else
		if(RxSerialMsg[2] == TxSerialMsg[3] || TxSerialMsg[3] == 0)// just judging CommandID TxSerial
	#endif
		{
			DBG_vPrintf(TRACE_APP_UART, " Rx Commandid And SerialId Is Matching\n");
			
			ZTIMER_eStop(u8TimerWriteSerial);
			ZTIMER_eStop(u8TimerSerialDioSet);

			u8UARTWriteCnt = 8;
			memset(TxSerialMsg,0,sizeof(TxSerialMsg));		

			APP_ButtonsStopWakeUp();
			
			App_CheckAtrributeFF33DataBuffer();
		}
		else
		{
			DBG_vPrintf(TRACE_APP_UART, " Rx Commandid And SerialId No Matching\n");
		}
#endif


}
void App_Handle_User_Write_Attribute(uint8 *palyload, uint16 size)//moons
{
	uint16 attributeId = 0;
	uint8 *p = palyload;
	//palyload frame
	//Frame control: 1 byte
	//Transaction Sequence Number: 1 byte
	//General Command Frame: 1 byte 0x02=>write attribute
	//Attribute Low byte: 1 byte
	//Attribute Height byte: 1 byte
	//Attribute Data type: 1 byte
	//Attribute Data type Len: 1 byte
	//Attribute Data : 1 byte

	attributeId |= *(palyload + 3);
	attributeId |= (*(palyload + 4)) << 8;
	
	#if 0
		DBG_vPrintf(TRACE_APP_UART, "\nHandleAttributesWrite size = %d, au8Storage = ", size);
		uint16 i = 0;
		for(i; i < size; i++)
		{
			DBG_vPrintf(TRACE_APP_UART, "%x ",  *palyload++);				
		}
		DBG_vPrintf(TRACE_APP_UART, "\nAttributeId = %x ",	attributeId);
	#endif
	
#ifdef TEMPORAEY_PASSWORD	
	if(attributeId == 0xFF33)
	{
		
		uint8 i = 0;
		for(i = 2; i < 20; i++)// set password to 0, report add tempoary password result!
		{
			sSensor.sDoorLockServerCluster.au8TemoaryPassword[i] = 0;	
		}
		
		if(size != 17)
		{
			DBG_vPrintf(TRACE_APP_UART, "\nWrite Attribute Palyload Size Error = %d", size);
			sSensor.sDoorLockServerCluster.au8TemoaryPassword[0] = 0xF1;//error code 0xF1 means illgeal bytes 
		}
		else//received bytes corrected!!
		{
			uint16 FunctionByte = 0, PasswordTime = 0, off = 7;
			FunctionByte = *(p + 7 ) | ((*(p + 8 )) << 8);
			PasswordTime = *(p + 15) | ((*(p + 16)) << 8);
			DBG_vPrintf(TRACE_APP_UART, "\nFunctionByte =  %x, Password Valid time = %x", FunctionByte, PasswordTime);

			uint8 DataType = 0;
			DataType = *(p + 5 );
			if(DataType == 0x42)
			{
				if(FunctionByte == 0x0000)
				{
					#if 0
						DBG_vPrintf(TRACE_APP_UART, "\nAdd New Tempoary Password: ");
						for(i = 0; i < 6; i++)
						{
							DBG_vPrintf(TRACE_APP_UART, "* ");//*(p + 9 + i));	
						}
						DBG_vPrintf(TRACE_APP_UART, "\nValid time = %x", PasswordTime);
					#endif
				}
				else if(FunctionByte == 0xFFFF)
				{
					DBG_vPrintf(TRACE_APP_UART, "\nRemove All Tempoary Password");
				}
				else
				{
					DBG_vPrintf(TRACE_APP_UART, "\nRemove No.%x Tempoary Password: ", FunctionByte);
				}
				TempoaryPasswordResendCnt = 0;
				App_SerialSendAddTempoaryPassword(p + 7);//send tempoary password to dooc lock!!
				return ;//no send the report!!
			}
			else
			{
				DBG_vPrintf(TRACE_APP_UART, "\nDataType error = %x", DataType);
				sSensor.sDoorLockServerCluster.au8TemoaryPassword[0] = 0xF2;//error code 0xF2 means illgeal data bytes!!
			}
		}

		#if 1
			DBG_vPrintf(TRACE_APP_UART, "\nsDoorLockServerCluster.au8TemoaryPasswor: ");
			for(i = 0; i < 20; i++)
			{
				DBG_vPrintf(TRACE_APP_UART, "%x ",  sSensor.sDoorLockServerCluster.au8TemoaryPassword[i]);
			}
			DBG_vPrintf(TRACE_APP_UART, "\n");
		#endif
		//if handle write success, it will not be run this place
		App_SendReportTempoaryPasswordState();//report add tempoary password error !!
	}
#endif
#if 0
	else if (attributeId == E_CLD_BAS_ATTR_ID_IR_DATA) 
	{
		if(*(p + 7) == 0xFB)
		{
			App_SerialSendEnableDoorLoockPwd(*(p + 8), p + 9);
		}
		
	#if 0
			uint8 i= 0;
			DBG_vPrintf(TRACE_WR_ATTR, "\n eAttributeDataType %0x u16AttributeId %0x",eAttributeDataType,u16AttributeId);
			DBG_vPrintf(TRACE_WR_ATTR,"\n Recviced : ");
			while(i++<32)
				DBG_vPrintf(TRACE_WR_ATTR,"%0x ",sSensor.sBasicServerCluster.au8BasicIRData[i]);	
			u16BasicAttrID =  u16AttributeId;
			ZTIMER_eStart(u8TimerBasicWriteAttr,ZTIMER_TIME_MSEC(20));// Ricky Call Back Basic Write Attribute
	#endif
	}
#endif
	//else if (attributeId == E_CLD_BAS_ATTR_ID_TRANSPRT_TRANS_CNT)
	if(attributeId == E_CLD_BAS_ATTR_ID_TRANSPRT_TRANS_CNT)
	{
		uint8 CachedataCnt = 0;
		CachedataCnt = *(p + 6);
		
		GetPollControlResponse = 0;// yes, get the cache all count response so clean the flag
		ZTIMER_eStop(u8TimerCheckGetCacheKeepRun);// and stop time out timer
		requestCacheStatus = E_NoWaitAnything;// reset get cache statu

		GateWayCacheAllCnt = CachedataCnt;
		
		if(GateWayCacheAllCnt > 30 || GateWayCacheAllCnt < 0)
		{
			GateWayCacheAllCnt = 30;
		}
		
		if(GateWayCacheAllCnt != 0)
		{

			if(GateWayCacheAllCnt * 20 > 255)
				App_PollRateCnt = 255;
			else
				App_PollRateCnt = GateWayCacheAllCnt * 20;
			
			DBG_vPrintf(TRUE, "E_WaitOneCacheDataResponse The Number Of Cache Is %d, \n Set App_PollRateCnt To %d\n", CachedataCnt, App_PollRateCnt);
			RequestNextCacheNum = 1;
			RecordRequestNextCacheNum = RequestNextCacheNum;//record the last time request cache data number, if gateway no response in 5s do request againt
			App_AskOneOfCache();// request one cache data from gateway
			requestCacheStatus = E_WaitOneCacheDataResponse;
			ZTIMER_eStart(u8TimerCheckGetCacheKeepRun, ZTIMER_TIME_SEC(4));// and start time out timer
		}
		else
		{
			//入网阶段的1分钟data request不允许被打断、远程开锁的1分钟data request也不允许被打断、do not change data request when OTA switch to new image 
			if(u8JoinNetworkFlowPath == 0 && App_SampleDoorLockWakeUpTimeOutDecide() && !bOTA_IsWaitToUpgrade())
			{
				if(bPeriodicEventWakeup == TRUE)//如果周期性的心跳醒来的话就可以直接休眠了，如果是按键唤醒就延时一会后休眠
				{
					//App_PollRateCnt = 2;//jabin remove 0->2 防止response未响应旧休眠，存在父节点时
				}
			}

			requestCacheStatus = E_NoWaitAnything;// reset get cache statu, moon jabin remove
			//ZTIMER_eStop(u8TimerCheckGetCacheKeepRun);
			//ZTIMER_eStart(u8TimerCheckGetCacheKeepRun, ZTIMER_TIME_SEC(2));//moon jabin 3->2
			
			DBG_vPrintf(TRUE, "E_NoWaitAnything Gateway no Cache Data!");
		}
	#if 0
			DBG_vPrintf(TRACE_WR_ATTR, "\n eAttributeDataType %0x u16AttributeId %0x",eAttributeDataType,u16AttributeId);
			DBG_vPrintf(TRACE_WR_ATTR,"\n u8TransportCnt : %0x \n",sSensor.sBasicServerCluster.u8TransportCnt);	
			u16BasicAttrID =  u16AttributeId;
			ZTIMER_eStart(u8TimerBasicWriteAttr,ZTIMER_TIME_MSEC(20));
	#endif
	}
	else if(attributeId == E_CLD_BAS_ATTR_ID_IR_DATA) 
	{
		if(*(p + 7) == 0xFB)
		{
			//App_SerialSendEnableDoorLoockPwd(*(p + 8), p + 9);
			//App_SendReportPollControlSendCacheAck();// send ack
			App_SendDataImmediatelyOrSaveIntoCacheBuffer(0, EnableDoorLock, *(p + 8), p + 9);
			//APP_CheckGateWayCacheAllCnt();
		}
		
		#if 0
			uint8 i= 0;
			DBG_vPrintf(TRACE_WR_ATTR, "\n eAttributeDataType %0x u16AttributeId %0x",eAttributeDataType,u16AttributeId);
			DBG_vPrintf(TRACE_WR_ATTR,"\n Recviced : ");
			while(i++<32)
				DBG_vPrintf(TRACE_WR_ATTR,"%0x ",sSensor.sBasicServerCluster.au8BasicIRData[i]);	
			u16BasicAttrID =  u16AttributeId;
			ZTIMER_eStart(u8TimerBasicWriteAttr,ZTIMER_TIME_MSEC(20));// Ricky Call Back Basic Write Attribute
		#endif
	}
	else if(attributeId == E_CLD_POLL_CONTROL_ATTR_ID_SLEEP_TIME) 
	{
		uint32 u32Ticks = 0;
		//DBG_vPrintf(TRUE, " Set Sleep Time : %d",u32Ticks);
		uint8 i = 0;
		p += 6;
		for(; i < 4; i++)
		{
			//DBG_vPrintf(TRUE, "%02x ", *p);
			u32Ticks |= ((uint32)(*p) << (i * 8));
			p++;
		}
		
		if(u32Ticks > 2)
		{
			u32SleepTimeTicks = u32Ticks;
			APP_AckPollControlSetSleepTime(0);
			reStarSystCount = 57600/u32SleepTimeTicks;
			//DBG_vPrintf(1," \n set sleep time reStarSystCount %d u32SleepTimeTicks %d\n",reStarSystCount, u32SleepTimeTicks);
			PDM_eSaveRecordData(PDM_ID_APP_SLEEP_TIME,&u32SleepTimeTicks,sizeof(uint32));
		}
		else
		{
			APP_AckPollControlSetSleepTime(1);
		}
		
		DBG_vPrintf(TRUE, " Set Sleep Time : %d ",u32Ticks);
	}
	else
	{
		DBG_vPrintf(TRACE_APP_UART, "\n NO Handle This Attribute %x ", attributeId);
	}
}
PUBLIC void App_AckSleepInTime(uint8 ack)
{	
	DBG_vPrintf(1, "\n App Ack Command Sleep In Time %x! ", ack);
	uint8 TxSerialMsg_[12] = {0};
	TxSerialMsg_[0]	= 12;
	TxSerialMsg_[1]	= 0xAA;
	TxSerialMsg_[2]	= 0x01;
	TxSerialMsg_[3]	= SleepInTime;
	TxSerialMsg_[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg_[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg_[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg_[7]	= (uint8)u32CmdCountID;
	TxSerialMsg_[8]	= 0x01;
	TxSerialMsg_[9]	= ack;
	TxSerialMsg_[10] = vUART_TxCharCRC(TxSerialMsg_);
	TxSerialMsg_[11] = 0x55;
	
	HAL_UART_Write(TxSerialMsg_);
	u32CmdCountID++;
}
PUBLIC void APP_SerialSendPwdOperationData(uint8 *p, uint16 size)//moon
{
	uint8 i = 0;

	#if 1
		uint8 *pp;
		pp = p;
		DBG_vPrintf(1, " Storage %d = ", size);
		for(; i < size; i++)
		{
			DBG_vPrintf(1, "%x ", *pp);
			pp++;
		}
		DBG_vPrintf(1, "\n");
	#endif

#if 1// for suning huitailong
	
	uint8 AppCmdId = 0, pwdVersion = 0, commandId = 0;

	DBG_vPrintf(1, "p[0]==%02x, p[1]==%02x\n", p[0], p[1]);
	
	if(p[0] == 0xFF && p[1] == 0xFD)// for all other command， 其它的一些不包含或者不需要服务器指令序列号的透传缓存的命令字
	{
		if(size > 3)// the least data is "FF FD C3 00"
		{
			commandId = p[2];
			p += 3;// subtract "FF FD"  + "commandId"
			size -= 3;// subtract 3 bytes of "FF FD" + "commandId"
		}
		else
		{
			return;// data no enought
		}
	}
	else if(p[0] == 0xFF && p[1] == 0xFC)// for password operation ， 这个专门用于服务器下发密码，带服务器指令序列
	{
		if(size > 20)
		{
			commandId = PwdOperation;// pwd operation		
			pwdVersion = p[2];// 01 : ying hua door lock; 02 : feibit other door lock
			AppCmdId = p[3];

			p += 4;// subtract "FF FC" + "pwdVersion" + "AppCmdId"
			size -= 4;// subtract 4 bytes of "FF FC" + "pwdVersion" + "AppCmdId"
			
			DBG_vPrintf(1, "\npwdVersion=%02x\n", pwdVersion);
			DBG_vPrintf(1, "\nAppCmdId=%02x\n", AppCmdId);

			if(pwdVersion == 0x01)// 兼容樱花公寓门锁
			{
				APP_SerialSendPwdOperationDataForYinghua(AppCmdId, size, p);// 由于前期协议制定的有所偏差，所以樱花公寓门锁需要转译一下字节序再传给门锁
				return;
			}
			else if(pwdVersion == 0x02)
			{
				// 飞比整合版本门锁的密码操作及下行指令的字节序与从服务器下发的一样，不再需要转字节序，直接判断立即传给门锁或者先放入缓存队列 
			}
		}
		else
		{
			return;// data no enought
		}
	}
	else if(p[0] == 0xFF && p[1] == 0xFB)// "FF FB" + "Version" + "AppCmdId" + "commandId"，除了下发密码，其他需要服务器指令序列号的指令，串口指令邋ID需大于或等于ServerAddDoorUserSeed
	{//后期由服务器下发的所有指令都需要服务器指令序列，否则应答服务器时服务器端的同事根本不知道应答的是哪一个的指令
		if(size > 4)
		{
			pwdVersion = p[2];// 
			AppCmdId = p[3];
			commandId = p[4];

			p += 5;// subtract "FF FC" + "pwdVersion" + "AppCmdId"
			size -= 5;// subtract 4 bytes of "FF FC" + "pwdVersion" + "AppCmdId"
			
			DBG_vPrintf(1, "\npwdVersion=%02x\n", pwdVersion);
			DBG_vPrintf(1, "\nAppCmdId=%02x\n", AppCmdId);
			DBG_vPrintf(1, "\ncommandId=%02x\n", commandId);
		}
		else
		{
			return;// data no enought
		}
	}
	else if(p[0] == 0xFF && p[1] == 0xFA)
	{
		//commandId = ServerSendMsgToLock;
		AppCmdId = p[3];
		DBG_vPrintf(1, "\ncommandId=%02x AppCmdId=%02x\n", commandId, AppCmdId);
		if( p[4] == 0x01 )
		{
			memset(RxSerialMsg, 0, sizeof(RxSerialMsg));
			RxSerialMsg[0] = 0xAA;
			RxSerialMsg[1] = 0x01;
			RxSerialMsg[2] = ServerSendMsgToLock; //门锁通用消息命令字

			RxSerialMsg[3] = 0x00;
			RxSerialMsg[4] = 0x00;
			RxSerialMsg[5] = 0x00;
			RxSerialMsg[6] = AppCmdId;

			RxSerialMsg[7] = 0X01;
			RxSerialMsg[8] = 0X01;

			memcpy(sSensor.sBasicServerCluster.au8Transport, RxSerialMsg, RxSerialMsg[1] + 8);
			sSensor.sBasicServerCluster.sTransport.u8Length = RxSerialMsg[1] + 8;
			sSensor.sBasicServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
			vSendBasicReport();
			u8RestartForOTA = 1;
			return;
		}else if( p[4] == 0x04 )
		{
			commandId = ServerSendMsgToLock;
			//AppCmdId = p[3];
			p += 4;
			size -= 28;

		}else{
			commandId = ServerSendMsgToLock;
			//AppCmdId = p[3];
			p += 4;
			size -= 28;
			DBG_vPrintf(1, "unkown commandid ");
		}

		//vAHI_SwReset();

	}
	else
	{
		APP_SerialSendPwdOperationDataForYinghua(PwdOperation, size, p);//兼容无服务器指令邋ＩＤ的版本
		return;// unknow command flag
	}

	App_SendDataImmediatelyOrSaveIntoCacheBuffer(AppCmdId, commandId, size, p);// 判断立即传给门锁或者先放入缓存队列

#endif

}

PUBLIC void APP_SerialSendPwdOperationDataForYinghua(uint8 AppCmdId, uint16 size, uint8 *p)//moon
{	
	uint8 i = 0;
	uint8 offset = 0;

	uint8 PwdOperationType = *(p + 13);//action used to pwd operation type
	uint8 PwdNumberLowByte = *(p + 0);
	uint8 PwdNumberHeightByte = *(p + 1);
	uint8 OperationThePwdType = 0;
	uint8 FirstPwdLen = *(p + 14);
	uint8 FirstPwdUsedTimes = *(p + 12);
	uint8 YingHuaPwdData[30] = {0};

	if(PwdOperationType == 1)// add one password
	{
		if(*(p + 4) == 0xFF && *(p + 5) == 0xFF && *(p + 6) == 0xFF && *(p + 7) == 0xFF 
			&& *(p + 8) == 0xFF && *(p + 9) == 0xFF && *(p + 10) == 0xFF && *(p + 11) == 0xFF
			&& FirstPwdUsedTimes == 0xFF)
		{
			OperationThePwdType = 2;// one forever password 
		}
		else if(FirstPwdUsedTimes == 0xFF)
		{
			OperationThePwdType = 3;//  aging password
		}
		else
		{
			OperationThePwdType = 1;// temporary password
		}
	}

	DBG_vPrintf(1, "\n PwdOperationType = %x ", PwdOperationType);
	DBG_vPrintf(1, "\n PwdNumberLowByte = %x ", PwdNumberLowByte);
	DBG_vPrintf(1, "\n PwdNumberHeightByte = %x ", PwdNumberHeightByte);
	DBG_vPrintf(1, "\n OperationThePwdType = %x ", OperationThePwdType);
	DBG_vPrintf(1, "\n FirstPwdLen = %x ", FirstPwdLen);
	DBG_vPrintf(1, "\n FirstPwdUsedTimes = %x \n", FirstPwdUsedTimes);

#if 1
	YingHuaPwdData[offset++] = PwdOperationType;// action switch to pwd operation type
	YingHuaPwdData[offset++] = PwdNumberLowByte;// user ID switch to pwd number
	YingHuaPwdData[offset++] = PwdNumberHeightByte;// user ID switch to pwd number

	YingHuaPwdData[offset++] = OperationThePwdType;// pwd type
	YingHuaPwdData[offset++] = FirstPwdLen;// pwd len
	YingHuaPwdData[offset++] = FirstPwdUsedTimes;// ped used times
	
	for(i = 0; i < FirstPwdLen; i++)//
	{
		YingHuaPwdData[offset++] = *(p + 15 + i);
	}	

	uint8 secondpwdlen = *(p + 15 + FirstPwdLen);
	DBG_vPrintf(1, "\n Second Pwd Len = %x \n", secondpwdlen);
	YingHuaPwdData[offset++] = secondpwdlen;
	for(i = 0; i < secondpwdlen; i++)//
	{
		YingHuaPwdData[offset++] = *(p + 15 + i + 1 + FirstPwdLen);
	}
		
	for(i = 0; i < 8; i++)// start time and end time
	{
		YingHuaPwdData[offset++] = *(p + 4 + i);
	}


	App_SendDataImmediatelyOrSaveIntoCacheBuffer(AppCmdId, PwdOperation, offset, YingHuaPwdData);
#else
	TxSerialMsg[0]	= 0;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0;
	TxSerialMsg[3]	= 0x73;
	#if 0
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	#else
	TxSerialMsg[4]	= 0;//(uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= 0;//(uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= 0;//(uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= AppCmdId;//(uint8)u32CmdCountID;
	#endif
	TxSerialMsg[8]	= 0x00;
	
	TxSerialMsg[9]	= PwdOperationType;// action switch to pwd operation type
	TxSerialMsg[10]	= PwdNumberLowByte;// user ID switch to pwd number
	TxSerialMsg[11]	= PwdNumberHeightByte;// user ID switch to pwd number

	TxSerialMsg[12]	= OperationThePwdType;// pwd type
	TxSerialMsg[13] = FirstPwdLen;// pwd len
	TxSerialMsg[14] = FirstPwdUsedTimes;// ped used times

	uint8 offset = 15;
	
	for(i = 0; i < FirstPwdLen; i++)//
	{
		TxSerialMsg[offset++] = *(p + 15 + i);
	}	

	uint8 secondpwdlen = *(p + 15 + FirstPwdLen);
	DBG_vPrintf(1, "\n Second Pwd Len = %x \n", secondpwdlen);
	TxSerialMsg[offset++] = secondpwdlen;
	for(i = 0; i < secondpwdlen; i++)//
	{
		TxSerialMsg[offset++] = *(p + 15 + i + 1 + FirstPwdLen);
	}
		
	for(i = 0; i < 8; i++)//
	{
		TxSerialMsg[offset++] = *(p + 4 + i);
	}

	TxSerialMsg[2] = 15 + FirstPwdLen + secondpwdlen;

	TxSerialMsg[0] = 1 + 10 + TxSerialMsg[2];

	TxSerialMsg[offset++] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[offset++] = 0x55;

	u8UARTWriteCnt= 0;

	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 
#endif
}
void App_SendDataImmediatelyOrSaveIntoCacheBuffer(uint8 AppCmdId, uint8 commandId, uint8 size, uint8 *p)//E_CLD_DOOR_LOCK_CMD_SET_YEAR_DAY_SCHEDULE
{
	uint8 num = 0;
	uint8 offset = 9;
	
	// write attribute 0xFF33`s data is right, try judging whether transmit to mcu immediately or save into u8ServerWriteAttributData	
	DBG_vPrintf(TRACE_APP_UART, "\nu8UARTWriteCnt=%d", u8UARTWriteCnt);
	if(u8UARTWriteCnt >= 7)
	{
		memset(TxSerialMsg, 0, sizeof(TxSerialMsg));
	}

	u32CmdCountID++;
	
	if(TxSerialMsg[0] == IS_EMPTY && TxSerialMsg[1] == IS_EMPTY
		&& TxSerialMsg[2] == IS_EMPTY && TxSerialMsg[3] == IS_EMPTY)// if transfer data buffer is empty
	{
		DBG_vPrintf(TRACE_APP_UART, "\nTransmit To Mcu Immediately");// transmit to mcu immediately
	}
	else
	{
		uint8 cnt = 0;
		DBG_vPrintf(TRACE_APP_UART, "\nTxSerialMsg[3] Is Not Empyt ComId = %02x ", TxSerialMsg[3]);// save into u8ServerWriteAttributData
		for(cnt = 0; cnt < SERVER_WA_BUFFFERSIZE; cnt++)
		{
			if(u8ServerWriteAttributData[ cnt ][0] == IS_EMPTY)// u8ServerWriteAttributData[i][0] is empty so save data into buffer
			{
				uint8 Cacheoffset = 0;
				DBG_vPrintf(TRACE_APP_UART, "\n\nSave Data Into u8ServerWriteAttributData[%d][x]", cnt);

				u8ServerWriteAttributData[ cnt ][0] = size;//commandDataLen;
				u8ServerWriteAttributData[ cnt ][1] = commandId;//commandID;

				if(commandId == PwdOperation || commandId >= ServerAddDoorUserSeed)// PwdOperation、ServerAddDoorUserSeed 这几个加了服务器指令序列号
				{
					u8ServerWriteAttributData[ cnt ][2] = AppCmdId;
					Cacheoffset = 3;					
					//u8ServerWriteAttributData[ cnt ][3 ~ end] is data
				}
				else// 这些是没有加服务器指令序列号的，所以无服务器的指令唯一标识或者判断
				{
					Cacheoffset = 2;
					//u8ServerWriteAttributData[ cnt ][2 ~ end] is data
				}

				for(num = 0; num < size; num++)
				{
					u8ServerWriteAttributData[ cnt ][Cacheoffset++] = *p++;
				}

			#if 0// print data to check
				DBG_vPrintf(TRACE_APP_UART, "\n\nu8ServerWriteAttributData[%d] :", cnt);
				for(num = 0; num < size + 2; num++)
				{
					DBG_vPrintf(TRACE_APP_UART, "%02x ", u8ServerWriteAttributData[ cnt ][num]);
				}
			#endif
				
				break;
			}
		}
		
		return;
	}
	
	TxSerialMsg[0]	= 0;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= size;// len;
	TxSerialMsg[3]	= commandId;// commond;
	
	DBG_vPrintf(1,"commandId 0x%02x ServerAddDoorUserSeed 0x%02x",commandId,ServerAddDoorUserSeed);
	if(commandId == PwdOperation || commandId >= ServerAddDoorUserSeed)//暂时未完成服务器指令ID
	{
		TxSerialMsg[4]	= 0;
		TxSerialMsg[5]	= 0;
		TxSerialMsg[6]	= 0;
		TxSerialMsg[7]	= (uint8)AppCmdId;
	}
	else
	{
		TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
		TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
		TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
		TxSerialMsg[7]	= (uint8)u32CmdCountID;
	}
	
	TxSerialMsg[8]	= 0x00;// ack byte

	for(num = 0; num < size; num++)
	{
		TxSerialMsg[offset++] = *p++;
	}

	TxSerialMsg[0] = 1 + 10 + size;

	TxSerialMsg[offset++] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[offset++] = 0x55;

	u8UARTWriteCnt = 0;

	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 

}
void App_CheckAtrributeFF33DataBuffer(void)
{
	#if 1 // check whether still have data no transmit to mcu
	uint8 cnt = 0;
	DBG_vPrintf(TRACE_APP_UART, " Check Wheather Still Data Need Transmit To MCU\n");// transmit to mcu immediately
	for(cnt = 0; cnt < SERVER_WA_BUFFFERSIZE; cnt++)
	{
		//[][0] = FF33wirte attribute [commandDataLen]
		//[][1] = FF33wirte attribute [commandID]
		
		//*********
		//[][2~end] = FF33wirte attribute [data] 
		//*********
		// OR
		//[][2] = FF33wirte attribute [AppCmdId]
		//[][3~end] 

		if(u8ServerWriteAttributData[ cnt ][0] != 0)// if this buffer have data so retrasmit to mcu
		{
			uint8 offset = 9, num = 0;
			uint8 Cacheoffset = 2;
			
			TxSerialMsg[0]	= 0;
			TxSerialMsg[1]	= 0xAA;
			TxSerialMsg[2]	= u8ServerWriteAttributData[ cnt ][0];// [commandDataLen]
			TxSerialMsg[3]	= u8ServerWriteAttributData[ cnt ][1];// [commandID]
			
			if(TxSerialMsg[3] == PwdOperation)
			{
				TxSerialMsg[4]	= 0;
				TxSerialMsg[5]	= 0;
				TxSerialMsg[6]	= 0;
				TxSerialMsg[7]	= u8ServerWriteAttributData[ cnt ][2];// [AppCmdId]
				Cacheoffset = 3;// u8ServerWriteAttributData[ cnt ][3 ~ end] is  [data]
			}
			else
			{
				TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
				TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
				TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
				TxSerialMsg[7]	= (uint8)u32CmdCountID;
				Cacheoffset = 2;// u8ServerWriteAttributData[ cnt ][2 ~ end] is  [data]
			}
			
			TxSerialMsg[8]	= 0x00;
			
			for(num = 0; num < u8ServerWriteAttributData[ cnt ][0]; num++)
			{
				TxSerialMsg[offset++] = u8ServerWriteAttributData[ cnt ][Cacheoffset + num];//u8ServerWriteAttributData[ cnt ][2 ~ end] is  [data] OR u8ServerWriteAttributData[ cnt ][3 ~ end] is  [data] 
			}

			TxSerialMsg[0] = 1 + 10 + TxSerialMsg[2];

			TxSerialMsg[offset++] = vUART_TxCharCRC(TxSerialMsg);
			TxSerialMsg[offset++] = 0x55;

			u8UARTWriteCnt = 0;

			ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 
			DBG_vPrintf(TRACE_APP_UART, "\n\n Send u8ServerWriteAttributData[%d][x] To Mcu", cnt);

			//u8ServerWriteAttributData[ cnt ][0] = IS_EMPTY;// clearn [commandDataLen]
			memset(&u8ServerWriteAttributData[ cnt ][0], 0, 64);
			break;
		}
	}
	#endif
}

void General_SendBasic(uint8 AppCmdCounter)
{
	memset(RxSerialMsg, 0, sizeof(RxSerialMsg));

	RxSerialMsg[0] = 0xAA;
	RxSerialMsg[1] = 0x01;
	RxSerialMsg[2] = DoorLockErrState; //门锁通用消息命令字

	RxSerialMsg[3] = 0x00;
	RxSerialMsg[4] = 0x00;
	RxSerialMsg[5] = 0x00;
	RxSerialMsg[6] = AppCmdCounter;

	RxSerialMsg[7] = 0x01;
	RxSerialMsg[8] = 0x03;
#if 1
	memcpy(sSensor.sBasicServerCluster.au8DoorLockMSG, RxSerialMsg, RxSerialMsg[1] + 8);
	sSensor.sBasicServerCluster.sDoorLockMSG.u8Length = RxSerialMsg[1] + 8;
	sSensor.sBasicServerCluster.sDoorLockMSG.u8MaxLength = sizeof(RxSerialMsg);
	vSendDoorLockMSGReport();
#else
	memcpy(sSensor.sBasicServerCluster.au8Transport, RxSerialMsg, RxSerialMsg[1] + 8);
	sSensor.sBasicServerCluster.sTransport.u8Length = RxSerialMsg[1] + 8;
	sSensor.sBasicServerCluster.sTransport.u8MaxLength = sizeof(RxSerialMsg);
	vSendBasicReport();
#endif
}
