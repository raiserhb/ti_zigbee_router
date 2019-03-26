


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

#include "pwrm.h"
#include "app_nwk_event_handler.h"
#include "app_event_handler.h"

#include "App_WL_DoorLock.h"
#include "app_SampleDoorLock_Uart.h"
#include "app_occupancy_buttons.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_APP_UART
	#define TRACE_APP_UART				FALSE
#else
	#define TRACE_APP_UART				TRUE
#endif
PRIVATE uint8 TempoaryPasswordBackup[10];
PRIVATE uint8 TempoaryPasswordBackup2[10];

extern uint8 GateWayCacheAllCnt;
extern uint8 RequestNextCacheNum;
extern uint8 RecordRequestNextCacheNum;
extern uint8 RetryRequestNextCacheNum;
extern uint8 crtreasknum;
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
PRIVATE uint8 RxSerial[80];		//Ricky ���ڴ�Ŵ��ڽ�������
PUBLIC uint8 RxSerialMsg[80];	//Ricky ���ڴ�����������
PRIVATE uint8 TxSerialMsg[64];	//Ricky ���ڴ�Ŵ��ڷ�������
PRIVATE uint8 TxSerial[64];		//Ricky ���ڴ���д��

#ifdef TEMPORAEY_PASSWORD	
char TempoaryPasswordResendCnt = 0;
extern uint8 u8TimerTemporayPasswordResend;
#endif

#define SERVER_WA_BUFFFERSIZE (10)
PUBLIC uint8 u8ServerWriteAttributData[SERVER_WA_BUFFFERSIZE][64];
//[][0] = FF33wirte attribute [commandDataLen]
//[][1] = FF33wirte attribute [commandID]

//*********
//[][2~end] = FF33wirte attribute [data] 
//*********
// OR
//[][2] = FF33wirte attribute [AppCmdId]
//[][3~end] = FF33wirte attribute [data] 
//*********

extern ts_SerialCache tsSerialBuffer;
extern uint8 GateWayCacheAllCnt;
extern uint8 GetPollControlResponse;
/****************************************************************************/
/***        Functions 		                                              ***/
/****************************************************************************/

PRIVATE uint8 vUART_RxCharCRC(uint8 *SerialMsg);
PUBLIC void APP_vProcessIncomingSerialCommands ( uint8 u8RxByte );
PUBLIC teSL_RxAcknowlegement bSL_ReadMessage(uint8 u8Data);
PUBLIC void UART_vSetBaudRate ( uint32 u32BaudRate );
PUBLIC bool_t UART_bTxReady();
PUBLIC void APP_SerialSendAcknowlegement(uint8 cmdType,bool Ack);

PUBLIC void App_SerialSendRemoteUnlock(uint8 *nPass, uint8 u8Cmd);
PUBLIC void App_SerialSendNetworkIndication(uint8 u8Decide);
PUBLIC void App_SerialSendTimeSynchronizer(UTCTime u32UtcSecs);
PUBLIC void App_SerialSendAddTempoaryPassword(uint8 *u8p);

#ifdef SKYWORTH
PUBLIC void APP_SerialSendFingerBreak(void);

PUBLIC void App_SerialSendRemoteBindUnlock(uint8 *nPass);
PUBLIC void APP_SerialSendModelVersion(void);
PUBLIC void App_DB_ConfigHeartPeriod(uint8 Length);
PUBLIC void App_SerialSendFristBindUnLockNotification(uint8 *nPass);

#endif

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
	bAHI_UartEnable(E_AHI_UART_1, TxSerial, (uint16)sizeof(TxSerial),
					RxSerial, (uint16)sizeof(RxSerial));

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

	DBG_vPrintf(TRACE_APP_UART, "\n vAHI_UartSetInterrupt");
    /* Turn on modem status, tx, rx interrupts */
    vAHI_UartSetInterrupt(E_AHI_UART_1, FALSE, FALSE, FALSE, TRUE, E_AHI_UART_FIFO_LEVEL_1);
	DBG_vPrintf(TRACE_APP_UART, "\n vAHI_UartSetInterrupt");

	//Ricky ��ջ���
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

PUBLIC void HAL_UART_Write(uint8 *WriteBuf)
{
	uint8 i;

	if(WriteBuf[1] == 0)
		return ;

	DBG_vPrintf(TRACE_APP_UART, "\n HAL_UART_Write");
	for(i=0;i <= WriteBuf[0];i++)
		DBG_vPrintf(TRACE_APP_UART, " %02x",WriteBuf[i]);

#if 1
	i=1;
	while(i <  WriteBuf[0])
	{
		while ( !UART_bTxReady () && ( u8AHI_UartReadLineStatus ( E_AHI_UART_1 ) & E_AHI_UART_LS_THRE ) );
		vAHI_UartWriteData(E_AHI_UART_1, WriteBuf[i]);
		while ( !UART_bTxReady() && !( u8AHI_UartReadLineStatus ( E_AHI_UART_1 ) & E_AHI_UART_LS_TEMT ) );

		i++;
	}
#else
	u16AHI_UartBlockWriteData(E_AHI_UART_1,WriteBuf+1,WriteBuf[0]-1);

#endif

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

/****************************************************************************
 *
 * NAME: vUART_SetTxEnable
 *
 * DESCRIPTION  Set transmit enabled status.
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
PUBLIC void vUART_SetTxEnable(bool_t bEnable)	/**< Transmit enabled status */
{
	/* Changing setting ? */
	if (bTxEnable != bEnable)
	{
		/* Set new value */
		bTxEnable = bEnable;
	}
}

/****************************************************************************
 *
 * NAME: vUART_SetRxEnable
 *
 * DESCRIPTION  Set receive enabled status.
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
PUBLIC void vUART_SetRxEnable(bool_t bEnable)	/**< Receive interrupt status */
{
	/* Changing setting ? */
	if (bRxEnable != bEnable)
	{
		/* Set new value */
		bRxEnable = bEnable;

		/* Update Rx interrupt setting */
	    vAHI_UartSetInterrupt(E_AHI_UART_1, bModemInt, FALSE, TRUE, bRxEnable, E_AHI_UART_FIFO_LEVEL_1);

	    /* Not using automatic flow control ? */
		#ifdef ENABLE_HW_SW_FLOW_CONTROL
	    	/* Set RTS manually */
	    	vUART_SetRts(bRxEnable);
		#endif
	}
}

/****************************************************************************
 *
 * NAME: bUART_GetRxEnable
 *
 * DESCRIPTION  Get receive enabled status.
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
PUBLIC bool_t bUART_GetRxEnable(void)
{
	return bRxEnable;
}

/****************************************************************************
 *
 * NAME: bUART_GetTxEnable
 *
 * DESCRIPTION  Get transmit enabled status.
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
PUBLIC bool_t bUART_GetTxEnable(void)
{
	return bTxEnable;
}

/****************************************************************************/
/***    Implementation                          */
/****************************************************************************/
uint8 u8RxCnt = 0;
PUBLIC void APP_vProcessIncomingSerialCommands ( uint8 u8RxByte )
{
	static uint8 u8CacheNum = 0;//, u8RxCnt = 0;
	//DBG_vPrintf(TRACE_APP_UART, "[%02x:%d] ", u8RxByte, u8RxCnt);
	//DBG_vPrintf(TRACE_APP_UART, "[%02x] ", u8RxByte);
	//RxSerial[u8RxCnt] = u8RxByte;

	if(ZTIMER_eGetState(u8TimerSerialTimeout) != E_ZTIMER_STATE_RUNNING)
	{
		//DBG_vPrintf(TRACE_APP_UART, "\n u8TimerSerialTimeout,ZTIMER_TIME_MSEC(60)");
		ZTIMER_eStart(u8TimerSerialTimeout,ZTIMER_TIME_MSEC(60));// Serial timeout
	}

	if(u8RxCnt == 0 && u8RxByte == 0xAA)
	{
		//DBG_vPrintf(TRACE_APP_UART, "0xAA ");
		tsSerialBuffer.u8SerialData[u8CacheNum][0] = u8RxByte;
		u8RxCnt = 1;
		return;
	}

	if(u8RxCnt == 1)
	{
		//DBG_vPrintf(TRACE_APP_UART, "Len=%02x ", u8RxByte);
		tsSerialBuffer.u8SerialData[u8CacheNum][1] = u8RxByte;
		u8RxCnt = 2;
	}
	else
	{
		//DBG_vPrintf(TRACE_APP_UART, "[%d:%02x] ", u8RxCnt, u8RxByte);
		
		tsSerialBuffer.u8SerialData[u8CacheNum][u8RxCnt++] = u8RxByte;
		
		if(u8RxCnt == tsSerialBuffer.u8SerialData[u8CacheNum][1] + 10)
		{
			ZTIMER_eStop(u8TimerSerialTimeout);
			
			ZQ_bQueueSend (&APP_msgSerialRx, &u8CacheNum);
			
			u8CacheNum++;
			if(u8CacheNum > 5)
				u8CacheNum = 0;
			
			u8RxCnt = 0;
		}
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

	DBG_vPrintf(TRACE_APP_UART, "\n vUART_RxCharCRC : %0x ",u8CRC);
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
	// ����д��ֻ��һ�����	Ӧ������������ź�
	ZTIMER_eStart(u8TimerWriteSerial, ZTIMER_TIME_MSEC(1));
#else
	memset(TxSerial,0,sizeof(TxSerial));
	memcpy(TxSerial,TxSerialMsg,TxSerialMsg[0]);

    vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
    vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);

	HAL_UART_Write(TxSerial);

	memset(TxSerial,0,sizeof(TxSerial));
#endif

}

PUBLIC void APP_SerialSendJoinIndication(uint8 Operation, uint8 u8Decide)
{
	// ����������ʾ
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

#ifdef SKYWORTH
PUBLIC void App_SerialSendRemoteBindUnlock(uint8 *nPass)
{
	uint8 i;
	DBG_vPrintf(TRACE_APP_UART, "\n App_SerialSendRemoteBindUnlock ");

	for(i = 0;i < 6;i++)
		DBG_vPrintf(TRACE_APP_UART, " %0x ",nPass[i]);
	DBG_vPrintf(TRACE_APP_UART, "\n");

	if (nPass[0] > 10)
		return;

	memset(TxSerialMsg,0,sizeof(TxSerialMsg));

	TxSerialMsg[0]	= 0x15;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0x0A;
	TxSerialMsg[3]	= BindUnlock;
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	TxSerialMsg[8]	= 0x00;

	for (i = 1; i <= nPass[0] ; i++)
		TxSerialMsg[8+i] = nPass[i];

	TxSerialMsg[19] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[20] = 0x55;

	u8UARTWriteCnt= 0;	//Ricky Test
	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1));

}
#endif

#ifdef SKYWORTH
PUBLIC void App_SerialSendFristBindUnLockNotification(uint8 *nPass)
{
	uint8 i;

	TxSerialMsg[0]  = 0x15;
	TxSerialMsg[1]  = 0xAA;
	TxSerialMsg[2]  = 0x0A;
	TxSerialMsg[3]  = BindUnlockCheck;
	TxSerialMsg[4]  = (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]  = (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]  = (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]  = (uint8)u32CmdCountID;
	TxSerialMsg[8]  = 0x00;	

	TxSerialMsg[9]  = 0x00;
	TxSerialMsg[10] = 0x00;
	TxSerialMsg[11] = 0x00;
	TxSerialMsg[12] = 0x00;
	TxSerialMsg[13] = 0x00;
	TxSerialMsg[14] = 0x00;
	TxSerialMsg[15] = 0x00;
	TxSerialMsg[16] = 0x00;
	TxSerialMsg[17] = 0x00;
	TxSerialMsg[18] = 0x00;


	for (i = 1; i <= nPass[0] ; i++)
		TxSerialMsg[8+i] = nPass[i];

	TxSerialMsg[19] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[20] = 0x55;

	u8UARTWriteCnt= 0;
	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 
}
#endif

PUBLIC void APP_SerialSendModelVersion(void)
{
	// �ظ�ģ��̼��汾
	#if 1
	//uint8 TxSerialMsg_[43];
	TxSerialMsg[0]	= 0x2B;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0x20;
	TxSerialMsg[3]	= ModelVersionReply;
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	TxSerialMsg[8]	= 0x00;
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

	u8UARTWriteCnt= 0;

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
#if 0
PRIVATE void App_SerialPrepareRecalculate(void)
{
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;

	if (TxSerialMsg[2] == 0x0A)
	{
		TxSerialMsg[19] = vUART_TxCharCRC(TxSerialMsg);
		TxSerialMsg[20] = 0x55;
	}
	else if(TxSerialMsg[2] == 0x01)
	{
		TxSerialMsg[10] = vUART_TxCharCRC(TxSerialMsg);
		TxSerialMsg[11] = 0x55;
	}
	else
	{
		TxSerialMsg[9 + TxSerialMsg[2]] = vUART_TxCharCRC(TxSerialMsg);
	}


}
#endif
#ifdef WONLY
PUBLIC void App_SampleDoorLockSerialIEEERequest(void)
{

	TxSerialMsg[0]  = 0x15;
	TxSerialMsg[1]  = 0xAA;
	TxSerialMsg[2]  = 0x0A;
	TxSerialMsg[3]  = RequestIEEE;
	TxSerialMsg[4]  = (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]  = (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]  = (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]  = (uint8)u32CmdCountID;
	TxSerialMsg[8]  = 0x00;	

	TxSerialMsg[9]  = 0x00;
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

	if (TRUE == APP_bNodeIsInRunningState())
		ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_SEC(5));
	else
		ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_SEC(1));

}
#endif

#ifdef SKYWORTH
PUBLIC void APP_SerialSendFingerBreak(void)
{
	// ָ�ƴ����ж�
	TxSerialMsg[0]	= 0x15;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0x0A;
	TxSerialMsg[3]	= FingerBreak;
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	TxSerialMsg[8]	= 0x00;

	TxSerialMsg[9]	= 0x00;
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

PUBLIC void App_DB_ConfigHeartPeriod(uint8 Length)
{
#if 1
	// д�봮��
	TxSerialMsg[0] = Length+1;
	uint8 *pTxSerial = &TxSerialMsg[1];
	memcpy(pTxSerial,sBasicWriteAttributePValue,Length+1);

	u8UARTWriteCnt= 7;
	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 
#endif
}

#endif
PUBLIC void APP_cbTimerSerialDioSet(void *pvParam)
{
	DBG_vPrintf(1,"\nWake up Lock MCU\n");
	APP_ButtonsWakeUpMCU();
	ZTIMER_eStart(u8TimerWriteSerial, ZTIMER_TIME_MSEC(UART_WAKEUP_OUTTIME) );
}

PUBLIC void APP_cbTimerWriteSerial(void *pvParam)
{
//	static uint8 au8WriteBuf[22];
//	DBG_vPrintf(TRACE_APP_UART, "\n APP_cbTimerWriteSerial");	

	if (App_PollRateCnt < 6)
	{ 
		vAppSampleDoorLockPollChange(6);
	}

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
			DBG_vPrintf(TRACE_APP_UART, "\n Rx Commandid And SerialId Is Matching\n ");
			
			ZTIMER_eStop(u8TimerWriteSerial);
			ZTIMER_eStop(u8TimerSerialDioSet);

			u8UARTWriteCnt = 8;
			memset(TxSerialMsg,0,sizeof(TxSerialMsg));		

			APP_ButtonsStopWakeUp();
			
			App_CheckAtrributeFF33DataBuffer();
		}
		else
		{
			DBG_vPrintf(TRACE_APP_UART, "\n Rx Commandid And SerialId No Matching\n ");
		}
#endif


}

PUBLIC void APP_AskDoorLockVerInfo(void)
{
	uint8 TxSerialMsg_[12];
	TxSerialMsg_[0]	= 0x0C;
	TxSerialMsg_[1]	= 0xAA;
	TxSerialMsg_[2]	= 0x01;
	TxSerialMsg_[3]	= AskDoorLockVerInfo;
	TxSerialMsg_[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg_[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg_[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg_[7]	= (uint8)u32CmdCountID;
	TxSerialMsg_[8]	= 0x00;//ack bit
	TxSerialMsg_[9]	= 0x00;// success 
	TxSerialMsg_[10] = vUART_TxCharCRC(TxSerialMsg_);
	TxSerialMsg_[11] = 0x55;
	u32CmdCountID++;
	HAL_UART_Write(TxSerialMsg_);

	//u8UARTWriteCnt = 8;// send one time

	//ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 

}

void App_Handle_User_Write_Attribute(uint8 *palyload, uint16 size)//moons
{
	uint16 attributeId = 0;
	uint8 *p = palyload;

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
			uint16 FunctionByte = 0, PasswordTime = 0;//offValue = 7;
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
	else if (attributeId == E_CLD_BAS_ATTR_ID_TRANSPRT_TRANS_CNT)
	{
		crtreasknum = 0;
		uint8 CachedataCnt = 0;
		CachedataCnt = *(p + 6);
		
		if(GateWayCacheAllCnt != 0)
			return;
		
		GateWayCacheAllCnt = CachedataCnt;
		
		if(GateWayCacheAllCnt > 20 || GateWayCacheAllCnt < 0)
		{
			GateWayCacheAllCnt = 20;
		}
		
		if(GateWayCacheAllCnt != 0)
		{

			if(GateWayCacheAllCnt * 20 > 255)
				App_PollRateCnt = 255;
			else
				App_PollRateCnt = GateWayCacheAllCnt * 20;
			
			DBG_vPrintf(TRUE, "\n\n The Number Of Cache Is %d, \n Set App_PollRateCnt To %d \n\n", CachedataCnt, App_PollRateCnt);

			RequestNextCacheNum = 1;
			App_AskOneOfCache();
			GetPollControlResponse = 0;

			RecordRequestNextCacheNum = RequestNextCacheNum;
			RetryRequestNextCacheNum = App_PollRateCnt - 10;// 5 second check
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
			crtreasknum = 0;
			//App_SerialSendEnableDoorLoockPwd(*(p + 8), p + 9);
			App_SendReportPollControlSendCacheAck();// send ack
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
	else
	{
		DBG_vPrintf(TRACE_APP_UART, "\n NO Handle This Attribute %x ", attributeId);
	}
}


#ifdef TEMPORAEY_PASSWORD	
PUBLIC void App_SerialSendAddTempoaryPassword(uint8 *u8p)
{
	DBG_vPrintf(TRACE_APP_UART, "\n TempoaryPasswordResendCnt = %x ", TempoaryPasswordResendCnt);		
	APP_ButtonsWakeUpMCU();

	//static uint8 TempoaryPasswordBackup[10];
	uint8 TxSerialMsg_[21];
	if(TempoaryPasswordResendCnt == 0)
	{
		DBG_vPrintf(TRACE_APP_UART, "\n Backup TAMPORAYPASSWORD\n");		
		memcpy(TempoaryPasswordBackup, u8p, 10);
		memcpy(TempoaryPasswordBackup2, u8p, 10);
		TempoaryPasswordResendCnt++;
		ZTIMER_eStart(u8TimerTemporayPasswordResend, ZTIMER_TIME_MSEC(100));
		return;
	}
	else
	{
		u8p = TempoaryPasswordBackup2;
		TempoaryPasswordResendCnt++;
	}
	
	TxSerialMsg_[0]  = 0x15;
	TxSerialMsg_[1]  = 0xAA;
	TxSerialMsg_[2]  = 0x0A;
	TxSerialMsg_[3]  = 0x8D;
	TxSerialMsg_[4]	 = (uint8)(u32CmdCountID>>24);
	TxSerialMsg_[5]	 = (uint8)(u32CmdCountID>>16);
	TxSerialMsg_[6]	 = (uint8)(u32CmdCountID>>8);
	TxSerialMsg_[7]	 = (uint8)u32CmdCountID;
	TxSerialMsg_[8]  = 0x01;
	TxSerialMsg_[9]  = (*u8p++);
	TxSerialMsg_[10] = (*u8p++);
	TxSerialMsg_[11] = (*u8p++);
	TxSerialMsg_[12] = (*u8p++);
	TxSerialMsg_[13] = (*u8p++);
	TxSerialMsg_[14] = (*u8p++);
	TxSerialMsg_[15] = (*u8p++);
	TxSerialMsg_[16] = (*u8p++);
	TxSerialMsg_[17] = (*u8p++);
	TxSerialMsg_[18] = (*u8p);

	TxSerialMsg_[19] = vUART_TxCharCRC(TxSerialMsg_);
	TxSerialMsg_[20] = 0x55;

	HAL_UART_Write(TxSerialMsg_);
	u32CmdCountID++;

	//u8UARTWriteCnt = 6;// meaning no resend
	//ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 

	if(TempoaryPasswordResendCnt < 7)// resend 3 times
	{
		DBG_vPrintf(TRACE_APP_UART, "\n Start Recv TAMPORAYPASSWORD ACK Timer = %x ", TempoaryPasswordResendCnt);		
		ZTIMER_eStart(u8TimerTemporayPasswordResend, ZTIMER_TIME_MSEC(100));
	}
	else
	{
		DBG_vPrintf(TRACE_APP_UART, "\n Stop Recv TAMPORAYPASSWORD ACK Timer = %x ", TempoaryPasswordResendCnt);
		App_Stop_ResendTempoaryPassword();
		//memset(TempoaryPasswordBackup, 0, 10);// reset to 0		
	}
	APP_ButtonsStopWakeUp();
}

void App_ResendTempoaryPassword(void *pvParam)
{
	uint8 *u8p = TempoaryPasswordBackup2;
	App_SerialSendAddTempoaryPassword(u8p);
	
}

void App_Stop_ResendTempoaryPassword(void)
{
	DBG_vPrintf(TRACE_APP_UART, "\n App Stop Resend Tempoary Password\n");

	TempoaryPasswordResendCnt = 0;
	if(E_ZTIMER_STATE_RUNNING == ZTIMER_eGetState(u8TimerTemporayPasswordResend))
	{
		ZTIMER_eStop(u8TimerTemporayPasswordResend);
	}
}


/****************************************************************************
 *
 * NAME: vSend Report DoorLock State 
 *
 * DESCRIPTION:
 * Method to send a report to all bound nodes.
 *
 ****************************************************************************/
PUBLIC void App_SendReportTempoaryPasswordState(void)
{
	uint8 u8ret = 0;
    PDUM_thAPduInstance myPDUM_thAPduInstance;
    tsZCL_Address sDestinationAddress;

    DBG_vPrintf(TRACE_APP_UART, "\nAPP Report: TempoaryPasswordState ");

    /* get buffer to write the response in*/
    myPDUM_thAPduInstance = hZCL_AllocateAPduInstance();
    /* no buffers - return*/
    if(myPDUM_thAPduInstance == PDUM_INVALID_HANDLE)
    {
        DBG_vPrintf(TRACE_APP_UART, "\nAPP Report: PDUM_INVALID_HANDLE");
    }

    /* Set the address mode to send to all bound device and don't wait for an ACK*/
    sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;

    /* Send the report with all attributes.*/
	u8ret = eZCL_ReportAttribute(&sDestinationAddress,CLOSURE_CLUSTER_ID_DOOR_LOCK,
								E_CLD_DOOR_LOCK_ATTR_ID_USER_ADD_TEMPOARYPASSWORD ,1,REPORT_EP,myPDUM_thAPduInstance);
    if (E_ZCL_SUCCESS != u8ret)
    {
    	DBG_vPrintf(TRACE_APP_UART, "\nAPP Report: Error Sending Report %d",u8ret);
    }

    /* free buffer and return*/
	PDUM_eAPduFreeAPduInstance(myPDUM_thAPduInstance);
    DBG_vPrintf(TRACE_APP_UART, "\nAPP Report: Report Sent");

}
#endif

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

	if(p[0] == 0xFF && p[1] == 0xFD)// for all other command
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
	else if(p[0] == 0xFF && p[1] == 0xFC)// for password operation 
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

			if(pwdVersion == 0x01)// ����ӣ����Ԣ����
			{
				APP_SerialSendPwdOperationDataForYinghua(AppCmdId, size, p);// ����ǰ��Э���ƶ�������ƫ�����ӣ����Ԣ������Ҫת��һ���ֽ����ٴ�������
				return;
			}
			else if(pwdVersion == 0x02)
			{
				// �ɱ����ϰ汾�������������������ָ����ֽ�����ӷ������·���һ����������Ҫת�ֽ���ֱ���ж������������������ȷ��뻺����� 
			}
		}
		else
		{
			return;// data no enought
		}
	}
	else
	{
		APP_SerialSendPwdOperationDataForYinghua(PwdOperation, size, p);//�����޷�����ָ����ɣĵİ汾
		return;// unknow command flag
	}

	App_SendDataImmediatelyOrSaveIntoCacheBuffer(AppCmdId, commandId, size, p);// �ж������������������ȷ��뻺�����

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

				if(commandId == PwdOperation)
				{
					u8ServerWriteAttributData[ cnt ][2] = AppCmdId;
					Cacheoffset = 3;					
					//u8ServerWriteAttributData[ cnt ][3 ~ end] is data
				}
				else
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
	
	if(commandId == PwdOperation)//��ʱδ��ɷ�����ָ��ID
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
	DBG_vPrintf(TRACE_APP_UART, "\n\n Check Wheather Still Data Need Transmit To MCU\n\n");// transmit to mcu immediately
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

#if 0
void App_SerialSendEnableDoorLoockPwd(uint8 cnt, uint8 *pwd)//moons
{
uint8 i = 0;
	TxSerialMsg[0]	= 11 + cnt;

	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= cnt;
	TxSerialMsg[3]	= EnableDoorLock;
	TxSerialMsg[4]	= (uint8)(u32CmdCountID>>24);
	TxSerialMsg[5]	= (uint8)(u32CmdCountID>>16);
	TxSerialMsg[6]	= (uint8)(u32CmdCountID>>8);
	TxSerialMsg[7]	= (uint8)u32CmdCountID;
	TxSerialMsg[8]	= 0x01;//ack bit
	for(i = 0; i <  cnt; i++)
	{
		TxSerialMsg[9 + i]	= *pwd;
		pwd++;
	}
	TxSerialMsg[cnt + 9] = vUART_TxCharCRC(TxSerialMsg);
	TxSerialMsg[cnt + 10] = 0x55;

	u8UARTWriteCnt = 0;// send one time

	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 

}
#endif
/**************************************************************************/
/***					File End										***/
/**************************************************************************/

