


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

#include "App_WL_DoorLock.h"
#include "app_SampleDoorLock_Uart.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
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

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE bool_t bRxEnable;			/**< UART receive enabled */
PRIVATE bool_t bTxEnable;			/**< UART transmit enabled */
PRIVATE bool_t bModemInt;			/**< UART modem status change interrupt enabled */
//PRIVATE bool_t bTxIntServiced;		/**< LAst UART transmit interrupt was serviced, expect another transmit interrupt to follow */

PRIVATE teSL_RxState eRxState = E_STATE_RX_WAIT_START;

PRIVATE uint8 u8UARTWriteCnt = 0;
PRIVATE uint32 u32CmdCountID = 1;

PRIVATE uint8 TxSerialMsg[32];	//Ricky 用于存放串口发送数据
PRIVATE uint8 TxSerial[32];		//Ricky 用于串口写入
PRIVATE uint8 RxSerial[32];		//Ricky 用于存放串口接收数据
PUBLIC uint8 RxSerialMsg[32];	//Ricky 用于处理串口数据


/****************************************************************************/
/***        Functions 		                                              ***/
/****************************************************************************/

PRIVATE uint8 vUART_RxCharCRC(uint8 *SerialMsg);
PUBLIC teSL_RxAcknowlegement bSL_ReadMessage(uint8 u8Data);
PRIVATE uint8 vFindSerialCommandIdentify(uint8 u8Data);
PUBLIC void UART_vSetBaudRate ( uint32 u32BaudRate );
PUBLIC bool_t UART_bTxReady();
PUBLIC void APP_UartAchieveMessagefromPrivate(void);
PUBLIC void App_SerialSendRemoteBindUnlock(uint8 *nPass);


/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

PUBLIC void APP_UartAchieveMessagefromPrivate(void)
{
	memcpy(RxSerialMsg,RxSerial,sizeof(RxSerial));
	memset(RxSerial,0,sizeof(RxSerial));

	DBG_vPrintf(TRACE_APP_UART, "\n APP_UartAchieveMessagefromPrivate ");
	uint8 i;
	for (i = 0;i < 21;i++)
		DBG_vPrintf(TRACE_APP_UART, " %0x",RxSerialMsg[i]);

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

	//Ricky 清空缓存
	vAppSerialCacheArea_Init();

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
    }

    /* Set the calculated clocks per bit */
    vAHI_UartSetClocksPerBit(E_AHI_UART_1, 11);
    /* Set the calculated divisor */
    vAHI_UartSetBaudDivisor(E_AHI_UART_1, 23);

	DBG_vPrintf(TRACE_APP_UART, "\n UART_vSetBaudRate %d,%d \n",u8ClocksPerBit,u16Divisor);
}

PUBLIC void HAL_UART_Write(uint8 *WriteBuf)
{
	uint8 i;

	if(WriteBuf[1] == 0)
		return ;

	DBG_vPrintf(TRACE_APP_UART, "\n HAL_UART_Write");
	for(i=0;i <= WriteBuf[0];i++)
		DBG_vPrintf(TRACE_APP_UART, " %0x",WriteBuf[i]);

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

PUBLIC void APP_vProcessIncomingSerialCommands ( uint8 u8RxByte )
{
	uint8 u8RxStatus = FALSE;
	teSL_RxAcknowlegement u8RxAcknowlegement = E_STATE_RX_WAIT;

	u8RxAcknowlegement = bSL_ReadMessage(u8RxByte);
	ZTIMER_eStart(u8TimerSerialTimeout,ZTIMER_TIME_MSEC(20));// Serial timeout

    if( E_STATE_RX_CMD_TRUE == u8RxAcknowlegement )
    {    // 串口接收完毕。
		DBG_vPrintf(TRACE_APP_UART, "\n Queue Send Serial To Process \n");
    	u8RxStatus = 1;
		ZQ_bQueueSend (&APP_msgSerialRx,&u8RxStatus);
    }
	else if (E_STATE_RX_CMD_FALSE == u8RxAcknowlegement)
	{	// 发送错误应答。
		u8RxStatus = 2;
		ZQ_bQueueSend (&APP_msgSerialRx,&u8RxStatus);
	}

}

/****************************************************************************
 *
 * NAME: bSL_ReadMessage
 *
 * DESCRIPTION:
 * Attempt to read a complete message from the serial link
 *
 * PARAMETERS :
 *
 * RETURNS:
 * TRUE if a complete valid message has been received
 *
 ****************************************************************************/
PUBLIC teSL_RxAcknowlegement bSL_ReadMessage(uint8 u8Data)
{
	static uint8 RX_Cnt = 1,Rx_SerialLen = 0;
	teSL_RxAcknowlegement eRxAcknowlegement = E_STATE_RX_WAIT;
	uint8 u8CRC = 0,i;

//	DBG_vPrintf(TRACE_APP_UART, "\n bSL_ReadMessage : %0x ",u8Data);

	switch(eRxState)
	{

		case E_STATE_RX_WAIT_START:

			if (u8Data == 0xAA)
			{
				RxSerial[0] = u8Data;
				eRxState++;
			}
			break;

		case E_STATE_RX_WAIT_LEN:

			if ((u8Data == 0x01) || (u8Data == 0x0A))
			{
				RxSerial[1] = u8Data;
				RX_Cnt = 0;
				eRxState++;
			}
			else
			{
				eRxState = E_STATE_RX_WAIT_ERROR;
			}
			break;

		case E_STATE_RX_WAIT_CMD:

//			if (vFindSerialCommandIdentify(u8Data))
			{
				RxSerial[2] = u8Data;
				eRxState++;
			}

			break;

		case E_STATE_RX_WAIT_COUNT:

			if (RX_Cnt <= 3)
			{
				RxSerial[RX_Cnt+3] = u8Data;
				RX_Cnt++;
			}
			if (RX_Cnt == 4)
			{
				eRxState++;
				RX_Cnt = 0;
			}
			break;

		case E_STATE_RX_WAIT_ACK:

			if ((u8Data == 0x00) || (u8Data == 0x01))
			{
				RxSerial[7] = u8Data;
				eRxState++;
			}
			else
			{
				eRxState = E_STATE_RX_WAIT_ERROR;
			}

			break;

		case E_STATE_RX_WAIT_DATA:

			if (RX_Cnt < RxSerial[1])
			{
				RxSerial[8+RX_Cnt] = u8Data;
				RX_Cnt++;
			}
			if (RX_Cnt == RxSerial[1])
			{
				eRxState++;
			}
			break;

		case E_STATE_RX_WAIT_CRC:

			u8CRC = vUART_RxCharCRC(RxSerial);

			if (u8CRC == u8Data)
			{
				RxSerial[RxSerial[1]+8] = u8Data;
				eRxState++;
			}
			else
			{
				eRxState = E_STATE_RX_WAIT_ERROR;
			}
			break;

		case E_STATE_RX_WAIT_TAIL:

			if(u8Data == 0x55)
			{
				RxSerial[RxSerial[1]+9] = u8Data;
				eRxState++;
			}
			else
				eRxState = E_STATE_RX_WAIT_ERROR;

			break;

		default :	// Error

			RX_Cnt = 0;
			eRxState = E_STATE_RX_WAIT_START;
			memset(RxSerial,0,sizeof(RxSerial));
			break;
	}
#if 0
	switch (eRxState)
	{
		case E_STATE_RX_WAIT_START:
			
			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_START");
			break;

    	case E_STATE_RX_WAIT_LEN:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_LEN");
			break;
			
		case E_STATE_RX_WAIT_CMD:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_CMD");
			break;

			case E_STATE_RX_WAIT_COUNT:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_COUNT");
			break;

			case E_STATE_RX_WAIT_ACK:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_ACK");
			break;
    	case E_STATE_RX_WAIT_DATA:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_DATA");
			break;
    	case E_STATE_RX_WAIT_CRC:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_CRC");
			break;
    	case E_STATE_RX_WAIT_TAIL:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_TAIL");
			break;
    	case E_STATE_RX_WAIT_SUCCESS:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_SUCCESS");
			break;
    	case E_STATE_RX_WAIT_ERROR:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_ERROR");
			break;
		default :
			DBG_vPrintf(TRACE_APP_UART, "\n default");
			break;
	}

#endif

	if (eRxState == E_STATE_RX_WAIT_SUCCESS)
	{
		eRxState = E_STATE_RX_WAIT_START;
			eRxAcknowlegement = E_STATE_RX_CMD_TRUE;

		if (FALSE == vFindSerialCommandIdentify(RxSerial[2]))
			eRxAcknowlegement = E_STATE_RX_CMD_FALSE;

	#if 1
		DBG_vPrintf(TRACE_APP_UART, "\n RxSerial : ");
		i = 0;
		while( i < 22)
			DBG_vPrintf(TRACE_APP_UART, "%0x ",RxSerial[i++]);
	#endif

	}
	else if(eRxState == E_STATE_RX_WAIT_ERROR)
	{
		// Send Error Acknowlegement
		eRxState = E_STATE_RX_WAIT_START;

		if (RxSerial[1] == 0x0A)
			eRxAcknowlegement = E_STATE_RX_CMD_FALSE;		

		memset(RxSerial,0,sizeof(RxSerial));
	}

	return(eRxAcknowlegement);
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

PRIVATE uint8 vUART_TxCharCRC(uint8 *SerialMsg)
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

PRIVATE uint8 vFindSerialCommandIdentify(uint8 u8Data)
{

	if (u8Data == 0)
		return FALSE;

	switch(u8Data)
	{
		case TransgressAlarm :
		case TamperAlarm :
		case PretendLock :
		case UnLockAlarm :
		case Coercion :
		case AlarmReset :

	#ifdef SKYWORTH
		case DoorRing :
	#endif

		case BatteryAlarm :
		case JoinNWK :
		case LeaveNWK :
		case RemoteOperation :
		case RemoteUnlock :
		case SetDoorLockTime :
		case WakeUp :
		case LoaclOperation :
		case LockReport :
		case NwkPrompt :
		case NwkIndication :
		case GetTime :
		case NormallyOpen :

			DBG_vPrintf(TRACE_APP_UART, "\n vFindSerialCommandIdentify ");
			return TRUE;
			break;

		default :
			return FALSE;
			break;
	}

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
	DBG_vPrintf(TRACE_APP_UART, "\n APP_SerialSendAcknowlegement ");
//	DBG_vPrintf(TRACE_APP_UART, "\n cmdType %0x ",cmdType);
#if 0
	// 串口写入只设一个入口	应答包不做唤醒信号
	ZTIMER_eStart(u8TimerWriteSerial, ZTIMER_TIME_MSEC(1));
#else
	memset(TxSerial,0,64);
	memcpy(TxSerial,TxSerialMsg,TxSerialMsg[0]);

    vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
    vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);

	HAL_UART_Write(TxSerial);

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

	u8UARTWriteCnt= 0;	//Ricky Test
	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1)); 
	DBG_vPrintf(TRACE_APP_UART, "\n App_SerialSendRemoteUnlock ");

}

PUBLIC void App_SerialSendRemoteBindUnlock(uint8 *nPass)
{
	if (nPass[0] > 10)
		return;

	uint8 i;

	memset(TxSerialMsg,0,sizeof(TxSerialMsg));

	TxSerialMsg[0]	= 0x15;
	TxSerialMsg[1]	= 0xAA;
	TxSerialMsg[2]	= 0x0A;
	TxSerialMsg[3]	= 0x67;
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
	DBG_vPrintf(TRACE_APP_UART, "\n App_SerialSendRemoteBindUnlock ");

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


}

PUBLIC void APP_cbTimerSerialDioSet(void)
{
	APP_ButtonsWakeUpMCU();
	ZTIMER_eStart(u8TimerWriteSerial, ZTIMER_TIME_MSEC(UART_WAKEUP_OUTTIME) );
}

PUBLIC void APP_cbTimerWriteSerial(void)
{
	static uint8 au8WriteBuf[22];
	DBG_vPrintf(TRACE_APP_UART, "\n APP_cbTimerWriteSerial");	

	APP_ButtonsStopWakeUp();

#if 1
    vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
    vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);
#endif

	memset(TxSerial,0,64);
	memcpy(TxSerial,TxSerialMsg,TxSerialMsg[0]);

	u32CmdCountID++;
	DBG_vPrintf(TRACE_APP_UART, "\n u8UARTWriteCnt : %0x ",u8UARTWriteCnt);

	HAL_UART_Write(TxSerial);

	if (u8UARTWriteCnt < 7)
	{
		u8UARTWriteCnt++;
		// Ricky Todo Recalculate CRC
		App_SerialPrepareRecalculate();		
		ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(800));
	}
	else
	{
		u8UARTWriteCnt = 8;
		memset(TxSerialMsg,0,sizeof(TxSerialMsg));		
		DBG_vPrintf(TRACE_APP_UART, "\n u8UARTWriteCnt : %0x ",u8UARTWriteCnt);
		
		ZTIMER_eStop(u8TimerWriteSerial);
		ZTIMER_eStop(u8TimerSerialDioSet);
		ZTIMER_eStop(u8TimerWriteSerial);
	}

}

PUBLIC void APP_cbTimerSerialTimeout(void)
{

	if ((eRxState != E_STATE_RX_WAIT_START) && (eRxState != E_STATE_RX_WAIT_SUCCESS))
	{
		eRxState = E_STATE_RX_WAIT_START;
		memset(RxSerialMsg,0,sizeof(RxSerialMsg));		
		DBG_vPrintf(TRACE_APP_UART, "\n APP_cbTimerSerialTimeout Set E_STATE_RX_WAIT_START ");
	}

}

PUBLIC void App_StopSerialPrepareEnterSleep(void)
{

	u8UARTWriteCnt = 8;
	memset(TxSerialMsg,0,sizeof(TxSerialMsg));		

	APP_ButtonsStopWakeUp();
	ZTIMER_eStop(u8TimerWriteSerial);
	ZTIMER_eStop(u8TimerSerialDioSet);

}

/**************************************************************************/
/***					File End										***/
/**************************************************************************/

