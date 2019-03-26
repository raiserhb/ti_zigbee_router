


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


/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE bool_t bRxEnable;			/**< UART receive enabled */
PRIVATE bool_t bTxEnable;			/**< UART transmit enabled */
PRIVATE bool_t bModemInt;			/**< UART modem status change interrupt enabled */
PRIVATE bool_t bTxIntServiced;		/**< LAst UART transmit interrupt was serviced, expect another transmit interrupt to follow */

PUBLIC uint8 TxSerialMsg[64];
PUBLIC uint8 RxSerialMsg[64];	//Ricky 用于存放串口接收数据

PRIVATE teSL_RxState eRxState = E_STATE_RX_WAIT_START;

/****************************************************************************/
/***        Functions 		                                              ***/
/****************************************************************************/
/*
#if JENNIC_CHIP_FAMILY == JN517x
PUBLIC void vUART_HandleUartInterrupt(uint32 u32Device, uint32 u32ItemBitmap);
#else
PUBLIC void vUART_HandleUartInterrupt(void);
#endif
*/
PRIVATE uint8 vUART_RxCharCRC(uint8 *SerialMsg,uint8 index);
PUBLIC teSL_RxAcknowlegement bSL_ReadMessage(uint8 u8Data);

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

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

#if 0
    vAHI_UartEnable(E_AHI_UART_1);
#else
	bAHI_UartEnable(E_AHI_UART_1, TxSerialMsg, (uint8)sizeof(TxSerialMsg),
					RxSerialMsg, (uint8)sizeof(RxSerialMsg));
#endif

    vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
    vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);

	vAHI_UartSetLocation(E_AHI_UART_1,TRUE);	// Ricky
	vAHI_UartSetBaudRate(E_AHI_UART_1,E_AHI_UART_RATE_115200);

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

}

PUBLIC void HAL_UART_Write(uint8 *WriteBuf)
{
	uint8 i;
	i = 1;

	if(WriteBuf[1] == 0)
		return ;

	while(i <=  WriteBuf[0])
	{
		vAHI_UartWriteData(E_AHI_UART_1, WriteBuf[i]);
		i++;
	}

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
		DBG_vPrintf(TRACE_APP_UART, "\n vUART_HandleUartInterrupt E_AHI_UART_INT_RXDATA");

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
	ZTIMER_eStart(u8TimerSerialTimeout,ZTIMER_TIME_MSEC(10));// Serial timeout

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

	DBG_vPrintf(TRACE_APP_UART, "\n bSL_ReadMessage : %0x ",u8Data);

	switch(eRxState)
	{

		case E_STATE_RX_WAIT_START:

			if (u8Data == 0xAA)
			{
				RxSerialMsg[0] = u8Data;
				eRxState++;
			}
			break;

		case E_STATE_RX_WAIT_HEAD:

			if (u8Data == 0x55)
			{
				RxSerialMsg[1] = u8Data;
				eRxState++;
			}
			else
			{
				eRxState = E_STATE_RX_WAIT_ERROR;
			}
			break;

		case E_STATE_RX_WAIT_LEN:

			if ((u8Data > 1) &&(u8Data < 38))
			{
				Rx_SerialLen = u8Data+1;
				RxSerialMsg[2] = u8Data;
				RX_Cnt = 1;
				eRxState++;
			}
			else
			{
				eRxState = E_STATE_RX_WAIT_ERROR;
			}
			break;

		case E_STATE_RX_WAIT_DATA:

			if (RX_Cnt <= Rx_SerialLen)
			{
				RxSerialMsg[2+RX_Cnt] = u8Data;
				RX_Cnt++;
			}
			if (RX_Cnt == Rx_SerialLen)
			{
				eRxState++;
			}
			break;

		case E_STATE_RX_WAIT_CRC:

			u8CRC = vUART_RxCharCRC(&RxSerialMsg[0]+2,Rx_SerialLen);
			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_CRC u8CRC = %02x,u8Data = %02x \n", u8CRC, u8Data);

			if (u8CRC == u8Data)
			{
				eRxState = E_STATE_RX_WAIT_SUCCESS;
			}
			else
			{
				eRxState = E_STATE_RX_WAIT_ERROR;
			}
			break;

		case E_STATE_RX_WAIT_TAIL:

			break;

		default :	// Error

			RX_Cnt = 1;
			break;
	}
#if 0
	switch (eRxState)
	{
		case E_STATE_RX_WAIT_START:
			
			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_START");
			break;
		case E_STATE_RX_WAIT_HEAD:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_HEAD");
			break;
    	case E_STATE_RX_WAIT_LEN:

			DBG_vPrintf(TRACE_APP_UART, "\n E_STATE_RX_WAIT_LEN");
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

#if (TRACE_APP_UART == TRUE)
		DBG_vPrintf(TRACE_APP_UART, "\n RxSerialMsg: ");
		i = 0;
		while( i < 38)
			DBG_vPrintf(TRACE_APP_UART, "%0x ",RxSerialMsg[i++]);
#endif

	}
	else if(eRxState == E_STATE_RX_WAIT_ERROR)
	{
		// Send Error Acknowlegement
		eRxState = E_STATE_RX_WAIT_START;
		eRxAcknowlegement = E_STATE_RX_CMD_FALSE;
	}

	return(eRxAcknowlegement);
}

PRIVATE uint8 vUART_RxCharCRC(uint8 *SerialMsg,uint8 index)
{
	uint8 u8CRC = 0;
	uint8 i;

	for (i = 0; i < index; i++)
	{
		u8CRC = u8CRC^SerialMsg[i];
	}

	return u8CRC;
}

PUBLIC void APP_SerialSendAcknowlegement(uint8 cmdType,bool Ack)
{
	memset(TxSerialMsg,0,64);

	TxSerialMsg[0] = 0x07;
	TxSerialMsg[1] = 0xAA;
	TxSerialMsg[2] = 0x55;
	TxSerialMsg[3] = 0x03;
	TxSerialMsg[4] = 0x04;
	TxSerialMsg[5] = cmdType;
	TxSerialMsg[6] = Ack;
	TxSerialMsg[7] = vUART_RxCharCRC(&TxSerialMsg[0]+3,5);

	DBG_vPrintf(TRACE_APP_UART, "\n APP_SerialSendAcknowlegement ");

	// 串口写入只设一个入口	应答包不做唤醒信号
//	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1));	
	ZTIMER_eStart(u8TimerWriteSerial, ZTIMER_TIME_MSEC(1));
}

PUBLIC void APP_SerialSendJoinIndication(bool data)
{
	memset(TxSerialMsg,0,64);

	TxSerialMsg[0] = 0x07;
	TxSerialMsg[1] = 0xAA;
	TxSerialMsg[2] = 0x55;
	TxSerialMsg[3] = 0x03;
	TxSerialMsg[4] = 0x01;
	TxSerialMsg[5] = 0x26;
	TxSerialMsg[6] = data;
	TxSerialMsg[7] = vUART_RxCharCRC(&TxSerialMsg[0]+3,5);

	ZTIMER_eStart(u8TimerSerialDioSet,ZTIMER_TIME_MSEC(1));	

}

PUBLIC void APP_cbTimerSerialDioSet(void)
{
	APP_ButtonsWakeUpMCU();
	ZTIMER_eStart(u8TimerWriteSerial, ZTIMER_TIME_MSEC(90));
}

PUBLIC void APP_cbTimerWriteSerial(void)
{
	DBG_vPrintf(TRACE_APP_UART, "\n APP_cbTimerWriteSerial");	
	DBG_vPrintf(TRACE_APP_UART, "\n TxSerialMsg :");

	APP_ButtonsStopWakeUp();
#if (TRACE_APP_UART == TRUE)
	uint8 i = 0;
	while( i < 20)
		DBG_vPrintf(TRACE_APP_UART, "%0x ",TxSerialMsg[i++]);
#endif

	HAL_UART_Write(TxSerialMsg);
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


/**************************************************************************/
/***					File End										***/
/**************************************************************************/

