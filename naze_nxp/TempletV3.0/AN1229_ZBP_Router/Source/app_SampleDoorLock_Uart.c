


/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "ZTimer.h"
#include "ZQueue.h"

#include "DBG.h"
#include "AppHardwareApi.h"
//#include "AppQueueApi.h"


#include "pwrm.h"
#include "app_SampleDoorLock_Uart.h"
#include "microspecific_ba.h"
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

PUBLIC uint8 TxSerialMsg[32];
PUBLIC uint8 RxSerialMsg[64];	//Ricky 用于存放串口接收数据


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
PRIVATE uint8 vUART_RxCharCRC(uint8 index);
PUBLIC bool bSL_ReadMessage(uint8 u8Data);

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

	vAHI_UartSetLocation(E_AHI_UART_1,TRUE);	// Ricky

#if 0
	vAHI_UartEnable(E_AHI_UART_1);
//	vAHI_UartTxOnly(E_AHI_UART_1,FALSE);

#else
	bAHI_UartEnable(E_AHI_UART_1, TxSerialMsg, (uint8)32,
					RxSerialMsg, (uint8)64);

	vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
	vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);
#endif

	vAHI_UartSetBaudRate(E_AHI_UART_1,E_AHI_UART_RATE_115200);
//	vAHI_UartSetControl(E_AHI_UART_1,E_AHI_UART_EVEN_PARITY,E_AHI_UART_PARITY_DISABLE,E_AHI_UART_WORD_LEN_8,E_AHI_UART_1_STOP_BIT,E_AHI_UART_RTS_HIGH);

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


#if 1
		vAHI_UartEnable(E_AHI_UART_1);
	
#else
		bAHI_UartEnable(E_AHI_UART_1, TxSerialMsg, (uint8)32,
						RxSerialMsg, (uint8)64);
	
		vAHI_UartReset(E_AHI_UART_1, TRUE, TRUE);
		vAHI_UartReset(E_AHI_UART_1, FALSE, FALSE);
#endif
	
		vAHI_UartSetBaudRate(E_AHI_UART_1,E_AHI_UART_RATE_115200);
//		vAHI_UartSetControl(E_AHI_UART_1,E_AHI_UART_EVEN_PARITY,E_AHI_UART_PARITY_DISABLE,E_AHI_UART_WORD_LEN_8,E_AHI_UART_1_STOP_BIT,E_AHI_UART_RTS_HIGH);
	
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


//	vAHI_Uart0RegisterCallback(vuart1int);

    /* Turn on modem status, tx, rx interrupts */
//	vAHI_UartSetInterrupt(E_AHI_UART_1, bModemInt, FALSE, TRUE, bRxEnable, E_AHI_UART_FIFO_LEVEL_1);
//    vAHI_UartSetInterrupt(E_AHI_UART_1, FALSE, FALSE, TRUE, TRUE, E_AHI_UART_FIFO_LEVEL_4);
    vAHI_UartSetInterrupt(E_AHI_UART_1, FALSE, FALSE, TRUE, TRUE, E_AHI_UART_FIFO_LEVEL_8);
	DBG_vPrintf(TRACE_APP_UART, "\n vAHI_UartSetInterrupt");
//	vAHI_InterruptSetPriority(MICRO_ISR_MASK_UART0,8);
//	vAHI_InterruptSetPriority(MICRO_ISR_MASK_UART1,8);

}

PUBLIC void HAL_UART_Write(uint8 *WriteBuf)
{
	if(WriteBuf == 0)
		return ;

	while(*WriteBuf != 0)
	{
		vAHI_UartWriteData(E_AHI_UART_1, *WriteBuf);
		WriteBuf++;
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
 
PUBLIC void vuart1int()
{

#if JENNIC_CHIP_FAMILY != JN517x
	uint32 u32ItemBitmap = ( ( *( (volatile uint32 *)( UART_START_ADR + 0x08 ) ) ) >> 1 ) & 0x0007;
#else
    if (u32Device)	// E_AHI_DEVICE_UART1
#endif
#if 0
	vAHI_UartWriteData(E_AHI_UART_1, 'a');

//    if (u32Device == E_AHI_DEVICE_UART1)
#else
    {

		#ifdef ENABLE_HW_SW_FLOW_CONTROL
			/* Read line status */
			uint8 u8LineStatus = u8AHI_UartReadLineStatus(E_AHI_UART_1);
		#endif

    	/* Data to receive ? */
        if (((u32ItemBitmap & 0x000000FF) == E_AHI_UART_INT_RXDATA) || 
			((u32ItemBitmap & 0x000000FF) == E_AHI_UART_INT_TIMEOUT))
        {
        	/* Receive character from UART */
		#if 1
			vUART_RxCharISR(u8AHI_UartReadData(E_AHI_UART_1));	//Ricky
		#else
			uint8 u8Byte;
			u8Byte = u8AHI_UartReadData(E_AHI_UART_1);
			ZQ_bQueueSend(&APP_msgSerialRx,&u8Byte);
		#endif
			DBG_vPrintf(TRACE_APP_UART, "\n u8AHI_UartReadData = %c ",u8AHI_UartReadData(E_AHI_UART_1));
        }
		#ifdef ENABLE_HW_SW_FLOW_CONTROL
			/* Modem status changed ? */
			else if (u32ItemBitmap == E_AHI_UART_INT_MODEM)
			{
				/* Read modem status */
				uint8 u8ModemStatus = u8AHI_UartReadModemStatus(E_AHI_UART_1);
				/* Has CTS changed ? */
				if (u8ModemStatus & E_AHI_UART_MS_DCTS)
				{
					/* Is CTS bit set meaning CTS has just been cleared ? */
					if (u8ModemStatus & 0x10)
					{
						/* Disable transmit */
						vUART_SetTxEnable(FALSE);
					}
					/* Is CTS bit is clear meaning CTS has just been set ? */
					else
					{
						/* Enable transmit */
						vUART_SetTxEnable(TRUE);
						/* OK to transmit now - begin transmitting again */
//						if (u8LineStatus & E_AHI_UART_LS_THRE) vUART_TxCharISR();
					}
				}
			}
		#endif
        /* Ready to transmit ? */
        else if (u32ItemBitmap == E_AHI_UART_INT_TX)
        {
//           	vUART_TxCharISR();
        }
    }
	#endif
}

/****************************************************************************
 *
 * NAME: vUART_RxCharISR
 *
 * DESCRIPTION  Receive character interrupt service routeine.
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
PUBLIC void vUART_RxCharISR(uint8 u8RxChar)		/**< Received character */
{

	DBG_vPrintf(TRACE_APP_UART, "\n vUART_RxCharISR ");
//	vSerialQ_AddItem(RX_QUEUE, u8RxChar);
//	ZQ_bQueueSend (&APP_msgSerialRx,&u8RxChar);

}

/****************************************************************************
 *
 * NAME: vUART_TxCharISR
 *
 * DESCRIPTION  Transmit character interrupt service routine.
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
PUBLIC void vUART_TxCharISR(void)
{
//    if(!bSerialQ_Empty(TX_QUEUE) && bTxEnable)
	{
//        vAHI_UartWriteData(E_AHI_UART_1, u8SerialQ_RemoveItem(TX_QUEUE));
		/* Note we serviced the interrupt, tx interrupts will continue */
		bTxIntServiced = TRUE;
	}
//	else
	{
		/* Note we didn't service the interrupt, tx interrupts will now stop */
		bTxIntServiced = FALSE;
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

PUBLIC void APP_vProcessIncomingSerialCommands ( uint8    u8RxByte )
{
	DBG_vPrintf(TRACE_APP_UART, "\n APP_vProcessIncomingSerialCommands ");

    if( TRUE == bSL_ReadMessage(u8RxByte) )
    {	// 串口接收完毕.

		DBG_vPrintf(TRACE_APP_UART, "\n APP_vProcessIncomingSerialCommands ");
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
PUBLIC bool bSL_ReadMessage(uint8 u8Data)	// Ricky Mark
{

    static teSL_RxState eRxState = E_STATE_RX_WAIT_START;
	static uint8 RX_Cnt = 1;
	static uint8 Rx_SerialLen = 0;
	uint8 u8CRC = 0;

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
				RxSerialMsg[2] = u8Data;
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
				Rx_SerialLen = u8Data;
				RxSerialMsg[3] = u8Data;
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
				RxSerialMsg[3+RX_Cnt] = u8Data;
				RX_Cnt++;
			}
			if (RX_Cnt == Rx_SerialLen)
			{
				eRxState++;
			}
			break;

		case E_STATE_RX_WAIT_CRC:

			u8CRC = vUART_RxCharCRC(Rx_SerialLen);
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

		default :	// Error 从头开始接收

			RX_Cnt = 1;
			eRxState = E_STATE_RX_WAIT_START;

			if (u8Data == 0xAA)
			{
					bSL_ReadMessage(u8Data);
			}
			else
			{
				memset(RxSerialMsg,0,sizeof(RxSerialMsg));
			}
			break;
	}

	DBG_vPrintf(TRACE_APP_UART, "\n bSL_ReadMessage : eRxState = ",eRxState);
	DBG_vPrintf(TRACE_APP_UART, "\n RxSerialMsg: ");

	uint8 i = 0;

	while( RxSerialMsg[i] != 0)
		DBG_vPrintf(TRACE_APP_UART, "%0x ",RxSerialMsg[i++]);

	if (eRxState == E_STATE_RX_WAIT_SUCCESS)
		return(TRUE);
	return(FALSE);
}

PRIVATE uint8 vUART_RxCharCRC(uint8 index)
{
	uint8 u8CRC = 0;
	uint8 i;

	for (i = 3; i < index; i++)
	{
		u8CRC ^= RxSerialMsg[i];
	}

	return u8CRC;
}

/**************************************************************************/
/***					File End										***/
/**************************************************************************/

