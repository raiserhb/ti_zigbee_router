


#ifndef APP_SAMPLE_DOORLOCK_UART_H_
#define APP_SAMPLE_DOORLOCK_UART_H_

#if JENNIC_CHIP_FAMILY == JN516x
	#if (UART == E_AHI_UART_0)
		#define UART_START_ADR      0x02003000UL
	#elif (UART == E_AHI_UART_1)
		#define UART_START_ADR      0x02004000UL
	#endif
#elif JENNIC_CHIP_FAMILY == JN517x
	#if (UART == E_AHI_UART_0)
		#define UART_START_ADR      0x40004000UL
	#elif (UART == E_AHI_UART_1)
		#define UART_START_ADR      0x40005000UL
	#endif
#endif


PUBLIC void APP_UARTInitialise(void);
PUBLIC void HAL_UART_Write(uint8 *WriteBuf);

PUBLIC void vuart1int();

typedef enum
{
    E_STATE_RX_WAIT_START,
	E_STATE_RX_WAIT_HEAD,
    E_STATE_RX_WAIT_LEN,
    E_STATE_RX_WAIT_DATA,
    E_STATE_RX_WAIT_CRC,
    E_STATE_RX_WAIT_TAIL,
    E_STATE_RX_WAIT_SUCCESS,
    E_STATE_RX_WAIT_ERROR,
}teSL_RxState;


#endif
