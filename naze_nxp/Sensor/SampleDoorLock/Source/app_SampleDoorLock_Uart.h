


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


//extern PUBLIC uint8 TxSerialMsg[64];
extern PUBLIC uint8 RxSerialMsg[32];


typedef enum
{
    E_STATE_RX_WAIT_START,
    E_STATE_RX_WAIT_LEN,
    E_STATE_RX_WAIT_CMD,
    E_STATE_RX_WAIT_COUNT,
    E_STATE_RX_WAIT_ACK,
    E_STATE_RX_WAIT_DATA,
    E_STATE_RX_WAIT_CRC,
    E_STATE_RX_WAIT_TAIL,
    E_STATE_RX_WAIT_SUCCESS,
    E_STATE_RX_WAIT_ERROR,
}teSL_RxState;

typedef enum
{
	E_STATE_RX_WAIT,
	E_STATE_RX_CMD_TRUE,
	E_STATE_RX_CMD_FALSE,
}teSL_RxAcknowlegement;

typedef enum
{
	TransgressAlarm = 0x20,		// 非法操作
	TamperAlarm = 0x22,			// 防拆报警
	PretendLock = 0x23,			// 假锁报警
	UnLockAlarm = 0x24,			// 未关门报警
	Coercion = 0x25,			// 胁迫报警
	AlarmReset = 0x29,			// 报警解除

#ifdef SKYWORTH
	DoorRing = 0x2A,			// 门铃
#endif

	BatteryAlarm = 0x30,		// 低压报警

	JoinNWK = 0x40,				// 加网
	LeaveNWK = 0x41,			// 退网

	RemoteOperation = 0x54,		// 远程开锁操作结果

	RemoteUnlock = 0x60,		//远程开锁
	SetDoorLockTime = 0x62,		//设置门锁时间

	WakeUp = 0x77,				// 门锁唤醒模块

	LoaclOperation = 0x80,		// 本地开锁
	LockReport = 0x81,			// 关锁上报
	NwkPrompt= 0x83,			// 入网退网提示
	NwkIndication = 0x84,		//网络状态显示
	GetTime = 0x85,				// 时间同步
	NormallyOpen = 0x86,		//门锁常开

}teSL_DoorLockCmd_t;



PUBLIC void APP_UARTInitialise(void);
PUBLIC void HAL_UART_Write(uint8 *WriteBuf);
PUBLIC void vUART_HandleUartInterrupt(void);
PUBLIC void APP_SerialSendJoinIndication(uint8 Operation, uint8 u8Decide);
PUBLIC void APP_cbTimerSerialDioSet(void);
PUBLIC void APP_cbTimerWriteSerial(void);
PUBLIC void APP_cbTimerSerialTimeout(void);
PUBLIC void App_StopSerialPrepareEnterSleep(void);
PUBLIC void APP_UartAchieveMessagefromPrivate(void);
PUBLIC void App_SerialSendRemoteBindUnlock(uint8 *nPass);


#endif
