


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
	TransgressAlarm = 0x20,		// �Ƿ�����
	TamperAlarm = 0x22,			// ���𱨾�
	PretendLock = 0x23,			// ��������
	UnLockAlarm = 0x24,			// δ���ű���
	Coercion = 0x25,			// в�ȱ���
	AlarmReset = 0x29,			// �������

#ifdef SKYWORTH
	DoorRing = 0x2A,			// ����
#endif

	BatteryAlarm = 0x30,		// ��ѹ����

	JoinNWK = 0x40,				// ����
	LeaveNWK = 0x41,			// ����

	RemoteOperation = 0x54,		// Զ�̿����������

	RemoteUnlock = 0x60,		//Զ�̿���
	SetDoorLockTime = 0x62,		//��������ʱ��

	WakeUp = 0x77,				// ��������ģ��

	LoaclOperation = 0x80,		// ���ؿ���
	LockReport = 0x81,			// �����ϱ�
	NwkPrompt= 0x83,			// ����������ʾ
	NwkIndication = 0x84,		//����״̬��ʾ
	GetTime = 0x85,				// ʱ��ͬ��
	NormallyOpen = 0x86,		//��������

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
