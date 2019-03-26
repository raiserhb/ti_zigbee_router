


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
extern PUBLIC uint8 RxSerialMsg[80];


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

#if 1//def SKYWORTH//moon
	DoorRing = 0x2A,			// 门铃
	BindUnlock = 0x66,
	BindUnlockCheck = 0x67,

	FingerRequest = 0x68,	// 指纹传输
	FingerTransfer = 0x69,
	FingerBreak = 0x6A,

	GetModelVersion = 0x70,
	ModelVersionReply = 0x71,
	CacheRequest = 0x72,
#endif

	PwdOperation = 0x73,//moon
	EnableDoorLock = 0x74, //moon
	AskDoorLockVerInfo = 0x75, //moon

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
	RequestIEEE = 0x87,		// 读取门锁串码
	ReviceIEEE = 0x88,

	RemoteManageUsers = 0x89,	// 用户管理
	ManageUsers = 0x8A,

#ifdef TEMPORAEY_PASSWORD	
	AddTempoaryPasswork = 0x8D, //添加临时密码
#endif

	SleepInTime = 0x8E,
	
	ServerAskKeyList = 0xC0,
	ServerOldOpenDoorList = 0xC1,
	ServerSetDoorSomeMode = 0xC2,
	DoorKeyPadLocked = 0xC3,
	DoorOtherIllgalAlarmReport = 0xC4,
	DoorUserListChangeReport = 0xC5,
	DoorSomeModeReport = 0xC6,
}teSL_DoorLockCmd_t;

PUBLIC void APP_vProcessIncomingSerialCommands ( uint8 u8RxByte );

PUBLIC void APP_UARTInitialise(void);
PUBLIC void HAL_UART_Write(uint8 *WriteBuf);
PUBLIC void vUART_HandleUartInterrupt(void);
PUBLIC void APP_SerialSendJoinIndication(uint8 Operation, uint8 u8Decide);
PUBLIC void APP_cbTimerSerialDioSet(void *pvParam);
PUBLIC void APP_cbTimerWriteSerial(void *pvParam);
PUBLIC void APP_cbTimerSerialTimeout(void *pvParam);
PUBLIC void App_StopSerialPrepareEnterSleep(void);
PUBLIC uint8 APP_UartAchieveMessagefromPrivate(uint8 u8RxBytes);
PUBLIC void APP_SerialSendAcknowlegement(uint8 cmdType,bool Ack);
PUBLIC void App_SerialSendNetworkIndication(uint8 u8Decide);
PUBLIC void App_SerialSendRemoteUnlock(uint8 *nPass, uint8 u8Cmd);
PUBLIC void App_SerialSendTimeSynchronizer(UTCTime u32UtcSecs);

#ifdef SKYWORTH
PUBLIC void APP_SerialSendFingerBreak(void);
PUBLIC void App_SerialSendRemoteBindUnlock(uint8 *nPass);
PUBLIC void App_DB_ConfigHeartPeriod(uint8 Length);
PUBLIC void App_SerialSendFristBindUnLockNotification(uint8 *nPass);
#endif

PUBLIC void APP_SerialSendModelVersion(void);

#ifdef TEMPORAEY_PASSWORD	
void App_ResendTempoaryPassword(void *pvParam);
void App_Stop_ResendTempoaryPassword(void);

void App_Handle_User_Write_Attribute(uint8 *palyload, uint16 size);//moons
PUBLIC void App_SendReportTempoaryPasswordState(void);//moons
#endif
PUBLIC void App_AckSleepInTime(uint8 ack);
#endif
PUBLIC void APP_SerialSendPwdOperationData(uint8 *p, uint16 size);//moon
PUBLIC void APP_SerialSendPwdOperationDataForYinghua(uint8 AppCmdId, uint16 size, uint8 *p);//moon
void App_CheckAtrributeFF33DataBuffer(void);
PUBLIC uint8 vUART_TxCharCRC(uint8 *SerialMsg);
PUBLIC void APP_AskDoorLockVerInfo(void);
void App_SendDataImmediatelyOrSaveIntoCacheBuffer(uint8 AppCmdId, uint8 commandId, uint8 size, uint8 *p);
//void App_SerialSendEnableDoorLoockPwd(uint8 cnt, uint8 *pwd);//moons
