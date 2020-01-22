
#include "App_OccupancySensor.h"


extern PUBLIC volatile uint8 RxSerialMsg[80];

typedef enum
{
	TransgressAlarm = 0x20,		// �Ƿ�����
	TamperAlarm = 0x22,			// ���𱨾�
	PretendLock = 0x23,			// �������� б�౨��
	UnLockAlarm = 0x24,			// δ���ű���
	Coercion = 0x25,			// в�ȱ���
	AlarmReset = 0x29,			// �������

#if 1//def SKYWORTH//moon
	DoorRing = 0x2A,			// ����
	BindUnlock = 0x66,
	BindUnlockCheck = 0x67,

	FingerRequest = 0x68,	// ָ�ƴ���
	FingerTransfer = 0x69,
	FingerBreak = 0x6A,

	GetModelVersion = 0x70,
	ModelVersionReply = 0x71,
	CacheRequest = 0x72,
#endif

	PwdOperation = 0x73,//moon
	EnableDoorLock = 0x74, //moon
	AskDoorLockVerInfo = 0x75, //moon

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
	RequestIEEE = 0x87,		// ��ȡ��������
	ReviceIEEE = 0x88,

	RemoteManageUsers = 0x89,	// �û�����
	ManageUsers = 0x8A,

#ifdef TEMPORAEY_PASSWORD	
	AddTempoaryPasswork = 0x8D, //�����ʱ����
#endif

	SleepInTime = 0x8E,
	
	ServerAskKeyList = 0xC0,
	ServerOldOpenDoorList = 0xC1,
	ServerSetDoorSomeMode = 0xC2,
	DoorKeyPadLocked = 0xC3,
	DoorOtherIllgalAlarmReport = 0xC4,
	DoorUserListChangeReport = 0xC5,
	DoorSomeModeReport = 0xC6,
	AskDoorLockRTC = 0xC7,
	DoorFingerInputLocked = 0xC8,
	DoorBackLocking = 0xC9,
	DoorReleaseBackLocking = 0xCA,
	DoorCardInputLocked = 0xCB,
	DoorMechanicalkeyOpenDoor = 0xCC,
	ServerAddDoorUserSeed = 0xCD,
	ServerDelDoorUserSeed = 0xCE,
	ServerAskDoorUserSeedList = 0xCF,
}teSL_DoorLockCmd_t;
PUBLIC void APP_vProcessIncomingSerialCommands ( uint8 u8RxByte );
PUBLIC void APP_UARTInitialise(void);
PUBLIC void HAL_UART_Write(const uint8 *WriteBuf);
PUBLIC void vUART_HandleUartInterrupt(void);
PUBLIC void APP_SerialSendJoinIndication(uint8 Operation, uint8 u8Decide);
PUBLIC void APP_cbTimerSerialDioSet(void *pvParam);
PUBLIC void APP_cbTimerWriteSerial(void *pvParam);
PUBLIC void APP_cbTimerSerialTimeout(void *pvParam);
PUBLIC void App_StopSerialPrepareEnterSleep(void);
PUBLIC uint8 APP_UartAchieveMessagefromPrivate(uint8 u8RxBytes);
PUBLIC void APP_SerialSendAcknowlegement(uint8 cmdType,bool Ack);
PUBLIC void App_SerialSendRemoteUnlock(uint8 *nPass, uint8 u8Cmd);
void App_Handle_User_Write_Attribute(uint8 *palyload, uint16 size);//moons
PUBLIC void App_AckSleepInTime(uint8 ack);
PUBLIC void APP_SerialSendPwdOperationData(uint8 *p, uint16 size);//moon
PUBLIC void APP_SerialSendPwdOperationDataForYinghua(uint8 AppCmdId, uint16 size, uint8 *p);//moon
void App_CheckAtrributeFF33DataBuffer(void);
PUBLIC uint8 vUART_TxCharCRC(uint8 *SerialMsg);
PUBLIC void App_SerialSendTimeSynchronizer(UTCTime u32UtcSecs);
PUBLIC void APP_SerialSendModelVersion(void);
PUBLIC void App_SerialSendNetworkIndication(uint8 u8Decide);
void App_SendDataImmediatelyOrSaveIntoCacheBuffer(uint8 AppCmdId, uint8 commandId, uint8 size, uint8 *p);//E_CLD_DOOR_LOCK_CMD_SET_YEAR_DAY_SCHEDULE

