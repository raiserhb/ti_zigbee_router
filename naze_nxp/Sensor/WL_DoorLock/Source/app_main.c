/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_main.c
 *
 * DESCRIPTION:        ZLO Main event handler (Implementation)
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2016. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <stdint.h>
#include "jendefs.h"
#include "ZQueue.h"
#include "ZTimer.h"
#include "portmacro.h"
#include "zps_apl_af.h"
#include "mac_vs_sap.h"
#include "AppHardwareApi.h"
#include "dbg.h"
#include "app_main.h"
#include "app_occupancy_buttons.h"
#include "app_events.h"
#include "app_event_handler.h"
#include "app_zcl_sensor_task.h"
#include "app_occupancy_sensor_state_machine.h"
#include "PDM.h"
#include "app_zlo_sensor_node.h"
#include "app_nwk_event_handler.h"
#include "app_blink_led.h"
#include "app_sleep_handler.h"
#ifdef APP_NTAG
#include "app_ntag.h"
#endif

#ifdef CLD_OTA
#include "app_ota_client.h"
#endif

#ifdef DB_DOORLOCK
#include "app_DB_DoorLock.h"
#endif
#include "App_WL_DoorLock.h"
#include "app_SampleDoorLock_Uart.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

//#ifndef DEBUG_APP
//#define TRACE_APP   FALSE
//#else
#define TRACE_APP   TRUE
//#endif

#define TIMER_QUEUE_SIZE			8
#define MLME_QUEQUE_SIZE			8
#define MCPS_QUEUE_SIZE				24
#define ZPS_QUEUE_SIZE				1
#define ZCL_QUEUE_SIZE				1
#define APP_QUEUE_SIZE				8
#define BDB_QUEUE_SIZE				3

#define RX_QUEUE_SIZE				32


#ifdef APP_NTAG
#define APP_ZTIMER_STORAGE           5 /* NTAG: Added timer */
#else
#define APP_ZTIMER_STORAGE           20	// Ricky, jabin 18->20
#endif

#if JENNIC_CHIP_FAMILY == JN517x
#define NVIC_INT_PRIO_LEVEL_SYSCTRL (1)
#define NVIC_INT_PRIO_LEVEL_BBC     (7)
#endif

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PUBLIC void vApp_SampleDoorLockStopAllTimer(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
PUBLIC uint8 u8TimerMinWake;
PUBLIC uint8 u8TimerPoll;
PUBLIC uint8 u8TimerButtonScan;			//Ricky 扫描IO 用于消抖和区分长按、短按
PUBLIC uint8 u8TimerStartJoinNWK;		//Ricky 启动加网
PUBLIC uint8 u8TimerSerialDioSet;		//Ricky 串口写入数据前的唤醒信号
PUBLIC uint8 u8TimerWriteSerial;		//Ricky 串口写入只留一个入口
PUBLIC uint8 u8TimerSerialTimeout;		//Ricky 串口接收超时退出
PUBLIC uint8 u8TimerLeaveInd;			//Ricky 退网广播Leave并重启
PUBLIC uint8 u8TimerSerialEvent;		//Ricky 串口事件延时
PUBLIC uint8 u8TimerWakeUpTimeOut;		//Ricky 远程开锁时间控制
PUBLIC uint8 u8TimerRemoteUnlockPemit;	//Ricky 控制远程开锁次数
PUBLIC uint8 u8TimerStartResend;		//Ricky 开始重发
PUBLIC uint8 u8TimerTemporayPasswordResend;		//moon
PUBLIC uint8 u8AskNumGetWay;			//再次发送网关缓存请求 jabin
PUBLIC uint8 u8DelayAskOneOfNumGetWay;			//延迟请求网关缓存 jabin

#ifdef SKYWORTH
//PUBLIC uint8 u8TimerRemoteBindUnlock;	//Ricky 
#endif

#ifdef DB_DOORLOCK
PUBLIC uint8 u8TimerHeartBeat;	//Ricky 心跳定时
#endif
PUBLIC uint8 u8TimerBasicWriteAttr;	//Ricky 处理缓存指令

PUBLIC uint8 u8TimerOtaStart;
//PUBLIC uint8 u8TimerBlink;
PUBLIC uint8 u8TimerTick;
#if APP_NTAG
PUBLIC uint8 u8TimerNtag;
#endif

PUBLIC tszQueue APP_msgZpsEvents;
PUBLIC tszQueue APP_msgZclEvents;
PUBLIC tszQueue APP_msgAppEvents;
PUBLIC tszQueue APP_msgBdbEvents;

PUBLIC tszQueue APP_msgSerialRx;

PUBLIC bool_t APP_bPersistantPolling = FALSE;
PUBLIC volatile bool_t APP_bInitialised = FALSE;

PUBLIC uint16 HeartBeatCount = 0;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

PRIVATE ZTIMER_tsTimer asTimers[APP_ZTIMER_STORAGE + BDB_ZTIMER_STORAGE];

PRIVATE zps_tsTimeEvent asTimeEvent[TIMER_QUEUE_SIZE];
PRIVATE MAC_tsMcpsVsDcfmInd asMacMcpsDcfmInd[MCPS_QUEUE_SIZE];
PRIVATE MAC_tsMlmeVsDcfmInd asMacMlmeVsDcfmInd[MLME_QUEQUE_SIZE];
PRIVATE ZPS_tsAfEvent asZpsStackEvent[ZPS_QUEUE_SIZE];
PRIVATE ZPS_tsAfEvent asZclStackEvent[ZCL_QUEUE_SIZE];

PRIVATE APP_tsEvent asAppEvent[APP_QUEUE_SIZE];
PRIVATE BDB_tsZpsAfEvent asBdbEvent[BDB_QUEUE_SIZE];

uint8 au8AtRxBuffer[RX_QUEUE_SIZE];

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/


extern void zps_taskZPS(void);
extern void PWRM_vManagePower(void);

/*start of file*/
/****************************************************************************
 *
 * NAME: APP_vMainLoop
 *
 * DESCRIPTION:
 * Main  execution loop
 *
 * RETURNS:
 * Never
 *
 ****************************************************************************/
PUBLIC void APP_vMainLoop(void)
{

    APP_bInitialised |= TRUE;

    /* idle task commences on exit from OS start call */
    while (TRUE)
    {
        zps_taskZPS();

        bdb_taskBDB();

        ZTIMER_vTask();

        APP_taskSensor();

        /* Re-load the watch-dog timer. Execution must return through the idle
        * task before the CPU is suspended by the power manager. This ensures
        * that at least one task / ISR has executed with in the watchdog period
        * otherwise the system will be reset.
        */
        vAHI_WatchdogRestart();

        /* See if we are able to sleep or not */
		vAttemptToSleep();

        /*
        * suspends CPU operation when the system is idle or puts the device to
        * sleep if there are no activities in progress
        */
        PWRM_vManagePower();

    }
}

/****************************************************************************
 *
 * NAME: APP_vSetUpHardware
 *
 * DESCRIPTION:
 * Set up interrupts
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vSetUpHardware(void)
{
#if (JENNIC_CHIP_FAMILY == JN517x)
    vAHI_SysCtrlRegisterCallback ( vISR_SystemController );
    u32AHI_Init();
    vAHI_InterruptSetPriority ( MICRO_ISR_MASK_BBC,        NVIC_INT_PRIO_LEVEL_BBC );
    vAHI_InterruptSetPriority ( MICRO_ISR_MASK_SYSCTRL, NVIC_INT_PRIO_LEVEL_SYSCTRL );
#endif

#if (JENNIC_CHIP_FAMILY == JN516x)

    DBG_vPrintf(TRACE_APP, "\n APP_vSetUpHardware ");
    TARGET_INITIALISE();
    /* clear interrupt priority level  */
    SET_IPL(0);
    portENABLE_INTERRUPTS();
#endif
}

/****************************************************************************
 *
 * NAME: APP_vInitResources
 *
 * DESCRIPTION:
 * Initialise resources (timers, queue's etc)
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vInitResources(void)
{

    /* Initialise the Z timer module */
    ZTIMER_eInit(asTimers, sizeof(asTimers) / sizeof(ZTIMER_tsTimer));

    /* Create Z timers */
#if (JENNIC_CHIP_FAMILY == JN516x)
    ZTIMER_eOpen(&u8TimerMinWake,				NULL,								NULL);
    ZTIMER_eOpen(&u8TimerPoll,					APP_cbTimerPoll,					NULL);	
    ZTIMER_eOpen(&u8TimerButtonScan,			APP_cbTimerButtonScan,				NULL); //Ricky
    ZTIMER_eOpen(&u8TimerStartJoinNWK,			APP_cbStartNWKJoin,					NULL); //Ricky
    ZTIMER_eOpen(&u8TimerSerialDioSet,			APP_cbTimerSerialDioSet,			NULL); //Ricky
    ZTIMER_eOpen(&u8TimerWriteSerial,			APP_cbTimerWriteSerial,				NULL); //Ricky
    ZTIMER_eOpen(&u8TimerSerialTimeout,			APP_cbTimerSerialTimeout,			NULL); //Ricky 
    ZTIMER_eOpen(&u8TimerLeaveInd,				App_cbTimerLeaveAndFactoryNew,		NULL); //Ricky
    //ZTIMER_eOpen(&u8TimerSerialEvent,			App_cbTimerProcessSerialEvent,		NULL); //Ricky
    ZTIMER_eOpen(&u8TimerWakeUpTimeOut,			App_cbTimerWakeUpTimeOut,			NULL); //Ricky
	ZTIMER_eOpen(&u8TimerRemoteUnlockPemit, 	App_cbTimerRemoteOperationPermit,	NULL); //Ricky
    ZTIMER_eOpen(&u8TimerStartResend,			App_cbTimerStartResend,				NULL); //Ricky
    ZTIMER_eOpen(&u8TimerBasicWriteAttr,		APP_cbTimerZCLBasicWriteAttr,	NULL); //Ricky
    ZTIMER_eOpen(&u8AskNumGetWay,				APP_cbGetNumGetWay,					NULL);
    ZTIMER_eOpen(&u8DelayAskOneOfNumGetWay,		APP_cbDelayAskOneOfNumGetWay,		NULL);

#ifdef TEMPORAEY_PASSWORD	
	ZTIMER_eOpen(&u8TimerTemporayPasswordResend,App_ResendTempoaryPassword, 		NULL); //Moon
#endif

#ifdef SKYWORTH
//	ZTIMER_eOpen(&u8TimerRemoteBindUnlock, 		App_SerialSendRemoteBindUnlock,		NULL); //Ricky
#endif

#ifdef CLD_OTA
    ZTIMER_eOpen(&u8TimerOtaStart,				APP_cbTimerOTAStart,				NULL); //Ricky
#endif
    ZTIMER_eOpen(&u8TimerTick,					APP_cbTimerZclTick,					NULL);
//	ZTIMER_eOpen(&u8TimerBlink,					vAPP_cbBlinkLED,				NULL);	//Ricky

#ifdef APP_NTAG
    ZTIMER_eOpen(&u8TimerNtag,					APP_cbNtagTimer,				NULL);
#endif

#elif (JENNIC_CHIP_FAMILY == JN517x)
    ZTIMER_eOpen(&u8TimerMinWake,       NULL,                   NULL,   ZTIMER_FLAG_PREVENT_SLEEP);
    ZTIMER_eOpen(&u8TimerPoll,          APP_cbTimerPoll,        NULL,   ZTIMER_FLAG_PREVENT_SLEEP);
    ZTIMER_eOpen(&u8TimerTick,          APP_cbTimerZclTick,     NULL,   ZTIMER_FLAG_PREVENT_SLEEP);
//	ZTIMER_eOpen(&u8TimerBlink,         vAPP_cbBlinkLED,        NULL,   ZTIMER_FLAG_PREVENT_SLEEP);
#ifdef APP_NTAG
    ZTIMER_eOpen(&u8TimerNtag,          APP_cbNtagTimer,        NULL,   ZTIMER_FLAG_PREVENT_SLEEP);
#endif

#endif

    /* create all the queues*/
    ZQ_vQueueCreate(&APP_msgBdbEvents,      BDB_QUEUE_SIZE,     sizeof(BDB_tsZpsAfEvent),   (uint8*)asBdbEvent);
    ZQ_vQueueCreate(&APP_msgZpsEvents,      ZPS_QUEUE_SIZE,     sizeof(ZPS_tsAfEvent),      (uint8*)asZpsStackEvent);
    ZQ_vQueueCreate(&APP_msgZclEvents,      ZCL_QUEUE_SIZE,     sizeof(ZPS_tsAfEvent),      (uint8*)asZclStackEvent);
    ZQ_vQueueCreate(&APP_msgAppEvents,      APP_QUEUE_SIZE,     sizeof(APP_tsEvent),        (uint8*)asAppEvent);
    ZQ_vQueueCreate(&zps_msgMlmeDcfmInd,    MLME_QUEQUE_SIZE,   sizeof(MAC_tsMlmeVsDcfmInd),(uint8*)asMacMlmeVsDcfmInd);
    ZQ_vQueueCreate(&zps_msgMcpsDcfmInd,    MCPS_QUEUE_SIZE,    sizeof(MAC_tsMcpsVsDcfmInd),(uint8*)asMacMcpsDcfmInd);
    ZQ_vQueueCreate(&zps_TimeEvents,        TIMER_QUEUE_SIZE,   sizeof(zps_tsTimeEvent),    (uint8*)asTimeEvent);
	ZQ_vQueueCreate (&APP_msgSerialRx,		RX_QUEUE_SIZE,		sizeof(uint8),				(uint8*)au8AtRxBuffer);	//Ricky

}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

PUBLIC void vApp_SampleDoorLockStopAllTimer(void)
{
    DBG_vPrintf(TRACE_APP, "\n vApp_SampleDoorLockStopAllTimer ");

	if (ZTIMER_eGetState(u8TimerMinWake) == E_ZTIMER_STATE_RUNNING)
	{
		DBG_vPrintf(TRACE_APP, "\n u8TimerMinWake ");
		ZTIMER_eStop(u8TimerMinWake);
	}

	if (ZTIMER_eGetState(u8TimerButtonScan) == E_ZTIMER_STATE_RUNNING)
	{
		DBG_vPrintf(TRACE_APP, "\n u8TimerButtonScan ");
		ZTIMER_eStop(u8TimerButtonScan);
	}

	if (ZTIMER_eGetState(u8TimerStartJoinNWK) == E_ZTIMER_STATE_RUNNING)
	{
		DBG_vPrintf(TRACE_APP, "\n u8TimerStartJoinNWK ");
		ZTIMER_eStop(u8TimerStartJoinNWK);
	}
	
	if (ZTIMER_eGetState(u8TimerSerialTimeout) == E_ZTIMER_STATE_RUNNING)
	{
		DBG_vPrintf(TRACE_APP, "\n u8TimerSerialTimeout ");
		ZTIMER_eStop(u8TimerSerialTimeout);
	}

	if (ZTIMER_eGetState(u8TimerWakeUpTimeOut) == E_ZTIMER_STATE_RUNNING)
	{
		DBG_vPrintf(TRACE_APP, "\n u8TimerWakeUpTimeOut ");
//		ZTIMER_eStop(u8TimerWakeUpTimeOut);
	}

	if (ZTIMER_eGetState(u8TimerRemoteUnlockPemit) == E_ZTIMER_STATE_RUNNING)
	{
		DBG_vPrintf(TRACE_APP, "\n u8TimerRemoteUnlockPemit ");
		ZTIMER_eStop(u8TimerRemoteUnlockPemit);
	}

	if (ZTIMER_eGetState(u8TimerOtaStart) == E_ZTIMER_STATE_RUNNING)
	{
		DBG_vPrintf(TRACE_APP, "\n u8TimerOtaStart ");
	//	ZTIMER_eStop(u8TimerOtaStart);
	}

}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
