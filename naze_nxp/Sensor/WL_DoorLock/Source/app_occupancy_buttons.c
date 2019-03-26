/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_occupancy_buttons.c
 *
 * DESCRIPTION:        DK4 (DR1175/DR1199) Button Press detection (Implementation)
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
#include <jendefs.h>
#include "ZTimer.h"
#include "ZQueue.h"
#include "app_main.h"
#include "DBG.h"
#include "AppHardwareApi.h"
#include "app_events.h"
#include "app_event_handler.h"	//Ricky
#include "app_nwk_event_handler.h"

#include "pwrm.h"
#include "app_occupancy_buttons.h"
#include "app_occupancy_sensor_state_machine.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_APP_BUTTON
    #define TRACE_APP_BUTTON               FALSE
#else
    #define TRACE_APP_BUTTON               TRUE
#endif

#define WAKE_FROM_DEEP_SLEEP     (1<<11)
#define DIO_STATE_NVM_LOCATION     0

/* Defines the number of bits sampled for a button debounce event. One sample taken for each bit set */
#define APP_BUTTON_SAMPLE_MASK          (0x1f)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PUBLIC void APP_ButtonsWakeUpMCU(void);
PUBLIC void APP_ButtonsStopWakeUp(void);
PRIVATE void Hal_Key_Poll(void);
PUBLIC void App_ReadDioStatusFromWakeup(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

#if (defined BUTTON_MAP_DR1199)

    PRIVATE uint32 u32PreviousDioState = APP_BUTTONS_DIO_MASK;
    PRIVATE const uint32 s_u32ButtonDIOLine[APP_BUTTONS_NUM] =
    {
    #if 1	//Ricky
        (1<<APP_BUTTONS_BUTTON_1),
        (1<<APP_BUTTONS_BUTTON_SW1),
        (1<<APP_BUTTONS_BUTTON_SW11),
        (1<<APP_BUTTONS_BUTTON_SW2),
        (1<<APP_BUTTONS_BUTTON_SW12),
        (1<<APP_BUTTONS_BUTTON_SW5),
	#endif

#ifdef ZPP_NTAG
        (1<<APP_BUTTONS_NFC_FD)          /* NTAG_FD/NCI_IRQ */
#endif

    };
#endif

PRIVATE uint32 u32DioInterrupts = 0;
PRIVATE volatile bool_t bDebouncing = FALSE;


/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void vApp_SampleDoorLockDoSetOutputLow(bool u8OLflgs)
{

#if 0
	vAHI_SpiDisable();
	vAHI_SpiSlaveDisable();
//	bAHI_EnableD0andD1Outputs(TRUE);
#endif

	DBG_vPrintf(TRACE_APP_BUTTON, "\n bAHI_DoEnableOutputs %d ",bAHI_DoEnableOutputs(u8OLflgs));
	vAHI_DoSetPullup(0,255);//E_AHI_DO_ALL_INT);
	vAHI_DoSetDataOut(255,0);//E_AHI_DO_ALL_INT,0);

}

/****************************************************************************
 *
 * NAME: APP_bButtonInitialise
 *
 * DESCRIPTION:
 * Button Initialization
 *
 * PARAMETER: void
 *
 * RETURNS: bool
 *
 ****************************************************************************/
PUBLIC void APP_bButtonInitialise(void)
{

	DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: APP_bButtonInitialise");

    /* Set DIO lines to inputs with buttons connected */
	// Ricky 5、12 端口输入，11脚输出，其余引脚输出低。
    vAHI_DioSetDirection(APP_BUTTONS_INPUT_MASK, (E_AHI_DIO_ALL_INT^APP_BUTTONS_INPUT_MASK));// APP_BUTTONS_OUTPUT_MASK);

    /* Turn on pull-ups for DIO lines with buttons connected */
	vAHI_DioSetPullup(APP_BUTTONS_PULLUP_MASK, (E_AHI_DIO_ALL_INT^APP_BUTTONS_PULLUP_MASK));//APP_BUTTONS_PULLDOWN_MASK);

	vAHI_DioSetOutput(0,(E_AHI_DIO_ALL_INT^(APP_BUTTONS_INPUT_MASK)));//APP_BUTTONS_OUTPUT_MASK);


//	if (FALSE == (u16AHI_PowerStatus() & WAKE_FROM_DEEP_SLEEP))
    {
        /* Set the edge detection for Rising edges */	
		vAHI_DioWakeEdge(APP_BUTTONS_RISINGEDGE_MASK,APP_BUTTONS_FALLINGEDGE_MASK);
		//vAHI_DioWakeEdge(APP_BUTTONS_FALLINGEDGE_MASK, APP_BUTTONS_RISINGEDGE_MASK);
    }

    /* Enable interrupts to occur on selected edge */
	vAHI_DioWakeEnable(APP_BUTTONS_INPUT_MASK, 0);//(E_AHI_DIO_ALL_INT^APP_BUTTONS_DIO_MASK));
	u32DioInterrupts = 0;

	// Ricky DO1 输出低
//	vApp_SampleDoorLockDoSetOutputLow(1);

}

#if (JENNIC_CHIP_FAMILY == JN516x)
/****************************************************************************
 *
 * NAME: vISR_SystemController (JN516x version )
 *
 * DESCRIPTION:
 * ISR called on system controller interrupt. This may be from the synchronous driver
 * (if used) or user pressing a button the the DK4 build
 *
 * PARAMETER:
 *
 * RETURNS:
 *
 ****************************************************************************/
PUBLIC void vISR_SystemController(void)
{

    /* clear pending DIO changed bits by reading register */
    uint8 u8WakeInt = u8AHI_WakeTimerFiredStatus();
	uint32 u32DioIntStat = u32AHI_DioInterruptStatus();
    u32DioInterrupts |= u32DioIntStat;//u32AHI_DioInterruptStatus();

    //DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: In %x vISR_SystemController",u8WakeInt);

	vAHI_ClearSystemEventStatus(~(0UL));	//Ricky add for Clear Interrupts 2017 0105

    if (u8WakeInt & E_AHI_WAKE_TIMER_MASK_0)
    {
        APP_tsEvent sButtonEvent;

        /* wake timer interrupt got us here */
//        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Wake Timer 0 Interrupt");
        vAHI_WakeTimerStop(E_AHI_WAKE_TIMER_0);

        /* Post a message to the stack so we aren't handling events
         * in interrupt context
         */
        sButtonEvent.eType = APP_E_EVENT_WAKE_TIMER;
        ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
    }

    if(u32DioInterrupts & APP_BUTTONS_DIO_MASK)
    {
//        DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: Dio Interrupt %0x ",u32DioIntStat);

        /* disable edge detection on all buttons until scan complete */
        vAHI_DioInterruptEnable(0, APP_BUTTONS_DIO_MASK);

        /* Begin debouncing the button press */
		bDebouncing = TRUE;
		ZTIMER_eStart(u8TimerButtonScan, ZTIMER_TIME_MSEC(1));
    }

    if (u8WakeInt & E_AHI_WAKE_TIMER_MASK_1)
    {

        /* wake timer interrupt got us here */
//        DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: Wake Timer 1 Interrupt");
        PWRM_vWakeInterruptCallback();

//		else
		{
			/* Post a message to the stack so we aren't handling events
			* in interrupt context*/
			APP_tsEvent sButtonEvent;
			sButtonEvent.eType = APP_E_EVENT_PERIODIC_REPORT;
			ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
		}

    }

}
#endif

#if (JENNIC_CHIP_FAMILY == JN517x)
/****************************************************************************
 *
 * NAME: vISR_SystemController (JN517x version )
 *
 * DESCRIPTION:
 * ISR called on system controller interrupt. This may be from the synchronous driver
 * (if used) or user pressing a button the the DK4 build
 *
 * PARAMETER:
 *
 * RETURNS:
 *
 ****************************************************************************/
PUBLIC void vISR_SystemController(uint32 u32DeviceId, uint32 u32ItemBitMap)
{}
#endif


/****************************************************************************
 *
 * NAME: vISR_Timer2
 *
 * DESCRIPTION:
 * Stub function to allow DK4 'bulbs' to build
 *
 * PARAMETER:
 *
 * RETURNS:
 *
 ****************************************************************************/
PUBLIC void vISR_Timer2( void)
{
	;
}

/****************************************************************************
 *
 * NAME: APP_ButtonsScanTask
 *
 * DESCRIPTION:
 * An Sensor specific Debounce task. In the real world this maybe
 * removed as the digital input will be driven from a sensor.
 *
 * PARAMETER:
 *
 * RETURNS:
 *
 ****************************************************************************/
PUBLIC void APP_cbTimerButtonScan(void *pvParam)
{
    bool_t bButtonDebounceComplete = TRUE;

    /* Clear any existing interrupt pending flags */
    (void)u32AHI_DioInterruptStatus();

	Hal_Key_Poll();

    /* If all buttons are in a stable state, stop the scan timer and set the new interrupt edge requirements */
	if(bButtonDebounceComplete == TRUE)
    {
		bDebouncing = FALSE;
        /* Stop the scan timer as we have finished */
        ZTIMER_eStop(u8TimerButtonScan);

		vAHI_DioWakeEdge(0,(APP_BUTTONS_DIO_MASK));	// Ricky
		vAHI_DioInterruptEnable(APP_BUTTONS_DIO_MASK,~(APP_BUTTONS_DIO_MASK));

	//	DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: vAHI_DioInterruptEnable %0x ",APP_BUTTONS_DIO_MASK);

    }
    else
    {
//        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Debounce in progress, timer continued");
        ZTIMER_eStart(u8TimerButtonScan, ZTIMER_TIME_MSEC(100));
    }

}

/****************************************************************************
 *
 * NAME: vActionOnButtonActivationAfterDeepSleep
 *
 * DESCRIPTION:
 * When we wake up, we have restarted so we need to manually check to see
 * what Dio woke us. Start the ButtonScanTask and disable wake interrupts
 *
 ****************************************************************************/
PUBLIC void vActionOnButtonActivationAfterDeepSleep(void)
{
//    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Interrupt Status = %08x", u32AHI_DioInterruptStatus());
    u32DioInterrupts |= APP_BUTTONS_INPUT_MASK;
    vAHI_DioWakeEnable(0, APP_BUTTONS_INPUT_MASK);

    /* Begin debouncing the buttons */
    bDebouncing = TRUE;
//    vAHI_TimerEnable(E_AHI_TIMER_0, 0, FALSE, TRUE, FALSE);
//    vAHI_Timer0RegisterCallback(vISR_Timer0);
//    vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 1);
}

/****************************************************************************
 *
 * NAME: vSaveDioStateBeforeDeepSleep
 *
 * DESCRIPTION:
 * Due to us going to sleep on a falling edge as well as a rising edge, we need
 * to save the Dio state into NVM so when we wake back up we know what edge we
 * had configured to wake us up.
 *
 ****************************************************************************/
PUBLIC void vSaveDioStateBeforeDeepSleep(void)
{
    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Writing %08x to NVM", u32PreviousDioState);
//    vAHI_WriteNVData(DIO_STATE_NVM_LOCATION, u32PreviousDioState);
//    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Written %08x to NVM", u32AHI_ReadNVData(DIO_STATE_NVM_LOCATION));
}

/****************************************************************************
 *
 * NAME: bGetPreSleepOccupancyState
 *
 * DESCRIPTION:
 * This function returns the last Dio edge of Switch 1 which determines what the last
 * Occupancy state was.
 *
 ****************************************************************************/
PUBLIC bool_t bGetPreSleepOccupancyState(void)
{
//    DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: Occupancy = %08x: ", u32PreviousDioState & APP_BUTTONS_BUTTON_SW1);

    /* If previously the occupied DIO pin was low, return occupied, else unoccupied */
    if((u32PreviousDioState & APP_BUTTONS_BUTTON_SW1) == 0)
    {
        DBG_vPrintf(TRACE_APP_BUTTON, "Occupied");
        return TRUE;
    }
    else
    {
        DBG_vPrintf(TRACE_APP_BUTTON, "Unoccupied");
        return FALSE;
    }

}


/****************************************************************************
 *
 * NAME: bButtonDebounceInProgress
 *
 * DESCRIPTION:
 * This function returns TRUE if the button debouncing routine is busy processing
 * a button press / release.
 *
 ****************************************************************************/
PUBLIC bool_t bButtonDebounceInProgress(void)
{
    return bDebouncing;
}
/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

PRIVATE void Hal_Key_Poll(void)
{
//	DBG_vPrintf(TRACE_APP_BUTTON, "\n u32AHI_DioReadInput %08x %08x \n",u32AHI_DioReadInput(),u32DioInterrupts);
	
	vAppSampleDoorLockPollChange(6);

	if(u32DioInterrupts & (1 << APP_BUTTONS_BUTTON_SW12))	//12
	{
		APP_tsEvent sButtonEvent;

		sButtonEvent.eType = APP_E_EVENT_BUTTON_DOWN;
		sButtonEvent.uEvent.sButton.u8Button = SW4_PRESSED;
		sButtonEvent.uEvent.sButton.u32DIOState = (1 << APP_BUTTONS_BUTTON_SW12);

		ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
		vStartPollTimer(ZTIMER_TIME_MSEC(500));
		DBG_vPrintf(TRACE_APP_BUTTON, "\n Hal_Key_Poll SW4_PRESSED");
	}
#if 0
	else if(u32DioInterrupts & (1 << APP_BUTTONS_BUTTON_SW3))
	{
		APP_tsEvent sButtonEvent;

		sButtonEvent.eType = APP_E_EVENT_BUTTON_DOWN;
		sButtonEvent.uEvent.sButton.u8Button = SW3_PRESSED;
		sButtonEvent.uEvent.sButton.u32DIOState = (1 << APP_BUTTONS_BUTTON_SW3);

		ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
		vStartPollTimer(ZTIMER_TIME_MSEC(100));

		DBG_vPrintf(TRACE_APP_BUTTON, "\n Hal_Key_Poll SW3_PRESSED");
	}
#endif
	else if(u32DioInterrupts & (1 << APP_BUTTONS_BUTTON_SW5))
	{
		APP_tsEvent sButtonEvent;

		sButtonEvent.eType = APP_E_EVENT_BUTTON_DOWN;
		sButtonEvent.uEvent.sButton.u8Button = SW5_PRESSED;
		sButtonEvent.uEvent.sButton.u32DIOState = (1 << APP_BUTTONS_BUTTON_SW5);

		ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
		vStartPollTimer(ZTIMER_TIME_MSEC(500));

		DBG_vPrintf(TRACE_APP_BUTTON, "\n Hal_Key_Poll SW5_PRESSED");
	}
//	else
	{
		/* disable edge detection on all buttons until scan complete */
	//	DBG_vPrintf(TRACE_APP_BUTTON, "\n Hal_Key_Poll %0x ", u32DioInterrupts);
//		vAHI_ClearSystemEventStatus(~(0UL));	//Ricky add for Clear Interrupts 2017 0105

		u32DioInterrupts = 0;
	}

}

PUBLIC void APP_ButtonsWakeUpMCU(void)
{
//	DBG_vPrintf(TRACE_APP_BUTTON, "\n APP_ButtonsWakeUpMCU ");
	vAHI_DioSetOutput(BV(APP_BUTTONS_BUTTON_SW11),0);
	//vAHI_DioSetOutput(0,BV(APP_BUTTONS_BUTTON_SW11));
}
PUBLIC void APP_ButtonsStopWakeUp(void)
{
//	DBG_vPrintf(TRACE_APP_BUTTON, "\n APP_ButtonsStopWakeUp ");
	vAHI_DioSetOutput(0,BV(APP_BUTTONS_BUTTON_SW11));
	//vAHI_DioSetOutput(BV(APP_BUTTONS_BUTTON_SW11),0);
}

PUBLIC void App_vSetUartPinOutputLow(void)
{
//	DBG_vPrintf(TRACE_APP_BUTTON, "\n App_vSetUartPinOutputLow ");
    vAHI_DioSetDirection(0, APP_BUTTONS_UART_MASK );

    /* Turn on pull-ups for DIO lines with buttons connected */
	vAHI_DioSetPullup(0, APP_BUTTONS_UART_MASK );
	vAHI_DioSetOutput(0, APP_BUTTONS_UART_MASK);

}

PUBLIC void App_ReadDioStatusFromWakeup(void)
{

	u32DioInterrupts = u32AHI_DioWakeStatus();
	DBG_vPrintf(TRACE_APP_BUTTON, "\n App_ReadDioStatusFromWakeup %0x ", u32DioInterrupts);
	Hal_Key_Poll();

}


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
