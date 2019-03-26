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

#if (JENNIC_CHIP_FAMILY == JN516x)
PRIVATE void vISR_Timer0(void);
#endif

#if (JENNIC_CHIP_FAMILY == JN517x)
PRIVATE void vISR_Timer0(uint32 u32DeviceId, uint32 u32ItemBitMap);
#endif

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

#if (defined BUTTON_MAP_DR1199)
    PRIVATE uint8 s_u8ButtonDebounce[APP_BUTTONS_NUM] =
    {
        APP_BUTTON_SAMPLE_MASK,
        APP_BUTTON_SAMPLE_MASK,
        APP_BUTTON_SAMPLE_MASK,
        APP_BUTTON_SAMPLE_MASK,
        APP_BUTTON_SAMPLE_MASK,
#ifdef APP_NTAG
        APP_BUTTON_SAMPLE_MASK
#endif
    };
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
    #else
	    (1<<APP_BUTTONS_BUTTON_1),
        (1<<APP_BUTTONS_BUTTON_SW1),
        (1<<APP_BUTTONS_BUTTON_SW2),
        (1<<APP_BUTTONS_BUTTON_SW3),
        (1<<APP_BUTTONS_BUTTON_SW4),
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
	// Ricky 5��12 �˿����룬11�����
    vAHI_DioSetDirection(APP_BUTTONS_INPUT_MASK, APP_BUTTONS_OUTPUT_MASK);//(E_AHI_DIO_ALL_INT^APP_BUTTONS_DIO_MASK));

    /* Turn on pull-ups for DIO lines with buttons connected */
	vAHI_DioSetPullup(APP_BUTTONS_PULLUP_MASK, APP_BUTTONS_PULLDOWN_MASK);
	
//	vAHI_DioSetPullup(BV(APP_BUTTONS_BUTTON_SW12), 0);
	vAHI_DioSetOutput(0,APP_BUTTONS_OUTPUT_MASK);


    if (FALSE == (u16AHI_PowerStatus() & WAKE_FROM_DEEP_SLEEP))
    {
        /* Set the edge detection for Rising edges */	
		vAHI_DioWakeEdge(APP_BUTTONS_RISINGEDGE_MASK,APP_BUTTONS_FALLINGEDGE_MASK);

    }
    else
    {
        u32PreviousDioState = u32AHI_ReadNVData(DIO_STATE_NVM_LOCATION);
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Woke from deep sleep, previous Dio State = %08x", u32PreviousDioState);
    }

    /* Enable interrupts to occur on selected edge */
	vAHI_DioWakeEnable(APP_BUTTONS_INPUT_MASK, 0);//(E_AHI_DIO_ALL_INT^APP_BUTTONS_DIO_MASK));
	u32DioInterrupts = 0;

	DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: %0x  %0x",APP_BUTTONS_DIO_MASK,u32AHI_DioInterruptStatus());

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

    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: In %x vISR_SystemController",u8WakeInt);

	vAHI_ClearSystemEventStatus(~(0UL));	//Ricky add for Clear Interrupts 2017 0105

    if (u8WakeInt & E_AHI_WAKE_TIMER_MASK_0)
    {
        APP_tsEvent sButtonEvent;

        /* wake timer interrupt got us here */
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Wake Timer 0 Interrupt");
        vAHI_WakeTimerStop(E_AHI_WAKE_TIMER_0);

        /* Post a message to the stack so we aren't handling events
         * in interrupt context
         */
        sButtonEvent.eType = APP_E_EVENT_WAKE_TIMER;
        ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
    }

    if(u32DioInterrupts & APP_BUTTONS_DIO_MASK)
    {
        DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: Dio Interrupt %0x ",u32DioIntStat);

        /* disable edge detection on all buttons until scan complete */
//        vAHI_DioInterruptEnable(0, APP_BUTTONS_DIO_MASK);

        /* Begin debouncing the button press */
		bDebouncing = TRUE;

#if 0	//Ricky
		vAHI_TimerEnable(E_AHI_TIMER_0, 0, FALSE, TRUE, FALSE);

        vAHI_Timer0RegisterCallback(vISR_Timer0);
		vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 1);

		PWRM_vWakeInterruptCallback();
#else
		ZTIMER_eStart(u8TimerButtonScan, ZTIMER_TIME_MSEC(1));
#endif

    }

    if (u8WakeInt & E_AHI_WAKE_TIMER_MASK_1)
    {

        /* wake timer interrupt got us here */
        DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: Wake Timer 1 Interrupt");
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
{

    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: In vISR_SystemController");

    if (u32ItemBitMap & E_AHI_SYSCTRL_WK0_MASK)
    {
        APP_tsEvent sButtonEvent;

        /* wake timer interrupt got us here */
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Wake Timer 0 Interrupt");
        vAHI_WakeTimerStop(E_AHI_WAKE_TIMER_0);

        /* Post a message to the stack so we aren't handling events
         * in interrupt context
         */
        sButtonEvent.eType = APP_E_EVENT_WAKE_TIMER;
        ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
    }

    if (u32ItemBitMap & APP_BUTTONS_DIO_MASK)
    {
        u32DioInterrupts |= (u32ItemBitMap & APP_BUTTONS_DIO_MASK);
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Dio Interrupt %0x", u32DioInterrupts & APP_BUTTONS_DIO_MASK);

        /* disable edge detection on all buttons until scan complete */
        vAHI_DioInterruptEnable(0, APP_BUTTONS_DIO_MASK);

        /* Begin debouncing the button press */
        bDebouncing = TRUE;
        vAHI_TimerEnable(E_AHI_TIMER_0, 0, FALSE, TRUE, FALSE);
        vAHI_Timer0RegisterCallback(vISR_Timer0);
        vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 1);
    }

    if (u32ItemBitMap & E_AHI_SYSCTRL_WK1_MASK)
    {
        APP_tsEvent sButtonEvent;

        /* wake timer interrupt got us here */
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Wake Timer 1 Interrupt");

        PWRM_vWakeInterruptCallback();

        /* Post a message to the stack so we aren't handling events
        * in interrupt context
        */
        sButtonEvent.eType = APP_E_EVENT_PERIODIC_REPORT;
        ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
    }

}
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

}

/****************************************************************************
 *
 * NAME: vISR_Timer0
 *
 * DESCRIPTION:
 * An Occupancy Sensor specific Debounce task. In the real world this maybe
 * removed as the digital input will be driven from a sensor.
 *
 * PARAMETER:
 *
 * RETURNS:
 *
 ****************************************************************************/
#if (JENNIC_CHIP_FAMILY == JN516x)
PRIVATE void vISR_Timer0(void)
#endif
#if (JENNIC_CHIP_FAMILY == JN517x)
PRIVATE void vISR_Timer0(uint32 u32DeviceId, uint32 u32ItemBitMap)
#endif
{
    uint8 u8Button;
    uint32 u32DioInput = 0;
    bool_t bButtonDebounceComplete = TRUE;

    /* Clear any existing interrupt pending flags */
    (void)u32AHI_DioInterruptStatus();

    if(u32DioInterrupts != 0)
    {
        u32DioInput = (u32PreviousDioState ^ u32DioInterrupts) & APP_BUTTONS_DIO_MASK;
    }
    else
    {
        u32DioInput = u32AHI_DioReadInput();
    }

    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: APP_ButtonsScanTask, Buttons = %08x, Interrupts = %08x, Previous = %08x", APP_BUTTONS_DIO_MASK & u32DioInput, (u32DioInterrupts & APP_BUTTONS_DIO_MASK), u32PreviousDioState);

    u32DioInterrupts = 0;


    /* Loop over all buttons to check their Dio states*/
    for (u8Button = 0; u8Button < APP_BUTTONS_NUM; u8Button++)
    {

        /* Shift the previous debounce checks and add the new debounce reading*/
        s_u8ButtonDebounce[u8Button] <<= 1;
        s_u8ButtonDebounce[u8Button] |= (u32DioInput & s_u32ButtonDIOLine[u8Button]) ? TRUE : FALSE;
        s_u8ButtonDebounce[u8Button] &= APP_BUTTON_SAMPLE_MASK;

        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Button %d, Debounce = %02x, Dio State = %08x", u8Button, s_u8ButtonDebounce[u8Button], u32PreviousDioState);

        /* If previously the button was down but now it is up, post an event to the queue */
        if (((u32PreviousDioState & s_u32ButtonDIOLine[u8Button]) == 0) && (s_u8ButtonDebounce[u8Button] == APP_BUTTON_SAMPLE_MASK))
        {
            APP_tsEvent sButtonEvent;
            sButtonEvent.eType = APP_E_EVENT_BUTTON_UP;
            sButtonEvent.uEvent.sButton.u8Button = u8Button;
            sButtonEvent.uEvent.sButton.u32DIOState = u32DioInput;

            /* Save the new state */
            u32PreviousDioState |= s_u32ButtonDIOLine[u8Button];
            DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Button UP=%d, Dio State = %08x", u8Button, u32PreviousDioState);

            /* Post a message to the stack*/
            ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);

        }
        /* If previously the button was up but now it is down, post an event to the queue */
        else if (((u32PreviousDioState & s_u32ButtonDIOLine[u8Button]) != 0) && (s_u8ButtonDebounce[u8Button] == 0x0))
        {
            APP_tsEvent sButtonEvent;
            sButtonEvent.eType = APP_E_EVENT_BUTTON_DOWN;
            sButtonEvent.uEvent.sButton.u8Button = u8Button;
            sButtonEvent.uEvent.sButton.u32DIOState = u32DioInput;

            /* Save the new state */
            u32PreviousDioState &= ~s_u32ButtonDIOLine[u8Button];

            DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Button DN=%d, Dio State = %08x", u8Button, u32PreviousDioState);

            /* Post a message to the stack*/
            ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);

        }

        /* Still debouncing this button, clear flag to indicate more samples are required */
        else if(((s_u8ButtonDebounce[u8Button] != 0) && (s_u8ButtonDebounce[u8Button] != APP_BUTTON_SAMPLE_MASK)))
        {
            bButtonDebounceComplete &= FALSE;
        }

    }


    /* If all buttons are in a stable state, stop the scan timer and set the new interrupt edge requirements */
    if(bButtonDebounceComplete == TRUE)
    {
        /* Stop the scan timer as we have finished */
        bDebouncing = FALSE;
        vAHI_TimerDisable(E_AHI_TIMER_0);

        /* Set the new interrupt edge requirements */
        vAHI_DioWakeEdge((APP_BUTTONS_DIO_MASK & ~u32PreviousDioState), (APP_BUTTONS_DIO_MASK & u32PreviousDioState));

        /* Re enable DIO wake interrupts on all buttons */
        vAHI_DioInterruptEnable(APP_BUTTONS_DIO_MASK, 0);

        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Debounce complete, timer stopped, interrupts re-enabled, previous state %08x", u32PreviousDioState);
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Wake edges: Rising=%08x Falling=%08x", (APP_BUTTONS_DIO_MASK & ~u32PreviousDioState), (APP_BUTTONS_DIO_MASK & u32PreviousDioState));

    }
    else
    {
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Debounce in progress, timer continued");
        vAHI_TimerEnable(E_AHI_TIMER_0, 0, FALSE, TRUE, FALSE);
#if (JENNIC_CHIP_FAMILY == JN516x)
        vAHI_Timer0RegisterCallback(vISR_Timer0);
#endif
        vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 16000);

    }

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
    uint8 u8Button;
    uint32 u32DioInput = 0;
    bool_t bButtonDebounceComplete = TRUE;

    /* Clear any existing interrupt pending flags */
    (void)u32AHI_DioInterruptStatus();

#if 0
    if(u32DioInterrupts != 0)
    {
        u32DioInput = (u32PreviousDioState ^ u32DioInterrupts) & APP_BUTTONS_DIO_MASK;
    }
    else
    {
        u32DioInput = u32AHI_DioReadInput();
    }

    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: APP_ButtonsScanTask, Buttons = %08x, Interrupts = %08x, Previous = %08x", APP_BUTTONS_DIO_MASK & u32DioInput, (u32DioInterrupts & APP_BUTTONS_DIO_MASK), u32PreviousDioState);

    u32DioInterrupts = 0;

    /* Loop over all buttons to check their Dio states*/
    for (u8Button = 0; u8Button < APP_BUTTONS_NUM; u8Button++)
    {

        /* Shift the previous debounce checks and add the new debounce reading*/
        s_u8ButtonDebounce[u8Button] <<= 1;
        s_u8ButtonDebounce[u8Button] |= (u32DioInput & s_u32ButtonDIOLine[u8Button]) ? TRUE : FALSE;
        s_u8ButtonDebounce[u8Button] &= APP_BUTTON_SAMPLE_MASK;

        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Button %d, Debounce = %02x, Dio State = %08x", u8Button, s_u8ButtonDebounce[u8Button], u32PreviousDioState);

        /* If previously the button was down but now it is up, post an event to the queue */
        if (((u32PreviousDioState & s_u32ButtonDIOLine[u8Button]) == 0) && (s_u8ButtonDebounce[u8Button] == APP_BUTTON_SAMPLE_MASK))
        {
            APP_tsEvent sButtonEvent;
            sButtonEvent.eType = APP_E_EVENT_BUTTON_UP;
            sButtonEvent.uEvent.sButton.u8Button = u8Button;
            sButtonEvent.uEvent.sButton.u32DIOState = u32DioInput;

            /* Save the new state */
            u32PreviousDioState |= s_u32ButtonDIOLine[u8Button];
            DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Button UP=%d, Dio State = %08x", u8Button, u32PreviousDioState);

            /* Post a message to the stack*/
            ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);

        }
        /* If previously the button was up but now it is down, post an event to the queue */
        else if (((u32PreviousDioState & s_u32ButtonDIOLine[u8Button]) != 0) && (s_u8ButtonDebounce[u8Button] == 0x0))
        {
            APP_tsEvent sButtonEvent;
            sButtonEvent.eType = APP_E_EVENT_BUTTON_DOWN;
            sButtonEvent.uEvent.sButton.u8Button = u8Button;
            sButtonEvent.uEvent.sButton.u32DIOState = u32DioInput;

            /* Save the new state */
            u32PreviousDioState &= ~s_u32ButtonDIOLine[u8Button];

            DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Button DN=%d, Dio State = %08x", u8Button, u32PreviousDioState);

            /* Post a message to the stack*/
            ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);

        }

        /* Still debouncing this button, clear flag to indicate more samples are required */
        else if(((s_u8ButtonDebounce[u8Button] != 0) && (s_u8ButtonDebounce[u8Button] != APP_BUTTON_SAMPLE_MASK)))
        {
            bButtonDebounceComplete &= FALSE;
        }

    }

#else
	Hal_Key_Poll();
#endif

    /* If all buttons are in a stable state, stop the scan timer and set the new interrupt edge requirements */
	if(bButtonDebounceComplete == TRUE)
    {
		bDebouncing = FALSE;
        /* Stop the scan timer as we have finished */
        ZTIMER_eStop(u8TimerButtonScan);

		vAHI_DioWakeEdge(0,(APP_BUTTONS_DIO_MASK));	// Ricky
		vAHI_DioInterruptEnable(APP_BUTTONS_DIO_MASK,~(APP_BUTTONS_DIO_MASK));

		DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: vAHI_DioInterruptEnable %0x ",APP_BUTTONS_DIO_MASK);

    }
    else
    {
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Debounce in progress, timer continued");
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
    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Interrupt Status = %08x", u32AHI_DioInterruptStatus());
    u32DioInterrupts |= APP_BUTTONS_DIO_MASK;
    vAHI_DioWakeEnable(0, APP_BUTTONS_DIO_MASK);

    /* Begin debouncing the buttons */
    bDebouncing = TRUE;
    vAHI_TimerEnable(E_AHI_TIMER_0, 0, FALSE, TRUE, FALSE);
    vAHI_Timer0RegisterCallback(vISR_Timer0);
    vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 1);
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
    vAHI_WriteNVData(DIO_STATE_NVM_LOCATION, u32PreviousDioState);
    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Written %08x to NVM", u32AHI_ReadNVData(DIO_STATE_NVM_LOCATION));
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
    DBG_vPrintf(TRACE_APP_BUTTON, "\n APP Button: Occupancy = %08x: ", u32PreviousDioState & APP_BUTTONS_BUTTON_SW1);

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
	
	App_PollRateCnt = 2;

	if(u32DioInterrupts & (1 << APP_BUTTONS_BUTTON_SW12))	//12
	{
		APP_tsEvent sButtonEvent;

		sButtonEvent.eType = APP_E_EVENT_BUTTON_DOWN;
		sButtonEvent.uEvent.sButton.u8Button = SW4_PRESSED;
		sButtonEvent.uEvent.sButton.u32DIOState = (1 << APP_BUTTONS_BUTTON_SW12);

		ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
		vStartPollTimer(ZTIMER_TIME_MSEC(100));
		DBG_vPrintf(TRACE_APP_BUTTON, "\n Hal_Key_Poll SW4_PRESSED");
	}
	else if(u32DioInterrupts & (1 << APP_BUTTONS_BUTTON_SW5))
	{
		APP_tsEvent sButtonEvent;

		sButtonEvent.eType = APP_E_EVENT_BUTTON_DOWN;
		sButtonEvent.uEvent.sButton.u8Button = SW5_PRESSED;
		sButtonEvent.uEvent.sButton.u32DIOState = (1 << APP_BUTTONS_BUTTON_SW5);

		ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
		vStartPollTimer(ZTIMER_TIME_MSEC(100));

		DBG_vPrintf(TRACE_APP_BUTTON, "\n Hal_Key_Poll SW5_PRESSED");
	}
//	else
	{
		/* disable edge detection on all buttons until scan complete */
		DBG_vPrintf(TRACE_APP_BUTTON, "\n Hal_Key_Poll %0x ", u32DioInterrupts);
//		vAHI_ClearSystemEventStatus(~(0UL));	//Ricky add for Clear Interrupts 2017 0105

		u32DioInterrupts = 0;
	}

}

PUBLIC void APP_ButtonsWakeUpMCU(void)
{
//	DBG_vPrintf(TRACE_APP_BUTTON, "\n APP_ButtonsWakeUpMCU ");
	vAHI_DioSetOutput(BV(APP_BUTTONS_BUTTON_SW11),0);

}
PUBLIC void APP_ButtonsStopWakeUp(void)
{
	DBG_vPrintf(TRACE_APP_BUTTON, "\n APP_ButtonsStopWakeUp ");
	vAHI_DioSetOutput(0,BV(APP_BUTTONS_BUTTON_SW11));

}

PUBLIC void App_vSetUartPinOutputLow(void)
{
	DBG_vPrintf(TRACE_APP_BUTTON, "\n App_vSetUartPinOutputLow ");
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
