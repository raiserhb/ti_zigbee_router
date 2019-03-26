/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_occupancy_buttons.h
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

#ifndef APP_OCCUPANCY_BUTTONS_H_
#define APP_OCCUPANCY_BUTTONS_H_

#include "AppHardwareAPI_JN5169.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

#define E_AHI_DO0_INT			(0x01)
#define E_AHI_DO1_INT			(0x02)
#define E_AHI_DO_ALL_INT		((E_AHI_DO0_INT) | (E_AHI_DO1_INT))

// Ricky add for Clear Interrupts
#define E_AHI_DIO_ALL_INT		((E_AHI_DIO0_INT) | (E_AHI_DIO1_INT) | (E_AHI_DIO2_INT) | (E_AHI_DIO3_INT)		|		\
								 (E_AHI_DIO4_INT) | (E_AHI_DIO5_INT) | (E_AHI_DIO6_INT) | (E_AHI_DIO7_INT)		|		\
								 (E_AHI_DIO8_INT) | (E_AHI_DIO9_INT) | (E_AHI_DIO10_INT) | (E_AHI_DIO11_INT)	|		\
								 (E_AHI_DIO12_INT) | (E_AHI_DIO13_INT) | (E_AHI_DIO14_INT) | (E_AHI_DIO15_INT)	|		\
								 (E_AHI_DIO16_INT) | (E_AHI_DIO17_INT) | (E_AHI_DIO18_INT) | (E_AHI_DIO19_INT) )


#if (defined BUTTON_MAP_DR1199)
    #ifdef APP_NTAG
        typedef enum {
            APP_E_BUTTONS_BUTTON_1 = 0,
            APP_E_BUTTONS_BUTTON_SW1,
            APP_E_BUTTONS_BUTTON_SW2,
            APP_E_BUTTONS_BUTTON_SW3,
            APP_E_BUTTONS_BUTTON_SW4,
            APP_E_BUTTONS_NFC_FD
        } APP_teButtons;

        #if (JENNIC_CHIP_FAMILY == JN516x)
            #define APP_BUTTONS_NUM             (6UL)
            #define APP_BUTTONS_BUTTON_1        (8)
            #define APP_BUTTONS_BUTTON_SW1      (11)
            #define APP_BUTTONS_BUTTON_SW2      (12)
            #define APP_BUTTONS_BUTTON_SW3      (17)
            #define APP_BUTTONS_BUTTON_SW4      (1)
            #define APP_BUTTONS_NFC_FD          (0)
        #endif

        #if (JENNIC_CHIP_FAMILY == JN517x)
            #define APP_BUTTONS_NUM             (6UL)
            #define APP_BUTTONS_BUTTON_1        (4)
            #define APP_BUTTONS_BUTTON_SW1      (12)
            #define APP_BUTTONS_BUTTON_SW2      (13)
            #define APP_BUTTONS_BUTTON_SW3      (18)
            #define APP_BUTTONS_BUTTON_SW4      (5)
            #define APP_BUTTONS_NFC_FD          (17)
        #endif

        #define APP_BUTTONS_DIO_MASK                    ((1 << APP_BUTTONS_BUTTON_1)|(1 << APP_BUTTONS_BUTTON_SW4) | (1 << APP_BUTTONS_BUTTON_SW3) | (1 << APP_BUTTONS_BUTTON_SW2) | (1 << APP_BUTTONS_BUTTON_SW1) | (1 << APP_BUTTONS_NFC_FD))
        #define APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP     ((1 << APP_BUTTONS_BUTTON_SW4) | (1 << APP_BUTTONS_BUTTON_SW3) | (1 << APP_BUTTONS_BUTTON_SW2) | (1 << APP_BUTTONS_BUTTON_SW1) | (1 << APP_BUTTONS_NFC_FD))
    #else
        // No ntag
        typedef enum {
            APP_E_BUTTONS_BUTTON_1 = 0,
            APP_E_BUTTONS_BUTTON_SW1,
            APP_E_BUTTONS_BUTTON_SW11,
            APP_E_BUTTONS_BUTTON_SW2,
            APP_E_BUTTONS_BUTTON_SW12,
            APP_E_BUTTONS_BUTTON_SW5
        } APP_teButtons;

        #if (JENNIC_CHIP_FAMILY == JN516x)
		
			#if 1
				#define APP_BUTTONS_NUM				(6UL)
				#define APP_BUTTONS_BUTTON_1		(8)
				#define APP_BUTTONS_BUTTON_SW1		(11)
				#define APP_BUTTONS_BUTTON_SW11		(12)	// 11
				#define APP_BUTTONS_BUTTON_SW2		(17)	// 2
				#define APP_BUTTONS_BUTTON_SW3 		(13)	// 11
				#define APP_BUTTONS_BUTTON_SW12		(1)		// 12
				#define APP_BUTTONS_BUTTON_SW5		(19)	//5	Ricky
				#define APP_BUTTONS_BUTTON_SW9		(11)	//5 Ricky
				#define APP_BUTTONS_BUTTON_SW10		(9)	//5 Ricky
				
			#else
				#define APP_BUTTONS_NUM             (6UL)
				#define APP_BUTTONS_BUTTON_1        (8)
				#define APP_BUTTONS_BUTTON_SW1      (11)
				#define APP_BUTTONS_BUTTON_SW2      (12)
				#define APP_BUTTONS_BUTTON_SW3      (17)
				#define APP_BUTTONS_BUTTON_SW4      (1)
				#define APP_BUTTONS_NFC_FD          (0)
			#endif

        #endif

        #if (JENNIC_CHIP_FAMILY == JN517x)
            #define APP_BUTTONS_NUM             (5UL)
            #define APP_BUTTONS_BUTTON_1        (4)
            #define APP_BUTTONS_BUTTON_SW1      (12)
            #define APP_BUTTONS_BUTTON_SW2      (13)
            #define APP_BUTTONS_BUTTON_SW3      (18)
            #define APP_BUTTONS_BUTTON_SW4      (5)
        #endif

		#define BV(x)									(1 << x)

	#ifdef HUTLON

    #define APP_BUTTONS_DIO_MASK					( BV(APP_BUTTONS_BUTTON_SW5) | BV(APP_BUTTONS_BUTTON_SW12))
	#define APP_BUTTONS_PULLUP_MASK					(0)
//	#define APP_BUTTONS_PULLDOWN_MASK 				( BV(APP_BUTTONS_BUTTON_SW5) | BV(APP_BUTTONS_BUTTON_SW12))
	#define APP_BUTTONS_INPUT_MASK 					( BV(APP_BUTTONS_BUTTON_SW5) | BV(APP_BUTTONS_BUTTON_SW12))
	#define APP_BUTTONS_OUTPUT_MASK					( BV(APP_BUTTONS_BUTTON_SW11) )
	#define APP_BUTTONS_RISINGEDGE_MASK 			( BV(APP_BUTTONS_BUTTON_SW5) | BV(APP_BUTTONS_BUTTON_SW12))
	#define APP_BUTTONS_FALLINGEDGE_MASK 			(0)
	#define APP_BUTTONS_UART_MASK					( BV(APP_BUTTONS_BUTTON_SW10)| BV(APP_BUTTONS_BUTTON_SW9))

	#endif

        #define APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP		((1 << APP_BUTTONS_BUTTON_SW12) | (1 << APP_BUTTONS_BUTTON_SW2) | (1 << APP_BUTTONS_BUTTON_SW11) | (1 << APP_BUTTONS_BUTTON_SW1))
    #endif


#endif

typedef enum {
    E_INTERRUPT_UNKNOWN,
    E_INTERRUPT_BUTTON,
    E_INTERRUPT_WAKE_TIMER_EXPIRY
} teInterruptType;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void vApp_SampleDoorLockDoSetOutputLow(bool u8OLflgs);
PUBLIC void APP_bButtonInitialise(void);

PUBLIC void APP_cbTimerButtonScan(void *pvParam); // Ricky

PUBLIC void vActionOnButtonActivationAfterDeepSleep(void);
PUBLIC void vSaveDioStateBeforeDeepSleep(void);
PUBLIC bool_t bGetPreSleepOccupancyState(void);
#if (JENNIC_CHIP_FAMILY == JN516x)
PUBLIC void vISR_SystemController(void);
#endif
#if (JENNIC_CHIP_FAMILY == JN517x)
PUBLIC void vISR_SystemController(uint32 u32DeviceId, uint32 u32ItemBitMap);
#endif
PUBLIC bool_t bButtonDebounceInProgress(void);

//PRIVATE void Hal_Key_Poll(void);
PUBLIC void App_vSetUartPinOutputLow(void);
PUBLIC void APP_ButtonsWakeUpMCU(void);
PUBLIC void APP_ButtonsStopWakeUp(void);
PUBLIC void App_ReadDioStatusFromWakeup(void);


/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#endif /*APP_OCCUPANCY_BUTTONS_H_*/
