/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_main.h
 *
 * DESCRIPTION:        ZLO Main Event Handler (Interface)
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

#ifndef APP_MAIN_H_
#define APP_MAIN_H_

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "ZQueue.h"
#include "bdb_api.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void APP_vInitResources(void);
PUBLIC void APP_vSetUpHardware(void);
PUBLIC void APP_vMainLoop(void);
PUBLIC void vApp_SampleDoorLockStopAllTimer(void);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

extern PUBLIC uint8 u8TimerMinWake;
extern PUBLIC uint8 u8TimerPoll;

extern PUBLIC uint8 u8TimerButtonScan;
extern PUBLIC uint8 u8TimerStartJoinNWK;
extern PUBLIC uint8 u8TimerSerialDioSet;
extern PUBLIC uint8 u8TimerWriteSerial;
extern PUBLIC uint8 u8TimerSerialTimeout;
extern PUBLIC uint8 u8TimerHeartBeat;
extern PUBLIC uint8 u8TimerBasicWriteAttr;
extern PUBLIC uint8 u8TimerOtaStart;
extern PUBLIC uint8 u8TimerLeaveInd;
extern PUBLIC uint8 u8TimerSerialEvent;
extern PUBLIC uint8 u8TimerWakeUpTimeOut;
extern PUBLIC uint8 u8TimerRemoteUnlockPemit;
extern PUBLIC uint8 u8TimerStartResend;
extern PUBLIC uint8 u8AskNumGetWay;
extern PUBLIC uint8 u8DelayAskOneOfNumGetWay;

#ifdef SKYWORTH
//extern PUBLIC uint8 u8TimerRemoteBindUnlock;
#endif

//extern PUBLIC uint8 u8TimerBlink;
extern PUBLIC uint8 u8TimerTick;
#ifdef APP_NTAG
extern PUBLIC uint8 u8TimerNtag;
#endif


extern PUBLIC tszQueue zps_msgMlmeDcfmInd;
extern PUBLIC tszQueue zps_msgMcpsDcfmInd;
extern PUBLIC tszQueue zps_TimeEvents;

extern PUBLIC tszQueue APP_msgZpsEvents;
extern PUBLIC tszQueue APP_msgZclEvents;
extern PUBLIC tszQueue APP_msgAppEvents;
extern PUBLIC tszQueue APP_msgBdbEvents;

extern PUBLIC tszQueue APP_msgSerialRx;

extern PUBLIC bool_t APP_bPersistantPolling;
extern PUBLIC uint16 HeartBeatCount;

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#endif /* APP_MAIN_H_ */






