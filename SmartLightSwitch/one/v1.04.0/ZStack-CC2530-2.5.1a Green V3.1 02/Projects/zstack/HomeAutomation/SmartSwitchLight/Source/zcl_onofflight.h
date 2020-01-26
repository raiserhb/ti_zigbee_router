/**************************************************************************************************
  Filename:       zcl_onofflight.h
  Revised:        $Date: 2009-12-29 18:31:22 -0800 (Tue, 29 Dec 2009) $
  Revision:       $Revision: 21416 $

  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


  Copyright 2006-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef ZCL_ONOFFLIGHT_H
#define ZCL_ONOFFLIGHT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"
#include "OSAL_Nv.h"
/*********************************************************************
 * CONSTANTS
 */
#define ONOFFLIGHT0_ENDPOINT            0x10
#define ONOFFLIGHT1_ENDPOINT            0x11
#define ONOFFLIGHT_ENDPONT_0xF0         0xF0
  
#define ONOFFLIGHT_MAX_ATTRIBUTES      12

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01
#define DEVICE_All        0x00
#define DEVICE_LIGHT      0X01

// Application Events
#define ONOFFLIGHT_IDENTIFY_TIMEOUT_EVT         0x0001
#define ZCL_DEVICE_SEND_ONOFF_STATE_EVENT       0x0002
#define ZCL_DEVICE_INIT_START_EVENT             0x0004
#define ZCL_DEVICE_INIT_DONE_EVENT              0x0008

#define ZCL_DEVICE_TOUCHPANEL_EVENT             0x0010
#define ZCL_DEVICE_RESTORE_INTERRUPT_EVENT      0x0020
#define ZCL_DEVICE_SOFT_RESTART_EVENT           0x0040
#define ZCL_DEVICE_EXIT_NETWORK_EVENT           0x1000
#define ZCL_DEVICE_LEDOFF_EVENT                 0x2000
#define ZCL_DEVICE_INTERRUPT_EVENT              0x4000  
  
#define ZCL_DEVICE_SEND_INIF_TIME_1S (1000+(osal_rand() & 0x00FF))
#define ZCL_DEVICE_SEND_INFO_TIME_1S (1000+(osal_rand() & 0x00FF) * 20)
#define ZCL_DEVICE_SEND_INFO_TIME_3S (3000+(osal_rand() & 0x00FF))
  
#define ZCL_DEVICE_SEND_INFO_TIME_200MS 200

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclOnOffLight0_SimpleDesc;
extern SimpleDescriptionFormat_t zclOnOffLight1_SimpleDesc;

extern CONST zclAttrRec_t zclOnOffLight0_Attrs[];
extern CONST zclAttrRec_t zclOnOffLight1_Attrs[];
extern uint8  zclOnOffLight0_OnOff;
extern uint8  zclOnOffLight1_OnOff;

extern uint16 zclOnOffLight_IdentifyTime;
extern uint8 ledcontorl;
/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclOnOffLight_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclOnOffLight_event_loop( byte task_id, UINT16 events );

void Device_App_MessageMSGCB(afIncomingMSGPacket_t *pkt);

void AppDeviceNwkJionState(void);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_ONOFFLIGHT_H */
