/**************************************************************************************************
  Filename:       zcl_sampleLight.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $


  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2006-2009 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

/*********************************************************************
  This device will be like a Light device.  This application is not
  intended to be a Light device, but will use the device description
  to implement this sample code.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"

#include "zcl_onofflight.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#include "zcl_light_control.h"
#include "zcl_device_info.h"

//#include "OSAL_Nv.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclOnOffLight_TaskID;
devStates_t zclDevice_NwkState;
int8 zclDevice_JoinNetworkOk = 6;
int8 zclDevice_InitDone = 0;
uint8 zcl_Led_Blink = 0;
endPointDesc_t DeviceApp_epDesc;
uint8 ledcontorl = 0;

uint8 g_u8sendDeviceInfoFlag = 0;//0x00默认发送；0x01收到服务器缺少设备信息的回执
uint8 ReJoinNetFlagInFlash[1] = {0xFF};//0x01:表示复位加入网络；0x02:表示断电重启
/*********************************************************************
 * GLOBAL FUNCTIONS
 */
/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 g_RestCount = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclOnOffLight_HandleKeys( byte shift, byte keys );
static void zclOnOffLight_BasicResetCB( void );
static void zclOnOffLight_IdentifyCB( zclIdentify_t *pCmd );
static void zclOnOffLight_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclOnOffLight0_OnOffCB(uint8 cmd );
static void zclOnOffLight_OnOffCB(uint8 light, uint8 cmd );
static void zclOnOffLight_ProcessIdentifyTimeChange( void );

// Functions to process ZCL Foundation incoming Command/Response messages 
static void zclOnOffLight_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclOnOffLight_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclOnOffLight_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclOnOffLight_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclOnOffLight_ProcessInDiscRspCmd( zclIncomingMsg_t *pInMsg );
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclOnOffLight0_CmdCallbacks =
{
  zclOnOffLight_BasicResetCB,              // Basic Cluster Reset command
  zclOnOffLight_IdentifyCB,                // Identify command  
  zclOnOffLight_IdentifyQueryRspCB,        // Identify Query Response command
  zclOnOffLight0_OnOffCB,                   // On/Off cluster command
  NULL,                                     // Level Control Move to Level command
  NULL,                                     // Level Control Move command
  NULL,                                     // Level Control Step command
  NULL,                                     // Group Response commands
  NULL,                                     // Scene Store Request command
  NULL,                                     // Scene Recall Request command
  NULL,                                     // Scene Response command
  NULL,                                     // Alarm (Response) command
  NULL,                                     // RSSI Location commands
  NULL,                                     // RSSI Location Response commands
};

/*********************************************************************
 * @fn          zclOnOffLight_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */

void zclOnOffLight_Init( byte task_id )
{
  zclOnOffLight_TaskID = task_id;
  zclDevice_JoinNetworkOk = 6;
  zclDevice_NwkState = DEV_INIT;
  zclDevice_InitDone = 0;
  // Set destination address to indirect
  //zclOnOffLight_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  //zclOnOffLight_DstAddr.endPoint = 0;
  //zclOnOffLight_DstAddr.addr.shortAddr = 0;
  
  DeviceApp_epDesc.endPoint = 0xF0;
  DeviceApp_epDesc.task_id = &zclOnOffLight_TaskID;
  DeviceApp_epDesc.simpleDesc
    = (SimpleDescriptionFormat_t *)&zclOnOffLight0_SimpleDesc;
  DeviceApp_epDesc.latencyReq = noLatencyReqs;
  
  // Register the endpoint description with the AF
  afRegister( &DeviceApp_epDesc );
  
  // This app is part of the Home Automation Profile
  zclHA_Init( &zclOnOffLight0_SimpleDesc );
  
  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( ONOFFLIGHT0_ENDPOINT, &zclOnOffLight0_CmdCallbacks );
  
  // Register the application's attribute list
  zcl_registerAttrList( ONOFFLIGHT0_ENDPOINT, ONOFFLIGHT_MAX_ATTRIBUTES, zclOnOffLight0_Attrs );
  
  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclOnOffLight_TaskID );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclOnOffLight_TaskID );
  
  
  zclDeviceInfoInit();
  zclLightControlInit();
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */

uint16 zclOnOffLight_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclOnOffLight_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
      case AF_INCOMING_MSG_CMD:
        Device_App_MessageMSGCB(MSGpkt);
        break;
      case ZCL_INCOMING_MSG:
        // Incoming ZCL Foundation command/response messages
        zclOnOffLight_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
        break;
      case KEY_CHANGE:
        zclOnOffLight_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        
        break;
      case ZDO_STATE_CHANGE:
        zclDevice_NwkState = (devStates_t)(MSGpkt->hdr.status);
        if (zclDevice_NwkState == DEV_ROUTER ) 
        {
          AppDeviceNwkJionState();
          osal_start_timerEx(zclOnOffLight_TaskID, 
                             ZCL_DEVICE_INFO_SEND_EVENT,
                             ZCL_DEVICE_SEND_INIF_TIME_1S);
          
        }
        break;
      default:
        break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if ( events & ONOFFLIGHT_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclOnOffLight_IdentifyTime > 0 )
      zclOnOffLight_IdentifyTime--;
    zclOnOffLight_ProcessIdentifyTimeChange();
    
    //return ( events ^ ONOFFLIGHT_IDENTIFY_TIMEOUT_EVT );
  }
  
  if (events & ZCL_DEVICE_SEND_ONOFF_STATE_EVENT) 
  {
    zclLightReportEvent(ONOFFLIGHT_ENDPONT_0xF0, &zclOnOffLight0_OnOff);  
  }
  
  //指示灯闪烁
  if (events & ZCL_DEVICE_EXIT_NETWORK_EVENT) 
  {
    if(ledcontorl == 0)
    {
      LED_RED_ON();
      osal_start_timerEx(zclOnOffLight_TaskID, 
                         ZCL_DEVICE_LEDOFF_EVENT,
                         200);
    }
  }
  if (events & ZCL_DEVICE_LEDOFF_EVENT) 
  {
    LED_RED_OFF();
    osal_start_timerEx(zclOnOffLight_TaskID, 
                       ZCL_DEVICE_EXIT_NETWORK_EVENT,
                       2000);
  }
  
  if (events & ZCL_DEVICE_INIT_DONE_EVENT) 
  {
    zclDevice_InitDone = 1;
    LED_RED_OFF();
    zclLightControlTouchPanelEnable();
    
    //初始化完成LED闪烁
    ledcontorl = 0;
    osal_start_timerEx(zclOnOffLight_TaskID,
                       ZCL_DEVICE_EXIT_NETWORK_EVENT,
                       1000);
  }
  
  if (events & ZCL_DEVICE_INFO_SEND_EVENT) 
  {
   zclSendDeviceInfo();
  }
  
  if (events & ZCL_DEVICE_HEARTBEAT_EVENT) 
  {
    zclSendHeartbeat();
  }  
  
  if (events & ZCL_DEVICE_INTERRUPT_EVENT) 
  {
     zclTouchPanelEvent();

  }
  
  //去抖200ms
  if (events & ZCL_DEVICE_TOUCHPANEL_EVENT) 
  {
    if( P0_0 == 0 ){
      osal_start_timerEx(zclOnOffLight_TaskID,
                         ZCL_DEVICE_INTERRUPT_EVENT,
                         100);
    }else{
      P0IEN |= BV(0);
    }
    //return (events ^ ZCL_DEVICE_LIGHT0_TOUCHPANEL_EVENT);
  }
  
  if (events & ZCL_DEVICE_RESTORE_INTERRUPT_EVENT) 
  {
    zclRestoreInterruptEvent();
    //return (events ^ ZCL_DEVICE_LIGHT1_TOUCHPANEL_EVENT);
  }
  
  if (events & ZCL_DEVICE_RESET_EVENT) 
  {
    zclFactoryResetEvent();
  }
  
  if(events & ZCL_DEVICE_RESET_1_EVENT)
  {
    if(P0_7 == 0)
    {
      g_RestCount++;
      if(g_RestCount>=3)
      {
        osal_start_timerEx(zclOnOffLight_TaskID, 
                           ZCL_DEVICE_RESET_EVENT,
                           100);
      }
      else
      {
        osal_start_timerEx(zclOnOffLight_TaskID, 
                           ZCL_DEVICE_RESET_1_EVENT,
                           100);
      }
    }
    else
    {
      P0IEN |= BV(7);
      g_RestCount=0;
    }
  }
  
  if(events & ZCL_DEVICE_SOFT_RESTART_EVENT)
  {
    if(zcl_Led_Blink >= 30)
    {
      zclFactoryReset(TRUE);
    }
    else
    {
      if(zcl_Led_Blink %2 == 0)
      {
        LED_RED_OFF();
      } else {
        LED_RED_ON();
      }
      zcl_Led_Blink++;
      osal_start_timerEx(zclOnOffLight_TaskID, 
                         ZCL_DEVICE_SOFT_RESTART_EVENT,
                         ZCL_FACTORY_RESET_HINT);
    }
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclOnOffLight_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclOnOffLight_HandleKeys( byte shift, byte keys )
{
 // zAddrType_t dstAddr;
  
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_2 )
  {
  }

  if ( keys & HAL_KEY_SW_3 )
  {
  }

  if ( keys & HAL_KEY_SW_4 )
  {
  }
}

/*********************************************************************
 * @fn      zclOnOffLight_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclOnOffLight_ProcessIdentifyTimeChange( void )
{
  if ( zclOnOffLight_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclOnOffLight_TaskID, ONOFFLIGHT_IDENTIFY_TIMEOUT_EVT, 1000 );
  }
  else
  {
    osal_stop_timerEx( zclOnOffLight_TaskID, ONOFFLIGHT_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclOnOffLight_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclOnOffLight_BasicResetCB( void )
{
  // Reset all attributes to default values
}

/*********************************************************************
 * @fn      zclOnOffLight_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclOnOffLight_IdentifyCB( zclIdentify_t *pCmd )
{
  zclOnOffLight_IdentifyTime = pCmd->identifyTime;
  zclOnOffLight_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclOnOffLight_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclOnOffLight_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
{
  // Query Response (with timeout value)
  (void)pRsp;
}


static void zclOnOffLight0_OnOffCB(uint8 cmd )
{
  zclOnOffLight_OnOffCB(0, cmd);
}
/*********************************************************************
 * @fn      zclOnOffLight_OnOffCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   cmd - COMMAND_ON, COMMAND_OFF or COMMAND_TOGGLE
 *
 * @return  none
 */
static void zclOnOffLight_OnOffCB(uint8 light, uint8 cmd )
{
  uint8 *zclOnOffLight_OnOff;
  
  if (light == 0)
    zclOnOffLight_OnOff = &zclOnOffLight0_OnOff;

  if ( cmd == COMMAND_ON ) 
  {
    *zclOnOffLight_OnOff = LIGHT_ON;
  } else if ( cmd == COMMAND_OFF ) 
  {
    *zclOnOffLight_OnOff = LIGHT_OFF;
  } 
  else 
  {
    if ( *zclOnOffLight_OnOff == LIGHT_OFF )
      *zclOnOffLight_OnOff = LIGHT_ON;
    else
      *zclOnOffLight_OnOff = LIGHT_OFF;
  }

  if ( *zclOnOffLight_OnOff == LIGHT_ON ) 
  {
    if (light == 0)
      LIGHT_TURN_ON_LIGHT0();
  } else {
    if (light == 0)
      LIGHT_TURN_OFF_LIGHT0();
  }
  
  if (light == 0) 
  {
    zclLightReportEvent(ONOFFLIGHT0_ENDPOINT, &zclOnOffLight0_OnOff);
  } 
}

/****************************************************************************** 
 * 
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclOnOffLight_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclOnOffLight_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
 
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclOnOffLight_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE    
    case ZCL_CMD_WRITE_RSP:
      zclOnOffLight_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclOnOffLight_ProcessInConfigReportCmd( pInMsg );
      break;
    
    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclOnOffLight_ProcessInConfigReportRspCmd( pInMsg );
      break;
    
    case ZCL_CMD_READ_REPORT_CFG:
      //zclOnOffLight_ProcessInReadReportCfgCmd( pInMsg );
      break;
    
    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclOnOffLight_ProcessInReadReportCfgRspCmd( pInMsg );
      break;
    
    case ZCL_CMD_REPORT:
      //zclOnOffLight_ProcessInReportCmd( pInMsg );
      break;
#endif   
    case ZCL_CMD_DEFAULT_RSP:
      zclOnOffLight_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER     
    case ZCL_CMD_DISCOVER_RSP:
      zclOnOffLight_ProcessInDiscRspCmd( pInMsg );
      break;
#endif  
    default:
      break;
  }
  
  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclOnOffLight_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclOnOffLight_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes 
    // attempt and, for each successfull request, the value of the requested 
    // attribute
  }

  return TRUE; 
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclOnOffLight_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclOnOffLight_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return TRUE; 
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclOnOffLight_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclOnOffLight_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
   
  // Device is notified of the Default Response command.
  (void)pInMsg;
  
  return TRUE; 
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclOnOffLight_ProcessInDiscRspCmd
 *
 * @brief   Process the "Profile" Discover Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclOnOffLight_ProcessInDiscRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverRspCmd_t *discoverRspCmd;
  uint8 i;
  
  discoverRspCmd = (zclDiscoverRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }
  
  return TRUE;
}
#endif // ZCL_DISCOVER


/****************************************************************************
****************************************************************************/


/***************************************************************************
*****************************************************************************/
void Device_App_MessageMSGCB(afIncomingMSGPacket_t *pkt)
{
  switch(pkt->clusterId)
  {
  case ZCL_CLUSTER_ID_PREMIT_JOIN_ZB_NET:
    if(pkt->cmd.Data[0] == 0xFF)
    {
      pkt->cmd.Data[0] = 120;            //pkt->cmd.Data[0] = 60;
    }
    NLME_PermitJoiningRequest(pkt->cmd.Data[0]);
    break;
  case ZCL_CLUSTER_ID_SEND_DEVICE_INFO:
    if(pkt->cmd.Data[0] == 0x01)
    {
    if(g_u8sendDeviceInfoFlag == 0)//设备未发送完消息之前不接受新的发送任务
      {
        g_u8sendDeviceInfoFlag=1;
        ZDApp_AnnounceNewAddress();
        osal_start_timerEx(zclOnOffLight_TaskID, 
                           ZCL_DEVICE_INFO_SEND_EVENT,
                           500);
      }
    }break;
       
    
  case ZCL_CLUSTER_ID_GEN_ON_OFF:
    {
      if(pkt->cmd.Data[3]== DEVICE_All || pkt->cmd.Data[3]== DEVICE_LIGHT)
      {
        if(pkt->cmd.Data[2] == COMMAND_ON)
        {
          LIGHT_TURN_ON_LIGHT0();
         
        }
        else if(pkt->cmd.Data[2] == COMMAND_OFF)
        {
          LIGHT_TURN_OFF_LIGHT0();
       
        }
        zclOnOffLight0_OnOff = pkt->cmd.Data[2];
        
        osal_start_timerEx(zclOnOffLight_TaskID, 
                           ZCL_DEVICE_SEND_ONOFF_STATE_EVENT,
                           ZCL_DEVICE_SEND_INFO_TIME_3S);
      }
    }
    break;
    
  default:
    break;
  }

}


//////////////////////////////////////////////////////////////////////////////////
//判断设备是否是新节点设备加入网络，是则打开设备
///////////////////////////////////////////////////////////////////////////////////
void AppDeviceNwkJionState(void)
{
  if(SUCCESS == osal_nv_read(ZDAPP_NV_SYSTEM_RESTART_FLAG,0,sizeof(ReJoinNetFlagInFlash),ReJoinNetFlagInFlash))
  {
    if(ReJoinNetFlagInFlash[0] == 0x01)//表示复位重启
    {
      NLME_PermitJoiningRequest(60);//上电网络允许加入
    }
    else
    {
      NLME_PermitJoiningRequest(0);//上电关闭网络允许加入
    }
    
  }
  else
  {
    ReJoinNetFlagInFlash[0] = 0x02;
    NLME_PermitJoiningRequest(60);//上电网络允许加入
    osal_nv_item_init(ZDAPP_NV_SYSTEM_RESTART_FLAG,sizeof(ReJoinNetFlagInFlash),NULL);
    osal_nv_write(ZDAPP_NV_SYSTEM_RESTART_FLAG,0,sizeof(ReJoinNetFlagInFlash),ReJoinNetFlagInFlash);
  }
}
