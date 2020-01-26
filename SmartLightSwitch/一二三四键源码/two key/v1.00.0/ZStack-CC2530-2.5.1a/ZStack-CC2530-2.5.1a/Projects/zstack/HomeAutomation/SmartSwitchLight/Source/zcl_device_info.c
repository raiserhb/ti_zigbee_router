#include "zcl.h"
#include "ZDObject.h"
#include "zcl_device_info.h"
#include "zcl_onofflight.h"
#include "zcl_light_control.h"
#include "OnBoard.h"
#include "ZGlobals.h"
#include "ZComDef.h"

extern byte zclOnOffLight_TaskID;
#define ZCL_TASK_ID zclOnOffLight_TaskID

afAddrType_t zclIASZoneMotionIR_Coord_nwkAddr;
static uint16 gDeviceInfoSendStatus = 0;

uint8 softeVision[3] = {0x01,0x09,0x00};//软件版本号
uint8 hardVision[2] = {0x00,0x00};//硬件北版本号
uint8 g_u8HeartBeatCount = 0;//心跳包计数

void zclDeviceInfoInit(void)
{
  gDeviceInfoSendStatus = 0;
  zclIASZoneMotionIR_Coord_nwkAddr.addrMode = afAddr16Bit;
  zclIASZoneMotionIR_Coord_nwkAddr.addr.shortAddr = 0x0000;
  zclIASZoneMotionIR_Coord_nwkAddr.endPoint = 0xF0;
}

static void zclActiveEPRsp(void)
{
  uint8 pBuf[64];
  byte cnt = 0;
  zAddrType_t srcAddr;
  
  srcAddr.addrMode = afAddr16Bit;
  srcAddr.addr.shortAddr = 0x0000;
  
  cnt = afNumEndPoints() - 1;
  afEndPoints( (uint8 *)pBuf, true );
  ZDP_ActiveEPRsp(0x0, &srcAddr, ZDP_SUCCESS,
                  NLME_GetShortAddr(), 
                  cnt, (uint8 *)pBuf, 0);
  gDeviceInfoSendStatus++;
  
  osal_start_timerEx(ZCL_TASK_ID, 
                     ZCL_DEVICE_INFO_SEND_EVENT,
                     ZCL_DEVICE_SEND_INFO_TIME_10S);
}

static void zclSimpleDescRsp(uint16 endpoint)
{
  zdoIncomingMsg_t *imMsg;
  uint16 nwkAddr = 0;
  
  nwkAddr = NLME_GetShortAddr();
  
  imMsg = (zdoIncomingMsg_t *)osal_msg_allocate( sizeof( zdoIncomingMsg_t ) + 3 );
  imMsg->srcAddr.addrMode = afAddr16Bit;
  imMsg->srcAddr.addr.shortAddr = 0x0000;
  imMsg->TransSeq = (uint8)0x00;
  imMsg->asdu = (byte*)(((byte*)imMsg) + sizeof( zdoIncomingMsg_t ));
  imMsg->asdu[0] = LO_UINT16( nwkAddr );
  imMsg->asdu[1] = HI_UINT16( nwkAddr );
  imMsg->asdu[2] = endpoint;
    
  ZDO_ProcessSimpleDescReq(imMsg);
  osal_msg_deallocate((uint8 *)imMsg);
  
  gDeviceInfoSendStatus++;
  osal_start_timerEx(ZCL_TASK_ID, 
                     ZCL_DEVICE_INFO_SEND_EVENT,
                     ZCL_DEVICE_SEND_INFO_TIME_10S);
}

static endPointDesc_t zclHeartbeatDesc = {
  ZCL_HEARTBEAT_ENDPOINT,
  &ZCL_TASK_ID,
  (SimpleDescriptionFormat_t *)&zclOnOffLight0_SimpleDesc,
  (afNetworkLatencyReq_t)0
};

static uint8 zclHeartbeatCounter = 0;

afStatus_t zclSendHeartbeat(void)
{
  afStatus_t stat;
  uint8 SendDataBuf[6]={0,0,0,0,0,0};
  
  SendDataBuf[0] = 0xFF;
  SendDataBuf[1] = softeVision[0];
  SendDataBuf[2] = softeVision[1];
  SendDataBuf[3] = softeVision[2];
  SendDataBuf[4] = hardVision[0];
  SendDataBuf[5] = hardVision[1];
  
  g_u8HeartBeatCount++;
  if(g_u8HeartBeatCount >= 2)
  {
    g_u8HeartBeatCount= 0;
    stat = AF_DataRequest(&zclIASZoneMotionIR_Coord_nwkAddr, 
                          &zclHeartbeatDesc,
                          ZCL_HEARTBEAT_CLUSTERID,
                          sizeof(SendDataBuf),
                          SendDataBuf,
                          (uint8 *)&zclHeartbeatCounter,
                          0,
                          AF_DEFAULT_RADIUS);
  }
  
  osal_start_timerEx(ZCL_TASK_ID, 
                     ZCL_DEVICE_HEARTBEAT_EVENT,
                     ZCL_HEARTBEAT_PERIOD);
  return stat;
  
}

void zclSendDeviceInfo(void)
{
  switch (gDeviceInfoSendStatus)
  {
  case 0:
    zclActiveEPRsp();
    break;
  case 1:
    zclSimpleDescRsp(ONOFFLIGHT0_ENDPOINT);
    break;
  case 2:
    zclSimpleDescRsp(ONOFFLIGHT1_ENDPOINT);
    abno_led = 0;
    LED_RED_OFF();
    ledcontrol = 1;
    break;
  default:
    gDeviceInfoSendStatus = 0;
    osal_start_timerEx(ZCL_TASK_ID, 
                     ZCL_DEVICE_HEARTBEAT_EVENT,
                     ZCL_HEARTBEAT_PERIOD);
    break;
  }
}

void zclFactoryReset(uint8 DataCleanFlag)
{
  static uint8 hasDoReset = 0;
  if (hasDoReset)
    return;
  
  hasDoReset = 1;
  if(DataCleanFlag)
  {
    zgWriteStartupOptions(ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE);
  }
  SystemResetSoft();
}