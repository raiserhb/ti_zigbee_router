#include "zcl_light_control.h"
#include "zcl_onofflight.h"
#include "zcl.h"
#include "zcl_device_info.h"

extern byte zclOnOffLight_TaskID;
static afAddrType_t zcl_Coord_nwkAddr;
static uint8 zcl_SeqNum = 0;
static uint8 zcl_interruptNo = 0;
void zclFactoryResetEvent(void);
static uint8 zcl_factoryReset = 0;
extern uint8 g_u8restFlag;
extern uint8 zcl_Led_Blink;
extern uint8 ReJoinNetFlagInFlash[1];
extern devStates_t zclDevice_NwkState;

/*********************************************************************
 * @fn      zclLightControlInit
 *
 * @brief   配置指示灯p0.5 配置输出IO口p0.2 p0.3 p0.4
 *          
 * @param   none
 *
 * @return  none
 */
void zclLightControlInit(void)
{

  P0SEL &= ~BV(5); //LED1
  P0DIR |= BV(5);

 ///init P1.4 and P1.5 for relay control///////////
  P0SEL &= ~BV(2);
  P0DIR |= BV(2);
  P0SEL &= ~BV(3);
  P0DIR |= BV(3); 
  P0SEL &= ~BV(4);
  P0DIR |= BV(4); 
  
  // default to turn off the light
  LIGHT_TURN_OFF_LIGHT0();
  LIGHT_TURN_OFF_LIGHT1(); 
  LIGHT_TURN_OFF_LIGHT2(); 
  ///end init for relay control////////

  
  osal_start_timerEx(zclOnOffLight_TaskID, 
                         ZCL_DEVICE_INIT_DONE_EVENT,
                         1000);
  
  zcl_Coord_nwkAddr.addrMode = afAddr16Bit;
  zcl_Coord_nwkAddr.addr.shortAddr = 0x0000;
  zcl_Coord_nwkAddr.endPoint = 0xF0;
}
/*********************************************************************
 * @fn      zclLightControlTouchPanelEnable
 *
 * @brief   配置按键p0.0 p0.1 p1.0  配置复位按键p0.7
 *          
 * @param   none
 *
 * @return  none
 */
void zclLightControlTouchPanelEnable(void)
{
  
  P0SEL &= ~BV(0);
  P0DIR &= ~BV(0);
  P0IEN |= BV(0);
  
  P0SEL &= ~BV(1);
  P0DIR &= ~BV(1);
  P0IEN |= BV(1);  
  
  P0SEL &= ~BV(7);
  P0DIR &= ~BV(7);
  P0IEN |= BV(7);
  
  P1SEL &= ~BV(0);
  P1DIR &= ~BV(0);
  P1INP &= ~BV(0);
  P1IEN |= BV(0); 
  
  PICTL |= (0x1 << 0);
  IEN1 |= (0x1 << 5);
  PICTL |= (0x1 << 1);
  IEN2 |= (0x1 << 4);
  P0IFG = 0;
  P0IF = 0;
  P1IFG = 0;
  P1IF = 0;
}
/*********************************************************************
 * @fn      zclLightReportEvent
 *
 * @brief   状态上报
 *          
 * @param   endpoint - 端点
 *          zclOnOffLight_OnOff - 当前状态
 *
 * @return  none
 */
void zclLightReportEvent(uint16 endpoint, uint8 *zclOnOffLight_OnOff)
{
  zclReportCmd_t *pReportCmd;

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_ON_OFF;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT8;
    pReportCmd->attrList[0].attrData = (void *)(zclOnOffLight_OnOff);
    
    if (afStatus_SUCCESS != zcl_SendReportCmd( endpoint, 
                                              &zcl_Coord_nwkAddr,
                                              ZCL_CLUSTER_ID_GEN_ON_OFF,
                                              pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, 
                                              zcl_SeqNum++ ))
    {
      zcl_SendReportCmd( endpoint, 
                        &zcl_Coord_nwkAddr,
                        ZCL_CLUSTER_ID_GEN_ON_OFF,
                        pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, 
                        zcl_SeqNum++ );
    }
    osal_mem_free( pReportCmd );
  }
}
/*********************************************************************
 * @fn      zclTouchPanelEvent
 *
 * @brief   按键开关
 *          
 * @param   none
 *
 * @return  none
 */
void zclTouchPanelEvent(void)
{
  int done = 0;
  if( zcl_interruptNo & 1 ) {
    if (P0_0 == 0) {
      if (zclOnOffLight0_OnOff == LIGHT_ON) {
        zclOnOffLight0_OnOff = LIGHT_OFF;
        LIGHT_TURN_OFF_LIGHT0();
      } else {
        zclOnOffLight0_OnOff = LIGHT_ON;
        LIGHT_TURN_ON_LIGHT0();
      }
      if (zclDevice_NwkState == DEV_ROUTER )
        zclLightReportEvent(ONOFFLIGHT0_ENDPOINT, &zclOnOffLight0_OnOff);
      //done = 1;
    }
    done = 1;
  }
  if( zcl_interruptNo & 2 ) {
    if (P0_1 == 0) {
      if (zclOnOffLight1_OnOff == LIGHT_ON) {
        zclOnOffLight1_OnOff = LIGHT_OFF;
        LIGHT_TURN_OFF_LIGHT1();
      } else {
        zclOnOffLight1_OnOff = LIGHT_ON;
        LIGHT_TURN_ON_LIGHT1();
      }  
      if (zclDevice_NwkState == DEV_ROUTER )
        zclLightReportEvent(ONOFFLIGHT1_ENDPOINT, &zclOnOffLight1_OnOff);
      //done = 1;
    }
    done = 1;
  }  
  
  if( zcl_interruptNo & 4 ) {
    if (P1_0 == 0) {
      if (zclOnOffLight2_OnOff == LIGHT_ON) {
        zclOnOffLight2_OnOff = LIGHT_OFF;
        LIGHT_TURN_OFF_LIGHT2();
      } else {
        zclOnOffLight2_OnOff = LIGHT_ON;
        LIGHT_TURN_ON_LIGHT2();
      }
      if (zclDevice_NwkState == DEV_ROUTER )
        zclLightReportEvent(ONOFFLIGHT2_ENDPOINT, &zclOnOffLight2_OnOff);
      //done = 1;
    }
    done = 1;
  }
  
  
  if(done) 
  {
    zcl_interruptNo = 0;
    osal_start_timerEx(zclOnOffLight_TaskID, 
                       ZCL_DEVICE_RESTORE_INTERRUPT_EVENT,
                       80);
  }
}
/*********************************************************************
 * @fn      zclRestoreInterruptEvent
 *
 * @brief   开中断
 *          
 * @param   none
 *
 * @return  none
 */
void zclRestoreInterruptEvent(void)
{
  
  P0IFG =0;
  P0IF = 0;
  P1IFG =0;
  P1IF = 0;
  P0IEN |= BV(0);
  P0IEN |= BV(1);
  P0IEN |= BV(7);
  P1IEN |= BV(0);
  
}
/*********************************************************************
 * @fn      zclFactoryResetEvent
 *
 * @brief   长按<6s指示灯常亮松开重启，长按>=6s复位指示灯快闪
 *          
 * @param   none
 *
 * @return  none
 */
void zclFactoryResetEvent(void)
{
  if( P0_7 == 0){
    if (zcl_factoryReset >= ZCL_FACTORY_RESET_TIMECOUNT)
    {
      if(zcl_Led_Blink == 0)
      {
        ReJoinNetFlagInFlash[0] = 0x01;//表示设备复位重启
        osal_nv_write(ZDAPP_NV_SYSTEM_RESTART_FLAG,0,sizeof(ReJoinNetFlagInFlash),ReJoinNetFlagInFlash);
        osal_start_timerEx(zclOnOffLight_TaskID, 
                           ZCL_DEVICE_SOFT_RESTART_EVENT,
                           ZCL_FACTORY_RESET_HINT);
        zcl_factoryReset=0;
      }
    }else{
      LED_RED_ON();
      ledcontrol=1;
      zcl_factoryReset++;
      osal_start_timerEx(zclOnOffLight_TaskID, 
                         ZCL_DEVICE_RESET_EVENT,
                         300);
    }
  }
  else 
  {
    if(( zcl_factoryReset < 20 ) && ( P0_7 == 1 )){
      LED_RED_OFF();
      P0IEN |= BV(7);
      zcl_factoryReset=0;
      SystemResetSoft();
    }
    LED_RED_OFF();
    P0IEN |= BV(7);
    zcl_factoryReset=0;
    if (zclDevice_NwkState != DEV_ROUTER ){
      ledcontrol=0;
      osal_start_timerEx(zclOnOffLight_TaskID, 
                         ZCL_DEVICE_EXIT_NETWORK_EVENT,
                         2000);
    }
  }
}

HAL_ISR_FUNCTION(zclLightControlPort0Isr, P0INT_VECTOR)
{
  HAL_ENTER_ISR();
  zcl_interruptNo=0;
  if (P0IFG & 0x01) {//p0_0
    P0IEN &= ~BV(0);
    zcl_interruptNo |= 1;
  }
  if (P0IFG & 0x02) {//p0_1
    P0IEN &= ~BV(1);
    zcl_interruptNo |= 2;
  }
  
  if (P0IFG & 0x80) {//p0_7
    if(zcl_interruptNo == 0)
    {
      P0IEN &= ~BV(7);
      zcl_factoryReset = 0;
      osal_start_timerEx(zclOnOffLight_TaskID, 
                         ZCL_DEVICE_RESET_1_EVENT,
                         100);
    }
  }
  
  if( zcl_interruptNo ) {
    osal_start_timerEx(zclOnOffLight_TaskID, 
                       ZCL_DEVICE_TOUCHPANEL_EVENT,
                       100);
  }
  
  P0IFG = 0;
  P0IF = 0;
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}
HAL_ISR_FUNCTION(zclLightControlPort1Isr, P1INT_VECTOR)
{
  HAL_ENTER_ISR();
  if (P1IFG & 0x01) {//p1_0
    P1IEN &= ~BV(0);
    zcl_interruptNo |= 4;
  }
 
  if( zcl_interruptNo ) {
    osal_start_timerEx(zclOnOffLight_TaskID, 
                       ZCL_DEVICE_TOUCHPANEL_EVENT,
                       100);
  }
  
  P1IFG = 0;
  P1IF = 0;
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}

