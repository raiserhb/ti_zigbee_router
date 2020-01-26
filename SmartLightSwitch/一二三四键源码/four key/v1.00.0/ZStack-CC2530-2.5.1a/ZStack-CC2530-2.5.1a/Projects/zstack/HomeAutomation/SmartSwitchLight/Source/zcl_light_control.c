#include "zcl_light_control.h"
#include "zcl_onofflight.h"
#include "zcl.h"
#include "zcl_device_info.h"
//#include <ioCC2530.h>

extern byte zclOnOffLight_TaskID;
static afAddrType_t zcl_Coord_nwkAddr;
static uint8 zcl_SeqNum = 0;
static uint8 zcl_interruptNo = 0;
void zclFactoryResetInit(void);
void zclFactoryResetEvent(void);
static uint8 zcl_factoryReset = 0;
extern uint8 g_u8restFlag;

/*
input   LED     LIGHT
P1_3 -> P0_7 -> P0_0

p1_2 -> P1_0 -> P0_1
*/

void zclLightControlInit(void)      //LED初始化
{
  //zclFactoryResetInit();

  P1SEL &= ~BV(7); //LED-RED
  P1DIR |= BV(7);
  
  ///init P1.4  P1.5 P1.6and P0.3 for relay control///////////
  P1SEL &= ~BV(4);      //功能：普通I/O口 
  P1DIR |= BV(4);       //方向：输出
  
  P0SEL &= ~BV(3);
  P0DIR |= BV(3); 
  
  P1SEL &= ~BV(5);
  P1DIR |= BV(5);
  
  P1SEL &= ~BV(6);
  P1DIR |= BV(6);
  
  //end init for control
  LED_RED_ON();
  LIGHT_TURN_OFF_LIGHT0();
  LIGHT_TURN_OFF_LIGHT1();
  LIGHT_TURN_OFF_LIGHT2();
  LIGHT_TURN_OFF_LIGHT3(); 

  //设备初始化完成，启动定时器
  osal_start_timerEx(zclOnOffLight_TaskID, 
                     ZCL_DEVICE_INIT_DONE_EVENT,                    
                     500);
  zcl_Coord_nwkAddr.addrMode = afAddr16Bit;       //协调器信息
  zcl_Coord_nwkAddr.addr.shortAddr = 0x0000;
  zcl_Coord_nwkAddr.endPoint = 0xF0;
}
/*
input   LED     LIGHT
P1_3 -> P0_7 -> P0_0

p1_2 -> P1_0 -> P0_1
*/
void zclLightControlTouchPanelEnable(void)
{
  //KEY_1
  P0SEL &= ~BV(4);          //普通I/O口
  P0DIR &= ~BV(4);          //输入
  P0INP &= ~BV(4);          
  P0IEN |= BV(4);           //使能中断
  
  //KEY_2
  P0SEL &= ~BV(5);
  P0DIR &= ~BV(5);
  P0INP &= ~BV(5);
  P0IEN |= BV(5);  
  
  //KEY_3
  P0SEL &= ~BV(6);
  P0DIR &= ~BV(6);
  P0INP &= ~BV(6);
  P0IEN |= BV(6);  
  
  //KEY_4
  P0SEL &= ~BV(7);
  P0DIR &= ~BV(7);
  P0INP &= ~BV(7);
  P0IEN |= BV(7);  
  
  PICTL |= (0x1 << 0);
  IEN1 |= (0x1 << 5);
  
  P0IFG = 0;
  P0IF = 0;
}

ZStatus_t retstatus = 0;
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
       LED_RED_ON();
     }
       
    osal_mem_free( pReportCmd );
  }
}

void zclTouchPanelEvent(void)      //按下开窗帘开，按下关窗帘关，按下暂停停止开/关动作
{
  int done = 0;
  //LED1
  if( zcl_interruptNo & 1 ) {
   if (P0_4 == 0) {
       if (zclOnOffLight0_OnOff == LIGHT_ON) {
        zclOnOffLight0_OnOff = LIGHT_OFF;
        LIGHT_TURN_OFF_LIGHT0();
       } else {
         zclOnOffLight0_OnOff = LIGHT_ON;
         LIGHT_TURN_ON_LIGHT0();
       }
       //if(zclDevice_NwkState == DEV_ROUTER)
         zclLightReportEvent(ONOFFLIGHT0_ENDPOINT, &zclOnOffLight0_OnOff); 
       done = 1;
   }
  }
  //LED2
  if( zcl_interruptNo & 0x20 ) {
    if (P0_5 == 0) {
      if (zclOnOffLight1_OnOff == LIGHT_ON) {
        zclOnOffLight1_OnOff = LIGHT_OFF;
        LIGHT_TURN_OFF_LIGHT1();
      } else {
        zclOnOffLight1_OnOff = LIGHT_ON;
        LIGHT_TURN_ON_LIGHT1();
      }  
      //if(zclDevice_NwkState == DEV_ROUTER)
        zclLightReportEvent(ONOFFLIGHT1_ENDPOINT, &zclOnOffLight1_OnOff);
      
      done = 1;
    }
  }  

  
  //LED3
 if( zcl_interruptNo & 0x40 ) {
    if (P0_6 == 0) {
    if (zclOnOffLight3_OnOff == LIGHT_ON) {
      zclOnOffLight3_OnOff = LIGHT_OFF;
       LIGHT_TURN_OFF_LIGHT3();
    } else {
      zclOnOffLight3_OnOff = LIGHT_ON;
      LIGHT_TURN_ON_LIGHT3();
    }
    //if(zclDevice_NwkState == DEV_ROUTER)
      zclLightReportEvent(ONOFFLIGHT3_ENDPOINT, &zclOnOffLight3_OnOff);
    done = 1;
    }
 }    
 // LED4
  if( zcl_interruptNo & 0x80 ) {
     if (P0_7 == 0) {
       if (zclOnOffLight2_OnOff == LIGHT_ON) {
        zclOnOffLight2_OnOff = LIGHT_OFF;
         LIGHT_TURN_OFF_LIGHT2();
      } else {
        zclOnOffLight2_OnOff = LIGHT_ON;
        LIGHT_TURN_ON_LIGHT2();
      }  
      //if(zclDevice_NwkState == DEV_ROUTER)
        zclLightReportEvent(ONOFFLIGHT2_ENDPOINT, &zclOnOffLight2_OnOff);
       done = 1;
     }
  }  
  
  zcl_interruptNo = 0;
  if(done) 
  {
    osal_start_timerEx(zclOnOffLight_TaskID, 
                       ZCL_DEVICE_RESTORE_INTERRUPT_EVENT,
                       80);
  } 
}

void zclRestoreInterruptEvent(void)
{
  //if(P0_4 = 0 || P0_4 = 1)    //单独在按下或者没按下的时候，存在延时触摸不灵敏
  //{
  //  P0IFG = 0;
  //  P0IF = 0; 
  //  P0IEN |= BV(4);
  //}
  
  P0IFG =0;         //每次中断之后都要重新允许中断
  P0IF = 0;
  P0IEN |= BV(4);
  //P0DIR &= ~BV(4);
  P0IEN |= BV(5);
  //P0DIR &= ~BV(5);
  P0IEN |= BV(6);
  //P0DIR &= ~BV(6);
  P0IEN |= BV(7);
  //P0DIR &= ~BV(7);
}

void zclFactoryResetInit(void)
{
  P2SEL &= ~BV(0);
  P2DIR &= ~BV(0);
  P2INP &= ~BV(7);
  P2INP &= ~BV(0);
  P2IEN |= BV(0);
  
  PICTL |= (0x1 << 3);
  IEN2 |= (0x1 << 1);
  P2IFG = 0;
  P2IF = 0;
}

void zclFactoryResetEvent(void)
{
    if (zcl_factoryReset > ZCL_FACTORY_RESET_TIMECOUNT) 
     {
       
         osal_start_timerEx(zclOnOffLight_TaskID, 
                            ZCL_DEVICE_SOFT_RESTART_EVENT,
                            ZCL_FACTORY_RESET_HINT);
     }
     
  else 
  {
    g_u8restFlag = 0;
    zcl_factoryReset = 0;
    P0IFG = 0;
    P0IF = 0;
    P0IEN |= BV(4);
    P0IEN |= BV(5);
    P0IEN |= BV(6);
    P0IEN |= BV(7);
    
  }
}  
//中断,所有外部中断入口
HAL_ISR_FUNCTION(zclLightControlPort0Isr, P0INT_VECTOR)
{
  HAL_ENTER_ISR();
  
  zcl_interruptNo = 0;
  if (P0IFG & 0x10) {     //判断相应管脚的中断标志位
    zcl_interruptNo |= 1;
    P0IEN &= ~BV(4);
    zcl_factoryReset ++; 
  }
  if (P0IFG & 0x20) {
    P0IEN &= ~BV(5);
    zcl_interruptNo |= BV(5);
    zcl_factoryReset = 0;
  }
  if (P0IFG & 0x40) {
    P0IEN &= ~BV(6);
    zcl_interruptNo |= BV(6);
    zcl_factoryReset = 0;
  }
  if (P0IFG & 0x80) {
    P0IEN &= ~BV(7);
    zcl_interruptNo |= BV(7);
    zcl_factoryReset = 0;
  }
  
  if( zcl_interruptNo ) {
    osal_start_timerEx(zclOnOffLight_TaskID, 
                       ZCL_DEVICE_TOUCHPANEL_EVENT,
                       20);
  }
  
  P0IFG = 0;
  P0IF = 0;
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}

