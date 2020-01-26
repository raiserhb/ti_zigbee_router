#ifndef __ZCL_LIGHT_CONTROL_H__
#define __ZCL_LIGHT_CONTROL_H__
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"

#include "zcl_onofflight.h"

#include "onboard.h"


//
//#define LIGHT_TURN_OFF_LIGHT0()       st( P0_2 = 0; P1_2 = 1; )
//#define LIGHT_TURN_ON_LIGHT0()        st( P0_2 = 1; P1_2 = 0; )
//
//#define LIGHT_TURN_OFF_LIGHT1()       st( P0_3 = 0; P1_0 = 1;)
//#define LIGHT_TURN_ON_LIGHT1()        st( P0_3 = 1; P1_0 = 0;)


#define LIGHT_TURN_OFF_LIGHT0()       st(P0_2 = 0;)
#define LIGHT_TURN_ON_LIGHT0()        st(P0_2 = 1;)

#define LED_RED_OFF() st (P0_5 = 0;)
#define LED_RED_ON()  st (P0_5 = 1;)

extern void zclLightControlInit(void);
extern void zclLightControlTouchPanelEnable(void);
extern void zclTouchPanelEvent(void);
extern void zclLightReportEvent(uint16 endpoint, uint8 *zclOnOffLight_OnOff);
extern void zclRestoreInterruptEvent(void);
extern void zclFactoryResetEvent(void);

#endif