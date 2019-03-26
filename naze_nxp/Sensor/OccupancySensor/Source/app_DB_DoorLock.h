
#ifndef _APP_DB_DOORLOCK_H_
#define _APP_DB_DOORLOCK_H_


/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "App_OccupancySensor.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
extern PUBLIC uint16 u16BasicAttrID;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
extern PUBLIC void APP_cbTimerZCLBasicWriteAttr(void);
PRIVATE void App_BasicWriteAttributeHandle(uint8 BasicCmd,uint8 Length);
PRIVATE void App_DB_ConfigHeartPeriod(uint8 Length);
PRIVATE void App_DB_ConfigCacheNumber(uint8 u8TransportCnt);
PUBLIC void APP_cbTimerSpecialHeartBeat(void);



/****************************************************************************/
/***        END OF FILE                                            ***/
/****************************************************************************/
#endif /*_APP_DB_DOORLOCK_H_*/


