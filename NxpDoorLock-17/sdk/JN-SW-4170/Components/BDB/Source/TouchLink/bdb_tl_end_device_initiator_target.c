/*###############################################################################
#
# MODULE:      BDB
#
# COMPONENT:   bdb_tl_end_device_initiator_target.c
#
# AUTHOR:      
#
# DESCRIPTION: BDB Touchlink implementation of End Device as Initiator/Target
#              
#
# $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/Components/BDB/Trunk/Source/TouchLink/bdb_tl_end_device_initiator_target.c $
#
# $Revision: 78033 $
#
# $LastChangedBy: nxp29772 $
#
# $LastChangedDate: 2016-03-22 09:56:12 +0000 (Tue, 22 Mar 2016) $
#
# $Id: bdb_tl_end_device_initiator_target.c 78033 2016-03-22 09:56:12Z nxp29772 $
#
###############################################################################
#
# This software is owned by NXP B.V. and/or its supplier and is protected
# under applicable copyright laws. All rights are reserved. We grant You,
# and any third parties, a license to use this software solely and
# exclusively on NXP products [NXP Microcontrollers such as JN514x, JN516x, JN517x].
# You, and any third parties must reproduce the copyright and warranty notice 
# and any other legend of ownership on each  copy or partial copy of the software.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE. 
# 
# Copyright NXP B.V. 2015-2016. All rights reserved
#
###############################################################################*/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "jendefs.h"
#include "bdb_api.h"
#include "bdb_tl.h"

#include <string.h>
#include <stdlib.h>
#include <appapi.h>
#include "pdum_apl.h"
#include "pdum_gen.h"
#include "pdm.h"
#include "dbg.h"
#include "pwrm.h"
#include "zps_apl_af.h"
#include "zps_apl_zdo.h"
#include "zps_apl_aib.h"
#include "zps_apl_zdp.h"
#include "zll_commission.h"



#include "app_main.h"

//#include "app_led_control.h"
//#include "zlo_controller_node.h"
#include "app_zlo_sensor_node.h"

#include "app_events.h"
#include "zcl_customcommand.h"
#include "mac_sap.h"
#include "ZTimer.h"
#include <rnd_pub.h>
#include <mac_pib.h>
#include <string.h>
#include <stdlib.h>

// todo deferal rules for 2 initiators


#include "PDM_IDs.h"

#ifndef DEBUG_COMMISSION
#define TRACE_COMMISSION		FALSE
#define TRACE_JOIN				FALSE
#define TRACE_SCAN				FALSE
#define TRACE_SCAN_REQ			FALSE
#define TRACE_TL_NEGATIVE		FALSE
#else
#define TRACE_COMMISSION		TRUE
#define TRACE_SCAN				TRUE
#define TRACE_JOIN				TRUE
#define TRACE_SCAN_REQ			TRUE
#define TRACE_TL_NEGATIVE		TRUE
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define ADJUST_POWER        TRUE
#define TL_SCAN_LQI_MIN    (110)
#define RSSI_CORRECTION    (10)

#define FACTORY_NEW_REMOTE		FACTORY_NEW
#define NOT_FACTORY_NEW_REMOTE	NOT_FACTORY_NEW

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum {
    E_IDLE,
    E_INFORM_APP,
    E_SCANNING,
    E_SCAN_DONE,
    E_SCAN_WAIT_ID,
    E_SCAN_WAIT_INFO,
    E_SCAN_WAIT_RESET_SENT,
    E_WAIT_START_RSP,
    E_WAIT_JOIN_RTR_RSP,
    E_WAIT_JOIN_ZED_RSP,
    E_WAIT_LEAVE,
    E_WAIT_LEAVE_RESET,
    E_WAIT_START_UP,
    E_ACTIVE
}eState;

typedef struct {
    eState eState;
    uint8 u8Count;
    uint8 u8Flags;
    bool_t bResponded;
    uint32 u32TransactionId;
    uint32 u32ResponseId;
    uint32 u32TheirTransactionId;
    uint32 u32TheirResponseId;
    bool_t bDoPrimaryScan;
    bool_t bIsFirstChannel;
    uint32 u32ScanChannels;
    bool_t bProfileInterOp;
} tsCommissionData;

typedef struct {
    ZPS_tsInterPanAddress       sSrcAddr;
    tsCLD_ZllCommission_ScanRspCommandPayload                   sScanRspPayload;
}tsZllScanData;

typedef struct {
     tsZllScanData sScanDetails;
    uint16 u16LQI;
}tsZllScanTarget;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PUBLIC void ZllCommissonCommandScanSend(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern tsZllState sZllState;

//extern tsZllRemoteState sZllState;
extern tsCLD_ZllDeviceTable sDeviceTable;
extern bool_t bFailedToJoin;

//APP_tsEventTouchLink        sTarget;
ZPS_tsInterPanAddress       sDstAddr;

uint64 au64IgnoreList[3];
tsZllScanTarget sScanTarget;

uint8 au8TempKeyStore[16];
PUBLIC uint8 u8TimerBdbTl;

uint8 TouchLinkRSSI = 0;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
tsCommissionData sCommission;

/* to hold address ranges over touchlink in case they need recovering */
uint16 u16TempAddrLow;
uint16 u16TempAddrHigh;
uint16 u16TempGroupLow;
uint16 u16TempGroupHigh;
bool bDoRejoin = FALSE;
bool bWithDiscovery = FALSE;
//PUBLIC bool_t bTLinkInProgress = FALSE;
PUBLIC bool_t bSendFactoryResetOverAir=FALSE;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
extern void vRemoveLight(uint16 u16Addr);;
/****************************************************************************
 *
 * NAME: 
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 *
 *
 * RETURNS:
 *
 *
 * NOTES:
 *
 ****************************************************************************/
PUBLIC void BDB_vTlInit(void)
{
    sCommission.eState = E_IDLE;
	sCommission.u32ScanChannels = (1<<13);
}

/****************************************************************************
 *
 * NAME: 
 *
 * DESCRIPTION: 
 *
 *
 * RETURNS: void
 * void
 *
 ****************************************************************************/
PUBLIC bool BDB_bTlTouchLinkInProgress( void)
{
    return (sCommission.eState > E_INFORM_APP) ? TRUE: FALSE;
}

/****************************************************************************
 *
 * NAME: 
 *
 * DESCRIPTION:
 *
 *
 * RETURNS: void
 * void
 *
 ****************************************************************************/
PUBLIC bool bIsTlStarted(void)
{
    return (sCommission.eState > E_INFORM_APP) ? TRUE: FALSE;
}

/****************************************************************************
 *
 * NAME:
 *
 * DESCRIPTION:
 * 
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
#define RCVD_DEST_ADDR ( ((tsCLD_ZllCommissionCustomDataStructure*)psEvent-> psCallBackEvent->psClusterInstance->pvEndPointCustomStructPtr)->sRxInterPanAddr.sDstAddr )

PUBLIC void BDB_vTlStateMachine( tsBDB_ZCLEvent *psEvent)
{}
/****************************************************************************
 *
 * NAME: BDB_vTlTimerCb
 *
 * DESCRIPTION: handles commissioning timer expiry events
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void BDB_vTlTimerCb(void *pvParam)
{
    tsBDB_ZCLEvent sEvent;
    sEvent.eType = BDB_E_ZCL_EVENT_TL_TIMER_EXPIRED;
    BDB_vTlStateMachine( &sEvent);
	
//	ZTIMER_eStart(u8TimerBdbTl, ZTIMER_TIME_MSEC(10));
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

PUBLIC void ZllCommissonCommandScanSend(void)
{
	uint8 u8Seq = 1;	
    tsCLD_ZllCommission_ScanRspCommandPayload	sScanRsp;

	sDstAddr.eMode = 2;
	sDstAddr.u16PanId = 0xFFFF;
	sDstAddr.uAddress.u16Addr = 0x0101;
	sDstAddr.uAddress.u64Addr = 0xFFFFFFFFFFFFFFFF;

	sScanRsp.u32TransactionId = 0;
	sScanRsp.u8RSSICorrection = 0x00;
	sScanRsp.u8ZigbeeInfo = 0x05;
	sScanRsp.u8ZllInfo = 0x00;
	sScanRsp.u16KeyMask = 0x0001;
	sScanRsp.u32ResponseId = 0x04030201;
	sScanRsp.u64ExtPanId = ZPS_u64AplZdoGetIeeeAddr();
	sScanRsp.u8NwkUpdateId = 0x00;
	sScanRsp.u8LogicalChannel = 0x00;
	sScanRsp.u16PanId  = 0x0101;
	sScanRsp.u16NwkAddr = 0x0101;
	sScanRsp.u8NumberSubDevices = 1;
	sScanRsp.u8TotalGroupIds = 0;
	sScanRsp.u8Endpoint = 0x10;
	sScanRsp.u16ProfileId = 0xC05E;
	sScanRsp.u16DeviceId = 0x0000;
	sScanRsp.u8Version = 0x02;
	sScanRsp.u8GroupIdCount = 0;


    DBG_vPrintf(TRACE_JOIN, "\n ZllCommissonCommandScanSend %d \n",
				eCLD_ZllCommissionCommandScanRspCommandSend_Feibit(&sDstAddr,&u8Seq,&sScanRsp) );

}

