/*###############################################################################
#
# MODULE:      BDB
#
# COMPONENT:   bdb_start.h
#
# AUTHOR:
#
# DESCRIPTION: BDB Network Initialisation API
#
#
# $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/Components/BDB/Trunk/Source/Common/bdb_start.h $
#
# $Revision: 77488 $
#
# $LastChangedBy: nxp46755 $
#
# $LastChangedDate: 2016-03-02 09:31:31 +0000 (Wed, 02 Mar 2016) $
#
# $Id: bdb_start.h 77488 2016-03-02 09:31:31Z nxp46755 $
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

#ifndef BDB_START_INCLUDED
#define BDB_START_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "bdb_api.h"
//#include "app_global_data.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum
{
    E_INIT_IDLE,
    E_INIT_WAIT_REJOIN,
}teInitState;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void BDB_vInit(BDB_tsInitArgs *psInitArgs);
PUBLIC void BDB_vSetKeys(void);
PUBLIC bool_t BDB_bIsBaseIdle(void);
PUBLIC void BDB_vStart(void);
PUBLIC void BDB_vInitStateMachine(BDB_tsZpsAfEvent *psZpsAfEvent);
PUBLIC uint8 BDB_u8PickChannel(uint32 u32ChannelMask);
PUBLIC void BDB_vRejoinCycle(bool_t bSkipDirectJoin);
PUBLIC void BDB_vRejoinSuccess(void);
PUBLIC void BDB_vRejoinTimerCb(void *pvParam);

PUBLIC void vBDB_SaveLogicalChannelFromBeacon(uint8 u8LogicalChannel);
PUBLIC void vBDB_ReadRecordLogicalChannelFromPDM(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern PUBLIC uint8 au8DefaultTCLinkKey[16];
extern PUBLIC uint8 au8DistributedLinkKey[16];
extern PUBLIC uint8 au8PreConfgLinkKey[16];
extern PUBLIC uint8 au8TouchLinkKey[16];

extern PUBLIC teInitState eInitState;
extern PUBLIC bool_t bAssociationJoin;
extern PUBLIC uint8 u8RejoinCycles;
extern PUBLIC uint8 u8TimerBdbRejoin;
extern PUBLIC uint32 u32BdbLogicalChannel;

#if defined __cplusplus
}
#endif

#endif  /* BDB_START_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/






