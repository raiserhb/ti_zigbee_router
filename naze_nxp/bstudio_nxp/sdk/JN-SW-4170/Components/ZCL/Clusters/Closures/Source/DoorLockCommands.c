/*****************************************************************************
 *
 * MODULE:             Door Lock Cluster
 *
 * COMPONENT:          DoorLockClientCommands.c
 *
 * AUTHOR:             Shweta Chauhan
 *
 * DESCRIPTION:        Send a door lock cluster command
 *
 * $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/SmartEnergy/Branches/HA1_x_1v0/ZCL/Clusters/General/Source/DoorLockCommands.c $
 *
 * $Revision: 53112 $
 *
 * $LastChangedBy: nxp57621 $
 *
 * $LastChangedDate: 2013-04-08 11:17:46 +0530 (Mon, 08 Apr 2013) $
 *
 * $Id: DoorLockCommands.c 53112 2013-04-08 05:47:46Z nxp57621 $
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139]. 
 * You, and any third parties must reproduce the copyright and warranty notice 
 * and any other legend of ownership on each  copy or partial copy of the software.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  
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
 * Copyright NXP B.V. 2012. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

#include "zcl.h"
#include "zcl_customcommand.h"

#include "DoorLock.h"
#include "DoorLock_internal.h"

#include "pdum_apl.h"
#include "zps_apl.h"
#include "zps_apl_af.h"


#include "dbg.h"

#ifdef DEBUG_CLD_DOOR_LOCK
#define TRACE_DOOR_LOCK    TRUE
#else
#define TRACE_DOOR_LOCK    FALSE
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Public Functions                                              ***/
/****************************************************************************/
#ifdef DOOR_LOCK_SERVER
/****************************************************************************
 **
 ** NAME:       eCLD_DoorLockCommandLockUnlockResponseSend
 **
 ** DESCRIPTION:
 ** Builds and sends a lock/unlock door response command
 **
 ** PARAMETERS:                 Name                           Usage
 ** uint8                       u8SourceEndPointId             Source EP Id
 ** uint8                       u8DestinationEndPointId        Destination EP Id
 ** tsZCL_Address              *psDestinationAddress           Destination Address
 ** uint8                      *pu8TransactionSequenceNumber   Sequence number Pointer
 ** tsCLD_DoorLock_LockUnlockResponsePayload  *psPayload             Payload
 **
 ** RETURN:
 ** teZCL_Status
 **
 ****************************************************************************/
PUBLIC  teZCL_Status  eCLD_DoorLockCommandLockUnlockResponseSend(
                    uint8                       u8SourceEndPointId,
                    uint8                       u8DestinationEndPointId,
                    tsZCL_Address               *psDestinationAddress,
                    uint8                       *pu8TransactionSequenceNumber,
                    teCLD_DoorLock_CommandID     eCommand,
                    tsCLD_DoorLock_LockUnlockResponsePayload *psPayload)
{

        tsZCL_TxPayloadItem asPayloadDefinition[] = {
        {1,                                     E_ZCL_ENUM8,     &psPayload->eStatus},
                                              };

    return eZCL_CustomCommandSend(u8SourceEndPointId,
                                  u8DestinationEndPointId,
                                  psDestinationAddress,
                                  CLOSURE_CLUSTER_ID_DOOR_LOCK,
                                  TRUE,
                                  (uint8)eCommand,
                                  pu8TransactionSequenceNumber,
                                  asPayloadDefinition,
                                  FALSE,
                                  0,
                                  sizeof(asPayloadDefinition) / sizeof(tsZCL_TxPayloadItem));

}
/*********************************************************************
 * @fn      zclClosures_SendDoorLockOperationEventNotification
 *
 * @brief   Call to send out a Operation Event Notification
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   pPayload:
 *           operationEventSource - Indicates where the event was triggered from
 *           operationEventCode - (Optional) a notification whenever there is a significant operation event on the lock
 *           userID - User ID is between 0 - [# PINs User supported attribute]
 *           pin - The PIN that is associated with the User ID who performed the event
 *           zigBeeLocalTime - Indicates when the event is triggered
 *           aData - Used to pass data associated with a particular event
 * @param   disableDefaultRsp - decides default response is necessary or not
 * @param   seqNum - sequence number of the command packet
 *
 * @return  ZStatus_t
 */
PUBLIC  teZCL_Status   eCLD_DoorLockCommandDoorLockOperationEventNotification( 
																uint8 srcEP, tsZCL_Address *dstAddr,
																uint8 u8DestinationEndPointId,
																zclDoorLockOperationEventNotification_t *pPayload)
{

	uint8 i;
	uint8 pBuf[16];  // variable length payload
	uint8 offset;
	uint8 calculatedArrayLen;
	uint8 calculatedBufSize;
	tsZCL_CharacterString spBufPayload;
	uint8 u8Seq = u8GetTransactionSequenceNumber();

//	set variable length if data is available
	calculatedArrayLen = pPayload->pData[0] + 1; // add first byte of string

//	determine total size of buffer
	calculatedBufSize = calculatedArrayLen + 9;//PAYLOAD_LEN_OPERATION_EVENT_NOTIFICATION;

  // over-the-air is always little endian. Break into a byte stream.
//	pBuf[0] = calculatedBufSize;
	pBuf[0] = pPayload->operationEventSource;
	pBuf[1] = pPayload->operationEventCode;
	pBuf[2] = U16_LOWER_U8( pPayload->userID );
	pBuf[3] = U16_UPPER_U8( pPayload->userID );
	pBuf[4] = pPayload->pin;
	pBuf[5] = U32_LOWEST_U8(pPayload->zigBeeLocalTime);
	pBuf[6] = U32_LOW_U8(pPayload->zigBeeLocalTime);
	pBuf[7] = U32_HIGH_U8(pPayload->zigBeeLocalTime);
	pBuf[8] = U32_HIGHEST_U8(pPayload->zigBeeLocalTime);

	offset = 9;//PAYLOAD_LEN_OPERATION_EVENT_NOTIFICATION;

	for ( i = 0; (i < calculatedArrayLen) ; i++ )
	{
		pBuf[offset++] = pPayload->pData[i];
	}

	spBufPayload.u8MaxLength = 15;
	spBufPayload.u8Length = calculatedBufSize;
	spBufPayload.pu8Data = pBuf;

	tsZCL_TxPayloadItem asPayloadDefinition[] = { {1, E_ZCL_CSTRING, &spBufPayload},};

	i = 0;
	DBG_vPrintf(TRACE_DOOR_LOCK, "\n pBuf Payload ");
	while (i < 12)
	{
		DBG_vPrintf(TRACE_DOOR_LOCK, " %0x ",pBuf[i]);
		i++;
	}

	tsZCL_Address sDestinationAddress;

	/* Set the address mode to send to all bound device and don't wait for an ACK*/
	sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;
	i = eZCL_CustomCommandSend(srcEP,
								u8DestinationEndPointId,
								&sDestinationAddress,
								CLOSURE_CLUSTER_ID_DOOR_LOCK,
								TRUE,
								COMMAND_CLOSURES_OPERATION_EVENT_NOTIFICATION,
								&u8Seq,
								asPayloadDefinition,
								FALSE,
								0,
								sizeof(asPayloadDefinition) / sizeof(tsZCL_TxPayloadItem));
	DBG_vPrintf(TRACE_DOOR_LOCK, "\n eCLD_DoorLockCommandDoorLockOperationEventNotification5 %d ",i);

	return i;
  
}

/****************************************************************************
 **
 ** NAME:       eCLD_DoorLockCommandLockUnlockRequestReceive
 **
 ** DESCRIPTION:
 ** handles rx of a lock/unlock request command
 **
 ** PARAMETERS:               Name                          Usage
 ** ZPS_tsAfEvent            *pZPSevent                   Zigbee stack event structure
 ** tsZCL_EndPointDefinition *psEndPointDefinition          EP structure
 ** tsZCL_ClusterInstance    *psClusterInstance             Cluster structure
 ** uint8                    *pu8TransactionSequenceNumber  Sequence number Pointer
 **
 ** RETURN:
 ** teZCL_Status
 **	// Ricky Mark
 ****************************************************************************/
PUBLIC  teZCL_Status eCLD_DoorLockCommandLockUnlockRequestReceive(
                    ZPS_tsAfEvent               *pZPSevent,
                    uint8                       *pu8TransactionSequenceNumber)
{

    return eZCL_CustomCommandReceive(pZPSevent,
                                     pu8TransactionSequenceNumber,
                                     0,
                                     0,
                                     E_ZCL_ACCEPT_EXACT | E_ZCL_DISABLE_DEFAULT_RESPONSE);

}
#endif

#ifdef DOOR_LOCK_CLIENT
/****************************************************************************
 **
 ** NAME:       eCLD_DoorLockCommandLockUnlockRequestSend
 **
 ** DESCRIPTION:
 ** Builds and sends a Lock Request command
 **
 ** PARAMETERS:                 Name                           Usage
 ** uint8                       u8SourceEndPointId             Source EP Id
 ** uint8                       u8DestinationEndPointId        Destination EP Id
 ** tsZCL_Address              *psDestinationAddress           Destination Address
 ** uint8                      *pu8TransactionSequenceNumber   Sequence number Pointer
 **
 ** RETURN:
 ** teZCL_Status
 **
 ****************************************************************************/
PUBLIC  teZCL_Status eCLD_DoorLockCommandLockUnlockRequestSend(
                                        uint8              u8SourceEndPointId,
                                        uint8           u8DestinationEndPointId,
                                        tsZCL_Address   *psDestinationAddress,
                                        uint8           *pu8TransactionSequenceNumber,
                                        teCLD_DoorLock_CommandID      eCommand)
{


    return eZCL_CustomCommandSend(u8SourceEndPointId,
                                  u8DestinationEndPointId,
                                  psDestinationAddress,
                                  CLOSURE_CLUSTER_ID_DOOR_LOCK,
                                  FALSE,
                                  (uint8)eCommand,
                                  pu8TransactionSequenceNumber,
                                  0,
                                  FALSE,
                                  0,
                                  0);

}


/****************************************************************************
 **
 ** NAME:       eCLD_DoorLockCommandLockUnlockResponseReceive
 **
 ** DESCRIPTION:
 ** handles rx of Lock/Unlock command Response
 **
 ** PARAMETERS:               Name                          Usage
 ** ZPS_tsAfEvent            *pZPSevent                   Zigbee stack event structure
 ** tsZCL_EndPointDefinition *psEndPointDefinition          EP structure
 ** tsZCL_ClusterInstance    *psClusterInstance             Cluster structure
 ** uint8                    *pu8TransactionSequenceNumber  Sequence number Pointer
 ** tsCLD_DoorLock_LockResponsePayload  *psPayload          Payload
 ** RETURN:
 ** teZCL_Status
 **
 ****************************************************************************/
PUBLIC  teZCL_Status eCLD_DoorLockCommandLockUnlockResponseReceive(
                    ZPS_tsAfEvent               *pZPSevent,
                    uint8                       *pu8TransactionSequenceNumber,
                    tsCLD_DoorLock_LockUnlockResponsePayload *psPayload)
{

    uint16 u16ActualQuantity;

    tsZCL_RxPayloadItem asPayloadDefinition[] = {
    {1,                                         &u16ActualQuantity,                     E_ZCL_ENUM8,    &psPayload->eStatus},
                                                };
                                                
    return eZCL_CustomCommandReceive(pZPSevent,
                                     pu8TransactionSequenceNumber,
                                     asPayloadDefinition,
                                     sizeof(asPayloadDefinition) / sizeof(tsZCL_RxPayloadItem),
                                     E_ZCL_ACCEPT_EXACT);

}

#endif
