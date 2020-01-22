/*****************************************************************************
 *
 * MODULE:             Door Lock Cluster
 *
 * COMPONENT:          DoorLock.h
 *
 * AUTHOR:             Shweta Chauhan
 * DESCRIPTION:        Header for DoorLock Cluster
 *
 * $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/SmartEnergy/Branches/HA1_x_1v0/ZCL/Clusters/General/Include/DoorLock.h $
 *
 * $Revision: 53326 $
 *
 * $LastChangedBy: nxp57621 $
 *
 * $LastChangedDate: 2013-04-17 11:51:06 +0530 (Wed, 17 Apr 2013) $
 *
 * $Id: DoorLock.h 53326 2013-04-17 06:21:06Z nxp57621 $
 *
 *****************************************************************************
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

#ifndef DOORLOCK_H
#define DOORLOCK_H

#include <jendefs.h>
#include "zcl.h"
#include "zcl_options.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Cluster ID's */
#define CLOSURE_CLUSTER_ID_DOOR_LOCK                        0x0101

/****************************************************************************/
/*             Door_Lock Control Cluster - Optional Attributes             */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* attributes to the door lock cluster.                                 */
/****************************************************************************/
#define PAYLOAD_LEN_OPERATION_EVENT_NOTIFICATION			9 // not including pData
#define COMMAND_CLOSURES_OPERATION_EVENT_NOTIFICATION		0x20 // O  zclDoorLockOperationalEventNotification_t

/* Door state attribute */
//#define CLD_DOOR_LOCK_ATTR_DOOR_STATE

/* number of door open evenrs attribute */
//#define CLD_DOOR_LOCK_ATTR_NUMBER_OF_DOOR_OPEN_EVENTS

/* number of door closed events attribute */
//#define CLD_DOOR_LOCK_ATTR_NUMBER_OF_DOOR_CLOSED_EVENTS

/* Number of minutes door has been open since this last time it transitioned from close to open */
//#define CLD_DOOR_LOCK_ATTR_NUMBER_OF_MINUTES_DOOR_OPENED

/* Zigbee Security Level*/
//#define CLD_DOOR_LOCK_ZIGBEE_SECUTRITY_LEVEL

#ifndef CLD_DOOR_LOCK_CLUSTER_REVISION
    #define CLD_DOOR_LOCK_CLUSTER_REVISION                         1
#endif 

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* Door Lock Command - Payload */
typedef enum 
{
    E_CLD_DOOR_LOCK_CMD_LOCK			= 0x00,	/* Mandatory */
    E_CLD_DOOR_LOCK_CMD_UNLOCK			= 0x01,	/* Mandatory */
    E_CLD_DOOR_LOCK_CMD_TOGGLE			= 0x02,
    E_CLD_DOOR_LOCK_CMD_UNLOCK_TIMEOUT	= 0x03,
    E_CLD_DOOR_LOCK_CMD_SET_PINCODE		= 0x05,
    E_CLD_DOOR_LOCK_CMD_SET_YEAR_DAY_SCHEDULE = 0x0E
} teCLD_DoorLock_CommandID;


typedef enum 
{
    /* Door Lock attribute set attribute ID's (A1) */
    E_CLD_DOOR_LOCK_ATTR_ID_LOCK_STATE                = 0x0000,             /* 0.Mandatory */
    E_CLD_DOOR_LOCK_ATTR_ID_LOCK_TYPE,                                      /* 1.Mandatory */
    E_CLD_DOOR_LOCK_ATTR_ID_ACTUATOR_ENABLED,                               /* 2.Mandatory */
    E_CLD_DOOR_LOCK_ATTR_ID_DOOR_STATE,                                     /* 3.Optional */
    E_CLD_DOOR_LOCK_ATTR_ID_NUMBER_OF_DOOR_OPEN_EVENTS,                     /* 4.Optional */
    E_CLD_DOOR_LOCK_ATTR_ID_NUMBER_OF_DOOR_CLOSED_EVENTS,                   /* 5.Optional */
    E_CLD_DOOR_LOCK_ATTR_ID_NUMBER_OF_MINUTES_DOOR_OPENED,                  /* 6.Optional */
    E_CLD_DOOR_LOCK_ATTR_ID_ZIGBEE_SECURITY_LEVEL     = 0x0034,
    E_CLD_DOOR_LOCK_ID_TRANSPRT_TRANS				  = 0x410C,
    E_CLD_DOOR_LOCK_ATTR_ID_USER_ADD_TEMPOARYPASSWORD = 0xFF33,             /*Moon*/
} teCLD_DoorLock_Cluster_AttrID; 

/* Lock State */
typedef enum 
{
    E_CLD_DOORLOCK_LOCK_STATE_NOT_FULLY_LOCKED  = 0x00,
    E_CLD_DOORLOCK_LOCK_STATE_LOCKED,
    E_CLD_DOORLOCK_LOCK_STATE_UNLOCKED,
    E_CLD_DOORLOCK_LOCK_STATE_UNDEFINED         = 0xFF
} teCLD_DoorLock_LockState;

typedef enum 
{
    E_CLD_DOORLOCK_LOCK_TYPE_DEAD_BOLT  = 0x00,
    E_CLD_DOORLOCK_LOCK_TYPE_MAGNETIC,
    E_CLD_DOORLOCK_LOCK_TYPE_OTHER,
    E_CLD_DOORLOCK_LOCK_TYPE_MORTISE,
    E_CLD_DOORLOCK_LOCK_TYPE_RIM,
    E_CLD_DOORLOCK_LOCK_TYPE_LATCH_BOLT,
    E_CLD_DOORLOCK_LOCK_TYPE_CYLINDRICAL_LOCK,
    E_CLD_DOORLOCK_LOCK_TYPE_TUBULAR_LOCK,
    E_CLD_DOORLOCK_LOCK_TYPE_INTERCONNECTED_LOCK,
    E_CLD_DOORLOCK_LOCK_TYPE_DEAD_LATCH,
    E_CLD_DOORLOCK_LOCK_TYPE_DOOR_FURNITURE
} teCLD_DoorLock_LockType;

#ifdef CLD_DOOR_LOCK_ATTR_DOOR_STATE
typedef enum 
{
    E_CLD_DOORLOCK_DOOR_STATE_OPEN  = 0x00,
    E_CLD_DOORLOCK_DOOR_STATE_CLOSED,
    E_CLD_DOORLOCK_DOOR_STATE_ERROR_JAMMED,
    E_CLD_DOORLOCK_DOOR_STATE_ERROR_FORCED_OPEN,
    E_CLD_DOORLOCK_DOOR_STATE_ERROR_UNSPECIFIED,
    E_CLD_DOORLOCK_DOOR_STATE_UNDEFINED = 0xFF
} teCLD_DoorLock_DoorState;
#endif

/* Door Lock Cluster */
typedef struct
{
#ifdef DOOR_LOCK_SERVER
    zenum8                   eLockState;
    zenum8                   eLockType;
    zbool                    bActuatorEnabled;
    
#ifdef CLD_DOOR_LOCK_ATTR_DOOR_STATE
    zenum8                   eDoorState;
#endif

#ifdef CLD_DOOR_LOCK_ATTR_NUMBER_OF_DOOR_OPEN_EVENTS
    zuint32                   u32NumberOfDoorOpenEvent;
#endif

#ifdef CLD_DOOR_LOCK_ATTR_NUMBER_OF_DOOR_CLOSED_EVENTS
    zuint32                   u32NumberOfDoorClosedEvent;
#endif

#ifdef CLD_DOOR_LOCK_ATTR_NUMBER_OF_MINUTES_DOOR_OPENED
    zuint16                   u16NumberOfMinutesDoorOpened;
#endif

#ifdef CLD_DOOR_LOCK_ZIGBEE_SECUTRITY_LEVEL
    zuint8                     u8ZigbeeSecurityLevel;
#endif

#ifdef CLD_DOOR_LOCK_ATTRIBUTE_REPORTING_STATUS
    zuint8                     u8AttributeReportingStatus;
#endif
#endif
	tsZCL_CharacterString	sTransport;
	uint8					au8Transport[80];
    zuint16                    u16ClusterRevision;
} tsCLD_DoorLock;


typedef struct
{
    zenum8                  eStatus;
}tsCLD_DoorLock_LockUnlockResponsePayload;

/* Definition of Door Lock Callback Event Structure */
typedef struct
{
    uint8                           u8CommandId;
        union
    {
        tsCLD_DoorLock_LockUnlockResponsePayload              *psLockUnlockResponsePayload;
    } uMessage;
} tsCLD_DoorLockCallBackMessage;

// Client Commands Received: Operation Event Notification
typedef struct
{
  uint8 operationEventSource;   // e.g. OPERATION_EVENT_SOURCE_KEYPAD
  uint8 operationEventCode;   // e.g. OPERATION_EVENT_CODE_UNKNOWN_OR_MFG_SPECIFIC
  uint16 userID;
  uint8 pin;
  uint32 zigBeeLocalTime;
  uint8 *pData;   // variable length string
} zclDoorLockOperationEventNotification_t;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC teZCL_Status eCLD_DoorLockCreateDoorLock(
                tsZCL_ClusterInstance                  *psClusterInstance,
                bool_t                                 bIsServer,
                tsZCL_ClusterDefinition                *psClusterDefinition,
                void                                   *pvEndPointSharedStructPtr,
                uint8                                  *pu8AttributeControlBits);

#ifdef CLD_DOOR_LOCK_ZIGBEE_SECUTRITY_LEVEL
PUBLIC teZCL_Status eCLD_DoorLockSetSecurityLevel(
                    uint8 u8SourceEndPointId,
                    bool bServer,
                    uint8 u8SecurityLevel);                
#endif

#ifdef DOOR_LOCK_CLIENT                  
PUBLIC teZCL_Status eCLD_DoorLockCommandLockUnlockRequestSend(
                    uint8                              u8SourceEndPointId,
                    uint8                           u8DestinationEndPointId,
                    tsZCL_Address                   *psDestinationAddress,
                    uint8                           *pu8TransactionSequenceNumber,
                    teCLD_DoorLock_CommandID          eCommand);
#endif

#ifdef DOOR_LOCK_SERVER 
PUBLIC teZCL_Status eCLD_DoorLockSetLockState(
                    uint8                       u8SourceEndPointId,
                    teCLD_DoorLock_LockState    eLock);
PUBLIC teZCL_Status eCLD_DoorLockGetLockState(
                    uint8                       u8SourceEndPointId,
                    teCLD_DoorLock_LockState    *eLock);
#endif

PUBLIC  teZCL_Status   eCLD_DoorLockCommandDoorLockOperationEventNotification( 
																uint8 srcEP, tsZCL_Address *dstAddr,
																uint8 u8DestinationEndPointId,
																zclDoorLockOperationEventNotification_t *pPayload);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

extern tsZCL_ClusterDefinition sCLD_DoorLock;
extern const tsZCL_AttributeDefinition asCLD_DoorLockClusterAttributeDefinitions[];
extern uint8 au8DoorLockAttributeControlBits[];
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* DOORLOCK_H */
