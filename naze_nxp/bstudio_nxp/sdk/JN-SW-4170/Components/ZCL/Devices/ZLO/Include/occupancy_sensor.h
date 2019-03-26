/*****************************************************************************
 *
 * MODULE:             Occupancy Sensor
 *
 * COMPONENT:          occupancy_sensor.h
 *
 * AUTHOR:             Lee Mitchell
 *
 * DESCRIPTION:        Header for ZigBee Occupancy sensor profile functions
 *
 * $HeadURL: $
 *
 * $Revision: $
 *
 * $LastChangedBy: $
 *
 * $LastChangedDate: $
 *
 * $Id: $
 *
 *****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products  [NXP Microcontrollers such as JN5168, JN5164,
 * JN5161, JN5148, JN5142, JN5139]. 
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

#ifndef OCCUPANCY_SENSOR_H
#define OCCUPANCY_SENSOR_H

#if defined __cplusplus
extern "C" {
#endif

#include <jendefs.h>
#include "zcl.h"
#include "zcl_options.h"
#include "Basic.h"
#include "Identify.h"
#include "OccupancySensing.h"
#include "Groups.h"

#include "PowerConfiguration.h"
#include "DoorLock.h"
#include "Time.h"
#include "IASWD.h"
#ifdef CLD_ALARMS
#include "Alarms.h"
#endif

#ifdef CLD_OTA
#include "OTA.h"
#endif
#include "PollControl.h"

#ifdef CLD_ZLL_COMMISSION
#include "zll_commission.h"
#endif

#include "DoorLock.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* Holds cluster instances */
typedef struct 
{
    /* All ZLO devices have 2 mandatory clusters - Basic(server) and Identify(server) */
#if (defined CLD_BASIC) && (defined BASIC_SERVER)
    tsZCL_ClusterInstance sBasicServer;
#endif

#if (defined CLD_POWER_CONFIGURATION) && (defined POWER_CONFIGURATION_SERVER)
    tsZCL_ClusterInstance sPowerConfigurationServer;
#endif

    /* Mandatory client clusters */
#if (defined CLD_IDENTIFY) && (defined IDENTIFY_CLIENT)
    tsZCL_ClusterInstance sIdentifyClient;
#endif

#if (defined CLD_IDENTIFY) && (defined IDENTIFY_SERVER)
    tsZCL_ClusterInstance sIdentifyServer;
#endif

    /* Recommended Optional client clusters */
#if (defined CLD_GROUPS) && (defined GROUPS_CLIENT)
    tsZCL_ClusterInstance sGroupsClient;
#endif

#if (defined CLD_ALARMS) && (defined ALARMS_CLIENT)
    tsZCL_ClusterInstance sAlarmsClient;
#endif

#if (defined CLD_ALARMS) && (defined ALARMS_SERVER)
    tsZCL_ClusterInstance sAlarmsServer;
#endif

#if (defined CLD_TIME && defined TIME_CLIENT)
    tsZCL_ClusterInstance sTimeClient;
#endif

#if (defined CLD_TIME && defined TIME_SERVER)
    tsZCL_ClusterInstance sTimeServer;
#endif

#if (defined CLD_DOOR_LOCK) && (defined DOOR_LOCK_SERVER)
    tsZCL_ClusterInstance sDoorLockServer;
#endif

#if (defined CLD_OCCUPANCY_SENSING) && (defined OCCUPANCY_SENSING_SERVER)
    tsZCL_ClusterInstance sOccupancySensingServer;
#endif

#if (defined CLD_IASWD) && (defined IASWD_SERVER)
		tsZCL_ClusterInstance sIASWDClient;
#endif

#if (defined CLD_IASWD) && (defined IASWD_CLIENT)
		tsZCL_ClusterInstance sIASWDServer;
#endif

    /* Optional server clusters */
#if (defined CLD_POLL_CONTROL) && (defined POLL_CONTROL_SERVER)
    tsZCL_ClusterInstance sPollControlServer;
#endif

#if (defined CLD_POLL_CONTROL) && (defined POLL_CONTROL_CLIENT)
    tsZCL_ClusterInstance sPollControlClient;
#endif
#if (defined CLD_OTA) && (defined OTA_CLIENT)
    /* Add  cluster instance for the OTA cluster */
    tsZCL_ClusterInstance sOTAClient;
#endif

#if (defined CLD_ZLL_COMMISSION) && (defined ZLL_COMMISSION_SERVER)
	tsZCL_ClusterInstance sZllCommissionServer;
#endif

} tsZLO_OccupancySensorDeviceClusterInstances;


/* Holds everything required to create an instance of an Occupancy Sensor */
typedef struct
{
    tsZCL_EndPointDefinition sEndPoint;

    /* Cluster instances */
    tsZLO_OccupancySensorDeviceClusterInstances sClusterInstance;

    /* Mandatory server clusters */
#if (defined CLD_BASIC) && (defined BASIC_SERVER)
    /* Basic Cluster - Server */
    tsCLD_Basic sBasicServerCluster;
#endif
    /* Optional server clusters */
#if (defined CLD_POWER_CONFIGURATION) && (defined POWER_CONFIGURATION_SERVER)
    /* Power Configuration Cluster - Server */
    tsCLD_PowerConfiguration sPowerConfigServerCluster;
 #endif
 
    /* Mandatory client clusters */
#if (defined CLD_IDENTIFY) && (defined IDENTIFY_CLIENT)
    /* Identify Cluster - Client */
    tsCLD_Identify sIdentifyClientCluster;
    tsCLD_IdentifyCustomDataStructure sIdentifyClientCustomDataStructure;
#endif

#if (defined CLD_IDENTIFY) && (defined IDENTIFY_SERVER)
    /* Identify Cluster - Server */
    tsCLD_Identify sIdentifyServerCluster;
    tsCLD_IdentifyCustomDataStructure sIdentifyServerCustomDataStructure;
#endif

#if (defined CLD_GROUPS) && (defined GROUPS_CLIENT)
    /* Groups Cluster - Client */
    tsCLD_Groups sGroupsClientCluster;
    tsCLD_GroupsCustomDataStructure sGroupsClientCustomDataStructure;
#endif

#if (defined CLD_ALARMS) && (defined ALARMS_CLIENT)
    /* Alarms Cluster - Client */
    tsCLD_Alarms sAlarmsClientCluster;
    tsCLD_AlarmsCustomDataStructure sAlarmsClientCustomDataStructure;
#endif

#if (defined CLD_ALARMS) && (defined ALARMS_SERVER)
    /* Alarms Cluster - Server */
    tsCLD_Alarms sAlarmsServerCluster;
    tsCLD_AlarmsCustomDataStructure sAlarmsServerCustomDataStructure;
#endif

#if (defined CLD_TIME) && (defined TIME_CLIENT)
    tsCLD_Time sTimeClientCluster;
#endif

#if (defined CLD_TIME) && (defined TIME_SERVER)
    tsCLD_Time sTimeServerCluster;
#endif

#if (defined CLD_DOOR_LOCK) && (defined DOOR_LOCK_SERVER)
    /* door lock Cluster - Server */
    tsCLD_DoorLock sDoorLockServerCluster;
#endif

#if (defined CLD_OCCUPANCY_SENSING) && (defined OCCUPANCY_SENSING_SERVER)
    /* Occupancy Sensing Cluster - Server */
    tsCLD_OccupancySensing sOccupancySensingServerCluster;
#endif

#if (defined CLD_IASWD) && (defined IASWD_SERVER)
		/* IAS WD Cluster - Server */
		tsCLD_IASWD sIASWDServerCluster;
		tsCLD_IASWD_CustomDataStructure sIASWDServerCustomDataStructure;
#endif

#if (defined CLD_IASWD) && (defined IASWD_CLIENT)
		/* IAS WD Cluster - Server */
		tsCLD_IASWD sIASWDClientCluster;
		tsCLD_IASWD_CustomDataStructure sIASWDClientCustomDataStructure;
#endif

    /* Optional server clusters */
#if (defined CLD_POLL_CONTROL) && (defined POLL_CONTROL_SERVER)
    tsCLD_PollControl sPollControlServerCluster;
    tsCLD_PollControlCustomDataStructure sPollControlServerCustomDataStructure;
#endif

    /* Recommended Optional client clusters */
#if (defined CLD_POLL_CONTROL) && (defined POLL_CONTROL_CLIENT)
    tsCLD_PollControl sPollControlClientCluster;
    tsCLD_PollControlCustomDataStructure sPollControlClientCustomDataStructure;
#endif

#if (defined CLD_OTA) && (defined OTA_CLIENT)
    tsCLD_AS_Ota sCLD_OTA;
    tsOTA_Common sCLD_OTA_CustomDataStruct;
#endif

#if (defined CLD_ZLL_COMMISSION) && (defined ZLL_COMMISSION_SERVER)
	tsCLD_ZllCommission 						sZllCommissionServerCluster;
	tsCLD_ZllCommissionCustomDataStructure		sZllCommissionServerCustomDataStructure;
#endif

} tsZLO_OccupancySensorDevice;


/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC teZCL_Status eZLO_RegisterOccupancySensorEndPoint(uint8 u8EndPointIdentifier,
                                              tfpZCL_ZCLCallBackFunction cbCallBack,
                                              tsZLO_OccupancySensorDevice *psDeviceInfo);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* OCCUPANCY_SENSOR_H */
