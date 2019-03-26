/*****************************************************************************
 *
 * MODULE:             Occupancy Sensor
 *
 * COMPONENT:          occupancy_sensor.c
 *
 * AUTHOR:             Lee Mitchell
 *
 * DESCRIPTION:        ZigBee Occupancy Sensor profile functions
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
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5164,
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

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <string.h>
#include "zps_apl.h"
#include "zcl_heap.h"
#include "occupancy_sensor.h"
#include "DBG.h"
#include "DoorLock.h"

/*IAS WD cluster*/
#include "IASWD.h"

#ifdef CLD_ALARMS
#include "Alarms.h"
#endif

#ifdef CLD_OTA
    #include "OTA.h"
    #include "app_ota_client.h"
#endif

#ifdef CLD_ZLL_COMMISSION
#include "zll_commission.h"
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
/***        Exported Functions                                            ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: eZLO_RegisterOccupancySensorEndPoint
 *
 * DESCRIPTION:
 * Registers an occupancy sensor device with the ZCL layer
 *
 * PARAMETERS:  Name                            Usage
 *              u8EndPointIdentifier            Endpoint being registered
 *              cbCallBack                      Pointer to endpoint callback
 *              psDeviceInfo                    Pointer to struct containing
 *                                              data for endpoint
 *
 * RETURNS:
 * teZCL_Status
 *
 ****************************************************************************/
PUBLIC teZCL_Status eZLO_RegisterOccupancySensorEndPoint(uint8 u8EndPointIdentifier,
                                              tfpZCL_ZCLCallBackFunction cbCallBack,
                                              tsZLO_OccupancySensorDevice *psDeviceInfo)
{

    /* Fill in end point details */
    psDeviceInfo->sEndPoint.u8EndPointNumber = u8EndPointIdentifier;
    psDeviceInfo->sEndPoint.u16ManufacturerCode = ZCL_MANUFACTURER_CODE;
    psDeviceInfo->sEndPoint.u16ProfileEnum = HA_PROFILE_ID;
    psDeviceInfo->sEndPoint.bIsManufacturerSpecificProfile = FALSE;
    psDeviceInfo->sEndPoint.u16NumberOfClusters = sizeof(tsZLO_OccupancySensorDeviceClusterInstances) / sizeof(tsZCL_ClusterInstance);
    psDeviceInfo->sEndPoint.psClusterInstance = (tsZCL_ClusterInstance*)&psDeviceInfo->sClusterInstance;
    psDeviceInfo->sEndPoint.bDisableDefaultResponse = 0;//ZCL_DISABLE_DEFAULT_RESPONSES;	//Ricky
    psDeviceInfo->sEndPoint.pCallBackFunctions = cbCallBack;

    /* Mandatory server clusters */
    #if (defined CLD_BASIC) && (defined BASIC_SERVER)
        /* Create an instance of a Basic cluster as a server */
        if(eCLD_BasicCreateBasic(&psDeviceInfo->sClusterInstance.sBasicServer,
                              TRUE,
                              &sCLD_Basic,
                              &psDeviceInfo->sBasicServerCluster,
                              &au8BasicClusterAttributeControlBits[0]) != E_ZCL_SUCCESS)
        {
            // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
            return E_ZCL_FAIL;
        } 

		vZCL_SetDefaultReporting(&psDeviceInfo->sClusterInstance.sBasicServer);
    #endif
		/* Optional server clusters */
	#if (defined CLD_POWER_CONFIGURATION) && (defined POWER_CONFIGURATION_SERVER)
		/* Create an instance of a Power Configuration cluster as a server */
		if(eCLD_PowerConfigurationCreatePowerConfiguration(&psDeviceInfo->sClusterInstance.sPowerConfigurationServer,
							  TRUE,
							  &sCLD_PowerConfiguration,
							  &psDeviceInfo->sPowerConfigServerCluster,
							  &au8PowerConfigurationAttributeControlBits[0]) != E_ZCL_SUCCESS)
		{
			return E_ZCL_FAIL;
		} 
	#endif

		/* Mandatory client clusters */
#if (defined CLD_IDENTIFY) && (defined IDENTIFY_CLIENT)
			/* Create an instance of an Identify cluster as a client */
			if(eCLD_IdentifyCreateIdentify(&psDeviceInfo->sClusterInstance.sIdentifyClient,
								  FALSE,
								  &sCLD_Identify,
								  &psDeviceInfo->sIdentifyClientCluster,
								  &au8IdentifyAttributeControlBits[0],
								  &psDeviceInfo->sIdentifyClientCustomDataStructure) != E_ZCL_SUCCESS)
			{
				// Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
				return E_ZCL_FAIL;
			}	 
#endif

    #if (defined CLD_IDENTIFY) && (defined IDENTIFY_SERVER)
        /* Create an instance of an Identify cluster as a server */
        if(eCLD_IdentifyCreateIdentify(&psDeviceInfo->sClusterInstance.sIdentifyServer,
                              TRUE,
                              &sCLD_Identify,
                              &psDeviceInfo->sIdentifyServerCluster,
                              &au8IdentifyAttributeControlBits[0],
                              &psDeviceInfo->sIdentifyServerCustomDataStructure) != E_ZCL_SUCCESS)
        {
            // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
            return E_ZCL_FAIL;
        }    
    #endif

    /* Recommended Optional client clusters */
#if (defined CLD_GROUPS) && (defined GROUPS_CLIENT)
        /* Create an instance of a Groups cluster as a client */
        if(eCLD_GroupsCreateGroups(&psDeviceInfo->sClusterInstance.sGroupsClient,
                              FALSE,
                              &sCLD_Groups,
                              &psDeviceInfo->sGroupsClientCluster,
                              &au8GroupsAttributeControlBits[0],
                              &psDeviceInfo->sGroupsClientCustomDataStructure,
                              &psDeviceInfo->sEndPoint) != E_ZCL_SUCCESS)
        {
            // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
            return E_ZCL_FAIL;
        } 
#endif

#if (defined CLD_ALARMS) && (defined ALARMS_CLIENT)
			/* Create an instance of an Alarms cluster as a server */
			if(eCLD_AlarmsCreateAlarms(&psDeviceInfo->sClusterInstance.sAlarmsClient,
								  FALSE,
								  &sCLD_Alarms,
								  &psDeviceInfo->sAlarmsClientCluster,
								  &au8AlarmsAttributeControlBits[0],
								  &psDeviceInfo->sAlarmsClientCustomDataStructure) != E_ZCL_SUCCESS)
			{
				return E_ZCL_FAIL;
			} 
#endif

#if (defined CLD_ALARMS) && (defined ALARMS_SERVER)
			/* Create an instance of an Alarms cluster as a server */
			if(eCLD_AlarmsCreateAlarms(&psDeviceInfo->sClusterInstance.sAlarmsServer,
								  TRUE,
								  &sCLD_Alarms,
								  &psDeviceInfo->sAlarmsServerCluster,
								  &au8AlarmsAttributeControlBits[0],
								  &psDeviceInfo->sAlarmsServerCustomDataStructure) != E_ZCL_SUCCESS)
			{
				return E_ZCL_FAIL;
			} 
#endif

#if (defined CLD_TIME && defined TIME_CLIENT)
    if (eCLD_TimeCreateTime(
                    &psDeviceInfo->sClusterInstance.sTimeClient,
                    FALSE,
                    &sCLD_Time,
                    &psDeviceInfo->sTimeClientCluster,
                    &au8TimeClusterAttributeControlBits[0]) != E_ZCL_SUCCESS)
    {
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_TIME && defined TIME_SERVER)
    if (eCLD_TimeCreateTime(
                    &psDeviceInfo->sClusterInstance.sTimeServer,
                    TRUE,
                    &sCLD_Time,
                    &psDeviceInfo->sTimeServerCluster,
                    &au8TimeClusterAttributeControlBits[0]) != E_ZCL_SUCCESS)
    {
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_DOOR_LOCK) && (defined DOOR_LOCK_SERVER)
		/* Create an instance of a door lock cluster as a server */
		if(eCLD_DoorLockCreateDoorLock(&psDeviceInfo->sClusterInstance.sDoorLockServer,
							  TRUE,
							  &sCLD_DoorLock,
							  &psDeviceInfo->sDoorLockServerCluster,
							  &au8DoorLockAttributeControlBits[0]) != E_ZCL_SUCCESS)
		{
			return E_ZCL_FAIL;
		}
#endif

    #if (defined CLD_OCCUPANCY_SENSING) && (defined OCCUPANCY_SENSING_SERVER)
        /* Create an instance of an Occupancy Sensing cluster as a server */
        eCLD_OccupancySensingCreateOccupancySensing(
                              &psDeviceInfo->sClusterInstance.sOccupancySensingServer,
                              TRUE,
                              &sCLD_OccupancySensing,
                              &psDeviceInfo->sOccupancySensingServerCluster,
                              &au8OccupancySensingAttributeControlBits[0]);
    #endif

    /* Optional server clusters */
    #if (defined CLD_POLL_CONTROL) && (defined POLL_CONTROL_SERVER)
        /* Create an instance of a Poll Control cluster as a server */
        if(eCLD_PollControlCreatePollControl(
                              &psDeviceInfo->sClusterInstance.sPollControlServer,
                              TRUE,
                              &sCLD_PollControl,
                              &psDeviceInfo->sPollControlServerCluster,
                              &au8PollControlAttributeControlBits[0],
                              &psDeviceInfo->sPollControlServerCustomDataStructure) != E_ZCL_SUCCESS)
        {
            // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
            return E_ZCL_FAIL;
        }    
    #endif

	#if (defined CLD_POLL_CONTROL) && (defined POLL_CONTROL_CLIENT)
    /* Create an instance of a Poll Control cluster as a client */
    if(eCLD_PollControlCreatePollControl(
                          &psDeviceInfo->sClusterInstance.sPollControlClient,
                          FALSE,
                          &sCLD_PollControl,
                          &psDeviceInfo->sPollControlClientCluster,
                          &au8PollControlAttributeControlBits[0],
                          &psDeviceInfo->sPollControlClientCustomDataStructure) != E_ZCL_SUCCESS)
    {
        return E_ZCL_FAIL;
    } 
	#endif
	#if (defined CLD_IASWD) && (defined IASWD_SERVER)
		/* Create an instance of a IAS WD cluster as a Server */
		if(eCLD_IASWDCreateIASWD(&psDeviceInfo->sClusterInstance.sIASWDServer,
								TRUE,
								&sCLD_IASWD,
								&psDeviceInfo->sIASWDServerCluster,
								&au8IASWDAttributeControlBits[0],
								&psDeviceInfo->sIASWDServerCustomDataStructure) != E_ZCL_SUCCESS)
		{
			return E_ZCL_FAIL;
		} 
	#endif

	#if (defined CLD_IASWD) && (defined IASWD_CLIENT)
		/* Create an instance of a IAS WD cluster as a client */
		if(eCLD_IASWDCreateIASWD(&psDeviceInfo->sClusterInstance.sIASWDClient,
								FALSE,
								&sCLD_IASWD,
								&psDeviceInfo->sIASWDClientCluster,
								&au8IASWDAttributeControlBits[0],
								&psDeviceInfo->sIASWDClientCustomDataStructure) != E_ZCL_SUCCESS)
		{
			return E_ZCL_FAIL;
		} 
	#endif

    #if(defined CLD_OTA) && (defined OTA_CLIENT)
       if (eOTA_Create(
           &psDeviceInfo->sClusterInstance.sOTAClient,
           FALSE,  /* client */
           &sCLD_OTA,
           &psDeviceInfo->sCLD_OTA,  /* cluster definition */
           u8EndPointIdentifier,
           NULL,
           &psDeviceInfo->sCLD_OTA_CustomDataStruct
           )!= E_ZCL_SUCCESS)

        {
            // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code            
            return E_ZCL_FAIL;
        }
    #endif

		#if (defined CLD_ZLL_COMMISSION) && (defined ZLL_COMMISSION_SERVER)
        /* Create an instance of a TL commissioning cluster as a server */
        if(eCLD_ZllCommissionCreateCommission(&psDeviceInfo->sClusterInstance.sZllCommissionServer,
                              TRUE,
                              &sCLD_ZllCommission,
                              NULL/*&psDeviceInfo->sZllCommissionServerCluster*/,
                              NULL/*(uint8*)&psDeviceInfo->sZllCommissionClusterAttributeStatus*/,
                              &psDeviceInfo->sZllCommissionServerCustomDataStructure) != E_ZCL_SUCCESS)
        {
            // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
            return E_ZCL_FAIL;
        }
    #endif

    return eZCL_Register(&psDeviceInfo->sEndPoint);
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

