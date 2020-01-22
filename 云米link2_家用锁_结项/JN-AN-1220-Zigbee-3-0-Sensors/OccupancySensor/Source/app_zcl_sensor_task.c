/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_zcl_sensor_task.c
 *
 * DESCRIPTION:        ZCL Occupancy sensor Control Behaviour (Implementation)
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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
 * Copyright NXP B.V. 2016. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <AppApi.h>
#include "ZTimer.h"
#include "app_main.h"
#include "pdum_apl.h"
#include "pdum_gen.h"
#include "PDM.h"
#include "dbg.h"
#include "pwrm.h"
#include "zps_apl_af.h"
#include "zps_apl_zdo.h"
#include "zps_apl_aib.h"
#include "zps_apl_zdp.h"
#include "rnd_pub.h"
#include "mac_pib.h"
#include "zcl_options.h"
#include "zcl.h"
#include "app_common.h"
#include "app_zlo_sensor_node.h"
#include "AHI_AES.h"
#include "app_events.h"
#include "LightingBoard.h"
#include "app_zcl_tick_handler.h"
#include "app_zcl_sensor_task.h"
#include "App_OccupancySensor.h"
#include "app_reporting.h"
#include "app_occupancy_buttons.h"
#include "app_blink_led.h"
#include "app_nwk_event_handler.h"
#ifdef CLD_OTA
 #include "app_ota_client.h"
#endif
#include "app_SampleDoorLock_Uart.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_ZCL
    #define TRACE_ZCL   TRUE
#else
    #define TRACE_ZCL   FALSE
#endif

#ifdef DEBUG_SENSOR_TASK
    #define TRACE_SENSOR_TASK   TRUE
#else
    #define TRACE_SENSOR_TASK   FALSE
#endif

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PRIVATE void APP_ZCL_cbGeneralCallback(tsZCL_CallBackEvent *psEvent);
PRIVATE void APP_ZCL_cbEndpointCallback(tsZCL_CallBackEvent *psEvent);
#ifdef CLD_OTA
PRIVATE void APP_ZCL_OTASetBlockRequestTime(tsZCL_CallBackEvent *psCallBackEvent);
PRIVATE void APP_ZCL_OTASetUpgradeTime(tsZCL_CallBackEvent *psCallBackEvent);
#endif
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern PDM_tsRecordDescriptor sDevicePDDesc;
extern tsDeviceDesc sDeviceDesc;
extern uint8 u8TimerRetryTimeSynchronization;
extern uint8 u8RetryTimeSynchronizationTimes;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: APP_ZCL_vInitialise
 *
 * DESCRIPTION:
 * Initialises ZCL related functions
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_ZCL_vInitialise(void)
{
    teZCL_Status eZCL_Status;

    /* Initialise ZCL */
    eZCL_Status = eZCL_Initialise(&APP_ZCL_cbGeneralCallback, apduZCL);
    if (eZCL_Status != E_ZCL_SUCCESS)
    {
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: Error eZCL_Initialise returned %d", eZCL_Status);
    }

    /* Register ZLO EndPoint */
    eZCL_Status = eApp_ZLO_RegisterEndpoint(&APP_ZCL_cbEndpointCallback);//moon
    if (eZCL_Status != E_ZCL_SUCCESS)
    {
            DBG_vPrintf(TRACE_SENSOR_TASK, "\nAPP_ZCL: Error: eApp_ZLO_RegisterEndpoint:%d", eZCL_Status);
    }

    DBG_vPrintf(TRACE_SENSOR_TASK, "\nAPP_ZCL: Chan Mask %08x", ZPS_psAplAibGetAib()->pau32ApsChannelMask[0]);
    DBG_vPrintf(TRACE_SENSOR_TASK, "\nAPP_ZCL: RxIdle TRUE");

    DBG_vPrintf(TRUE, "\nAPP_ZCL_vInitialise: Start tick timer\n");
    ZTIMER_eStart(u8TimerTick, ZCL_TICK_TIME);

    vAPP_ZCL_DeviceSpecific_Init();

#ifdef CLD_OTA
    vAppInitOTA();
#endif
}

/****************************************************************************
 *
 * NAME: ZCL_Task
 *
 * DESCRIPTION:
 * ZCL Task for the sensor
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_cbTimerZclTick( void *pvParam)
{
	//sDBG_vPrintf(TRACE_ZCL, "\nCancle ZTIMER_eStart(u8TimerTick, ZCL_TICK_TIME)\n");
	//sreturn;
    /*
     * If the 1 second tick timer has expired, restart it and pass
     * the event on to ZCL
     */
    vAPP_ZCL_Tick();//moon
    ZTIMER_eStart(u8TimerTick, ZCL_TICK_TIME);
	DBG_vPrintf(TRACE_ZCL, "\nZTIMER_eStart(u8TimerTick, ZCL_TICK_TIME)\n");
}

/****************************************************************************
 *
 * NAME: APP_ZCL_vEventHandler
 *
 * DESCRIPTION:
 * Sends the stack event to the ZCL event handler.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_ZCL_vEventHandler(ZPS_tsAfEvent* psStackEvent)
{
    tsZCL_CallBackEvent sCallBackEvent;

    sCallBackEvent.pZPSevent = psStackEvent;
    sCallBackEvent.eEventType = E_ZCL_CBET_ZIGBEE_EVENT;
    vZCL_EventHandler(&sCallBackEvent);
}

PUBLIC void vAPP_ZCL_Tick(void)
{
    ZPS_tsAfEvent sStackEvent;
    tsZCL_CallBackEvent sCallBackEvent;
    sCallBackEvent.pZPSevent = &sStackEvent;

    vDecrementTickCount();
    sCallBackEvent.eEventType = E_ZCL_CBET_TIMER;
    vZCL_EventHandler(&sCallBackEvent);

#ifdef CLD_OTA
     /* Update for 1 second (1000ms) */
	 DBG_vPrintf(TRACE_ZCL, "\nvRunAppOTAStateMachine\n");
     vRunAppOTAStateMachine(1000);
#endif

}
/****************************************************************************
 *
 * NAME: APP_ZCL_cbGeneralCallback
 *
 * DESCRIPTION:
 * General callback for ZCL events
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_ZCL_cbGeneralCallback(tsZCL_CallBackEvent *psEvent)
{

    switch (psEvent->eEventType)
    {

    case E_ZCL_CBET_UNHANDLED_EVENT:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EVT Unhandled Event");
        break;

    case E_ZCL_CBET_READ_ATTRIBUTES_RESPONSE:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EVT Read attributes response");
        break;

    case E_ZCL_CBET_READ_REQUEST:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EVT Read request");
        break;

    case E_ZCL_CBET_DEFAULT_RESPONSE:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EVT Default response");
        break;

    case E_ZCL_CBET_ERROR:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EVT Error");
        break;

    case E_ZCL_CBET_TIMER:
        break;

    case E_ZCL_CBET_ZIGBEE_EVENT:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EVT ZigBee");
        break;

    case E_ZCL_CBET_CLUSTER_CUSTOM:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EP EVT Custom");
        break;

    default:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: Invalid event type");
        break;

    }
}

/****************************************************************************
 *
 * NAME: APP_ZCL_cbEndpointCallback
 *
 * DESCRIPTION:
 * Endpoint specific callback for ZCL events
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_ZCL_cbEndpointCallback(tsZCL_CallBackEvent *psEvent)
{

    switch (psEvent->eEventType)
    {

    case E_ZCL_CBET_REPORT_INDIVIDUAL_ATTRIBUTES_CONFIGURE:
        {
            tsZCL_AttributeReportingConfigurationRecord    *psAttributeReportingRecord= &psEvent->uMessage.sAttributeReportingConfigurationRecord;
            DBG_vPrintf(TRACE_ZCL,"\nAPP_ZCL: Individual Configure attribute for Cluster = %d",psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum);
            DBG_vPrintf(TRACE_ZCL,"\nAPP_ZCL: eAttributeDataType = %d",psAttributeReportingRecord->eAttributeDataType);
            DBG_vPrintf(TRACE_ZCL,"\nAPP_ZCL: u16AttributeEnum = %d",psAttributeReportingRecord->u16AttributeEnum );
            DBG_vPrintf(TRACE_ZCL,"\nAPP_ZCL: u16MaximumReportingInterval = %d",psAttributeReportingRecord->u16MaximumReportingInterval );
            DBG_vPrintf(TRACE_ZCL,"\nAPP_ZCL: u16MinimumReportingInterval = %d",psAttributeReportingRecord->u16MinimumReportingInterval );
            DBG_vPrintf(TRACE_ZCL,"\nAPP_ZCL: u16TimeoutPeriodField = %d",psAttributeReportingRecord->u16TimeoutPeriodField );
            DBG_vPrintf(TRACE_ZCL,"\nAPP_ZCL: u8DirectionIsReceived = %d",psAttributeReportingRecord->u8DirectionIsReceived );
            DBG_vPrintf(TRACE_ZCL,"\nAPP_ZCL: uAttributeReportableChange = %d",psAttributeReportingRecord->uAttributeReportableChange );
            if (E_ZCL_SUCCESS == psEvent->eZCL_Status)
            {
                if(MEASUREMENT_AND_SENSING_CLUSTER_ID_OCCUPANCY_SENSING == psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum)
                {
                    vSaveReportableRecord(MEASUREMENT_AND_SENSING_CLUSTER_ID_OCCUPANCY_SENSING,psAttributeReportingRecord);
                }
            }
        }
        break;

    case E_ZCL_CBET_UNHANDLED_EVENT:
    case E_ZCL_CBET_READ_ATTRIBUTES_RESPONSE:
    case E_ZCL_CBET_READ_REQUEST:

        DBG_vPrintf(TRACE_ZCL, "\n APP_ZCL: E_ZCL_CBET_READ_REQUEST or RESPONSE ");
		break;
		
    case E_ZCL_CBET_DEFAULT_RESPONSE:
        DBG_vPrintf(1, "\n\n APP_ZCL: E_ZCL_CBET_DEFAULT_RESPONSE - for %04x \n",
						psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum);

		if(psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum == 0x0020)
		{
			//APP_RecvPollControlResponse();
		}
		App_SampleDoorLockCommandResend(psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum);//u16NumberOfAttributes //moon
		break;
		
    case E_ZCL_CBET_ERROR:
    case E_ZCL_CBET_TIMER:
    case E_ZCL_CBET_ZIGBEE_EVENT:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EP EVT No action");
        break;

    case E_ZCL_CBET_READ_INDIVIDUAL_ATTRIBUTE_RESPONSE:
        DBG_vPrintf(TRACE_SENSOR_TASK, "\nAPP_ZCL: Read Attrib Rsp %d %02x", psEvent->uMessage.sIndividualAttributeResponse.eAttributeStatus,
            *((uint8*)psEvent->uMessage.sIndividualAttributeResponse.pvAttributeData));
        break;

    case E_ZCL_CBET_CLUSTER_CUSTOM:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EP EVT: Custom %04x", psEvent->uMessage.sClusterCustomMessage.u16ClusterId);

        switch (psEvent->uMessage.sClusterCustomMessage.u16ClusterId)
        {

        case GENERAL_CLUSTER_ID_BASIC:
        {
            tsCLD_BasicCallBackMessage *psCallBackMessage = (tsCLD_BasicCallBackMessage*)psEvent->uMessage.sClusterCustomMessage.pvCustomData;
            if (psCallBackMessage->u8CommandId == E_CLD_BASIC_CMD_RESET_TO_FACTORY_DEFAULTS )
            {
                DBG_vPrintf(TRACE_ZCL, "Basic Factory Reset Received\n");
                /* resetting the sensor structure back to zero*/
                memset(&sSensor,0,sizeof(tsZLO_OccupancySensorDevice));
                vAPP_ZCL_DeviceSpecific_Init();
				#if 0//def CLD_OTA
                    vAppInitOTA();
                #endif
                eApp_ZLO_RegisterEndpoint(&APP_ZCL_cbEndpointCallback);
				ZTIMER_eStart(u8TimerLeaveInd,ZTIMER_TIME_MSEC(3000));
            }
        }
        break;

        case GENERAL_CLUSTER_ID_IDENTIFY:
            {
                tsCLD_IdentifyCallBackMessage *psCallBackMessage = (tsCLD_IdentifyCallBackMessage*)psEvent->uMessage.sClusterCustomMessage.pvCustomData;

                if (psCallBackMessage->u8CommandId == E_CLD_IDENTIFY_CMD_IDENTIFY)
                {
                    DBG_vPrintf(TRACE_ZCL, "\nEP E_CLD_IDENTIFY_CMD_IDENTIFY");
                    /* provide callback to BDB handler for identify query response on initiator*/
                    if(psEvent->psClusterInstance->bIsServer == FALSE)
                    {
                        tsBDB_ZCLEvent  sBDBZCLEvent;
                        DBG_vPrintf(TRACE_ZCL, "\nCallBackBDB");
                        sBDBZCLEvent.eType = BDB_E_ZCL_EVENT_IDENTIFY_QUERY;
                        sBDBZCLEvent.psCallBackEvent = psEvent;
                        BDB_vZclEventHandler(&sBDBZCLEvent);
                    }
                    else
                    {
                        if(sSensor.sIdentifyServerCluster.u16IdentifyTime == 0)
                        {
                            vStopBlinkTimer();
                        }
                    }

                }
                else if((psCallBackMessage->u8CommandId == E_CLD_IDENTIFY_CMD_TRIGGER_EFFECT) &&
                        psEvent->psClusterInstance->bIsServer)
                {
                    DBG_vPrintf(TRACE_ZCL, "Trigger Effect ID %d Vr %d\r\n",
                            psCallBackMessage->uMessage.psTriggerEffectRequestPayload->eEffectId,
                            psCallBackMessage->uMessage.psTriggerEffectRequestPayload->u8EffectVarient);
                }
            }
            break;
		 #ifdef CLD_OTA
            case OTA_CLUSTER_ID:
            {
                 tsOTA_CallBackMessage *psCallBackMessage = (tsOTA_CallBackMessage *)psEvent->uMessage.sClusterCustomMessage.pvCustomData;
                 vHandleAppOtaClient(psCallBackMessage);
				 DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL:APP_bPersistantPolling=%d,eEventId=%d,eOTA_GetState()=%d,bOTA_IsWaitToUpgrade()=%d\n", 
				 											APP_bPersistantPolling, psCallBackMessage->eEventId, eOTA_GetState(), bOTA_IsWaitToUpgrade());
				#if 1 //moon
				if((APP_bPersistantPolling != TRUE) &&
				     (((psCallBackMessage->eEventId == E_CLD_OTA_COMMAND_QUERY_NEXT_IMAGE_RESPONSE) || (psCallBackMessage->eEventId == E_CLD_OTA_INTERNAL_COMMAND_POLL_REQUIRED)) ||
				      (eOTA_GetState() == OTA_IDLE) || bOTA_IsWaitToUpgrade()))
				{
					//DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: Stopped Poll");
					DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: No Stopped Poll");
					//ZTIMER_eStop(u8TimerPoll);
				}
				#endif
            }
            break;
        #endif

        case GENERAL_CLUSTER_ID_GROUPS:
            DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: for groups cluster");
            break;

        case 0x1000:
            DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: - for 0x1000");
            break;

        default:
            DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: for unknown cluster %d", psEvent->uMessage.sClusterCustomMessage.u16ClusterId);
            break;
        }
        break;

    case E_ZCL_CBET_CLUSTER_UPDATE:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: Update Id %04x", psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum);
        if (psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum == GENERAL_CLUSTER_ID_IDENTIFY)
        {
            vAPP_ZCL_DeviceSpecific_UpdateIdentify();
        }
        break;

	case E_ZCL_CBET_WRITE_ATTRIBUTES:
		DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: E_ZCL_CBET_WRITE_ATTRIBUTES ");		
        if ((psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum == GENERAL_CLUSTER_ID_TIME) && 
			(psEvent->psClusterInstance->psClusterDefinition->psAttributeDefinition->u16AttributeEnum == E_CLD_TIME_ATTR_ID_TIME))
        {
			#if 0		
				uint16 i = 0, size = psEvent->pZPSevent->uEvent.sApsDataIndEvent.hAPduInst->u16Size;
				uint8 *palyload = psEvent->pZPSevent->uEvent.sApsDataIndEvent.hAPduInst->au8Storage;
				DBG_vPrintf(TRACE_ZCL, "\nHandleAttributesWrite size = %d, au8Storage = ", size);
				for(i; i < size; i++)
				{
					DBG_vPrintf(TRACE_ZCL, "%02x ",  *palyload++);				
				}
				DBG_vPrintf(TRACE_ZCL, "\n");
			#endif
			// Ricky
			App_StopSerialPrepareEnterSleep();
			#if 0
			vZCL_SetUTCTime(sSensor.sTimeServerCluster.utctTime);
			DBG_vPrintf(TRACE_ZCL, "\n WriteTime %04x,%04x",sSensor.sTimeServerCluster.utctTime,u32ZCL_GetUTCTime());
			App_SerialSendTimeSynchronizer(u32ZCL_GetUTCTime());
			#else
			zutctime zb_utctTime = 0;
			uint8 *pzb_utctTime = psEvent->pZPSevent->uEvent.sApsDataIndEvent.hAPduInst->au8Storage, j = 0;
			pzb_utctTime += 6;
			for(j = 0; j < 4; j++)
			{
				zb_utctTime = zb_utctTime | (((zutctime)*pzb_utctTime) << (8 * j));
				DBG_vPrintf(TRACE_ZCL, "\npzb_utctTime=%x, %x", zb_utctTime, *pzb_utctTime);				
				pzb_utctTime++;
			}
			vZCL_SetUTCTime(zb_utctTime);
			App_SerialSendTimeSynchronizer(zb_utctTime);
			u8RetryTimeSynchronizationTimes = ~0;
			ZTIMER_eStop(u8TimerRetryTimeSynchronization);//moon
			#endif

			vAppSampleDoorLockPollChange(10);
			vStartPollTimer(ZTIMER_TIME_MSEC(500));
		}
		else if(psEvent->pZPSevent->uEvent.sApsDataIndEvent.u16ClusterId == 0x0101 || psEvent->pZPSevent->uEvent.sApsDataIndEvent.u16ClusterId == 0x0000)
		{	
			DBG_vPrintf(TRACE_ZCL, "\n HandleAttributesWrite %x ", psEvent->pZPSevent->uEvent.sApsDataIndEvent.u16ClusterId);
			App_Handle_User_Write_Attribute(psEvent->pZPSevent->uEvent.sApsDataIndEvent.hAPduInst->au8Storage, psEvent->pZPSevent->uEvent.sApsDataIndEvent.hAPduInst->u16Size);
		}
		else
		{
			DBG_vPrintf(TRACE_ZCL, "\n No HandleAttributesWrite %x ", psEvent->pZPSevent->uEvent.sApsDataIndEvent.u16ClusterId);
		}
		break;

    default:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EP EVT Invalid event type 0x%04x", psEvent->eEventType);
        break;
    }

}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
