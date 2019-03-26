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
#include <appapi.h>
#include "ZTimer.h"
#include "app_main.h"
#include "pdum_apl.h"
#include "pdum_gen.h"
#include "pdm.h"
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
#include "ahi_aes.h"
#include "app_events.h"
#include "LightingBoard.h"
#include "app_zcl_tick_handler.h"
#include "app_zcl_sensor_task.h"
#include "app_blink_led.h"

#include "App_WL_DoorLock.h"
#include "app_reporting.h"
#include "app_occupancy_buttons.h"
#include "app_sleep_handler.h"//Ricky
#include "app_SampleDoorLock_Uart.h"
#include "app_nwk_event_handler.h"

#include "Bdb_api.h"

#ifdef CLD_OTA
    #include "app_ota_client.h"
#endif


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
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern PDM_tsRecordDescriptor sDevicePDDesc;
extern tsDeviceDesc sDeviceDesc;
extern uint8 crtreasknum;//δ�յ�response�ط�
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
    eZCL_Status = eApp_ZLO_RegisterEndpoint(&APP_ZCL_cbEndpointCallback);
    if (eZCL_Status != E_ZCL_SUCCESS)
    {
            DBG_vPrintf(TRACE_SENSOR_TASK, "\nAPP_ZCL: Error: eApp_ZLO_RegisterEndpoint:%d", eZCL_Status);
    }

    DBG_vPrintf(TRACE_SENSOR_TASK, "\nAPP_ZCL: Chan Mask %08x", ZPS_psAplAibGetAib()->apsChannelMask);
    DBG_vPrintf(TRACE_SENSOR_TASK, "\nAPP_ZCL: RxIdle TRUE");

    ZTIMER_eStart(u8TimerTick, ZCL_TICK_TIME);

    vAPP_ZCL_DeviceSpecific_Init();

#ifdef CLD_OTA	//Ricky 2017 0505
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
    /*
     * If the 1 second tick timer has expired, restart it and pass
     * the event on to ZCL
     */
    vAPP_ZCL_Tick();

//	if (u8NumerOfActiveTaskTimer())	//Ricky	
		ZTIMER_eStart(u8TimerTick, ZCL_TICK_TIME);
//	else
	{
//		DBG_vPrintf(TRACE_ZCL, "\n u8NumerOfActiveTaskTimer : %d ",u8NumerOfActiveTaskTimer());
//		ZTIMER_eStop(u8TimerTick);
	}

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

    case E_ZCL_CBET_DEFAULT_RESPONSE:	//Ricky Mark Default Response
        DBG_vPrintf(TRACE_ZCL, "\n APP_ZCL: EVT Default response");
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
#ifndef SB
//	#define SB
#endif
PRIVATE void APP_ZCL_cbEndpointCallback(tsZCL_CallBackEvent *psEvent)
{
	DBG_vPrintf(TRACE_ZCL,"\n APP_ZCL_cbEndpointCallback : %d ",psEvent->eEventType);

    switch (psEvent->eEventType)
    {

    case E_ZCL_CBET_REPORT_INDIVIDUAL_ATTRIBUTES_CONFIGURE:	//Ricky Mark Config Report
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
			//crtreasknum = 0;
			APP_RecvPollControlResponse();
		}
		App_SampleDoorLockCommandResend(psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum);//u16NumberOfAttributes

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
                eApp_ZLO_RegisterEndpoint(&APP_ZCL_cbEndpointCallback);
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

        case GENERAL_CLUSTER_ID_GROUPS:
            DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: for groups cluster");
            break;

	#ifdef CLD_OTA	//Ricky 2017 0505
		case OTA_CLUSTER_ID:
		{
			tsOTA_CallBackMessage *psCallBackMessage = (tsOTA_CallBackMessage *)psEvent->uMessage.sClusterCustomMessage.pvCustomData;
			vHandleAppOtaClient(psCallBackMessage);
		}
			break;
	#endif

	#ifdef BDB_SUPPORT_TOUCHLINK

		case COMMISSIONING_CLUSTER_ID_TL_COMMISSIONING :

			DBG_vPrintf(TRACE_ZCL, "\n COMMISSIONING_CLUSTER_ID_TL_COMMISSIONING");

	//		tsBDB_ZCLEvent sEvent;
			SensorTaskEvent.eType = BDB_E_ZCL_EVENT_TL_IPAN_MSG;
			SensorTaskEvent.psCallBackEvent = psEvent;
			BDB_vZclEventHandler(&SensorTaskEvent);
			break;
	#endif

		case CLOSURE_CLUSTER_ID_DOOR_LOCK:	//Ricky

		#ifdef SB
			vAppSampleDoorLockPollChange(10);
			vStartPollTimer(500);
		#endif
            DBG_vPrintf(TRACE_ZCL, "\n APP_ZCL: - for %0x \n",psEvent->uMessage.sClusterCustomMessage.u16ClusterId);
			App_SampleDoorLockCommandHandle(psEvent->pZPSevent->uEvent.sApsDataIndEvent.hAPduInst->au8Storage+1);

			break;
        default:
            DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: for unknown cluster %d", psEvent->uMessage.sClusterCustomMessage.u16ClusterId);
            break;
        }
        break;

	case E_ZCL_CBET_WRITE_ATTRIBUTES:

		DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: E_ZCL_CBET_WRITE_ATTRIBUTES ");		

        if ((psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum == GENERAL_CLUSTER_ID_TIME) && 
			(psEvent->psClusterInstance->psClusterDefinition->psAttributeDefinition->u16AttributeEnum == E_CLD_TIME_ATTR_ID_TIME))
        {
			// Ricky
			App_StopSerialPrepareEnterSleep();
			vZCL_SetUTCTime(sSensor.sTimeServerCluster.utctTime);
			DBG_vPrintf(TRACE_ZCL, "\n WriteTime %04x,%04x",sSensor.sTimeServerCluster.utctTime,u32ZCL_GetUTCTime());
			App_SerialSendTimeSynchronizer(u32ZCL_GetUTCTime());

			vAppSampleDoorLockPollChange(20);
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

    case E_ZCL_CBET_CLUSTER_UPDATE:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: Update Id %04x", psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum);
        if (psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum == GENERAL_CLUSTER_ID_IDENTIFY)
        {
            vAPP_ZCL_DeviceSpecific_UpdateIdentify();
        }
        break;

    default:
        DBG_vPrintf(TRACE_ZCL, "\nAPP_ZCL: EP EVT Invalid event type");
        break;
    }

}


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
