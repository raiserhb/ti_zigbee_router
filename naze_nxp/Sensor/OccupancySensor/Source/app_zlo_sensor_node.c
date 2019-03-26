/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_zlo_sensor_node.c
 *
 * DESCRIPTION:        ZLO Demo : Stack <-> Occupancy Sensor App Interaction
 *                     (Implementation)
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
#include "ZQueue.h"
#include "app_main.h"
#include "pdum_apl.h"
#include "pdum_gen.h"
#include "pdm.h"
#include "dbg.h"
#include "dbg_uart.h"
#include "pwrm.h"
#include "zps_gen.h"
#include "rnd_pub.h"
#include "app_common.h"
#include "app_blink_led.h"
#include "groups.h"
#include "PDM_IDs.h"

#include "app_zlo_sensor_node.h"
#include "app_zcl_sensor_task.h"
#include "app_zbp_utilities.h"
#include "app_events.h"
#include "zcl_customcommand.h"
#include "app_occupancy_sensor_state_machine.h"
#include "zcl_common.h"
#include "app_reporting.h"
#include "app_occupancy_buttons.h"
#include "app_sleep_handler.h"
#include "app_event_handler.h"
#include "app_nwk_event_handler.h"
#include "app_blink_led.h"
#include "bdb_api.h"
#include "bdb_fb_api.h"
#include "Bdb_options.h"
#include "bdb_start.h"


#ifdef JN517x
#include "AHI_ModuleConfiguration.h"
#endif

#include "app_SampleDoorLock_Uart.h"

#ifdef CLD_OTA
    #include "OTA.h"
    #include "app_ota_client.h"
#endif


/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_SENSOR_NODE
    #define TRACE_SENSOR_NODE   TRUE
#else
    #define TRACE_SENSOR_NODE   FALSE
#endif

#define WAKE_FROM_DEEP_SLEEP    (1<<11)


PUBLIC uint32 u32BattVoltage = 0;


/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void app_vStartNodeFactoryNew(void);
PRIVATE void app_vRestartNode (void);
PRIVATE void vAppHandleZdoEvents( BDB_tsZpsAfEvent *psZpsAfEvent);
PRIVATE uint8 app_u8GetSensorEndpoint( void);
PRIVATE void vDeletePDMOnButtonPress(uint8 u8ButtonDIO);
PRIVATE void vHandleZdoLeaveRequest(uint8 u8Action, uint64 u64TargetAddr, uint8 u8Flags);
PRIVATE void vAppHandleAfEvent( BDB_tsZpsAfEvent* psZpsAfEvent);
PUBLIC void APP_ADCReadBattreyVoltage(void);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
tsZllState sZllState = { FACTORY_NEW, E_STARTUP, 0 };

PUBLIC PDM_tsRecordDescriptor sDevicePDDesc;
PUBLIC  tsDeviceDesc           sDeviceDesc;
PUBLIC bool_t bBDBJoinFailed = FALSE;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
uint8 u8NoQueryCount = 0;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

#ifdef CLD_OTA
PUBLIC teNODE_STATES eGetNodeState(void)
{
    return sZllState.eNodeState;	//Ricky sZllState -> sDeviceDesc
}
#endif

/****************************************************************************
 *
 * NAME: APP_vInitialiseNode
 *
 * DESCRIPTION:
 * Initialises the application related functions
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vInitialiseNode(void)
{
    PDM_teStatus eStatusReportReload;
    DBG_vPrintf(TRACE_SENSOR_NODE, "\nAPP Sensor Node: APP_vInitialiseNode");

    APP_vInitLeds();

	APP_ADCReadBattreyVoltage();	//Ricky

    /*Initialise the application buttons*/
    APP_bButtonInitialise();

	APP_UARTInitialise();//	Ricky 2016 1222

#ifdef CLD_OTA	//Ricky 2017 0505
	vLoadOTAPersistedData();
#endif

    /* We need to get the previous state out off NVM and save it*/
//    sSensor.sOccupancySensingServerCluster.u8Occupancy = bGetPreSleepOccupancyState();
//    APP_vSetLED(LED1, sSensor.sOccupancySensingServerCluster.u8Occupancy);

    DBG_vPrintf(TRACE_SENSOR_NODE, "\nAPP Sensor Node: u16AHI_PowerStatus() = 0x%04x", u16AHI_PowerStatus());

    /* Restore any report data that is previously saved to flash */
    eStatusReportReload = eRestoreReports();
    uint16 u16ByteRead;
    PDM_eReadDataFromRecord(PDM_ID_APP_SENSOR,
                            &sDeviceDesc,
                            sizeof(tsDeviceDesc),
                            &u16ByteRead);

    /* Set security state */
    ZPS_vDefaultKeyInit();

#ifdef JN517x
    /* Default module configuration: change E_MODULE_DEFAULT as appropriate */
      vAHI_ModuleConfigure(E_MODULE_DEFAULT);
#endif
    /* Initialize ZBPro stack */
	DBG_vPrintf(TRACE_SENSOR_NODE, "\n ZPS_eAplAfInit : %0x ",ZPS_eAplAfInit());

	DBG_vPrintf(TRACE_SENSOR_NODE, "\n APP Sensor Node: APP_ZCL_vInitialise ");
    APP_ZCL_vInitialise();

    /* Set end device age out time to 11 days 9 hours & 4 mins */
	ZPS_bAplAfSetEndDeviceTimeout(ZED_TIMEOUT_UNDEFINED);	// Ricky

    /*Load the reports from the PDM or the default ones depending on the PDM load record status*/
    if(eStatusReportReload !=PDM_E_STATUS_OK )
    {
        /*Load Defaults if the data was not correct*/
        vLoadDefaultConfigForReportable();
    }
    /*Make the reportable attributes */
    vMakeSupportedAttributesReportable();

    /* If the device state has been restored from flash, re-start the stack
     * and set the application running again.
     */
    sBDB.sAttrib.u32bdbPrimaryChannelSet = BDB_PRIMARY_CHANNEL_SET;	//Ricky
    sBDB.sAttrib.u32bdbSecondaryChannelSet = BDB_SECONDARY_CHANNEL_SET;
    BDB_tsInitArgs sInitArgs;
    sInitArgs.hBdbEventsMsgQ = &APP_msgBdbEvents;

    BDB_vInit(&sInitArgs);
	DBG_vPrintf(TRACE_SENSOR_NODE, "\n BDB_vInit");

    if (APP_bNodeIsInRunningState())
    {
        app_vRestartNode();
        sBDB.sAttrib.bbdbNodeIsOnANetwork = TRUE;
	#ifdef CLD_OTA
		if (u32BattVoltage >= 26)
		{
			// Ricky OTA入口
			DBG_vPrintf(TRACE_SENSOR_NODE, "\n Start OTA ! ");
			u8OTAStartMachine = TRUE;
			sZllState.eNodeState = E_RUNNING;
			vRunAppOTAStateMachine();
		}
	#endif

    }
    else
    {
        DBG_vPrintf(TRACE_SENSOR_NODE, "\n Factory New Start");
        app_vStartNodeFactoryNew();
        sBDB.sAttrib.bbdbNodeIsOnANetwork = FALSE;
    }

    /*In case of a deep sleep device any button wake up would cause a PDM delete , only check for DIO8
     * pressed for deleting the context */
    if (FALSE == (u16AHI_PowerStatus() & WAKE_FROM_DEEP_SLEEP))
    {
//        vDeletePDMOnButtonPress(APP_BUTTONS_BUTTON_1);
    }

    /* Register callback that will handle ZDP (mgmt) leave requests */
    ZPS_vAplZdoRegisterZdoLeaveActionCallback(vHandleZdoLeaveRequest);

    #ifdef PDM_EEPROM
        vDisplayPDMUsage();
    #endif

    APP_vInitialiseTasks();
}


/****************************************************************************
 *
 * NAME: APP_vInitialiseTasks
 *
 * DESCRIPTION:
 * This is the main App Initialise.
 * Task that checks the power status and starts tasks based on those results
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vInitialiseTasks(void)
{
    DBG_vPrintf(TRACE_SENSOR_NODE, "\n APP Initialise Tasks: Power Status = %d", u16AHI_PowerStatus());

    if (APP_bNodeIsInRunningState())
    {
        vActionOnButtonActivationAfterDeepSleep();
    }
    else
    {
//    	DBG_vPrintf(TRACE_SENSOR_NODE, "\n APP Initialise Tasks: APP_JOINING_BLINK_TIME");
        /* We are factory new so start the blink timers*/
//        vStartBlinkTimer(APP_JOINING_BLINK_TIME);	// Ricky
    }
}

/****************************************************************************
 *
 * NAME: APP_vBdbCallback
 *
 * DESCRIPTION:
 * Callbak from the BDB
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vBdbCallback(BDB_tsBdbEvent *psBdbEvent)
{
    vAHI_WatchdogRestart(); // JV - Assuming bdb_taskBDB() context might take longer; especially during initialisation. ToDo - discuss with Richard towards removal from here

    switch(psBdbEvent->eEventType)
    {

    case BDB_EVENT_NONE:
        break;

    case BDB_EVENT_ZPSAF:                // Use with BDB_tsZpsAfEvent
        DBG_vPrintf(TRACE_SENSOR_NODE,"\n BDB: ZpsAF Event: %d\n", psBdbEvent->uEventData.sZpsAfEvent.sStackEvent.eType);
        vAppHandleAfEvent(&psBdbEvent->uEventData.sZpsAfEvent);
        break;

    case BDB_EVENT_INIT_SUCCESS:
        DBG_vPrintf(TRACE_SENSOR_NODE,"\n APP: BdbInitSuccessful\n");
        break;

    case BDB_EVENT_FAILURE_RECOVERY_FOR_REJOIN: //Recovery on rejoin failure
        DBG_vPrintf(TRACE_SENSOR_NODE,"\n BDB EVT Recovery On Rejoin Failure\n");
//		vHandleFailedRejoin();
		// Ricky 定时启动Rejoin

        break;

    case BDB_EVENT_REJOIN_FAILURE: // only for ZED
        DBG_vPrintf(TRACE_SENSOR_NODE, "\n BDB EVT INIT failed to rejoin\n");
        vHandleFailedToJoin();
        break;

    case BDB_EVENT_REJOIN_SUCCESS: // only for ZED	
        DBG_vPrintf(TRACE_SENSOR_NODE, "\n BDB EVT INIT Rejoin success\n");
        bBDBJoinFailed = FALSE;
        vHandleNetworkJoinEndDevice();
        break;

    case BDB_EVENT_NWK_STEERING_SUCCESS:	//Ricky 显示加网

        DBG_vPrintf(TRACE_SENSOR_NODE,"\n GoRunningState\n!");
        bBDBJoinFailed = FALSE;
        vHandleNetworkJoinAndRejoin();

		APP_SerialSendJoinIndication(0);
		vSendReportPowerConfig();

        break;

    case BDB_EVENT_NO_NETWORK:
        DBG_vPrintf(TRACE_SENSOR_NODE,"\n BDB EVT No Network!\n");
        vHandleFailedToJoin();

		APP_SerialSendJoinIndication(1);	//Ricky 
		
        break;

    case BDB_EVENT_NWK_JOIN_SUCCESS:
        DBG_vPrintf(TRACE_SENSOR_NODE,"\n BDB EVT Join Success!\n");
        break;

    case BDB_EVENT_APP_START_POLLING:	// Ricky ignore
        DBG_vPrintf(TRACE_SENSOR_NODE,"\n Start Polling!\n");
        /* Start fast seconds polling */
		vStartPollTimer(500);
        break;

    case BDB_EVENT_FB_HANDLE_SIMPLE_DESC_RESP_OF_TARGET:
        DBG_vPrintf(TRACE_SENSOR_NODE,"\n Simple descriptor %d %d %04x %04x %d \n",psBdbEvent->uEventData.psFindAndBindEvent->u8TargetEp,
                psBdbEvent->uEventData.psFindAndBindEvent->u16TargetAddress,
                psBdbEvent->uEventData.psFindAndBindEvent->u16ProfileId,
                psBdbEvent->uEventData.psFindAndBindEvent->u16DeviceId,
                psBdbEvent->uEventData.psFindAndBindEvent->u8DeviceVersion);
        break;

    case BDB_EVENT_FB_CHECK_BEFORE_BINDING_CLUSTER_FOR_TARGET:
        DBG_vPrintf(TRACE_SENSOR_NODE,"Check For Binding Cluster %d \n",psBdbEvent->uEventData.psFindAndBindEvent->uEvent.u16ClusterId);
        break;

    case BDB_EVENT_FB_CLUSTER_BIND_CREATED_FOR_TARGET:
        DBG_vPrintf(TRACE_SENSOR_NODE,"Bind Created for cluster %d \n",psBdbEvent->uEventData.psFindAndBindEvent->uEvent.u16ClusterId);
        break;

    case BDB_EVENT_FB_BIND_CREATED_FOR_TARGET:
        {
            DBG_vPrintf(TRACE_SENSOR_NODE,"Bind Created for target EndPt %d \n",psBdbEvent->uEventData.psFindAndBindEvent->u8TargetEp);
            u8NoQueryCount = 0;
            uint8 u8Seq;
            tsZCL_Address sAddress;
            tsCLD_Identify_IdentifyRequestPayload sPayload;

            sPayload.u16IdentifyTime = 0;
            sAddress.eAddressMode = E_ZCL_AM_SHORT_NO_ACK;
            sAddress.uAddress.u16DestinationAddress = psBdbEvent->uEventData.psFindAndBindEvent->u16TargetAddress;

            eCLD_IdentifyCommandIdentifyRequestSend(psBdbEvent->uEventData.psFindAndBindEvent->u8InitiatorEp,
                                                    psBdbEvent->uEventData.psFindAndBindEvent->u8TargetEp,
                                                    &sAddress,
                                                    &u8Seq,
                                                    &sPayload);
        }
        break;

    case BDB_EVENT_FB_GROUP_ADDED_TO_TARGET:
        DBG_vPrintf(TRACE_SENSOR_NODE,"Group Bind Created\n");
        break;

    case BDB_EVENT_FB_ERR_BINDING_TABLE_FULL:
        DBG_vPrintf(TRACE_SENSOR_NODE,"ERR: Bind Table Full\n");
        break;

    case BDB_EVENT_FB_ERR_BINDING_FAILED:
        DBG_vPrintf(TRACE_SENSOR_NODE,"ERR: Bind\n");
        break;

    case BDB_EVENT_FB_ERR_GROUPING_FAILED:
        DBG_vPrintf(TRACE_SENSOR_NODE,"ERR: Group\n");
        break;

    case BDB_EVENT_FB_NO_QUERY_RESPONSE:
        DBG_vPrintf(TRACE_SENSOR_NODE,"ERR: No Query response\n");
        //Example to stop further query repeating
        if(u8NoQueryCount >= 2)
        {
            u8NoQueryCount = 0;
            BDB_vFbExitAsInitiator();
        }
        else
        {
            u8NoQueryCount++;
        }
        break;

    case BDB_EVENT_FB_TIMEOUT:
        DBG_vPrintf(TRACE_SENSOR_NODE,"ERR: TimeOut\n");
        break;

    default:
        DBG_vPrintf(TRACE_SENSOR_NODE, "BDB EVT default evt %d\n", psBdbEvent->eEventType);
        break;

    }

}

/****************************************************************************
 *
 * NAME: APP_app_zlo_sensor_Task
 *
 * DESCRIPTION:
 * Checks to see what event has triggered the task to start and calls the
 * appropriate function. This handles App events, Stack events, timer activations
 * and manual activations.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_taskSensor(void)
{
	uint8 u8RxByte;
    APP_tsEvent sAppEvent;
    sAppEvent.eType = APP_E_EVENT_NONE;

    /*Collect the application events*/
    if (ZQ_bQueueReceive(&APP_msgAppEvents, &sAppEvent) == TRUE)
    {
        DBG_vPrintf(TRACE_SENSOR_NODE, "\nAPP ZLO Sensor Task: App Event %d", sAppEvent.eType);
#ifdef APP_NTAG
        /* Is this a button event on SW3 ? */
        if ( ( sAppEvent.eType == APP_E_EVENT_BUTTON_DOWN || sAppEvent.eType == APP_E_EVENT_BUTTON_UP )
                && (sAppEvent.uEvent.sButton.u8Button == APP_E_BUTTONS_NFC_FD) )
        {
            /* Always pass this on for processing */
            vAppHandleAppEvent(sAppEvent);
        }
        /* Other event (handle as normal) ? */
        else
#endif
        {
            if(bBDBJoinFailed)
            {
		//		vStartBlinkTimer(APP_JOINING_BLINK_TIME);	//Ricky 门锁不需要指示灯
                if(APP_bNodeIsInRunningState())
                {
                    // TODO kick BDB for rejoin
                    DBG_vPrintf(TRACE_SENSOR_NODE, "\n Call BDB vStart\n");
                    sBDB.sAttrib.bbdbNodeIsOnANetwork = TRUE;
                    BDB_vStart();
                }
                else
                {
                    //Retrigger the network steering as sensor is not part of a network
                    vAppHandleStartup();
                }
            }
            else
            {
                vAppHandleAppEvent(sAppEvent);
            }
        }
    }

	if ( TRUE == ZQ_bQueueReceive(&APP_msgSerialRx, &u8RxByte) )
	{

		if(u8RxByte == 1)
		{
			APP_SerialSendAcknowlegement(RxSerialMsg[3],0);
			DBG_vPrintf(TRACE_SENSOR_NODE, "\n Serial Success to Handle ");

			if (u8RejoinCycles)
				u8RejoinCycles = 2;
			vAppHandleSerialEvent();
	//		memset(RxSerialMsg,0,sizeof(RxSerialMsg));
		}
		else if (u8RxByte == 2)
		{
			APP_SerialSendAcknowlegement(RxSerialMsg[3],1);
			DBG_vPrintf(TRACE_SENSOR_NODE, "\n Serial Acknowlegement Error ");
		}
	}

}


/****************************************************************************
 *
 * NAME: vAppHandleAfEvent
 *
 * DESCRIPTION:
 * Handles AF Events.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vAppHandleAfEvent( BDB_tsZpsAfEvent *psZpsAfEvent)
{

    if (psZpsAfEvent->u8EndPoint == app_u8GetSensorEndpoint())
    {
        DBG_vPrintf(TRACE_SENSOR_NODE, "\nAPP ZLO Sensor Task: ZCL Event");
        APP_ZCL_vEventHandler( &psZpsAfEvent->sStackEvent);
    }
    else if (psZpsAfEvent->u8EndPoint == OCCUPANCYSENSOR_ZDO_ENDPOINT)
    {
        DBG_vPrintf(TRACE_SENSOR_NODE, "\nAPP ZLO Sensor Task: Main APP Handle");
        vAppHandleZdoEvents( psZpsAfEvent);
    }

    /* Ensure Freeing of Apdus */
    if (psZpsAfEvent->sStackEvent.eType == ZPS_EVENT_APS_DATA_INDICATION)
    {
        PDUM_eAPduFreeAPduInstance(psZpsAfEvent->sStackEvent.uEvent.sApsDataIndEvent.hAPduInst);
    }
    else if ( psZpsAfEvent->sStackEvent.eType == ZPS_EVENT_APS_INTERPAN_DATA_INDICATION )
    {
        PDUM_eAPduFreeAPduInstance(psZpsAfEvent->sStackEvent.uEvent.sApsInterPanDataIndEvent.hAPduInst);
    }

}


/****************************************************************************
 *
 * NAME: vHandleZdoLeaveRequest
 *
 * DESCRIPTION:
 * Callback that will handle ZDP (mgmt) leave requests
 *
 * RETURNS:
 * None
 *
 ****************************************************************************/
PRIVATE void vHandleZdoLeaveRequest(uint8 u8Action, uint64 u64TargetAddr, uint8 u8Flags)
{
    DBG_vPrintf(TRACE_SENSOR_NODE, "\n%s - Addr: 0x%016llx, Action: 0x%02x Flags: 0x%02x\n", __FUNCTION__, u64TargetAddr, u8Action, u8Flags);

    /* Check this request is for us */
    if ((u64TargetAddr == ZPS_u64AplZdoGetIeeeAddr()) || (u64TargetAddr == 0ULL))
    {
        /* We respond to NLME leave requests elsewhere.. */
        if (ZPS_LEAVE_ORIGIN_MGMT_LEAVE == u8Action)
        {
            /*what to do if rejoin is set to true i.e. u8Flags = 1....?*/
            if (0 == u8Flags)
            {
                /* No need to wait for ZPS_EVENT_NWK_LEAVE_CONFIRM event which will
                   The parent will do the needful */
                APP_vFactoryResetRecords();
                vAHI_SwReset();
            }
            else
            {
                /*No need to do anything because the stack will generate one rejoin -
                 * if that is success full its fine
                 * else the rejoin statemachine will take care of rejoining. */
            }
        }
    }
}

/****************************************************************************
 *
 * NAME: vDeletePDMOnButtonPress
 *
 * DESCRIPTION:
 * PDM context clearing on button press
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDeletePDMOnButtonPress(uint8 u8ButtonDIO)
{
    bool_t bDeleteRecords = FALSE;
    uint32 u32Buttons = u32AHI_DioReadInput() & (1 << u8ButtonDIO);
    (void)u32AHI_DioInterruptStatus();
    if (u32Buttons == 0)
    {
        bDeleteRecords = TRUE;
    }
    else
    {
        bDeleteRecords = FALSE;
    }
    /* If required, at this point delete the network context from flash, perhaps upon some condition
     * For example, check if a button is being held down at reset, and if so request the Persistent
     * Data Manager to delete all its records:
     * e.g. bDeleteRecords = vCheckButtons();
     * Alternatively, always call PDM_vDelete() if context saving is not required.
     */
    if(bDeleteRecords)
    {
        sBDB.sAttrib.bbdbNodeIsOnANetwork = FALSE;
        if (ZPS_E_SUCCESS !=  ZPS_eAplZdoLeaveNetwork(0, FALSE,FALSE)) {
            /* Leave failed, probably lost parent, so just reset everything */
            DBG_vPrintf(TRACE_SENSOR_NODE,"\nAPP Sensor Node: Deleting the PDM");
            APP_vFactoryResetRecords();
            vAHI_SwReset();
        }

    }
}



/****************************************************************************
 *
 * NAME: vAppHandleZdoEvents
 *
 * DESCRIPTION:
 * This is the main state machine which decides whether to call up the startup
 * or running function. This depends on whether we are in the network on not.
 *
 * PARAMETERS:
 * ZPS_tsAfEvent sAppStackEvent Stack event information.
 *
 ****************************************************************************/
PRIVATE void vAppHandleZdoEvents( BDB_tsZpsAfEvent *psZpsAfEvent)
{
#ifdef DEBUG_SENSOR_NODE
    //vDisplayNWKTransmitTable();
#endif

    /* Handle events depending on node state */
    switch (sDeviceDesc.eNodeState)
    {

    case E_STARTUP:
        vAppHandleStartup();
        break;

    case E_RUNNING:
        vAppHandleRunning( &(psZpsAfEvent->sStackEvent) );
        break;

    case E_JOINING_NETWORK:
            break;

    default:
        break;

    }

#ifdef CLD_OTA
        if ((psZpsAfEvent->sStackEvent.uEvent.sApsDataIndEvent.eStatus == ZPS_E_SUCCESS) &&
                (psZpsAfEvent->sStackEvent.uEvent.sApsDataIndEvent.u8DstEndpoint == 0))
        {
            // Data Ind for ZDp Ep
            if (ZPS_ZDP_MATCH_DESC_RSP_CLUSTER_ID == psZpsAfEvent->sStackEvent.uEvent.sApsDataIndEvent.u16ClusterId)
            {
                vHandleMatchDescriptor(&psZpsAfEvent->sStackEvent);
            } else if (ZPS_ZDP_IEEE_ADDR_RSP_CLUSTER_ID == psZpsAfEvent->sStackEvent.uEvent.sApsDataIndEvent.u16ClusterId) {
                vHandleIeeeAddressRsp(&psZpsAfEvent->sStackEvent);
            }
        }
#endif


}

/****************************************************************************
 * NAME: app_vRestartNode
 *
 * DESCRIPTION:
 * Start the Restart the ZigBee Stack after a context restore from
 * the EEPROM/Flash
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void app_vRestartNode (void)
{

    /* The node is in running state indicates that
     * the EZ Mode state is as E_EZ_DEVICE_IN_NETWORK*/

    DBG_vPrintf(TRACE_SENSOR_NODE, "\nAPP Non Factory New Start");

    ZPS_vSaveAllZpsRecords();
}


/****************************************************************************
 *
 * NAME: app_vStartNodeFactoryNew
 *
 * DESCRIPTION:
 * Start the ZigBee Stack for the first ever Time.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void app_vStartNodeFactoryNew(void)
{
    sBDB.sAttrib.u32bdbPrimaryChannelSet = BDB_PRIMARY_CHANNEL_SET;
    sBDB.sAttrib.u32bdbSecondaryChannelSet = BDB_SECONDARY_CHANNEL_SET;
    //BDB_eNsStartNwkSteering(); done later on  only if no OOB commissionning  ???

    sDeviceDesc.eNodeState = E_JOINING_NETWORK;
    /* Stay awake for joining */
    DBG_vPrintf(TRACE_SENSOR_NODE, "\nAPP Sensor Node: Factory New Start");
}

/****************************************************************************
 *
 * NAME: app_u8GetSensorEndpoint
 *
 * DESCRIPTION:
 * Return the application endpoint
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PRIVATE uint8 app_u8GetSensorEndpoint( void)
{
    return OCCUPANCYSENSOR_SENSOR_ENDPOINT;
}

/****************************************************************************
 *
 * NAME: APP_vFactoryResetRecords
 *
 * DESCRIPTION: reset application and stack to factory new state
 *              preserving the outgoing nwk frame counter
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vFactoryResetRecords(void)
{
    sDeviceDesc.eNodeState = E_STARTUP;

#ifdef CLD_OTA	//Ricky 20170505
		sZllState.bValid = FALSE;
		sZllState.u64IeeeAddrOfServer = 0;
		sZllState.u16NwkAddrOfServer = 0xffff;
		sZllState.u8OTAserverEP = 0xff;
		vOTAResetPersist();
#endif

    /* clear out the stack */
    ZPS_vDefaultStack();
    BDB_vSetKeys();

    /* save everything */
    PDM_eSaveRecordData(PDM_ID_APP_SENSOR,
                            &sDeviceDesc,
                            sizeof(tsDeviceDesc));

    ZPS_vSaveAllZpsRecords();
}


PUBLIC bool_t APP_bNodeIsInRunningState(void)
{
    DBG_vPrintf(TRACE_SENSOR_NODE, "\nAPP Sensor Node: NodeState=%d", sDeviceDesc.eNodeState);
    return (sDeviceDesc.eNodeState == E_RUNNING) ? TRUE:FALSE;
}

PUBLIC void APP_ADCReadBattreyVoltage(void)
{
	
	// local value placeholders
	uint16 u16BattVoltage = 0;

	vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE,
					 E_AHI_AP_INT_DISABLE,
					 E_AHI_AP_SAMPLE_8,
					 E_AHI_AP_CLOCKDIV_500KHZ,
					 E_AHI_AP_INTREF);

	/* spin on reg not enabled */
	while (!bAHI_APRegulatorEnabled());

	vAHI_AdcEnable(E_AHI_ADC_SINGLE_SHOT, 
					E_AHI_AP_INPUT_RANGE_2, 
					E_AHI_ADC_SRC_VOLT);	//enable adc

	// start sampling adc
	vAHI_AdcStartSample();	
	// keep polling until we get a value
	while(bAHI_AdcPoll());	

#if (JENNIC_CHIP_FAMILY == JN516x)
	u16BattVoltage = u16AHI_AdcRead() >> 2;
#endif

	u32BattVoltage = (u16BattVoltage *3600) >> 8;  /*  in Millivolts */
	u32BattVoltage /= 100;
    DBG_vPrintf(TRACE_SENSOR_NODE, "\n APP_ADCReadBattreyVoltage %d ",u32BattVoltage);

	vAHI_AdcDisable();

	if (u32BattVoltage < 25)
		sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = 1;
	else
		sSensor.sPowerConfigServerCluster.u32BatteryAlarmState = 0;

//	return u32BattVoltage;
}

#ifdef CLD_OTA
PUBLIC tsOTA_PersistedData sGetOTACallBackPersistdata(void)
{
    return sSensor.sCLD_OTA_CustomDataStruct.sOTACallBackMessage.sPersistedData;
}
#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
