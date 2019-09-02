/*****************************************************************************
 *
 * MODULE:             JN-AN-1220
 *
 * COMPONENT:          app_ota_client.c
 *
 * DESCRIPTION:        OTA Client Implementation
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5169,
 * JN5179, JN5189].
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
 * Copyright NXP B.V. 2016-2018. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

#include "dbg.h"
#include "PDM.h"
#include "PDM_IDs.h"
#include <string.h>


#include "pdum_gen.h"


#include "OTA.h"
#include "zcl_options.h"


#include "app_ota_client.h"
#include "app_zlo_sensor_node.h"
#include "app_common.h"
#include "rnd_pub.h"

#ifdef LTOSensor
    #include "App_LTOSensor.h"
#endif

#ifdef LightSensor
    #include "App_LightSensor.h"
#endif

#ifdef OccupancySensor
    #include "App_OccupancySensor.h"
    #include "app_zcl_sensor_task.h"
#endif
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#ifndef DEBUG_APP_OTA
    #define TRACE_APP_OTA               FALSE
    #define TRACE_APP_OTA_STATE         FALSE
	#define OTA_LNT  					FALSE
#else
    #define TRACE_APP_OTA               TRUE
    #define TRACE_APP_OTA_STATE         TRUE
	#define OTA_LNT 					TRUE	
#endif

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void         vInitAndDisplayKeys(void);
PRIVATE bool_t       bMatchReceived(void);
PRIVATE teZCL_Status eClientQueryNextImageRequest(
                     uint8 u8SourceEndpoint,
                     uint8 u8DestinationEndpoint,
                     uint16 u16DestinationAddress,
                     uint32 u32FileVersion,
                     uint16 u16HardwareVersion,
                     uint16 u16ImageType,
                     uint16 u16ManufacturerCode,
                     uint8 u8FieldControl);
PRIVATE ZPS_teStatus eSendOTAMatchDescriptor(uint16 u16ProfileId);
PRIVATE void         vManageOTAState(void);
PRIVATE void         vGetIEEEAddress( void);
PRIVATE void         vOTAPersist(void);
PRIVATE void         vResetOTADiscovery(void);
PRIVATE void         vManageDLProgressState(void);
PRIVATE uint8        u8VerifyLinkKey(tsOTA_CallBackMessage *psCallBackMessage);
PUBLIC void          vDumpFlashData(uint32 u32FlashByteLocation, uint32 u32EndLocation);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
tsOTA_PersistedData sOTA_PersistedData;

/* mac address */
PUBLIC    uint8 au8MacAddress[]  __attribute__ ((section (".ro_mac_address"))) = {
                  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/* Pre-configured Link Key */
PUBLIC uint8 s_au8LnkKeyArray[16] __attribute__ ((section (".ro_se_lnkKey"))) =
    { 0x5a, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6c, 0x6c, 0x69, 0x61, 0x6e, 0x63, 0x65, 0x30, 0x39 };

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE teOTA_State eOTA_State = OTA_FIND_SERVER;
PRIVATE uint32      u32OTARetry;
PRIVATE bool_t      bWaitUgradeTime;
PRIVATE uint32      u32OtaTimerMs;
PRIVATE uint32      u32OtaTimeoutMs;

PRIVATE uint8 au8MacAddressVolatile[8];
PRIVATE tsNvmDefs sNvmDefs;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
extern PUBLIC void vStartPollTimer(uint32 u32PollInterval);
extern PUBLIC uint8 u8AppOtaControler;
extern PUBLIC bool_t bBDBJoinFailed;
extern PUBLIC uint8 APP_OTArestart(uint8 cmd);
/****************************************************************************
 *
 * NAME: vAppInitOTA
 *
 * DESCRIPTION:
 * Initialises application OTA client related data structures and calls
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vAppInitOTA(void)
{
    teZCL_Status eZCL_Status;
    uint8 au8CAPublicKey[22];
    uint8 u8CurrentImageSector;

    uint8 u8SrcEp = app_u8GetDeviceEndpoint();
    vInitAndDisplayKeys();

    #if (defined OTA_INTERNAL_STORAGE)
        uint8 u8StartSector[1] = {8};
        sNvmDefs.u32SectorSize = 32*1024; /* Sector Size = 32K*/
        sNvmDefs.u8FlashDeviceType = E_FL_CHIP_INTERNAL;
    #else
        uint8 u8StartSector[1] = {0};
        sNvmDefs.u32SectorSize = 64*1024; /* Sector Size = 64K*/
        sNvmDefs.u8FlashDeviceType = E_FL_CHIP_ST_M25P20_A; // E_FL_CHIP_AUTO works in 69
    #endif



    eZCL_Status = eOTA_UpdateClientAttributes(u8SrcEp,0);
    if (eZCL_Status != E_ZCL_SUCCESS)
    {
        DBG_vPrintf(TRACE_APP_OTA, "eOTA_UpdateClientAttributes returned error 0x%x\n", eZCL_Status);
    }

    u8CurrentImageSector = u32REG_SysRead(REG_SYS_FLASH_REMAP) & 0xf;

    if (u8CurrentImageSector)
    {
    	vREG_SysWrite(REG_SYS_FLASH_REMAP,  0xfedcba98);
        #ifndef JN5168
        	vREG_SysWrite(REG_SYS_FLASH_REMAP2, 0x76543210);
        #endif
    }

    vOTA_FlashInit(NULL,&sNvmDefs);

    #if (defined OTA_INTERNAL_STORAGE)
        eZCL_Status = eOTA_AllocateEndpointOTASpace(
                                u8SrcEp,
                                u8StartSector,
                                OTA_MAX_IMAGES_PER_ENDPOINT,
								8,                                 // max sectors per image
                                FALSE,
                                au8CAPublicKey);
    #else
        eZCL_Status = eOTA_AllocateEndpointOTASpace(
                                u8SrcEp,
                                u8StartSector,
                                OTA_MAX_IMAGES_PER_ENDPOINT,
                                4,
                                FALSE,
                                au8CAPublicKey);
    #endif


    if (eZCL_Status != E_ZCL_SUCCESS)
    {
        DBG_vPrintf(TRACE_APP_OTA, "eAllocateEndpointOTASpace returned error 0x%x", eZCL_Status);
    }
    else
    {

        #if TRACE_APP_OTA
            tsOTA_ImageHeader          sOTAHeader;
            eOTA_GetCurrentOtaHeader(u8SrcEp,FALSE,&sOTAHeader);
            DBG_vPrintf(TRACE_APP_OTA,"\n\nCurrent Image Details \n");
            DBG_vPrintf(TRACE_APP_OTA,"File ID = 0x%08x\n",sOTAHeader.u32FileIdentifier);
            DBG_vPrintf(TRACE_APP_OTA,"Header Ver ID = 0x%04x\n",sOTAHeader.u16HeaderVersion);
            DBG_vPrintf(TRACE_APP_OTA,"Header Length ID = 0x%04x\n",sOTAHeader.u16HeaderLength);
            DBG_vPrintf(TRACE_APP_OTA,"Header Control Field = 0x%04x\n",sOTAHeader.u16HeaderControlField);
            DBG_vPrintf(TRACE_APP_OTA,"Manufac Code = 0x%04x\n",sOTAHeader.u16ManufacturerCode);
            DBG_vPrintf(TRACE_APP_OTA,"Image Type = 0x%04x\n",sOTAHeader.u16ImageType);
            DBG_vPrintf(TRACE_APP_OTA,"File Ver = 0x%08x\n",sOTAHeader.u32FileVersion);
            DBG_vPrintf(TRACE_APP_OTA,"Stack Ver = 0x%04x\n",sOTAHeader.u16StackVersion);
            DBG_vPrintf(TRACE_APP_OTA,"Image Len = 0x%08x\n\n\n",sOTAHeader.u32TotalImage);
        #endif

        eZCL_Status = eOTA_RestoreClientData( u8SrcEp, &sOTA_PersistedData, 1);
        DBG_vPrintf(TRACE_APP_OTA,"OTA PDM Status = %d \n",eZCL_Status);
        DBG_vPrintf(TRACE_APP_OTA, "Valid %d Nwk Addr %04x Ieee %016llx\n",
                sDeviceDesc.bValid, sDeviceDesc.u16NwkAddrOfServer, sDeviceDesc.u64IeeeAddrOfServer);
        if (sDeviceDesc.bValid == TRUE)
        {
            if(sOTA_PersistedData.sAttributes.u8ImageUpgradeStatus == E_CLD_OTA_STATUS_DL_IN_PROGRESS)
                eOTA_State = OTA_DL_PROGRESS;
            else
                eOTA_State = OTA_QUERYIMAGE;
            u32OtaTimerMs = 0;
            u32OtaTimeoutMs = OTA_INIT_TIME_MS;
            DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: QUERYIMAGE, %d, vAppInitOTA()", u32OtaTimeoutMs);
            DBG_vPrintf(TRACE_APP_OTA, "Start With Server\n");
        }
        else
        {
            eOTA_State = OTA_FIND_SERVER;
            u32OtaTimerMs = 0;
            u32OtaTimeoutMs = OTA_INIT_TIME_MS;
            DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: FIND_SERVER, %d, vAppInitOTA()", u32OtaTimeoutMs);
            DBG_vPrintf(TRACE_APP_OTA,  "Start No Server\n");
        }
        u32OTARetry = 0;
    }
    bWaitUgradeTime = FALSE;
}
/****************************************************************************
 *
 * NAME: vLoadOTAPersistedData
 *
 * DESCRIPTION:
 * Loads back OTA persisted data from PDM in application at start up.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vLoadOTAPersistedData(void)
{
    /*Restore OTA Persistent Data*/
    PDM_teStatus eStatusOTAReload;
    uint16 u16ByteRead;

    eStatusOTAReload = PDM_eReadDataFromRecord(PDM_ID_OTA_DATA,
                &sOTA_PersistedData,
                sizeof(tsOTA_PersistedData), &u16ByteRead);

    DBG_vPrintf(TRACE_APP_OTA,"eStatusOTAReload=%d size %d\n",eStatusOTAReload, sizeof(tsOTA_PersistedData) );

    if(sOTA_PersistedData.u32RequestBlockRequestTime != 0)
    {
        sOTA_PersistedData.u32RequestBlockRequestTime = 1;  //
        DBG_vPrintf(TRACE_APP_OTA, "Set block time to 1\n");
    }
    /*Make retries 0*/
    sOTA_PersistedData.u8Retry = 0;
}

/****************************************************************************
 *
 * NAME: vRunAppOTAStateMachine
 *
 * DESCRIPTION:
 * Timely checks for the OTA upgrade when the device state is running.
 * This is called from a timer ticking at rate of 1sec
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vRunAppOTAStateMachine(uint32 u32TimeMs)
{
	DBG_vPrintf(TRACE_APP_OTA, "\nStartl vRunAppOTAStateMachine u8AppOtaControler=%d, bBDBJoinFailedr=%d\n", u8AppOtaControler, bBDBJoinFailed);
	if(u8AppOtaControler == 1 && bBDBJoinFailed == 0)
	{
		//DBG_vPrintf(TRACE_APP_OTA_STATE, " vRunAppOTAStateMachine 1\n");
	    if( E_RUNNING == sDeviceDesc.eNodeState)
	    {
	        /* Update timer */
	        u32OtaTimerMs += u32TimeMs;
	        /* Process OTA state */
	        vManageOTAState();
	    }
	}
}

/****************************************************************************
 *
 * NAME: bOTADeepSleepAllowed
 *
 * DESCRIPTION:
 * Checks to see if deep sleep is allowed
 *
 * RETURNS:
 * FALSE - if in a wait or downloading state
 * TRUE  - otherwise and an attempt to discover has been completed (even if failed)
 *
 ****************************************************************************/
PUBLIC bool_t bOTADeepSleepAllowed(void)
{
    bool_t bReturn = FALSE;

    /* Node is starting up ? */
    if( E_STARTUP == sDeviceDesc.eNodeState)
    {
        /* Allow deep sleep - we don't care about OTA at this point */
        bReturn = TRUE;
    }
    /* In an idle state not wiating to upgrade ? */
    else if (eOTA_State == OTA_IDLE && bWaitUgradeTime == FALSE)
    {
        /* Allow sleep if we have attempted a discovery */
        bReturn = TRUE;
    }

    return bReturn;
}

/****************************************************************************
 *
 * NAME: vHandleAppOtaClient
 *
 * DESCRIPTION:
 * Handles the OTA Cluster Client events.
 * This is called from the EndPoint call back in the application
 * when an OTA event occurs.
 *
 * INPUT:
 * tsOTA_CallBackMessage *psCallBackMessage Pointer to cluster callback message
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandleAppOtaClient(tsOTA_CallBackMessage *psCallBackMessage)
{
    tsZCL_Address sZCL_Address;

    if(psCallBackMessage->eEventId == E_CLD_OTA_COMMAND_QUERY_NEXT_IMAGE_RESPONSE )
    {
        DBG_vPrintf(TRACE_APP_OTA,"\n\n\nQuery Next Image Response \n");
        //u32OTARetry = 0;//jabin remove
        if (sDeviceDesc.bValid)
        {
            if (psCallBackMessage->uMessage.sQueryImageResponsePayload.u8Status != E_ZCL_SUCCESS)
            {
                DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"Query Status 0x%02x\n", psCallBackMessage->uMessage.sQueryImageResponsePayload.u8Status);
                /* Got Response, nothing there, wait for idle period */
                //eOTA_State = OTA_IDLE;
                eOTA_State = OTA_QUERYIMAGE_WAIT;//jabin OTA_IDLE->OTA_QUERYIMAGE_WAIT
                u32OtaTimerMs = 0;
                u32OtaTimeoutMs = OTA_IDLE_TIME_MS;
                DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vHandleAppOtaClient(QNIRsp != SUCC %d)", u32OtaTimeoutMs, psCallBackMessage->uMessage.sQueryImageResponsePayload.u8Status);
            }
            else
            {
                /* there is an image, image check to follow */
                DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"Image Size 0x%08x\n",psCallBackMessage->uMessage.sQueryImageResponsePayload.u32ImageSize);
                DBG_vPrintf(TRACE_APP_OTA,"File Ver  0x%08x \n",psCallBackMessage->uMessage.sQueryImageResponsePayload.u32FileVersion);
                DBG_vPrintf(TRACE_APP_OTA,"Manufacture Code 0x%04x  \n",psCallBackMessage->uMessage.sQueryImageResponsePayload.u16ManufacturerCode);
                DBG_vPrintf(TRACE_APP_OTA,"Image Type 0x%04x\n\n\n",psCallBackMessage->uMessage.sQueryImageResponsePayload.u16ImageType);
                /* Re-time query image state awaiting image check */
                u32OTARetry = 0;//jabin
                eOTA_State = OTA_QUERYIMAGE;
                u32OtaTimerMs = 0;
                u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
                DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: QUERYIMAGE, %d, vHandleAppOtaClient(QNIRsp == SUCC %d)", u32OtaTimeoutMs, psCallBackMessage->uMessage.sQueryImageResponsePayload.u8Status);
            }
        }
        else
        {
            eOTA_State = OTA_FIND_SERVER;
            u32OtaTimerMs = 0;
            u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
            DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "Image Response while lost server\n");
            DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: FIND_SERVER, %d, vHandleAppOtaClient(QNIRsp no server)", u32OtaTimeoutMs);
        }
    }

    if(psCallBackMessage->eEventId == E_CLD_OTA_INTERNAL_COMMAND_VERIFY_IMAGE_VERSION )
    {
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"\n\nVerify the version \n");
        DBG_vPrintf(TRACE_APP_OTA,"Current Ver = 0x%08x\n",psCallBackMessage->uMessage.sImageVersionVerify.u32CurrentImageVersion );
        DBG_vPrintf(TRACE_APP_OTA,"Notified Ver = 0x%08x\n",psCallBackMessage->uMessage.sImageVersionVerify.u32NotifiedImageVersion);

        if (psCallBackMessage->uMessage.sImageVersionVerify.u32CurrentImageVersion ==
            psCallBackMessage->uMessage.sImageVersionVerify.u32NotifiedImageVersion    )
        {
                psCallBackMessage->uMessage.sImageVersionVerify.eImageVersionVerifyStatus =E_ZCL_FAIL;
                /* go back to waiting */
                DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "UNWANTED IMAGE\n");
                eOTA_State = OTA_IDLE;
                u32OtaTimerMs = 0;
                u32OtaTimeoutMs = OTA_IDLE_TIME_MS;
                DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vHandleAppOtaClient(verify unwanted)", u32OtaTimeoutMs);
        }
        else
        {
            /* down load about to start */
            eOTA_State = OTA_DL_PROGRESS;
            u32OtaTimerMs = 0;
            u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
            DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: DL_PROGRESS, %d, vHandleAppOtaClient(verify wanted)", u32OtaTimeoutMs);
            DBG_vPrintf(OTA_LNT, "Accept Image\n");
        }
    }

    #ifdef OTA_STRING_COMPARE
        if(psCallBackMessage->eEventId == E_CLD_OTA_INTERNAL_COMMAND_VERIFY_STRING )
        {
            int i=0;
            DBG_vPrintf(TRACE_APP_OTA, "Status = %d\n",psCallBackMessage->uMessage.sImageIdentification.eStatus);

            DBG_vPrintf(TRACE_APP_OTA, "Current Header String:\n");
            for(i=0;i<32;i++)
                DBG_vPrintf(TRACE_APP_OTA, "%c ",*(psCallBackMessage->uMessage.sImageIdentification.pu8Current +i));

            DBG_vPrintf(TRACE_APP_OTA, "\nNotified Header String: \n");
            for(i=0;i<32;i++)
                DBG_vPrintf(TRACE_APP_OTA, "%c ",*(psCallBackMessage->uMessage.sImageIdentification.puNotified +i));

            if(psCallBackMessage->uMessage.sImageIdentification.eStatus == E_ZCL_FAIL)
            {
                DBG_vPrintf(TRACE_APP_OTA,  "Mismatch\n\n\n");
                eOTA_State = OTA_IDLE;
                u32OtaTimerMs = 0;
                u32OtaTimeoutMs = OTA_IDLE_TIME_MS;
                DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vHandleAppOtaClient(string mismatch)", u32OtaTimeoutMs);
            }
        }
    #endif

    if(psCallBackMessage->eEventId == E_CLD_OTA_COMMAND_UPGRADE_END_RESPONSE )
    {
    	DBG_vPrintf(1," Upgrade End Response 1 \n");
        //DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"\n\n\nUpgrade End Response \n");
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"Upgrade Time : 0x%08x\n",psCallBackMessage->uMessage.sUpgradeResponsePayload.u32UpgradeTime);
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"Current Time : 0x%08x\n",psCallBackMessage->uMessage.sUpgradeResponsePayload.u32CurrentTime);
        DBG_vPrintf(TRACE_APP_OTA,"File Version : 0x%08x\n",psCallBackMessage->uMessage.sUpgradeResponsePayload.u32FileVersion);
        DBG_vPrintf(TRACE_APP_OTA,"Image Type   : 0x%04x\n",psCallBackMessage->uMessage.sUpgradeResponsePayload.u16ImageType);
        DBG_vPrintf(TRACE_APP_OTA,"Manufacturer : 0x%04x\n",psCallBackMessage->uMessage.sUpgradeResponsePayload.u16ManufacturerCode);
        if(0xFFFFFFFF != psCallBackMessage->uMessage.sUpgradeResponsePayload.u32UpgradeTime )
        {
            if(psCallBackMessage->uMessage.sUpgradeResponsePayload.u32UpgradeTime > psCallBackMessage->uMessage.sUpgradeResponsePayload.u32CurrentTime)
            {
                psCallBackMessage->uMessage.sUpgradeResponsePayload.u32UpgradeTime -= psCallBackMessage->uMessage.sUpgradeResponsePayload.u32CurrentTime;
                DBG_vPrintf(OTA_LNT, "FUTURE UPGRADE %d\n", psCallBackMessage->uMessage.sUpgradeResponsePayload.u32UpgradeTime);
            }
            else
            {
                /* If upgrade time is in past , upgrade in one second*/
                psCallBackMessage->uMessage.sUpgradeResponsePayload.u32UpgradeTime = 5;
            }
            psCallBackMessage->uMessage.sUpgradeResponsePayload.u32CurrentTime = 0;
            DBG_vPrintf(OTA_LNT, "DO IT IN 5 SECOND\n");
			vStartPollTimer(500);
			vAppSampleDoorLockPollChange(psCallBackMessage->uMessage.sUpgradeResponsePayload.u32UpgradeTime * 2 + 2);//moon;
        }
        bWaitUgradeTime = TRUE;
        /* Don't allow deep sleeping while waiting for timer */
        if(sOTA_PersistedData.sAttributes.u32FileOffset != 0)
        {
        	eOTA_State = OTA_DL_PROGRESS;
        }
        u32OtaTimerMs = 0;
		#if 1
		u32OtaTimeoutMs = 0;//moon
		#else
        if ((psCallBackMessage->uMessage.sUpgradeResponsePayload.u32UpgradeTime * 1000) > OTA_BUSY_TIME_MS)
        {
            /* Wait upgrade time plus busy time */
            u32OtaTimeoutMs = (psCallBackMessage->uMessage.sUpgradeResponsePayload.u32UpgradeTime * 1000) + OTA_BUSY_TIME_MS;
        }
        else
        {
            /* Wait busy time */
            u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
        }
		#endif
        
        //APP_OTArestart(1);//jabin
        DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: DL_PROGRESS, %d, vHandleAppOtaClient(EndRsp)", u32OtaTimeoutMs);
    }

    if(psCallBackMessage->eEventId == E_CLD_OTA_INTERNAL_COMMAND_SAVE_CONTEXT )
    {
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"\nSave Context\n");
        vOTAPersist();
    }

    if(psCallBackMessage->eEventId == E_CLD_OTA_INTERNAL_COMMAND_SWITCH_TO_UPGRADE_DOWNGRADE )
    {
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"\nSwitching to New Image\n");
    }

    if(psCallBackMessage->eEventId ==  E_CLD_OTA_INTERNAL_COMMAND_OTA_DL_ABORTED)
    {
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"DL complete INvalid Image\n\n");
        eOTA_State = OTA_IDLE;
        u32OtaTimerMs = 0;
        u32OtaTimeoutMs = OTA_IDLE_TIME_MS;
        DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vHandleAppOtaClient(aborted)", u32OtaTimeoutMs);
    }
    if(psCallBackMessage->eEventId ==  E_CLD_OTA_INTERNAL_COMMAND_RESET_TO_UPGRADE)
    {
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"E_CLD_OTA_INTERNAL_COMMAND_RESET_TO_UPGRADE\n\n");
    }
    if(psCallBackMessage->eEventId ==  E_CLD_OTA_COMMAND_UPGRADE_END_RESPONSE)
    {
    	DBG_vPrintf(1," Upgrade End Response 2 \n");
    	APP_OTArestart(1);//jabin
        //DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"E_CLD_OTA_COMMAND_UPGRADE_END_RESPONSE\n\n");
    }
    if(psCallBackMessage->eEventId == E_CLD_OTA_COMMAND_BLOCK_RESPONSE)
    {

        eOTA_State = OTA_DL_PROGRESS;
        u32OtaTimerMs = 0;
        u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
        DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: DL_PROGRESS, %d, vHandleAppOtaClient(BlkRsp)", u32OtaTimeoutMs);
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT, "Block Resp Status = %02x file offset %d\n",
                psCallBackMessage->uMessage.sImageBlockResponsePayload.u8Status,
                psCallBackMessage->uMessage.sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u32FileOffset);
        if(psCallBackMessage->uMessage.sImageBlockResponsePayload.u8Status == OTA_STATUS_SUCCESS)
        {
            psCallBackMessage->uMessage.sImageBlockResponsePayload.u8Status = u8VerifyLinkKey(psCallBackMessage);
            if(OTA_STATUS_ABORT == psCallBackMessage->uMessage.sImageBlockResponsePayload.u8Status)
            {
                /* Send an Abort from Application, the ZCL is not doing it.*/
                tsOTA_UpgradeEndRequestPayload sUpgradeEndRequestPayload;

                sUpgradeEndRequestPayload.u32FileVersion = psCallBackMessage->uMessage.sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u32FileVersion;
                sUpgradeEndRequestPayload.u16ImageType = psCallBackMessage->uMessage.sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u16ImageType;
                sUpgradeEndRequestPayload.u16ManufacturerCode = psCallBackMessage->uMessage.sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u16ManufacturerCode;
                sUpgradeEndRequestPayload.u8Status = psCallBackMessage->uMessage.sImageBlockResponsePayload.u8Status;
#if OTA_ACKS_ON == TRUE
                sZCL_Address.eAddressMode = E_ZCL_AM_SHORT;
#else
                sZCL_Address.eAddressMode = E_ZCL_AM_SHORT_NO_ACK;
#endif
                sZCL_Address.uAddress.u16DestinationAddress = psCallBackMessage->sPersistedData.u16ServerShortAddress;

                eOTA_ClientUpgradeEndRequest(
                        app_u8GetDeviceEndpoint(),                                         // 0uint8 u8SourceEndpoint,
                         psCallBackMessage->sPersistedData.u8DstEndpoint,       // uint8  u8DestinationEndpoint,
                         &sZCL_Address,                                         // tsZCL_Address
                         &sUpgradeEndRequestPayload);
            }
        }
    }
    if(psCallBackMessage->eEventId ==  E_CLD_OTA_INTERNAL_COMMAND_OTA_DL_ABORTED)
    {
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"E_CLD_OTA_INTERNAL_COMMAND_OTA_DL_ABORTED\n\n");
    }
}

/****************************************************************************
 *
 * NAME: vDumpFlashData
 *
 * DESCRIPTION:
 * Dumps the OTA downloaded data on debug terminal at a chunk of 16 bytes
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vDumpFlashData(uint32 u32FlashByteLocation, uint32 u32EndLocation)
{
    uint8 au8Data[16];

    for (;u32FlashByteLocation<=u32EndLocation;u32FlashByteLocation +=16 )
    {
        bAHI_FullFlashRead(u32FlashByteLocation,16,au8Data);

        uint8 u8Len;
        DBG_vPrintf(TRACE_APP_OTA,"0x%08x : ",u32FlashByteLocation);
        for (u8Len=0;u8Len<16;u8Len++)
        {
            DBG_vPrintf(TRACE_APP_OTA,"%02x ",au8Data[u8Len]);
        }
        DBG_vPrintf(TRACE_APP_OTA,"\n");

        volatile uint32 u32Delay=10000;
        for(u32Delay=10000;u32Delay>0;u32Delay--);

        vAHI_WatchdogRestart();
    }
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: u8VerifyLinkKey
 *
 * DESCRIPTION:
 * Verifies link key once first 1K data is downloaded and saved in flash
 *
 * RETURNS:
 * staus of the verification
 *
 ****************************************************************************/
PRIVATE uint8 u8VerifyLinkKey(tsOTA_CallBackMessage *psCallBackMessage)
{
    static bool_t bKeyVerified = FALSE;

    //DBG_vPrintf(TRACE_APP_OTA, "Block Resp Offset = %d \n",psCallBackMessage->uMessage.sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u32FileOffset);
    //DBG_vPrintf(TRACE_APP_OTA, "Flash_start = 0x%08x\n",&_flash_start);
    //DBG_vPrintf(TRACE_APP_OTA, "LinkKey Location = 0x%08x\n",(uint32)&FlsLinkKey);

    /*Assumption : First 1 K downloaded and saved to external flash*/
    if ((bKeyVerified == FALSE)
    &&  (psCallBackMessage->uMessage.sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u32FileOffset > APP_OTA_OFFSET_WRITEN_BEFORE_LINKKEY_VERIFICATION ))
    {
        bKeyVerified = TRUE;

        volatile int i;
        uint8 au8DownloadedLnkKey[0x10];
        uint8 au8Key[0x10];

        uint32 u32LnkKeyLocation = (uint32)(&(_FlsLinkKey)) - (uint32)(&(_flash_start));
        #ifdef OTA_INTERNAL_STORAGE
            #if (defined JENNIC_CHIP_FAMILY_JN516x) || (defined JENNIC_CHIP_FAMILY_JN517x)
                u32LnkKeyLocation += psCallBackMessage->u8ImageStartSector[0] * sNvmDefsStruct.u32SectorSize;
            #else
                /* For JN518x a segment has the same definition as a Page in this implementation
                   1 Page = 32 FLASH words = 32 * 16 = 512 Bytes AHI uses 32K sector sizes
                  (32*1024)/512 gives a factor of 64 */
                u32LnkKeyLocation += sNvmDefsStruct.u32SectorSize * psCallBackMessage->u8ImageStartSector[0]  * 64;
            #endif
        #endif
        DBG_vPrintf(TRACE_APP_OTA,"Link Key Offset in External Flash = 0x%08x\n",u32LnkKeyLocation);
        bAHI_FullFlashRead(u32LnkKeyLocation,0x10,au8DownloadedLnkKey);

        /*Get a copy of the Lnk Key*/
        for(i=0;i<16;i++)
        {
            au8Key[i] = s_au8LnkKeyArray[i];
        }

        #ifndef OTA_INTERNAL_STORAGE
        {
            extern uint32 _enc_start;
            /* for internal storage of the image the key will never be encrypted
             * as the downloaded image is decrypted before placing it in the internal flash
             */
            uint32 u32CustomerSettings = *((uint32*)FL_INDEX_SECTOR_CUSTOMER_SETTINGS);
            bool_t bEncExternalFlash = (u32CustomerSettings & FL_INDEX_SECTOR_ENC_EXT_FLASH) ? FALSE : TRUE;

            if(bEncExternalFlash)
            {
                uint8 au8Iv[0x10];
                uint8 au8DataOut[0x10];

                uint32 u32IVLocation = 0x10;
                tsReg128 sKey;

                /*Get the downloaded IV from Ext Flash */
                bAHI_FullFlashRead(u32IVLocation,0x10,au8Iv);
                DBG_vPrintf(TRACE_APP_OTA,"The Plain IV :\n");

                for (i=0;i<0x10;i++)
                    DBG_vPrintf(TRACE_APP_OTA,"au8Iv[%d]: 0x%02x\n",i,au8Iv[i]);

                DBG_vPrintf(TRACE_APP_OTA,"The Enc Offset Address = 0x%08x\n",((uint32)(&(_enc_start))));
                uint32 u32LnkKeyFromEncStart =  ((uint32)(&(_FlsLinkKey)) - ((uint32)(&(_enc_start))));
                DBG_vPrintf(TRACE_APP_OTA,"The The Total Bytes between Enc Offset and LnkKey = 0x%08x\n", u32LnkKeyFromEncStart );
                uint8 u8IVIncrement = (uint8)(u32LnkKeyFromEncStart>>4);
                DBG_vPrintf(TRACE_APP_OTA,"The IV should be increased by = 0x%08x\n", u8IVIncrement);

                /*Increase IV*/
                au8Iv[15] = au8Iv[15] + u8IVIncrement;

                /*Get the EFUSE keys*/
                uint32 *pu32KeyPtr = (uint32*)FL_INDEX_SECTOR_ENC_KEY_ADDRESS;

                sKey.u32register0 = *pu32KeyPtr;
                sKey.u32register1 = *(pu32KeyPtr+1);
                sKey.u32register2 = *(pu32KeyPtr+2);
                sKey.u32register3 = *(pu32KeyPtr+3);

                DBG_vPrintf(TRACE_APP_OTA,"The Key is :\n");
                DBG_vPrintf(TRACE_APP_OTA,"sKey.u32register0: 0x%08x\n",sKey.u32register0);
                DBG_vPrintf(
,"sKey.u32register1: 0x%08x\n",sKey.u32register1);
                DBG_vPrintf(TRACE_APP_OTA,"sKey.u32register2: 0x%08x\n",sKey.u32register2);
                DBG_vPrintf(TRACE_APP_OTA,"sKey.u32register3: 0x%08x\n",sKey.u32register3);

                /*Encrypt the IV*/
                vOTA_EncodeString(&sKey,au8Iv,au8DataOut);

                /* Encrypt the Internal Key */
                for(i=0;i<16;i++)
                {
                    au8Key[i] = au8Key[i] ^ au8DataOut[i];
                }
            }
        }
        #endif

        /*Comparing the Keys*/
        for(i=0;i<0x10;i++)
        {
            DBG_vPrintf(TRACE_APP_OTA,"Internal Key[%d] = %02x Downloaded Key[%d] = 0x%02x \n",i,au8Key[i],i,au8DownloadedLnkKey[i]);

            /*Compare to see if they match else its an invalid image*/
            if(au8Key[i]!=au8DownloadedLnkKey[i])
            {
                DBG_vPrintf(TRACE_APP_OTA,"Key Mismatch, Abort DownLoad\n");
                bKeyVerified=FALSE;
                return OTA_STATUS_ABORT;
            }
        }

        #ifdef OTA_CLIENT_ABORT_TH
            /*First Link key check will fail  if the TH is defined to test Client initiated Aborts.*/
            volatile static bool_t bClientAbort = TRUE;

            if(TRUE == bClientAbort)
            {
                bClientAbort = FALSE;
                return OTA_STATUS_ABORT;
            }
        #endif
    }

    return OTA_STATUS_SUCCESS;
}

/****************************************************************************
 *
 * NAME: vInitAndDisplayKeys
 *
 * DESCRIPTION:
 * Initialize Keys and displays the content
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vInitAndDisplayKeys(void)
{
	/*ZPS_vSetOverrideLocalMacAddress((uint64 *)&au8MacAddress);*/
	volatile uint8 u8Index;
	for(u8Index=0;u8Index<8;u8Index++)
		au8MacAddressVolatile[u8Index] = au8MacAddress[u8Index];

    #if TRACE_APP_OTA==TRUE
        uint8 i;
        DBG_vPrintf(TRACE_APP_OTA, "MAC Address at address = %08x\n\n\n",au8MacAddress);
        for (i =0;i<8;i++)
        {
        	DBG_vPrintf(TRACE_APP_OTA, "%02x ",au8MacAddress[i] );
        }
        DBG_vPrintf(TRACE_APP_OTA, "\n\n\n Link Key ");
        for (i =0;i<16;i++)
        {
            DBG_vPrintf(TRACE_APP_OTA, "%02x ",s_au8LnkKeyArray[i] );
        }
        DBG_vPrintf(TRACE_APP_OTA, "\n\n\n");
    #endif
}

/****************************************************************************
 *
 * NAME: vHandleMatchDescriptor
 *
 * DESCRIPTION:
 * Checks for the OTA cluster match during OTA server discovery, if a match
 * found it will make an entry in the local discovery table.This table will be
 * used to query image requests by the client.
 *
 *
 * INPUT:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandleMatchDescriptor( ZPS_tsAfEvent  * psStackEvent)
{
    ZPS_tsAplZdpMatchDescRsp sMatchDescRsp;
    uint32 u32Location;
    PDUM_thAPduInstance hAPduInst = psStackEvent->uEvent.sApsDataIndEvent.hAPduInst;
    uint16 u16ShortAddress;

    u32Location = 1;   // first loc has seq no
    sMatchDescRsp.u8Status = ((pdum_tsAPduInstance*)hAPduInst)->au8Storage[u32Location++];
    if (sMatchDescRsp.u8Status == 0)
    {
        APDU_BUF_READ16_INC(sMatchDescRsp.u16NwkAddrOfInterest,hAPduInst, u32Location);
        sMatchDescRsp.u8MatchLength = ((pdum_tsAPduInstance*)hAPduInst)->au8Storage[u32Location++];
        if(psStackEvent->uEvent.sApsDataIndEvent.u8SrcAddrMode == ZPS_E_ADDR_MODE_SHORT)
        {
            u16ShortAddress = psStackEvent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr;
        }
        else
        {
            u16ShortAddress = ZPS_u16NwkNibFindNwkAddr(ZPS_pvAplZdoGetNwkHandle(),psStackEvent->uEvent.sApsDataIndEvent.uSrcAddress.u64Addr);
        }
        if(sMatchDescRsp.u16NwkAddrOfInterest == ZPS_E_BROADCAST_RX_ON)
        {

            sMatchDescRsp.u16NwkAddrOfInterest = u16ShortAddress;
        }

        DBG_vPrintf(TRACE_APP_OTA,  "Match Ep Count %d\n", sMatchDescRsp.u8MatchLength);
        if(sMatchDescRsp.u8MatchLength != 0)
        {

            sDeviceDesc.u8OTAserverEP = ((pdum_tsAPduInstance*)hAPduInst)->au8Storage[u32Location];
            sDeviceDesc.u16NwkAddrOfServer = sMatchDescRsp.u16NwkAddrOfInterest;
            DBG_vPrintf(TRACE_APP_OTA,"\n\nNwk Address oF server = %04x\n", sDeviceDesc.u16NwkAddrOfServer);

            DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,"OTA Server EP# = %d\n", sDeviceDesc.u8OTAserverEP);

            eOTA_State = OTA_IEEE_LOOK_UP;
            u32OtaTimerMs = 0;
            u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
            DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IEEE_LOOKUP, %d, vHandleMatchDescriptor()", u32OtaTimeoutMs);
        }
    }
}

/****************************************************************************
 *
 * NAME: vHandleIeeeAddressRsp
 *
 * DESCRIPTION:
 * Handles IEEE address look up query query
 * Makes an entry in the application OTA discovery table. Later this is used
 * for by the OTA requests.
 *
 * This function is called from the application OTA handler with stack event
 * as input.
 *
 *
 * INPUT:
 * ZPS_tsAfEvent  * psStackEvent   Pointer to the stack event
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vHandleIeeeAddressRsp( ZPS_tsAfEvent  * psStackEvent)
{
    ZPS_tsAplZdpIeeeAddrRsp sIeeeAddrRsp;
    uint32 u32Location = 1;                     // first location has seq no, so skip
    PDUM_thAPduInstance hAPduInst = psStackEvent->uEvent.sApsDataIndEvent.hAPduInst;

    sIeeeAddrRsp.u8Status = ((pdum_tsAPduInstance*)hAPduInst)->au8Storage[u32Location++];
    DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "IEEE Rsp Status %02x ", sIeeeAddrRsp.u8Status);

    if (sIeeeAddrRsp.u8Status == 0)
    {
        u32Location += PDUM_u16APduInstanceReadNBO(hAPduInst, u32Location, "l",
                                                   &sIeeeAddrRsp.u64IeeeAddrRemoteDev);
        APDU_BUF_READ16_INC(sIeeeAddrRsp.u16NwkAddrRemoteDev,hAPduInst, u32Location);

        DBG_vPrintf(TRACE_APP_OTA,  "Addr of sever %04x Addr Rem Dev %04x\n ",
            sDeviceDesc.u16NwkAddrOfServer,
            sIeeeAddrRsp.u16NwkAddrRemoteDev);
        if( sDeviceDesc.u16NwkAddrOfServer == sIeeeAddrRsp.u16NwkAddrRemoteDev)
        {
            /*Make an entry in the OTA server tables*/
            sDeviceDesc.u64IeeeAddrOfServer = sIeeeAddrRsp.u64IeeeAddrRemoteDev;
            DBG_vPrintf(TRACE_APP_OTA|1,"Entry Added NWK Addr 0x%04x IEEE Addr 0x%016llx",
                    sDeviceDesc.u16NwkAddrOfServer, sDeviceDesc.u64IeeeAddrOfServer);
            ZPS_eAplZdoAddAddrMapEntry( sDeviceDesc.u16NwkAddrOfServer,
                    sDeviceDesc.u64IeeeAddrOfServer,
                                        FALSE);
            sDeviceDesc.bValid = TRUE;
            PDM_eSaveRecordData(PDM_ID_APP_SENSOR,&sDeviceDesc,sizeof(tsDeviceDesc));
            eOTA_SetServerAddress( app_u8GetDeviceEndpoint(), sDeviceDesc.u64IeeeAddrOfServer, sDeviceDesc.u16NwkAddrOfServer
                            );

            eOTA_State = OTA_QUERYIMAGE;
            u32OtaTimerMs = 0;
            u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
            DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: QUERYIMAGE, %d, vHandleIeeeAddressRsp()", u32OtaTimeoutMs);
            u32OTARetry = 0;
            DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "SERVER DISCOVERED\n");
        }
        else
        {
            DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "NOT VALID\n" );
        }
    }
    else
    {
        DBG_vPrintf(TRACE_APP_OTA,  "IEEE req failed\n");
    }
}
/****************************************************************************
 *
 * NAME: vGetIEEEAddress
 *
 * DESCRIPTION:
 * Finds an IEEE address on the local node by calling Stack API, if no entries
 * found it request the IEEE look up on air.
 *
 *
 * INPUT:
 * uint8 u8Index   Index to the discovery table point to the NWK address of
 *                 the discovered server
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vGetIEEEAddress( void)
{
    /* Always query for address Over the air - Uncomment if required for local look up first*/

    /* If there is no address map existing, then do a look up */
    PDUM_thAPduInstance hAPduInst;
    hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

    if (hAPduInst == PDUM_INVALID_HANDLE)
    {
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT, "IEEE Address Request - PDUM_INVALID_HANDLE\n");
    }
    else
    {
        ZPS_tuAddress uDstAddr;
        bool bExtAddr;
        uint8 u8SeqNumber;
        ZPS_teStatus eStatus;
        ZPS_tsAplZdpIeeeAddrReq sZdpIeeeAddrReq;

        uDstAddr.u16Addr = sDeviceDesc.u16NwkAddrOfServer;
        bExtAddr = FALSE;
        sZdpIeeeAddrReq.u16NwkAddrOfInterest = sDeviceDesc.u16NwkAddrOfServer;
        sZdpIeeeAddrReq.u8RequestType = 0;
        sZdpIeeeAddrReq.u8StartIndex = 0;

        eStatus= ZPS_eAplZdpIeeeAddrRequest(
                                        hAPduInst,
                                        uDstAddr,
                                        bExtAddr,
                                        &u8SeqNumber,
                                        &sZdpIeeeAddrReq);
        if (eStatus)
        {
            PDUM_eAPduFreeAPduInstance(hAPduInst);
            DBG_vPrintf(TRACE_APP_OTA|OTA_LNT, "Address Request failed: 0x%02x\n", eStatus);
        }
    }
}

/****************************************************************************
 *
 * NAME: bMatchReceived
 *
 * DESCRIPTION:
 * Validation function for the match and sets the valid field true in the
 * discovery table if there is valid OTA Server found in the network
 *
 *
 * INPUT:
 *
 * RETURNS:
 * TRUE/FALSE
 *
 ****************************************************************************/
PRIVATE bool_t bMatchReceived(void)
{
    return (sDeviceDesc.bValid == TRUE)? TRUE: FALSE;
}

/****************************************************************************
 *
 * NAME: eClientQueryNextImageRequest
 *
 * DESCRIPTION:
 * Query nest image request application wrapper.
 *
 *
 * INPUT:
 *    uint8 u8SourceEndpoint
 *  uint8 u8DestinationEndpoint,
 *    uint16 u16DestinationAddress,
 *  uint32 u32FileVersion,
 *  uint16 u16HardwareVersion,
 *  uint16 u16ImageType,
 *  uint16 u16ManufacturerCode,
 *  uint8 u8FieldControl
 *
 * RETURNS:
 * ZCL status of the call
 *
 ****************************************************************************/
PRIVATE teZCL_Status eClientQueryNextImageRequest(
                    uint8 u8SourceEndpoint,
                    uint8 u8DestinationEndpoint,
                    uint16 u16DestinationAddress,
                    uint32 u32FileVersion,
                    uint16 u16HardwareVersion,
                    uint16 u16ImageType,
                    uint16 u16ManufacturerCode,
                    uint8 u8FieldControl)
{
    teZCL_Status eStatus;
    tsZCL_Address sAddress;
    sAddress.eAddressMode = E_ZCL_AM_SHORT;
    sAddress.uAddress.u16DestinationAddress = u16DestinationAddress;

    tsOTA_QueryImageRequest sRequest;
    sRequest.u32CurrentFileVersion = u32FileVersion;
    sRequest.u16HardwareVersion = u16HardwareVersion;
    sRequest.u16ImageType = u16ImageType;
    sRequest.u16ManufacturerCode = u16ManufacturerCode;
    sRequest.u8FieldControl = u8FieldControl;

    eStatus = eOTA_ClientQueryNextImageRequest(
            u8SourceEndpoint,
            u8DestinationEndpoint,
            &sAddress,
            &sRequest);

    DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "query Next ret %02x\n", eStatus);
    return eStatus;
}

/****************************************************************************
 *
 * NAME: eSendOTAMatchDescriptor
 *
 * DESCRIPTION:
 * Sends the OTA match descriptor for OTA server discovery as a broadcast.
 *
 *
 * INPUT:
 *  uint16 u16ProfileId Profile Identifier
 *
 * RETURNS:
 * ZPS status of the call
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eSendOTAMatchDescriptor(uint16 u16ProfileId)
{
    uint16 au16InClusters[]={OTA_CLUSTER_ID};
    uint8 u8TransactionSequenceNumber;
    ZPS_tuAddress uDestinationAddress;
    ZPS_tsAplZdpMatchDescReq sMatch;

    sMatch.u16ProfileId = u16ProfileId;
    sMatch.u8NumInClusters=sizeof(au16InClusters)/sizeof(uint16);
    sMatch.pu16InClusterList=au16InClusters;
    sMatch.pu16OutClusterList=NULL;
    sMatch.u8NumOutClusters=0;
    sMatch.u16NwkAddrOfInterest=0xFFFD;

    uDestinationAddress.u16Addr = 0xFFFD;

    PDUM_thAPduInstance hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

    if (hAPduInst == PDUM_INVALID_HANDLE)
    {
        DBG_vPrintf(TRACE_APP_OTA, "Allocate PDU ERR:\n");
        return (ZPS_teStatus)PDUM_E_INVALID_HANDLE;
    }

    ZPS_teStatus eStatus = ZPS_eAplZdpMatchDescRequest(
                            hAPduInst,
                            uDestinationAddress,
                            FALSE,
                            &u8TransactionSequenceNumber,
                            &sMatch);

    if (eStatus)
    {
        PDUM_eAPduFreeAPduInstance(hAPduInst);
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT, "Match ERR: 0x%x", eStatus);
    }
    else
    {
        DBG_vPrintf(OTA_LNT, "MATCH D Sent OK\n");
    }

    return eStatus;
}

/****************************************************************************
 *
 * NAME: vManageOTAState
 *
 * DESCRIPTION:
 * Simple State Machine to move the OTA state from Dicovery to Download.
 * It alos implements a simple mechanism of time out.
 *
 * INPUT:
 * uint32 u32OTAQueryTime
 *
 * RETURNS:
 *
 *
 ****************************************************************************/
PRIVATE void vManageOTAState(void)
{
    ZPS_teStatus eStatus;
    uint8 u8SrcEp = app_u8GetDeviceEndpoint();
	
	DBG_vPrintf(TRACE_APP_OTA, "eOTA_State=%d\n", eOTA_State);	
	DBG_vPrintf(TRACE_APP_OTA, "u32OtaTimerMs=%d, u32OtaTimeoutMs=%d\n", u32OtaTimerMs, u32OtaTimeoutMs);

	if(bWaitUgradeTime != TRUE)//moon
	{
		vAppSampleDoorLockPollChange(120);//moon;
		vStartPollTimer(500);//jabin 300->500
	}
   
    switch(eOTA_State)
    {
        case OTA_IDLE:
        {
            if(u32OtaTimerMs >= u32OtaTimeoutMs)
            {
                //if (sDeviceDesc.bValid == TRUE)
                if(1)
                {
                    eOTA_State = OTA_QUERYIMAGE;
                    u32OtaTimerMs = 0;
                    u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
                    DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: QUERYIMAGE, %d, vManageOTAState(IDLE to)", u32OtaTimeoutMs);
                    DBG_vPrintf(TRACE_APP_OTA, "Idle start with server\n");
                }
                else
                {
                    eOTA_State = OTA_FIND_SERVER;
                    u32OtaTimerMs = 0;
                    u32OtaTimeoutMs = OTA_INIT_TIME_MS;
                    DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: FIND_SERVER, %d, vManageOTAState(IDLE to)", u32OtaTimeoutMs);
                    DBG_vPrintf(TRACE_APP_OTA,  "Idle start without server\n");
                }
            }
        }
        break;

        case OTA_FIND_SERVER:
        {
            if(u32OtaTimerMs >= u32OtaTimeoutMs)
            {
                ZPS_teStatus eStatus;
                eStatus = eSendOTAMatchDescriptor(HA_PROFILE_ID);
                if ( eStatus)
                {
                    eOTA_State = OTA_IDLE;
                    u32OtaTimerMs = 0;
                    //u32OtaTimeoutMs = OTA_IDLE_TIME_MS;//nxp                  
                    u32OtaTimeoutMs = OTA_BUSY_TIME_MS;//moon
                    DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vManageOTAState(FIND_SERVER to)", u32OtaTimeoutMs);
                    vResetOTADiscovery();
                    DBG_vPrintf(TRACE_APP_OTA|OTA_LNT, "Send Match Error 0x%02x\n",eStatus);
                }
                else
                {
                    /*Wait for the discovery to complete */
                    eOTA_State = OTA_FIND_SERVER_WAIT;
                    u32OtaTimerMs = 0;
                    u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
                    DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: FIND_SERVER_WAIT, %d, vManageOTAState(FIND_SERVER to)", u32OtaTimeoutMs);
                    DBG_vPrintf(OTA_LNT, "Wait Server Rsp\n");
                }
            }
        }
        break;

        case OTA_FIND_SERVER_WAIT:
        {
            if( bMatchReceived() )
            {
                eOTA_State = OTA_QUERYIMAGE;
                u32OtaTimerMs = 0;
                u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
                DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: QUERYIMAGE, %d, vManageOTAState(FIND_SERVER_WAIT to)", u32OtaTimeoutMs);
                u32OTARetry = 0;
            }
            else if(u32OtaTimerMs >= u32OtaTimeoutMs)
            {
                u32OTARetry = 0;
                eOTA_State = OTA_IDLE;
                u32OtaTimerMs = 0;
                u32OtaTimeoutMs = OTA_IDLE_TIME_MS;
                DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vManageOTAState(FIND_SERVER_WAIT to)", u32OtaTimeoutMs);
                DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "MATCH TIME OUT\n");
				
				AppDisableuOtaControler();//moon
            }
        }
        break;

        case OTA_IEEE_LOOK_UP:
        {
            if(u32OtaTimerMs >= u32OtaTimeoutMs)
            {

                vGetIEEEAddress();
                eOTA_State = OTA_IEEE_WAIT;
                u32OtaTimerMs = 0;
                u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
                DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IEEE_WAIT, %d, vManageOTAState(IEEE_LOOK_UP to)", u32OtaTimeoutMs);
            }
        }
        break;

        case OTA_IEEE_WAIT:
        {
            if( bMatchReceived() )
            {
                eOTA_State = OTA_QUERYIMAGE;
                u32OtaTimerMs = 0;
                u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
                DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: QUERYIMAGE, %d, vManageOTAState(IEEE_WAIT to)", u32OtaTimeoutMs);
                u32OTARetry = 0;
            }
            else if(u32OtaTimerMs >= u32OtaTimeoutMs)
            {
                eOTA_State = OTA_IDLE;
                u32OtaTimerMs = 0;
                u32OtaTimeoutMs = OTA_IDLE_TIME_MS;
                DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vManageOTAState(IEEE_WAIT to)", u32OtaTimeoutMs);
                DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "IEEE TIME OUT\n");
				
				AppDisableuOtaControler();//moon
            }

        }
        break;

        case OTA_QUERYIMAGE:
        {
            if(u32OtaTimerMs >= u32OtaTimeoutMs)
            {
                if(sDeviceDesc.bValid)
                {
                    tsOTA_ImageHeader          sOTAHeader;
                    eOTA_GetCurrentOtaHeader( u8SrcEp,FALSE,&sOTAHeader);
                    DBG_vPrintf(TRACE_APP_OTA,"\n\nFile ID = 0x%08x\n",sOTAHeader.u32FileIdentifier);
                    DBG_vPrintf(TRACE_APP_OTA,"Header Ver ID = 0x%04x\n",sOTAHeader.u16HeaderVersion);
                    DBG_vPrintf(TRACE_APP_OTA,"Header Length ID = 0x%04x\n",sOTAHeader.u16HeaderLength);
                    DBG_vPrintf(TRACE_APP_OTA,"Header Control Filed = 0x%04x\n",sOTAHeader.u16HeaderControlField);
                    DBG_vPrintf(TRACE_APP_OTA,"Manufac Code = 0x%04x\n",sOTAHeader.u16ManufacturerCode);
                    DBG_vPrintf(TRACE_APP_OTA,"Image Type = 0x%04x\n",sOTAHeader.u16ImageType);
                    DBG_vPrintf(TRACE_APP_OTA,"File Ver = 0x%08x\n",sOTAHeader.u32FileVersion);
                    DBG_vPrintf(TRACE_APP_OTA,"Stack Ver = 0x%04x\n",sOTAHeader.u16StackVersion);
                    DBG_vPrintf(TRACE_APP_OTA,"Image Len = 0x%08x\n\n",sOTAHeader.u32TotalImage);

                    /*Set server address */
                    eOTA_SetServerAddress(
                            u8SrcEp,
                            sDeviceDesc.u64IeeeAddrOfServer,
                            sDeviceDesc.u16NwkAddrOfServer);

                    eStatus = eClientQueryNextImageRequest(
                            u8SrcEp,
                            sDeviceDesc.u8OTAserverEP,
                            sDeviceDesc.u16NwkAddrOfServer,
                            sOTAHeader.u32FileVersion,
                            0,
                            sOTAHeader.u16ImageType,
                            sOTAHeader.u16ManufacturerCode,
                            0);

                    DBG_vPrintf(OTA_LNT, "Query Image Status Send %02x\n", eStatus);
                    eOTA_State = OTA_QUERYIMAGE_WAIT;
                    u32OtaTimerMs = 0;
                    u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
                    DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: QUERYIMAGE_WAIT, %d, vManageOTAState(QUERYIMAGE to)", u32OtaTimeoutMs);
                    DBG_vPrintf(1, "\r\nOTA_STATE: QUERYIMAGE_WAIT, %d, vManageOTAState(QUERYIMAGE to)", u32OtaTimeoutMs);
                }
                else
                {
                    DBG_vPrintf(TRACE_APP_OTA,  "OTA Server not valid %d\n", sDeviceDesc.bValid);
                    DBG_vPrintf(1,  "OTA Server not valid %d\n", sDeviceDesc.bValid);
                    u32OTARetry = 0;
                    eOTA_State = OTA_IDLE;
                    u32OtaTimerMs = 0;
                    u32OtaTimeoutMs = OTA_IDLE_TIME_MS;
                    DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vManageOTAState(QUERYIMAGE to)", u32OtaTimeoutMs);
					
					AppDisableuOtaControler();//moon
                }
            }
        }
        break;

        case OTA_QUERYIMAGE_WAIT:
        {
            if(u32OtaTimerMs >= u32OtaTimeoutMs)
            {
                DBG_vPrintf(TRACE_APP_OTA,  "IMAGE QUERY WAIT TIMEOUT -> ");
                u32OTARetry++;
                DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "Retry %d ", u32OTARetry);
                DBG_vPrintf(1,  "Retry %d ", u32OTARetry);
                if (u32OTARetry > 2)
                {
                    eOTA_State = OTA_IDLE;
                    u32OtaTimerMs = 0;
                    u32OtaTimeoutMs = OTA_IDLE_TIME_MS;
                    DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vManageOTAState(QUERYIMAGE_WAIT to)", u32OtaTimeoutMs);
                    u32OTARetry = 0;
                    vResetOTADiscovery();
                    DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "CLEAR OUT\n");

                    //APP_OTArestart(2);//jabin
					AppDisableuOtaControler();//moon
                }
                else
                {
                    eOTA_State = OTA_QUERYIMAGE;
                    u32OtaTimerMs = 0;
                    u32OtaTimeoutMs = OTA_BUSY_TIME_MS;
                    DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: QUERYIMAGE, %d, vManageOTAState(QUERYIMAGE_WAIT to)", u32OtaTimeoutMs);
                    DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "TRY AGAIN Timeout=%d\n", u32OtaTimeoutMs);
                }
            }
        }
        break;

        case OTA_DL_PROGRESS:
        {
            vManageDLProgressState();
        }
        break;

        default:
        {
            /* Do nothing */
            ;
        }
        break;
    }
}

/****************************************************************************
 *
 * NAME: vManageDLProgressState
 *
 * DESCRIPTION:
 * Manages the DL progress state, mainly to get the state machine out of it
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vManageDLProgressState(void)
{
#ifdef LightSensor
    u32OtaTimeoutMs = 30000; /* As light sensor state machine will try for block request 5 times */
#endif

    if (((u32OtaTimerMs >= u32OtaTimeoutMs) || (sOTA_PersistedData.sAttributes.u8ImageUpgradeStatus == E_CLD_OTA_STATUS_NORMAL))
    &&  (bWaitUgradeTime == FALSE))
    {
        eOTA_State = OTA_IDLE;
        u32OtaTimerMs = 0;
        u32OtaTimeoutMs = OTA_IDLE_TIME_MS;
        DBG_vPrintf(TRACE_APP_OTA_STATE, "\r\nOTA_STATE: IDLE, %d, vManageDLProgressState(to)", u32OtaTimeoutMs);
        DBG_vPrintf(TRACE_APP_OTA|OTA_LNT,  "OTA In Progress CLEAR OUT\n");
        DBG_vPrintf(1,  "OTA In Progress CLEAR OUT\n");
    }
}

/****************************************************************************
 *
 * NAME: vResetOTADiscovery
 *
 * DESCRIPTION:
 * Resets OTA discovery so that a fresh discovery is possible
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vResetOTADiscovery(void)
{
    sDeviceDesc.u64IeeeAddrOfServer = 0;
    sDeviceDesc.u16NwkAddrOfServer = 0xffff;
    sDeviceDesc.bValid = FALSE;
    PDM_eSaveRecordData(PDM_ID_APP_SENSOR,&sDeviceDesc,sizeof(tsDeviceDesc));
}

/****************************************************************************
 *
 * NAME: vOTAPersist
 *
 * DESCRIPTION:
 * Persists OTA data when called by the OTA client ebvent, this is required to
 * restore the down load in case of power failure.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/

PRIVATE void vOTAPersist(void)
{
    sOTA_PersistedData = sGetOTACallBackPersistdata();
    PDM_eSaveRecordData(PDM_ID_OTA_DATA, &sOTA_PersistedData, sizeof(tsOTA_PersistedData));
}

/****************************************************************************
 *
 * NAME: vOTAResetPersist
 *
 * DESCRIPTION:
 * clear and save persistant data related to the OTA record
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vOTAResetPersist(void)
{
    sOTA_PersistedData = sGetOTACallBackPersistdata();
    memset(&sOTA_PersistedData, 0, sizeof(tsOTA_PersistedData));
    PDM_eSaveRecordData(PDM_ID_OTA_DATA, &sOTA_PersistedData, sizeof(tsOTA_PersistedData));
}


PUBLIC teOTA_State eOTA_GetState(void)
{
   return eOTA_State;
}

PUBLIC bool_t bOTA_IsWaitToUpgrade(void)
{
   return bWaitUgradeTime;
}
/****************************************************************************
 *
 * NAME: vWarmStartOTA
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/

PUBLIC void vWarmStartOTA(void)
{
	vOTA_FlashInit(NULL,&sNvmDefs);

}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

