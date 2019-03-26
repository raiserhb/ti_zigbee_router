/*****************************************************************************
 *
 * MODULE:             Over The Air Upgrade
 *
 * COMPONENT:          OTA_client.c
 *
 * AUTHOR:             Faisal Bhaiyat
 *
 * DESCRIPTION:        Over The Air Upgrade
 *
 * $HeadURL: http://svn/sware/Projects/SmartEnergy/Trunk/ZCL/Clusters/OTA/Source/OTA_client.c $
 *
 * $Revision:  $
 *
 * $LastChangedBy: fbhai $
 *
 * $LastChangedDate: $
 *
 * $Id: ota_client.c  $
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
#include "zcl.h"
#include "zcl_options.h"
#include "OTA.h"
#include "OTA_private.h"

#include "dbg.h"

//Ricky
#ifdef DEBUG_APP_OTA
#define TRACE_OTA_DEBUG TRUE
#define TRACE_OTA_DEBUG TRUE
#define TRACE_VERIF TRUE
#else
#define TRACE_OTA_DEBUG FALSE
#define TRACE_OTA_DEBUG FALSE
#define TRACE_VERIF FALSE
#endif

#ifdef OTA_CLIENT
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
 **
 ** NAME:       eOTA_ClientQueryNextImageRequest
 **
 ** DESCRIPTION:
 ** sends query next image request command
 **
 ** PARAMETERS:               Name                           Usage
 ** uint8                    u8SourceEndPointId            Source EP Id
 ** uint8                    u8DestinationEndPointId       Destination EP Id
 ** tsZCL_Address           *psDestinationAddress          Destination Address
 ** tsOTA_QueryImageRequest *psQueryImageRequest           command payload
 **
 ** RETURN:
 ** teZCL_Status
 ****************************************************************************/
PUBLIC  teZCL_Status eOTA_ClientQueryNextImageRequest(
                    uint8 u8SourceEndpoint,
                    uint8 u8DestinationEndpoint,
                    tsZCL_Address *psDestinationAddress,
                    tsOTA_QueryImageRequest *psQueryImageRequest)
{
    teZCL_Status eZCL_Status;
    tsZCL_ClusterInstance *psClusterInstance;
    tsZCL_EndPointDefinition *psEndPointDefinition;
    tsOTA_Common *psCustomData;
    uint8 u8SequenceNumber ;
    bool_t bDisableDefaultResponseState=FALSE;
//    tsZCL_TxPayloadItem asActPayloadDefinition[5] = {0};
    uint8 u8ItemsInPayload = 0;
    
    if((eZCL_Status =
        eOtaFindCluster(u8SourceEndpoint,
                         &psEndPointDefinition,
                           &psClusterInstance,
                           &psCustomData,
                           FALSE))
                           == E_ZCL_SUCCESS)
    {
        if (psClusterInstance->bIsServer)
        {
            eZCL_Status = E_ZCL_FAIL;
        }
        else
        {
                tsZCL_TxPayloadItem asPayloadDefinition[] = { {1, E_ZCL_UINT8,   &psQueryImageRequest->u8FieldControl},
                                                              {1, E_ZCL_UINT16,   &psQueryImageRequest->u16ManufacturerCode},
                                                              {1, E_ZCL_UINT16,   &psQueryImageRequest->u16ImageType},
                                                              {1, E_ZCL_UINT32,   &psQueryImageRequest->u32CurrentFileVersion},
                                                              {1, E_ZCL_UINT16,   &psQueryImageRequest->u16HardwareVersion}
                                                            };

            bDisableDefaultResponseState = psEndPointDefinition->bDisableDefaultResponse;
            psEndPointDefinition->bDisableDefaultResponse = OTA_CLIENT_DISABLE_DEFAULT_RESPONSE;

            if(psQueryImageRequest->u8FieldControl& 0x1)
            {
                u8ItemsInPayload=5;
            }
            else
            {
                u8ItemsInPayload=4;             
            }
            
            eZCL_Status = eZCL_CustomCommandSend(u8SourceEndpoint,
                             u8DestinationEndpoint,
                             psDestinationAddress,
                             OTA_CLUSTER_ID,
                             FALSE,
                             E_CLD_OTA_COMMAND_QUERY_NEXT_IMAGE_REQUEST,
                             &u8SequenceNumber,
                             asPayloadDefinition,//asActPayloadDefinition,
                             FALSE,
                             0,
                             u8ItemsInPayload);
        }
    }
    

    if(E_ZCL_SUCCESS == eZCL_Status)
    {
        eOtaClientReqSeqNoUpdate(u8SourceEndpoint,u8SequenceNumber);
    }

    psEndPointDefinition->bDisableDefaultResponse = bDisableDefaultResponseState;
    return eZCL_Status;
}

/****************************************************************************
 **
 ** NAME:       eOTA_ClientImageBlockRequest
 **
 ** DESCRIPTION:
 ** sends image block request command
 **
 ** PARAMETERS:               Name                           Usage
 ** uint8                    u8SourceEndPointId            Source EP Id
 ** uint8                    u8DestinationEndPointId       Destination EP Id
 ** tsZCL_Address           *psDestinationAddress          Destination Address
 ** tsOTA_BlockRequest      *psOtaBlockRequest             command payload
 **
 ** RETURN:
 ** teZCL_Status
 ****************************************************************************/
PUBLIC  teZCL_Status eOTA_ClientImageBlockRequest(
                    uint8 u8SourceEndpoint,
                    uint8 u8DestinationEndpoint,
                    tsZCL_Address *psDestinationAddress,
                    tsOTA_BlockRequest *psOtaBlockRequest)
{
    teZCL_Status eZCL_Status;
    tsZCL_ClusterInstance *psClusterInstance;
    tsZCL_EndPointDefinition *psEndPointDefinition;
    tsOTA_Common *psCustomData;
    uint8 u8SequenceNumber;
    bool_t bDisableDefaultResponseState=FALSE;

    uint8 u8ItemsInPayload = 0;
    
    if((eZCL_Status =
        eOtaFindCluster(u8SourceEndpoint,
                           &psEndPointDefinition,
                           &psClusterInstance,
                           &psCustomData,
                           FALSE))
                           == E_ZCL_SUCCESS)
    {
        if (psClusterInstance->bIsServer)
        {
            eZCL_Status = E_ZCL_FAIL;
			DBG_vPrintf(TRACE_OTA_DEBUG,"ERROR:SERVER node trying to do request\n");
        }
        else
        {
            bDisableDefaultResponseState = psEndPointDefinition->bDisableDefaultResponse;
            psEndPointDefinition->bDisableDefaultResponse = OTA_CLIENT_DISABLE_DEFAULT_RESPONSE;

            tsZCL_TxPayloadItem asPayloadDefinition[] = { {1, E_ZCL_UINT8,   &psOtaBlockRequest->u8FieldControl},
                                  {1, E_ZCL_UINT16,   &psOtaBlockRequest->u16ManufactureCode},
                                  {1, E_ZCL_UINT16,   &psOtaBlockRequest->u16ImageType},
                                  {1, E_ZCL_UINT32,   &psOtaBlockRequest->u32FileVersion},
                                  {1, E_ZCL_UINT32,   &psOtaBlockRequest->u32FileOffset},
                                  {1, E_ZCL_UINT8,    &psOtaBlockRequest->u8MaxDataSize},
                                  {1, E_ZCL_UINT64,   &psOtaBlockRequest->u64RequestNodeAddress},
                                  {1, E_ZCL_UINT16,   &psOtaBlockRequest->u16BlockRequestDelay}
                                };
            
            if((psOtaBlockRequest->u8FieldControl& 0x1)&&(psOtaBlockRequest->u8FieldControl& 0x2))
            {
                u8ItemsInPayload=8;                                
            }
            else if(psOtaBlockRequest->u8FieldControl& 0x1)
            {
                u8ItemsInPayload=7;                             
            }
            else if(psOtaBlockRequest->u8FieldControl& 0x2)
            {
                tsZCL_TxPayloadItem asActPayloadDefinition[] = { 
                                                  {1, E_ZCL_UINT16,   &psOtaBlockRequest->u16BlockRequestDelay}
                                                };
                u8ItemsInPayload=7;
                memcpy(&asPayloadDefinition[6],asActPayloadDefinition,sizeof(asActPayloadDefinition));
            }
            else
            {
                u8ItemsInPayload=6;                                          
            }
            
            eZCL_Status = eZCL_CustomCommandSend(u8SourceEndpoint,
                            u8DestinationEndpoint,
                            psDestinationAddress,
                            OTA_CLUSTER_ID,
                            FALSE,
                            E_CLD_OTA_COMMAND_BLOCK_REQUEST,
                            &u8SequenceNumber,
                            asPayloadDefinition,
                            FALSE,
                            0,
                            u8ItemsInPayload);

			if(eZCL_Status != E_ZCL_SUCCESS)
			{
				DBG_vPrintf(TRACE_OTA_DEBUG,"ERROR: %x status returned %x stack error\n", eZCL_Status , eZCL_GetLastZpsError());
			}
        }
    }
    if(E_ZCL_SUCCESS == eZCL_Status)
    {
        eOtaClientReqSeqNoUpdate(u8SourceEndpoint,u8SequenceNumber);
    }

    psEndPointDefinition->bDisableDefaultResponse = bDisableDefaultResponseState;
    return eZCL_Status;
}

/****************************************************************************
 **
 ** NAME:       eOTA_ClientImagePageRequest
 **
 ** DESCRIPTION:
 ** sends image page request command
 **
 ** PARAMETERS:               Name                           Usage
 ** uint8                    u8SourceEndPointId            Source EP Id
 ** uint8                    u8DestinationEndPointId       Destination EP Id
 ** tsZCL_Address           *psDestinationAddress          Destination Address
 ** tsOTA_ImagePageRequest  *psOtaPageRequest              command payload
 **
 ** RETURN:
 ** teZCL_Status
 ****************************************************************************/
PUBLIC  teZCL_Status eOTA_ClientImagePageRequest(
                    uint8 u8SourceEndpoint,
                    uint8 u8DestinationEndpoint,
                    tsZCL_Address *psDestinationAddress,
                    tsOTA_ImagePageRequest *psOtaPageRequest)
{
    teZCL_Status eZCL_Status;
    tsZCL_ClusterInstance *psClusterInstance;
    tsZCL_EndPointDefinition *psEndPointDefinition;
    tsOTA_Common *psCustomData;
    uint8 u8SequenceNumber;
    bool_t bDisableDefaultResponseState=FALSE;

    uint8 u8ItemsInPayload = 0;

    if((eZCL_Status =
        eOtaFindCluster(u8SourceEndpoint,
                           &psEndPointDefinition,
                           &psClusterInstance,
                           &psCustomData,
                           FALSE))
                           == E_ZCL_SUCCESS)
    {
        if (psClusterInstance->bIsServer)
        {
            eZCL_Status = E_ZCL_FAIL;
        }
        else
        {
            bDisableDefaultResponseState = psEndPointDefinition->bDisableDefaultResponse;
            psEndPointDefinition->bDisableDefaultResponse = OTA_CLIENT_DISABLE_DEFAULT_RESPONSE;
            tsZCL_TxPayloadItem asPayloadDefinition[] = { {1, E_ZCL_UINT8,   &psOtaPageRequest->u8FieldControl},
                                              {1, E_ZCL_UINT16,   &psOtaPageRequest->u16ManufactureCode},
                                              {1, E_ZCL_UINT16,   &psOtaPageRequest->u16ImageType},
                                              {1, E_ZCL_UINT32,   &psOtaPageRequest->u32FileVersion},
                                              {1, E_ZCL_UINT32,   &psOtaPageRequest->u32FileOffset},
                                              {1, E_ZCL_UINT8,    &psOtaPageRequest->u8MaxDataSize},
                                              {1, E_ZCL_UINT16,   &psOtaPageRequest->u16PageSize},
                                              {1, E_ZCL_UINT16,   &psOtaPageRequest->u16ResponseSpacing},
                                              {1, E_ZCL_UINT64,   &psOtaPageRequest->u64RequestNodeAddress},
                                            };

            if(psOtaPageRequest->u8FieldControl& 0x1)
            {
                u8ItemsInPayload = 9;
            }
            else
            {
                u8ItemsInPayload = 8;
            }
            eZCL_Status = eZCL_CustomCommandSend(u8SourceEndpoint,
                                     u8DestinationEndpoint,
                                     psDestinationAddress,
                                     OTA_CLUSTER_ID,
                                     FALSE,
                                     E_CLD_OTA_COMMAND_PAGE_REQUEST,
                                     &u8SequenceNumber,
                                     asPayloadDefinition,//asActPayloadDefinition,
                                     FALSE,
                                     0,
                                     u8ItemsInPayload);
        }
    }
    if(E_ZCL_SUCCESS == eZCL_Status)
    {
        eOtaClientReqSeqNoUpdate(u8SourceEndpoint,u8SequenceNumber);
    }

    psEndPointDefinition->bDisableDefaultResponse = bDisableDefaultResponseState;
    return eZCL_Status;
}

/****************************************************************************
 **
 ** NAME:       eOTA_ClientUpgradeEndRequest
 **
 ** DESCRIPTION:
 ** sends upgrade end request command
 **
 ** PARAMETERS:                         Name                           Usage
 ** uint8                           u8SourceEndPointId            Source EP Id
 ** uint8                           u8DestinationEndPointId       Destination EP Id
 ** tsZCL_Address                  *psDestinationAddress          Destination Address
 ** tsOTA_UpgradeEndRequestPayload *psUpgradeEndRequestPayload    Command Payload
 **
 ** RETURN:
 ** teZCL_Status
 ****************************************************************************/
PUBLIC  teZCL_Status eOTA_ClientUpgradeEndRequest(
                    uint8 u8SourceEndpoint,
                    uint8 u8DestinationEndpoint,
                    tsZCL_Address *psDestinationAddress,
                    tsOTA_UpgradeEndRequestPayload *psUpgradeEndRequestPayload)
{
    teZCL_Status eZCL_Status;
    tsZCL_ClusterInstance *psClusterInstance = NULL;
    tsZCL_EndPointDefinition *psEndPointDefinition = NULL;
    tsOTA_Common *psCustomData = NULL;
    uint8 u8SequenceNumber;
    bool_t bDisableDefaultResponseState=FALSE;

    tsZCL_TxPayloadItem asPayloadDefinition[] = { {1, E_ZCL_UINT8,   &psUpgradeEndRequestPayload->u8Status},
                                                  {1, E_ZCL_UINT16,   &psUpgradeEndRequestPayload->u16ManufacturerCode},
                                                  {1, E_ZCL_UINT16,   &psUpgradeEndRequestPayload->u16ImageType},
                                                  {1, E_ZCL_UINT32,   &psUpgradeEndRequestPayload->u32FileVersion},
                                                };

    if((eZCL_Status =
        eOtaFindCluster(u8SourceEndpoint,
                         &psEndPointDefinition,
                         &psClusterInstance,
                           &psCustomData,
                           FALSE))
                           == E_ZCL_SUCCESS)
    {
        if (psClusterInstance->bIsServer)
        {
            eZCL_Status = E_ZCL_FAIL;
        }
        else
        {
            bDisableDefaultResponseState = psEndPointDefinition->bDisableDefaultResponse;
            psEndPointDefinition->bDisableDefaultResponse = OTA_CLIENT_DISABLE_DEFAULT_RESPONSE;

            eZCL_Status = eZCL_CustomCommandSend(u8SourceEndpoint,
                                                 u8DestinationEndpoint,
                                                 psDestinationAddress,
                                                 OTA_CLUSTER_ID,
                                                 FALSE,
                                                 E_CLD_OTA_COMMAND_UPGRADE_END_REQUEST,
                                                 &u8SequenceNumber,
                                                 asPayloadDefinition,
                                                 FALSE,
                                                 0,
                                                 sizeof(asPayloadDefinition) / sizeof(tsZCL_TxPayloadItem));
        }
    }
    if(E_ZCL_SUCCESS == eZCL_Status)
    {
        eOtaClientReqSeqNoUpdate(u8SourceEndpoint,u8SequenceNumber);
    }

    psEndPointDefinition->bDisableDefaultResponse = bDisableDefaultResponseState;
    return eZCL_Status;
}

/****************************************************************************
 **
 ** NAME:       eOTA_ClientQuerySpecificFileRequest
 **
 ** DESCRIPTION:
 ** sends Query Specific File request command
 **
 ** PARAMETERS:                         Name                           Usage
 ** uint8                           u8SourceEndPointId            Source EP Id
 ** uint8                           u8DestinationEndPointId       Destination EP Id
 ** tsZCL_Address                  *psDestinationAddress          Destination Address
 ** tsOTA_QuerySpecificFileRequestPayload *psQuerySpecificFileRequestPayload    Command Payload
 **
 ** RETURN:
 ** teZCL_Status
 ****************************************************************************/
PUBLIC  teZCL_Status eOTA_ClientQuerySpecificFileRequest(
                    uint8 u8SourceEndpoint,
                    uint8 u8DestinationEndpoint,
                    tsZCL_Address *psDestinationAddress,
                    tsOTA_QuerySpecificFileRequestPayload *psQuerySpecificFileRequestPayload)
{
    teZCL_Status eZCL_Status;
    tsZCL_ClusterInstance *psClusterInstance;
    tsZCL_EndPointDefinition *psEndPointDefinition;
    tsOTA_Common *psCustomData;
    uint8 u8SequenceNumber;
    bool_t bDisableDefaultResponseState=FALSE;

    tsZCL_TxPayloadItem asPayloadDefinition[] = { {1, E_ZCL_UINT64,   &psQuerySpecificFileRequestPayload->u64RequestNodeAddress},
                                                  {1, E_ZCL_UINT16,   &psQuerySpecificFileRequestPayload->u16ManufacturerCode},
                                                  {1, E_ZCL_UINT16,   &psQuerySpecificFileRequestPayload->u16ImageType},
                                                  {1, E_ZCL_UINT32,   &psQuerySpecificFileRequestPayload->u32FileVersion},
                                                  {1, E_ZCL_UINT16,   &psQuerySpecificFileRequestPayload->u16CurrentZibgeeStackVersion}
                                                };

    if((eZCL_Status =
        eOtaFindCluster(u8SourceEndpoint,
                         &psEndPointDefinition,
                         &psClusterInstance,
                           &psCustomData,
                           FALSE))
                           == E_ZCL_SUCCESS)
    {
        if (psClusterInstance->bIsServer)
        {
            eZCL_Status = E_ZCL_FAIL;
        }
        else
        {
            bDisableDefaultResponseState = psEndPointDefinition->bDisableDefaultResponse;
            psEndPointDefinition->bDisableDefaultResponse = OTA_CLIENT_DISABLE_DEFAULT_RESPONSE;

            eZCL_Status = eZCL_CustomCommandSend(u8SourceEndpoint,
                                                 u8DestinationEndpoint,
                                                 psDestinationAddress,
                                                 OTA_CLUSTER_ID,
                                                 FALSE,
                                                 E_CLD_OTA_COMMAND_QUERY_SPECIFIC_FILE_REQUEST,
                                                 &u8SequenceNumber,
                                                 asPayloadDefinition,
                                                 FALSE,
                                                 0,
                                                 sizeof(asPayloadDefinition) / sizeof(tsZCL_TxPayloadItem));
        }
    }
    if(E_ZCL_SUCCESS == eZCL_Status)
    {
        eOtaClientReqSeqNoUpdate(u8SourceEndpoint,u8SequenceNumber);
    }

    psEndPointDefinition->bDisableDefaultResponse = bDisableDefaultResponseState;
    return eZCL_Status;
}
/****************************************************************************
 **
 ** NAME:       eOTA_SpecificFileUpgradeEndRequest
 **
 ** DESCRIPTION:
 ** sends Query Specific File Upgrade End request command
 **
 ** PARAMETERS:                         Name                           Usage
 ** uint8                           u8SourceEndPointId            Source EP Id
 ** uint8                           u8Status                      Status
 **
 ** RETURN:
 ** teZCL_Status
 ****************************************************************************/
PUBLIC  teZCL_Status eOTA_SpecificFileUpgradeEndRequest(uint8 u8SourceEndPointId, uint8 u8Status)
{
    teZCL_Status eStatus = E_ZCL_SUCCESS;
    tsZCL_ClusterInstance *psClusterInstance;
    tsZCL_EndPointDefinition *psEndPointDefinition;
    tsOTA_Common *psCustomData;
    tsOTA_UpgradeEndRequestPayload sEndRequest;
    tsZCL_Address sZCL_Address;

	#if OTA_ACKS_ON == TRUE
	    sZCL_Address.eAddressMode = E_ZCL_AM_SHORT;
	#else
	    sZCL_Address.eAddressMode = E_ZCL_AM_SHORT_NO_ACK;
	#endif
    sZCL_Address.uAddress.u16DestinationAddress = psCustomData->sOTACallBackMessage.sPersistedData.u16ServerShortAddress;
    if((eStatus = eOtaFindCluster(u8SourceEndPointId,
                                  &psEndPointDefinition,
                                  &psClusterInstance,
                                  &psCustomData,
                                  FALSE))
                                  == E_ZCL_SUCCESS)
    {

        if(!psCustomData->sOTACallBackMessage.sPersistedData.bIsSpecificFile)
        {
            return(E_ZCL_FAIL);
        }

        sEndRequest.u32FileVersion = psCustomData->sOTACallBackMessage.sPersistedData.sAttributes.u32DownloadedFileVersion;

        sEndRequest.u16ImageType = psCustomData->sOTACallBackMessage.sPersistedData.sAttributes.u16ImageType;

        sEndRequest.u16ManufacturerCode = psCustomData->sOTACallBackMessage.sPersistedData.sAttributes.u16ManfId;
        sEndRequest.u8Status = u8Status;
        if(u8Status == OTA_STATUS_SUCCESS)
        {
             /* change state to dowmload complete */
            vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_DL_COMPLETE, psEndPointDefinition, psCustomData);
        }
        else
        {
            /* change state to normal */
            vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_NORMAL, psEndPointDefinition, psCustomData);
        }
         /* Send block end request */
        eStatus = eOTA_ClientUpgradeEndRequest(psCustomData->sReceiveEventAddress.u8DstEndpoint,
                                     psCustomData->sReceiveEventAddress.u8SrcEndpoint,
                                     &sZCL_Address,
                                     &sEndRequest);
        if(u8Status == OTA_STATUS_SUCCESS)
        {  /* if it is a success case then enable retries and wait for response */
            psCustomData->sOTACallBackMessage.sPersistedData.u8Retry = 0;
            psCustomData->sOTACallBackMessage.sPersistedData.u32RequestBlockRequestTime = u32ZCL_GetUTCTime()+ OTA_TIME_INTERVAL_BETWEEN_END_REQUEST_RETRIES+1;
        }
        else
        {
            psCustomData->sOTACallBackMessage.sPersistedData.u32RequestBlockRequestTime = 0;
        }
        /* request has been sent make sure we poll for the response too otherwise might miss it
           only valid for end device*/
        eOtaSetEventTypeAndGiveCallBack(psCustomData, E_CLD_OTA_INTERNAL_COMMAND_POLL_REQUIRED,psEndPointDefinition);
    }
    return eStatus;
}
#if (OTA_MAX_CO_PROCESSOR_IMAGES != 0)
/****************************************************************************
 **
 ** NAME:       eOTA_CoProcessorUpgradeEndRequest
 **
 ** DESCRIPTION:
 ** sends upgrade end request command for the co-processor
 **
 ** PARAMETERS:                         Name                           Usage
 ** uint8                           u8SourceEndPointId            Source EP Id
 ** uint8                           u8Status                      status
 **
 ** RETURN:
 ** teZCL_Status
 ****************************************************************************/
PUBLIC  teZCL_Status eOTA_CoProcessorUpgradeEndRequest(uint8 u8SourceEndPointId, uint8 u8Status)
{
    teZCL_Status eStatus = E_ZCL_SUCCESS;
    tsZCL_ClusterInstance *psClusterInstance;
    tsZCL_EndPointDefinition *psEndPointDefinition;
    tsOTA_Common *psCustomData;
    tsOTA_UpgradeEndRequestPayload sEndRequest;
    tsOTA_ImageHeader sOTAHeader ;
    tsZCL_Address sZCL_Address;


    if((eStatus = eOtaFindCluster(u8SourceEndPointId,
                                  &psEndPointDefinition,
                                  &psClusterInstance,
                                  &psCustomData,
                                  FALSE))
                                  == E_ZCL_SUCCESS)
    {

        if(!psCustomData->sOTACallBackMessage.sPersistedData.bIsCoProcessorImage)
        {
            return(E_ZCL_FAIL);
        }
        sOTAHeader = sOtaGetHeader(&asCommonCoProcessorOTAHeader[psCustomData->sOTACallBackMessage.sPersistedData.u8CoProcessorOTAHeaderIndex][0]);
        sEndRequest.u32FileVersion = psCustomData->sOTACallBackMessage.sPersistedData.sAttributes.u32DownloadedFileVersion;

        sEndRequest.u16ImageType = sOTAHeader.u16ImageType;

        sEndRequest.u16ManufacturerCode = sOTAHeader.u16ManufacturerCode;
        sEndRequest.u8Status = u8Status;
        if(u8Status == OTA_STATUS_SUCCESS)
        {
            /* Decrement downloadable images */
            psCustomData->sOTACallBackMessage.sPersistedData.u8NumOfDownloadableImages--;

            /* Check Any images are pedning for downloading */
            if((!psCustomData->sOTACallBackMessage.sPersistedData.u8NumOfDownloadableImages)||(!bIsCoProcessorImgUpgdDependent))
            {
               sEndRequest.u8Status = OTA_STATUS_SUCCESS;

               /* change state to dowmload complete */
               vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_DL_COMPLETE, psEndPointDefinition, psCustomData);
            }
            else
            {
                sEndRequest.u8Status = OTA_REQUIRE_MORE_IMAGE;
            }
        }
        else
        {
            /* change state to normal */
            vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_NORMAL, psEndPointDefinition, psCustomData);
        }


	#if OTA_ACKS_ON == TRUE
	    sZCL_Address.eAddressMode = E_ZCL_AM_SHORT;
	#else
	    sZCL_Address.eAddressMode = E_ZCL_AM_SHORT_NO_ACK;
	#endif
	    sZCL_Address.uAddress.u16DestinationAddress = psOTA_Common->sOTACallBackMessage.sPersistedData.u16ServerShortAddress;
		
         /* Send block end request */
        eStatus = eOTA_ClientUpgradeEndRequest(psCustomData->sReceiveEventAddress.u8DstEndpoint,
                                     psCustomData->sReceiveEventAddress.u8SrcEndpoint,
                                     &sZCL_Address,
                                     &sEndRequest);
        if(sEndRequest.u8Status == OTA_STATUS_SUCCESS)
        {  /* if it is a success case then enable retries and wait for response */
            psCustomData->sOTACallBackMessage.sPersistedData.u8Retry = 0;
            psCustomData->sOTACallBackMessage.sPersistedData.u32RequestBlockRequestTime = u32ZCL_GetUTCTime()+ OTA_TIME_INTERVAL_BETWEEN_END_REQUEST_RETRIES+1;
        }
        else if(sEndRequest.u8Status == OTA_REQUIRE_MORE_IMAGE)
        {
            /* Give a call back to the user for requesting remaining images */
            eOtaSetEventTypeAndGiveCallBack(psCustomData, E_CLD_OTA_INTERNAL_COMMAND_REQUEST_QUERY_NEXT_IMAGES,psEndPointDefinition);

            /* Change the state to Normal State */
            vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_NORMAL,psEndPointDefinition,psCustomData);
        }
        else
        {
            psCustomData->sOTACallBackMessage.sPersistedData.u32RequestBlockRequestTime = 0;
        }
        /* request has been sent make sure we poll for the response too otherwise might miss it
            only valid for end device*/
        eOtaSetEventTypeAndGiveCallBack(psCustomData, E_CLD_OTA_INTERNAL_COMMAND_POLL_REQUIRED,psEndPointDefinition);
    }
    return eStatus;
}
#endif
/****************************************************************************
 **
 ** NAME:       eOTA_ClientSwitchToNewImage
 **
 ** DESCRIPTION:
 ** Client switch to new downloaded upgrade image
 **
 ** PARAMETERS:                         Name                           Usage
 ** uint8                           u8SourceEndPointId            Source EP Id
 **
 ** RETURN:
 ** teZCL_Status
 ****************************************************************************/
PUBLIC  teZCL_Status eOTA_ClientSwitchToNewImage(uint8 u8SourceEndPointId)
{
    teZCL_Status eStatus = E_ZCL_SUCCESS;
    tsZCL_ClusterInstance *psClusterInstance;
    tsZCL_EndPointDefinition *psEndPointDefinition;
    tsOTA_Common *psOTA_Common;


    if((eStatus = eOtaFindCluster(u8SourceEndPointId,
                                  &psEndPointDefinition,
                                  &psClusterInstance,
                                  &psOTA_Common,
                                  FALSE))
                                  == E_ZCL_SUCCESS)
    {
        if(!psOTA_Common->sOTACallBackMessage.sPersistedData.bIsNullImage)
        {
            uint8 au8MagicNumbers[12] = {0};
            uint32 u32Offset;
            uint8 u8NextFreeImageLocation = psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation;
#if JENNIC_CHIP_FAMILY == JN516x
            uint8 au8Data[16] = {0};
#endif
            vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_NORMAL,psEndPointDefinition,psOTA_Common);
            vOTA_SetImageValidityFlag(psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation,psOTA_Common,TRUE, psEndPointDefinition);

            /* Read Magic Numbers */
            u32Offset =  (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize) + OTA_FLS_VALIDITY_OFFSET;
            vOtaFlashLockRead(psEndPointDefinition, psOTA_Common,u32Offset,OTA_FLS_MAGIC_NUMBER_LENGTH, au8MagicNumbers);

            if(bOtaIsImageValid(au8MagicNumbers))
            {
                /* persisted data changed send event to the  application to save it*/
                eOtaSetEventTypeAndGiveCallBack(psOTA_Common, E_CLD_OTA_INTERNAL_COMMAND_SAVE_CONTEXT,psEndPointDefinition);

                eOtaSetEventTypeAndGiveCallBack(psOTA_Common, E_CLD_OTA_INTERNAL_COMMAND_RESET_TO_UPGRADE,psEndPointDefinition);

                /* Invalidate current image and validate upgrade image */
#if JENNIC_CHIP_FAMILY == JN514x
                vOTA_SetImageValidityFlag(psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation,psOTA_Common,FALSE, psEndPointDefinition);
#endif

#if JENNIC_CHIP_FAMILY == JN516x
                /* Invalidate Internal Flash Header */
                bAHI_FlashInit(E_FL_CHIP_INTERNAL, NULL); /* Pass Internal Flash */
                /* Erase Internal Flash Header */
                DBG_vPrintf(TRACE_VERIF, "DELETE HEADER\n");
                bAHI_FullFlashProgram(0x00,16,au8Data);
#endif
                vOtaSwitchLoads();
            }
            else
            {
                DBG_vPrintf(TRACE_VERIF, "Image header check FAILED\n");
                eOtaSetEventTypeAndGiveCallBack(psOTA_Common, E_CLD_OTA_INTERNAL_COMMAND_FAILED_VALIDATING_UPGRADE_IMAGE,psEndPointDefinition);
            }
        }
    }
    return eStatus;
}

/****************************************************************************
 **
 ** NAME:       eOTA_HandleImageVerification
 **
 ** DESCRIPTION:
 ** After image verification according to status provided it copies serialization data
 ** and sends the upgrade end request
 **
 ** PARAMETERS:                         Name                           Usage
 ** uint8                           u8SourceEndPointId            Source EP Id
 ** uint8                           u8DstEndpoint                 Destination EP Id
 ** teZCL_Status                    eImageVerificationStatus      image verification status
 **
 ** RETURN:
 ** teZCL_Status
 ****************************************************************************/
PUBLIC  teZCL_Status eOTA_HandleImageVerification(uint8 u8SourceEndpoint,
                                                                  uint8 u8DstEndpoint,
                                                                  teZCL_Status eImageVerificationStatus)
{
    teZCL_Status eStatus;
    uint32 u32Offset,u32Id;
    bool_t bValid = FALSE;
#if JENNIC_CHIP_FAMILY == JN514x
    uint8 *pu8Start = (uint8*)&FlsStart;
#endif
#if JENNIC_CHIP_FAMILY == JN516x
    uint8 *pu8Start = (uint8*)&_flash_start;
#endif
    uint8 *pu8LinkKey =    (uint8*)&_FlsLinkKey;
#ifndef OTA_NO_CERTIFICATE
    uint8 *pu8Cert =       (uint8*)&FlsZcCert;
    uint8 *pu8PrvKey =     (uint8*)&FlsPrivateKey;
#else
    uint8 *pu8Cert =       (uint8*)&_FlsLinkKey + 16;
#ifdef COPY_PRIVATE_KEY     
    uint8 *pu8PrvKey =     (uint8*)&_FlsLinkKey + 16;
#endif    
#endif

#ifdef OTA_COPY_MAC_ADDRESS
    uint8 *pu8MacAddress = (uint8*)&_FlsMACAddress;
#endif

#ifdef OTA_MAINTAIN_CUSTOM_SERIALISATION_DATA
    uint8 *pu8CustData =   (uint8*)&FlsCustomDatabeg;
    uint8 *pu8CustDataEnd = (uint8*)&FlsCustomDataEnd;
    uint8 u8SizeOfCustomData = pu8CustDataEnd - pu8CustData;
#endif
    uint8 au8Value[OTA_AES_BLOCK_SIZE];
    tsOTA_UpgradeEndRequestPayload sEndRequest;

#if JENNIC_CHIP_FAMILY == JN516x
    uint32 u32CustomerSettings = *((uint32*)FL_INDEX_SECTOR_CUSTOMER_SETTINGS);
    bool_t bEncExternalFlash = (u32CustomerSettings & FL_INDEX_SECTOR_ENC_EXT_FLASH)?FALSE:TRUE;
#endif
    tsZCL_ClusterInstance *psClusterInstance;
    tsZCL_EndPointDefinition *psEndPointDefinition;
    tsOTA_Common *psOTA_Common;
    tsZCL_Address sZCL_Address;


    /* These arrays will be used for the Decryption of the Encrypted data on flash
       and is used in different cases. */
#ifndef OTA_UNIT_TEST_FRAMEWORK
    uint8 au8ivector[OTA_AES_BLOCK_SIZE], au8DataOut[OTA_AES_BLOCK_SIZE],u8Loop;
    uint32 u32TempOffset;
#if JENNIC_CHIP_FAMILY == JN514x
    // These arrays will be used for reading old Image IV data
    uint8 au8Oivector[OTA_AES_BLOCK_SIZE], au8ODataOut[OTA_AES_BLOCK_SIZE];
#endif
    tsReg128 sOtaUseKey =  eOTA_retOtaUseKey();
#endif
    uint32 u32OtaOffset = eOTA_OtaOffset();

    if((eStatus = eOtaFindCluster(u8SourceEndpoint,
                                  &psEndPointDefinition,
                                  &psClusterInstance,
                                  &psOTA_Common,
                                  FALSE))
                                  == E_ZCL_SUCCESS)
    {
        vReverseMemcpy((uint8*)&sEndRequest.u16ImageType,&psOTA_Common->sOTACallBackMessage.sPersistedData.au8Header[12],sizeof(uint16));
        vReverseMemcpy((uint8*)&sEndRequest.u16ManufacturerCode,&psOTA_Common->sOTACallBackMessage.sPersistedData.au8Header[10],sizeof(uint16));
        vReverseMemcpy((uint8*)&sEndRequest.u32FileVersion,&psOTA_Common->sOTACallBackMessage.sPersistedData.au8Header[14],sizeof(uint32));

        u32Offset = u32OtaOffset;
        u32Offset +=  (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
        vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32Offset,OTA_AES_BLOCK_SIZE, au8Value);
#ifndef OTA_UNIT_TEST_FRAMEWORK
#if JENNIC_CHIP_FAMILY == JN514x
        if (u32REG_SysRead(OTA_EFUSE_REG_CNTRL + 3) & 0x1)
#endif
#if JENNIC_CHIP_FAMILY == JN516x
        if (bEncExternalFlash)
#endif
        {   // Read IV of Old Image
            u32TempOffset = (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
            u32TempOffset += OTA_IV_LOCATION;
            /* Read the IV */
            vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32TempOffset,OTA_AES_BLOCK_SIZE, au8ivector);
            au8ivector[15] = au8ivector[15]+(uint8)((u32OtaOffset - OTA_ENC_OFFSET)/OTA_AES_BLOCK_SIZE);
            //bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8ivector,(tsReg128 *)au8DataOut);
			vOTA_EncodeString(&sOtaUseKey, au8ivector,au8DataOut);

            for(u8Loop=0;u8Loop<OTA_AES_BLOCK_SIZE;u8Loop++)
            {
                au8Value[u8Loop] = au8DataOut[u8Loop]^au8Value[u8Loop];
            }
        }
#endif
        vReverseMemcpy((uint8*)&u32Id,&au8Value[0],sizeof(uint32));
		DBG_vPrintf(TRACE_OTA_DEBUG, "\n u32Id : %08x, OTA_FILE_IDENTIFIER:%08x \n",u32Id,OTA_FILE_IDENTIFIER);

        if(u32Id != OTA_FILE_IDENTIFIER)
        {
            DBG_vPrintf(TRACE_OTA_DEBUG, "OTA FILE IDENTIFIER NOT MATCHED \n");
        }

        if(u32Id == OTA_FILE_IDENTIFIER && (eImageVerificationStatus == E_ZCL_SUCCESS))
        {
            /* copy of serializetion is only for own image */
            u32Offset = pu8Cert - pu8LinkKey;
            if(u32Offset > 0)
            {
#ifndef OTA_UNIT_TEST_FRAMEWORK
                uint8 au8TempBuffer[50]; /* for copying data from active image */

                /* Check if the device is Encrypted.  If yes the device is encrypted, we have to take care of IV vector as well
                 * If the IV vector of the Client and the Upgrade Image is different we cannot just copy the Serialization Data
                 * Directly. We need to Decrypt existing data and encrypt it again with the new IV vector values.
                 */
#if JENNIC_CHIP_FAMILY == JN514x
                if (u32REG_SysRead(OTA_EFUSE_REG_CNTRL + 3) & 0x1)
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                if (bEncExternalFlash)
#endif
                {

                    DBG_vPrintf(TRACE_OTA_DEBUG, "OTA 200 ..\n");
#ifdef OTA_COPY_MAC_ADDRESS
#if JENNIC_CHIP_FAMILY == JN516x
                    /* Copy Mac Address from from Internal flash and encrypt it. Then copy to external flash */
                    memcpy(au8TempBuffer, pu8MacAddress, 8);

                    /* Read IV vector from external flash */
                    u32TempOffset = (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    u32TempOffset += OTA_IV_LOCATION;
                    vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32TempOffset,OTA_AES_BLOCK_SIZE, au8ivector);

                    au8ivector[15] = au8ivector[15]+((OTA_6X_MAC_OFFSET - OTA_ENC_OFFSET)/OTA_AES_BLOCK_SIZE);
                    bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8ivector,(tsReg128 *)au8DataOut);

                    for(u8Loop=0;u8Loop<16;u8Loop++)
                              au8TempBuffer[u8Loop] = (au8TempBuffer[u8Loop] ^ au8DataOut[u8Loop]) ;

                    u32Offset = OTA_6X_MAC_OFFSET;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    DBG_vPrintf(TRACE_VERIF, "Copy MAC TO %08x\n", u32Offset);
                    vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common,u32Offset,16, au8TempBuffer);
#endif
#endif
                    /*
                     * We read the IV data from the image presently in execution in the Device
                     */
#if JENNIC_CHIP_FAMILY == JN514x
                    u32TempOffset = (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
                    u32TempOffset += OTA_IV_LOCATION;
                    vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32TempOffset,OTA_AES_BLOCK_SIZE, au8Oivector);

                    // The IV offset is beginning of the Link Key Data on Old Image
                    au8Oivector[15] = au8Oivector[15]+(uint8)((u32OtaOffset - OTA_ENC_OFFSET)/OTA_AES_BLOCK_SIZE) +  (80 / OTA_AES_BLOCK_SIZE);
                    bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8Oivector,(tsReg128 *)au8ODataOut);
#endif
                    u32TempOffset = (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    u32TempOffset += OTA_IV_LOCATION;
                    vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32TempOffset,OTA_AES_BLOCK_SIZE, au8ivector);

                    au8ivector[15] = au8ivector[15]+((u32OtaOffset - OTA_ENC_OFFSET)/OTA_AES_BLOCK_SIZE) + (80 / OTA_AES_BLOCK_SIZE);
                    //bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8ivector,(tsReg128 *)au8DataOut);
					vOTA_EncodeString(&sOtaUseKey,au8ivector,au8DataOut);

                    // For Copying Link Key
#if JENNIC_CHIP_FAMILY == JN514x
                    u32Offset = pu8LinkKey - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
                    vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32Offset,16, au8TempBuffer);

                    for(u8Loop=0;u8Loop<16;u8Loop++)
                              au8TempBuffer[u8Loop] = (au8TempBuffer[u8Loop]^au8ODataOut[u8Loop]) ^ au8DataOut[u8Loop] ;
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                    memcpy(au8TempBuffer, pu8LinkKey, 16);

                    for(u8Loop=0;u8Loop<16;u8Loop++)
                              au8TempBuffer[u8Loop] = (au8TempBuffer[u8Loop] ^ au8DataOut[u8Loop]);
#endif

                    u32Offset = pu8LinkKey - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    DBG_vPrintf(TRACE_VERIF, "Copy key to %08x\n", u32Offset);
                    vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common,u32Offset,16, au8TempBuffer);

                    // For Copying Certificate

#if JENNIC_CHIP_FAMILY == JN514x
                    u32Offset = pu8Cert - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
                    vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32Offset,48, au8TempBuffer);
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                    memcpy(au8TempBuffer, pu8Cert, 48);
#endif
                    for(u8Loop=0;u8Loop<48;u8Loop++)
                    {
                        if((u8Loop % OTA_AES_BLOCK_SIZE) == 0 )
                        {
                            au8ivector[15]++;
#if JENNIC_CHIP_FAMILY == JN514x
                            au8Oivector[15]++;
                            bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8Oivector,(tsReg128 *)au8ODataOut);
#endif
                            //bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8ivector,(tsReg128 *)au8DataOut);
							vOTA_EncodeString(&sOtaUseKey,au8ivector,au8DataOut);
                        }
#if JENNIC_CHIP_FAMILY == JN514x
                        au8TempBuffer[u8Loop] = (au8TempBuffer[u8Loop]^au8ODataOut[(u8Loop%16)]) ^ au8DataOut[(u8Loop %16)] ;
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                        au8TempBuffer[u8Loop] = au8TempBuffer[u8Loop] ^ au8DataOut[(u8Loop %16)] ;
#endif
                    }
                    u32Offset = pu8Cert - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common,u32Offset,48, au8TempBuffer);

                    // For Copying Private Key
#ifdef COPY_PRIVATE_KEY                     
#if JENNIC_CHIP_FAMILY == JN514x
                    u32Offset = pu8PrvKey - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
                    vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32Offset, 21, au8TempBuffer);
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                    memcpy(au8TempBuffer, pu8PrvKey, 21);
#endif
                    for(u8Loop=0;u8Loop<21;u8Loop++)
                    {
                        if((u8Loop % OTA_AES_BLOCK_SIZE) == 0 )
                        {
                            au8ivector[15]++;
#if JENNIC_CHIP_FAMILY == JN514x
                            au8Oivector[15]++;
                            bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8Oivector,(tsReg128 *)au8ODataOut);
#endif
                            bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8ivector,(tsReg128 *)au8DataOut);
                        }
#if JENNIC_CHIP_FAMILY == JN514x
                        au8TempBuffer[u8Loop] = (au8TempBuffer[u8Loop]^au8ODataOut[(u8Loop%16)]) ^ au8DataOut[(u8Loop %16)] ;
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                        au8TempBuffer[u8Loop] = (au8TempBuffer[u8Loop] ^ au8DataOut[(u8Loop %16)]) ;
#endif
                    }
                    u32Offset = pu8PrvKey - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common,u32Offset,21, au8TempBuffer);
#endif                    
                }
                else
                {
                    DBG_vPrintf(TRACE_OTA_DEBUG, "OTA 31 ..\n");

#ifdef OTA_COPY_MAC_ADDRESS
#if JENNIC_CHIP_FAMILY == JN516x
#ifndef OTA_INTERNAL_STORAGE
                    memcpy(au8TempBuffer, pu8MacAddress, 8);
                    u32Offset = pu8MacAddress - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    DBG_vPrintf(TRACE_VERIF, "Copy MAC2 TO %08x\n", u32Offset);
                    vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common,u32Offset,8, au8TempBuffer);
#else
                    memset(au8TempBuffer, 0, 16);
                    memcpy(au8TempBuffer, pu8MacAddress, 8);
                    u32Offset = pu8MacAddress - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    DBG_vPrintf(TRACE_VERIF, "Copy MAC2 TO %08x 16 bytes\n", u32Offset);
                    vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common, u32Offset, 16, au8TempBuffer);
#endif
#endif
#endif
                    // copy Link Key
                    /* read link key from active image */
#if JENNIC_CHIP_FAMILY == JN514x
                    u32Offset = pu8LinkKey - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
                    vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32Offset,16, au8TempBuffer);
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                    memcpy(au8TempBuffer, pu8LinkKey, 16);
#endif
                    /* copy link key to new upgrade image */
                    u32Offset = pu8LinkKey - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    DBG_vPrintf(TRACE_VERIF, "Copy Key2 to %08x\n", u32Offset);
                    vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common,u32Offset,16, au8TempBuffer);
#ifndef OTA_NO_CERTIFICATE
                    // copy certs Key
                    /* read certificates from active image */
#if JENNIC_CHIP_FAMILY == JN514x
                    u32Offset = pu8Cert - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
                    vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32Offset,48, au8TempBuffer);
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                    memcpy(au8TempBuffer, pu8Cert, 48);
#endif
                    /* copy certificate to new upgrade image */
                    u32Offset = pu8Cert - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    DBG_vPrintf(TRACE_VERIF, "READ4 -> 48 from %08x\n", u32Offset);
                    vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common,u32Offset,48, au8TempBuffer);

                    // copy private Key
                    /* read private key from active image */
#if JENNIC_CHIP_FAMILY == JN514x
                    u32Offset = pu8PrvKey - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
                    vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32Offset, 21, au8TempBuffer);
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                    memcpy(au8TempBuffer, pu8PrvKey, 21);
#endif
                    /* copy private key to new upgrade image */
                    u32Offset = pu8PrvKey - pu8Start;
                    u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                    DBG_vPrintf(TRACE_VERIF, "VERIF1 -> write 21 bytes to %08x\n", u32Offset);
                    vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common,u32Offset,21, au8TempBuffer);
#endif
                }
#endif

#ifdef OTA_COPY_MAC_ADDRESS
#if JENNIC_CHIP_FAMILY == JN514x
                u32Offset = pu8MacAddress - pu8Start;
                u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
                vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32Offset, 8, au8TempBuffer);

                u32Offset = pu8MacAddress - pu8Start;
                u32Offset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common,u32Offset,8, au8TempBuffer);
#endif
#endif

#ifdef OTA_MAINTAIN_CUSTOM_SERIALISATION_DATA
                if(u8SizeOfCustomData > 0)
                {
                    uint8 u8NoOf16BytesBlocks = u8SizeOfCustomData/16;

                    uint8 u8LoopCount,u8IntLoopCount;
                    uint32 u32ReadOffset = pu8CustData - pu8Start;
                    uint32 u32WriteOffset = pu8CustData - pu8Start;

                    // Check if Device is Encrypted
#if JENNIC_CHIP_FAMILY == JN514x
                    if (u32REG_SysRead(OTA_EFUSE_REG_CNTRL + 3) & 0x1)
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                    if (bEncExternalFlash)
#endif
                    {
#if JENNIC_CHIP_FAMILY == JN514x
                        u32TempOffset = (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
                        u32TempOffset += OTA_IV_LOCATION;

                        vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32TempOffset,OTA_AES_BLOCK_SIZE, au8Oivector);
                        au8Oivector[15] = au8Oivector[15]+(uint8)((u32OtaOffset - OTA_ENC_OFFSET)/OTA_AES_BLOCK_SIZE) +  (80 / OTA_AES_BLOCK_SIZE) + ( (pu8CustData - pu8LinkKey) / OTA_AES_BLOCK_SIZE);

                        bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8Oivector,(tsReg128 *)au8ODataOut);
#endif

                        u32TempOffset = (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);
                        u32TempOffset += OTA_IV_LOCATION;
                        vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, u32TempOffset,OTA_AES_BLOCK_SIZE, au8ivector);

                        au8ivector[15] = au8ivector[15]+((u32OtaOffset - OTA_ENC_OFFSET)/OTA_AES_BLOCK_SIZE) + (80 / OTA_AES_BLOCK_SIZE) + ( (pu8CustData - pu8LinkKey) / OTA_AES_BLOCK_SIZE);

                        bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8ivector,(tsReg128 *)au8DataOut);
#if JENNIC_CHIP_FAMILY == JN514x
                        u32ReadOffset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
#endif
                        u32WriteOffset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);

                        for(u8LoopCount = 0; u8LoopCount < u8NoOf16BytesBlocks; u8LoopCount++)
                        {
#if JENNIC_CHIP_FAMILY == JN514x
                            vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, (u32ReadOffset + (16*u8LoopCount)), 16, au8TempBuffer);
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                            memcpy(au8TempBuffer, (pu8CustData+(16*u8LoopCount)), 16);
#endif

                            for(u8IntLoopCount=0; u8IntLoopCount < 16;u8IntLoopCount++)
                            {
#if JENNIC_CHIP_FAMILY == JN514x
                                 au8TempBuffer[u8IntLoopCount] = (au8TempBuffer[u8IntLoopCount]^au8ODataOut[(u8IntLoopCount)]) ^ au8DataOut[(u8IntLoopCount)] ;
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                                 au8TempBuffer[u8IntLoopCount] = (au8TempBuffer[u8IntLoopCount] ^ au8DataOut[(u8IntLoopCount)]);
#endif
                            }

                            vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common, (u32WriteOffset + (16*u8LoopCount)), 16, au8TempBuffer);

                            au8ivector[15]++;
#if JENNIC_CHIP_FAMILY == JN514x
                            au8Oivector[15]++;
                            bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8Oivector,(tsReg128 *)au8ODataOut);
#endif
                            bACI_ECBencodeStripe(&sOtaUseKey,TRUE,(tsReg128 *)au8ivector,(tsReg128 *)au8DataOut);
                        }

                        /* copy remaining data */
                        u8NoOf16BytesBlocks = u8SizeOfCustomData%16;
#if JENNIC_CHIP_FAMILY == JN514x
                        vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, (u32ReadOffset + (16*u8LoopCount)), u8NoOf16BytesBlocks, au8TempBuffer);
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                        memcpy(au8TempBuffer, (pu8CustData+(16*u8LoopCount)), u8NoOf16BytesBlocks);
#endif
                        for(u8LoopCount = 0; u8LoopCount < u8NoOf16BytesBlocks; u8LoopCount++)
                        {
#if JENNIC_CHIP_FAMILY == JN514x
                            au8TempBuffer[u8LoopCount] = (au8TempBuffer[u8LoopCount]^au8ODataOut[(u8LoopCount)]) ^ au8DataOut[(u8LoopCount)] ;
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                            au8TempBuffer[u8LoopCount] = (au8TempBuffer[u8LoopCount] ^ au8DataOut[(u8LoopCount)]);
#endif
                        }

                        vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common, (u32WriteOffset + (16*u8LoopCount)), u8NoOf16BytesBlocks, au8TempBuffer);
                    }
                    else
                    {
#if JENNIC_CHIP_FAMILY == JN514x
                        u32ReadOffset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation] * sNvmDefsStruct.u32SectorSize);
#endif
                        u32WriteOffset += (psOTA_Common->sOTACallBackMessage.u8ImageStartSector[psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation] * sNvmDefsStruct.u32SectorSize);

                        for(u8LoopCount = 0; u8LoopCount < u8NoOf16BytesBlocks; u8LoopCount++)
                        {
#if JENNIC_CHIP_FAMILY == JN514x
                            vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, (u32ReadOffset + (16*u8LoopCount)), 16, au8TempBuffer);
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                            memcpy(au8TempBuffer, (pu8CustData+(16*u8LoopCount)), 16);
#endif
                            vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common, (u32WriteOffset + (16*u8LoopCount)), 16, au8TempBuffer);

                        }

                        /* copy remaining data */
                        u8NoOf16BytesBlocks = u8SizeOfCustomData%16;
#if JENNIC_CHIP_FAMILY == JN514x
                        vOtaFlashLockRead(psEndPointDefinition, psOTA_Common, (u32ReadOffset + (16*u8LoopCount)), u8NoOf16BytesBlocks, au8TempBuffer);
#endif
#if JENNIC_CHIP_FAMILY == JN516x
                        memcpy(au8TempBuffer, (pu8CustData+(16*u8LoopCount)), 16);
#endif
                        vOtaFlashLockWrite(psEndPointDefinition, psOTA_Common, (u32WriteOffset + (16*u8LoopCount)), u8NoOf16BytesBlocks, au8TempBuffer);
                    }
                }
#endif
            }

            //vOTA_SetImageValidityFlag(psOTA_Common->sOTACallBackMessage.u8CurrentActiveImageLocation,psOTA_Common,FALSE, psEndPointDefinition);
            bValid = bOtaIsSerializationDataValid( psEndPointDefinition, psOTA_Common,
                                                   psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation);

            /* Check Status, if it is not true, there is no upgrade from new image */
            if(bValid)
            {
#if (OTA_MAX_CO_PROCESSOR_IMAGES != 0)
                /* Decrement downloadable images */
                psOTA_Common->sOTACallBackMessage.sPersistedData.u8NumOfDownloadableImages--;
                if((!psOTA_Common->sOTACallBackMessage.sPersistedData.u8NumOfDownloadableImages)||(!bIsCoProcessorImgUpgdDependent))
                {
                    sEndRequest.u8Status = OTA_STATUS_SUCCESS;
                }
                else
                {
                    sEndRequest.u8Status = OTA_REQUIRE_MORE_IMAGE;
                }
#else
                sEndRequest.u8Status = OTA_STATUS_SUCCESS;
#endif
                DBG_vPrintf(TRACE_VERIF, "IMAGE IS VALID\n");
            }
            else
            {
                /* Set Status as Invalid Image */
                /* even though signature verification is passed but serialization copy is failed */
                sEndRequest.u8Status = OTA_STATUS_IMAGE_INVALID;

                /* Give a callback to the user, serialization copying failed */
                eOtaSetEventTypeAndGiveCallBack(psOTA_Common, E_CLD_OTA_INTERNAL_COMMAND_FAILED_COPYING_SERIALIZATION_DATA,psEndPointDefinition);

                vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_NORMAL,psEndPointDefinition,psOTA_Common);
                DBG_vPrintf(TRACE_VERIF, "IMAGE IS NOT VALID\n");
                /* Give a call back to the user */
                eOtaSetEventTypeAndGiveCallBack(psOTA_Common, E_CLD_OTA_INTERNAL_COMMAND_OTA_DL_ABORTED,psEndPointDefinition);
            }
        }
        else
        {
            sEndRequest.u8Status = OTA_STATUS_IMAGE_INVALID;
            vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_NORMAL,psEndPointDefinition,psOTA_Common);
            //vOTA_SetImageValidityFlag(psOTA_Common->sOTACallBackMessage.u8NextFreeImageLocation,psOTA_Common,FALSE, psEndPointDefinition);

            /* Give a call back to the user */
            eOtaSetEventTypeAndGiveCallBack(psOTA_Common, E_CLD_OTA_INTERNAL_COMMAND_OTA_DL_ABORTED,psEndPointDefinition);
        }
		#if OTA_ACKS_ON == TRUE
		        sZCL_Address.eAddressMode = E_ZCL_AM_SHORT;
		#else
		        sZCL_Address.eAddressMode = E_ZCL_AM_SHORT_NO_ACK;
		#endif
        sZCL_Address.uAddress.u16DestinationAddress = psOTA_Common->sOTACallBackMessage.sPersistedData.u16ServerShortAddress;

        eOTA_ClientUpgradeEndRequest(u8SourceEndpoint,
                                     u8DstEndpoint,
                                     &sZCL_Address,
                                     &sEndRequest);
        if(sEndRequest.u8Status == OTA_STATUS_SUCCESS)
        {  /* if it is a success case then enable retries and wait for response */
            psOTA_Common->sOTACallBackMessage.sPersistedData.u8Retry = 0;
            psOTA_Common->sOTACallBackMessage.sPersistedData.u32RequestBlockRequestTime = u32ZCL_GetUTCTime()+ OTA_TIME_INTERVAL_BETWEEN_END_REQUEST_RETRIES+1;
            vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_DL_COMPLETE,psEndPointDefinition,psOTA_Common);
        }
#if (OTA_MAX_CO_PROCESSOR_IMAGES != 0)
        else if(sEndRequest.u8Status == OTA_REQUIRE_MORE_IMAGE)
        {
            /* Give a call back to the user for requesting remaining images */
            eOtaSetEventTypeAndGiveCallBack(psOTA_Common, E_CLD_OTA_INTERNAL_COMMAND_REQUEST_QUERY_NEXT_IMAGES,psEndPointDefinition);

            /* Change the state to Normal State */
            vOtaClientUpgMgrMapStates(E_CLD_OTA_STATUS_NORMAL,psEndPointDefinition,psOTA_Common);
        }
#endif
        else
        {
             psOTA_Common->sOTACallBackMessage.sPersistedData.u32RequestBlockRequestTime = 0;
        }
        /* request has been sent make sure we poll for the response too otherwise might miss it
           only valid for end device*/
        eOtaSetEventTypeAndGiveCallBack(psOTA_Common, E_CLD_OTA_INTERNAL_COMMAND_POLL_REQUIRED,psEndPointDefinition);
    }
    else
    {
        return E_ZCL_FAIL;
    }
    return E_ZCL_SUCCESS;
}
#endif
/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
