/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_reporting.c
 *
 * DESCRIPTION:        ZLO Demo : OccupancySensor Report (Implementation)
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
#include <string.h>
#include "dbg.h"
#include "ZTimer.h"
#include "app_main.h"
#include "pdum_gen.h"
#include "pdm.h"
#include "pdum_gen.h"
#include "app_common.h"
#include "PDM_IDs.h"
#include "zcl_options.h"
#include "app_zbp_utilities.h"
#include "zcl_common.h"
#include "app_reporting.h"

#include "zcl.h"
#include "App_OccupancySensor.h"
#include "PowerConfiguration.h"

#ifdef DB_DOORLOCK
#include "App_DB_DoorLock.h"
#endif
#include "app_sampledoorlock_uart.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_REPORT
    #define TRACE_REPORT   FALSE
#else
    #define TRACE_REPORT   TRUE
#endif
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/*There is just one attributes at this point - Occupancy Attribute */

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern const uint8 u8MyEndpoint;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/*Just one reports for time being*/
PRIVATE tsReports asSavedReports[OCCUPANCY_NUMBER_OF_REPORTS];

PUBLIC uint8 sBasicWriteAttributePValue[64];
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/


/****************************************************************************
 *
 * NAME: eRestoreReports
 *
 * DESCRIPTION:
 * Loads the reporting information from the EEPROM/PDM
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC PDM_teStatus eRestoreReports( void )
{
    /* Restore any report data that is previously saved to flash */
    uint16 u16ByteRead;
    PDM_teStatus eStatusReportReload = PDM_eReadDataFromRecord(PDM_ID_APP_REPORTS,
                                                              asSavedReports,
                                                              sizeof(asSavedReports),
                                                              &u16ByteRead);

    DBG_vPrintf(TRACE_REPORT,"\nAPP Report: eStatusReportReload=%d",eStatusReportReload);
    /* Restore any application data previously saved to flash */

    return  (eStatusReportReload);
}

/****************************************************************************
 *
 * NAME: vMakeSupportedAttributesReportable
 *
 * DESCRIPTION:
 * Makes the attributes reportable for Occupancy attribute
 *
 * RETURNS:
 * void
 *	//Ricky Mark
 ****************************************************************************/
PUBLIC void vMakeSupportedAttributesReportable(void)
{
    uint16 u16AttributeEnum;
    uint16 u16ClusterId;
    uint8 i;

    tsZCL_AttributeReportingConfigurationRecord*    psAttributeReportingConfigurationRecord;

    for(i=0; i<OCCUPANCY_NUMBER_OF_REPORTS; i++)
    {
        u16AttributeEnum=asSavedReports[i].sAttributeReportingConfigurationRecord.u16AttributeEnum;
        u16ClusterId= asSavedReports[i].u16ClusterID;
        psAttributeReportingConfigurationRecord = &(asSavedReports[i].sAttributeReportingConfigurationRecord);
        DBG_vPrintf(TRACE_REPORT, "Cluster %04x Attribute %04x Min %d Max %d IntV %d Direct %d Change %d\n",
                u16ClusterId,
                u16AttributeEnum,
                asSavedReports[i].sAttributeReportingConfigurationRecord.u16MinimumReportingInterval,
                asSavedReports[i].sAttributeReportingConfigurationRecord.u16MaximumReportingInterval,
                asSavedReports[i].sAttributeReportingConfigurationRecord.u16TimeoutPeriodField,
                asSavedReports[i].sAttributeReportingConfigurationRecord.u8DirectionIsReceived,
                asSavedReports[i].sAttributeReportingConfigurationRecord.uAttributeReportableChange.zint8ReportableChange);
        eZCL_CreateLocalReport( u8MyEndpoint, u16ClusterId, 0, TRUE, psAttributeReportingConfigurationRecord);
    }
}

/****************************************************************************
 *
 * NAME: vLoadDefaultConfigForReportable
 *
 * DESCRIPTION:
 * Loads a default configuration
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vLoadDefaultConfigForReportable(void)
{
    memset(asSavedReports, 0 ,sizeof(asSavedReports));
    int i;
    for (i=0; i<OCCUPANCY_NUMBER_OF_REPORTS; i++)
    {
        asSavedReports[i] = asDefaultReports[i];
    }

#if TRACE_REPORT

    DBG_vPrintf(TRACE_REPORT,"\nLoaded Defaults Records \n");
    for(i=0; i <OCCUPANCY_NUMBER_OF_REPORTS; i++)
    {
        DBG_vPrintf(TRACE_REPORT,"Cluster %04x Type %d Attr %04x Min %d Max %d IntV %d Direct %d Change %d\n",
                asSavedReports[i].u16ClusterID,
                asSavedReports[i].sAttributeReportingConfigurationRecord.eAttributeDataType,
                asSavedReports[i].sAttributeReportingConfigurationRecord.u16AttributeEnum,
                asSavedReports[i].sAttributeReportingConfigurationRecord.u16MinimumReportingInterval,
                asSavedReports[i].sAttributeReportingConfigurationRecord.u16MaximumReportingInterval,
                asSavedReports[i].sAttributeReportingConfigurationRecord.u16TimeoutPeriodField,
                asSavedReports[i].sAttributeReportingConfigurationRecord.u8DirectionIsReceived,
                asSavedReports[i].sAttributeReportingConfigurationRecord.uAttributeReportableChange.zuint8ReportableChange);
    }
#endif

    /*Save this Records*/
    PDM_eSaveRecordData(PDM_ID_APP_REPORTS,
                        asSavedReports,
                        sizeof(asSavedReports));

}


/****************************************************************************
 *
 * NAME: vSaveReportableRecord
 *
 * DESCRIPTION:
 * Loads a default configuration
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vSaveReportableRecord(  uint16 u16ClusterID,
                                    tsZCL_AttributeReportingConfigurationRecord* psAttributeReportingConfigurationRecord)
{
    int iIndex = 0;

    /*For MeasuredValue attribute in Illuminance Measurement Cluster*/
    asSavedReports[iIndex].u16ClusterID=u16ClusterID;
    memcpy( &(asSavedReports[iIndex].sAttributeReportingConfigurationRecord),
            psAttributeReportingConfigurationRecord,
            sizeof(tsZCL_AttributeReportingConfigurationRecord) );

    DBG_vPrintf(TRACE_REPORT,"Cluster %04x Type %d Attrib %04x Min %d Max %d IntV %d Direction %d Change %d\n",
            asSavedReports[iIndex].u16ClusterID,
            asSavedReports[iIndex].sAttributeReportingConfigurationRecord.eAttributeDataType,
            asSavedReports[iIndex].sAttributeReportingConfigurationRecord.u16AttributeEnum,
            asSavedReports[iIndex].sAttributeReportingConfigurationRecord.u16MinimumReportingInterval,
            asSavedReports[iIndex].sAttributeReportingConfigurationRecord.u16MaximumReportingInterval,
            asSavedReports[iIndex].sAttributeReportingConfigurationRecord.u16TimeoutPeriodField,
            asSavedReports[iIndex].sAttributeReportingConfigurationRecord.u8DirectionIsReceived,
            asSavedReports[iIndex].sAttributeReportingConfigurationRecord.uAttributeReportableChange.zuint8ReportableChange );

    /*Save this Records*/
    PDM_eSaveRecordData(PDM_ID_APP_REPORTS,
                        asSavedReports,
                        sizeof(asSavedReports));
}

/****************************************************************************
 *
 * NAME: vSendImmediateReport
 *
 * DESCRIPTION:
 * Method to send a report to all bound nodes.
 *
 ****************************************************************************/
PUBLIC void vSendImmediateReport(void)
{
    PDUM_thAPduInstance myPDUM_thAPduInstance;
    tsZCL_Address sDestinationAddress;

    DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Sending Report");

    /* get buffer to write the response in*/
    myPDUM_thAPduInstance = hZCL_AllocateAPduInstance();
    /* no buffers - return*/
    if(myPDUM_thAPduInstance == PDUM_INVALID_HANDLE)
    {
        DBG_vPrintf(TRACE_REPORT, "\nAPP Report: PDUM_INVALID_HANDLE");
    }

    /* Set the address mode to send to all bound device and don't wait for an ACK*/
    sDestinationAddress.eAddressMode = E_ZCL_AM_BOUND_NO_ACK;

    /* Send the report with all attributes.*/
    if (E_ZCL_SUCCESS != eZCL_ReportAllAttributes(&sDestinationAddress,
												  MEASUREMENT_AND_SENSING_CLUSTER_ID_OCCUPANCY_SENSING,
												  1,
												  0,
												  myPDUM_thAPduInstance))
    {

    	DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Error Sending Report");
    }

    /* free buffer and return*/
    PDUM_eAPduFreeAPduInstance(myPDUM_thAPduInstance);

    DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Report Sent");
}

/****************************************************************************
 *
 * NAME: vSendReportPowerConfig
 *
 * DESCRIPTION:
 * Method to send a report to all bound nodes.
 *
 ****************************************************************************/
PUBLIC void vSendReportPowerConfig(void)
{
    PDUM_thAPduInstance myPDUM_thAPduInstance;
    tsZCL_Address sDestinationAddress;

    DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Sending PowerConfig Report");

    /* get buffer to write the response in*/
    myPDUM_thAPduInstance = hZCL_AllocateAPduInstance();
    /* no buffers - return*/
    if(myPDUM_thAPduInstance == PDUM_INVALID_HANDLE)
    {
        DBG_vPrintf(TRACE_REPORT, "\nAPP Report: PDUM_INVALID_HANDLE");
    }

    /* Set the address mode to send to all bound device and don't wait for an ACK*/
    sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;

    /* Send the report with all attributes.*/
    if (E_ZCL_SUCCESS != eZCL_ReportAllAttributes(&sDestinationAddress,
												  GENERAL_CLUSTER_ID_POWER_CONFIGURATION,
												  1,
												  11,
												  myPDUM_thAPduInstance))
    {
    	DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Error Sending Report");
    }

    /* free buffer and return*/
    PDUM_eAPduFreeAPduInstance(myPDUM_thAPduInstance);

    DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Report Sent");
}

/****************************************************************************
 *
 * NAME: vSendBasicReport
 *
 * DESCRIPTION:
 * Method to send a report to all bound nodes.
 *
 ****************************************************************************/
PUBLIC void vSendBasicReport(void)
{
    PDUM_thAPduInstance myPDUM_thAPduInstance;
    tsZCL_Address sDestinationAddress;

    DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Sending Basic Report");

    /* get buffer to write the response in*/
    myPDUM_thAPduInstance = hZCL_AllocateAPduInstance();
    /* no buffers - return*/
    if(myPDUM_thAPduInstance == PDUM_INVALID_HANDLE)
    {
        DBG_vPrintf(TRACE_REPORT, "\nAPP Report: PDUM_INVALID_HANDLE");
    }

    /* Set the address mode to send to all bound device and don't wait for an ACK*/
    sDestinationAddress.eAddressMode = E_ZCL_AM_SHORT;
	sDestinationAddress.uAddress.u16DestinationAddress = 0x0000;
#if 1
    /* Send the report with all attributes.*/
    if (E_ZCL_SUCCESS != eZCL_ReportAttribute(&sDestinationAddress,
												GENERAL_CLUSTER_ID_BASIC,
												0x410C,
												1,
												11,
												myPDUM_thAPduInstance))
    {
    	DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Error Sending Report");
    }

#else
	RxSerialMsg[0]=1;
	RxSerialMsg[1]=0;
	tsZCL_CharacterString cStringTmp;
	tsZCL_TxPayloadItem psPayloadDefinition;
	psPayloadDefinition.u16Quantity = 1;//E_CLD_BAS_ATTR_ID_TRANSPRT_TRANS;
	psPayloadDefinition.eType = E_ZCL_CSTRING;
	psPayloadDefinition.pvData = &cStringTmp;
	cStringTmp.u8MaxLength=32;
	cStringTmp.u8Length=1;
	cStringTmp.pu8Data=RxSerialMsg;

	uint8 u8seq = u8GetTransactionSequenceNumber();
    DBG_vPrintf(TRACE_REPORT, "\n psPayloadDefinition ");

	if (E_ZCL_SUCCESS != eZCL_CustomCommandSend(
												1,								//uint8               u8SourceEndPointId,
												0,								//uint8               u8DestinationEndPointId,
												&sDestinationAddress,			//tsZCL_Address       *psDestinationAddress,
						                        GENERAL_CLUSTER_ID_BASIC,		//uint16              u16ClusterId,
						                        TRUE,							//bool_t              bDirection,
						                        E_ZCL_REPORT_ATTRIBUTES,		//uint8               u8CommandId,
						                        &u8seq,							//uint8               *pu8TransactionSequenceNumber,
						                        &psPayloadDefinition,			//tsZCL_TxPayloadItem *psPayloadDefinition,
						                        0,								//bool_t              bIsManufacturerSpecific,
						                        0x117E,							//uint16              u16ManufacturerCode,
						                        1								//uint8               u8ItemsInPayload)
												)
		)
	{
    	DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Error Sending Report");
	}
#endif

    DBG_vPrintf(TRACE_REPORT, "\n PDUM_eAPduFreeAPduInstance");

    /* free buffer and return*/
	PDUM_eAPduFreeAPduInstance(myPDUM_thAPduInstance);
    DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Report Sent");

}


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
