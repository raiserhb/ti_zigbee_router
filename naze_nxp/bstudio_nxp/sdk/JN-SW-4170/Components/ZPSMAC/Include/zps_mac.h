/*****************************************************************************
 *
 * MODULE:             Zigbee Protocol Stack MAC Shim
 *
 * COMPONENT:          zps_mac.h
 *
 * AUTHOR:             RCC
 *
 * DESCRIPTION:        MAC shim layer to provide PDU manager i/f to NWK layer
 *
 * $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/Zigbee%20Protocol%20Stack/Modules/MAC/Branches/ZBPRO_R20_v1.1_NoRTOS/Include/zps_mac.h $
 *
 * $Revision: 77430 $
 *
 * $LastChangedBy: nxp29741 $
 *
 * $LastChangedDate: 2016-02-29 10:57:17 +0000 (Mon, 29 Feb 2016) $
 *
 * $Id: zps_mac.h 77430 2016-02-29 10:57:17Z nxp29741 $
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
 ****************************************************************************/

#ifndef _zps_mac_h_
#define _zps_mac_h_

#ifdef __cplusplus
extern "C" {
#endif

/***********************/
/**** INCLUDE FILES ****/
/***********************/

#include "jendefs.h"
#include "mac_sap.h"

/************************/
/**** MACROS/DEFINES ****/
/************************/

/**************************/
/**** TYPE DEFINITIONS ****/
/**************************/
typedef enum MAC_teRadioType_tag				/* Indicates MAC Radio Frequency types */
{
	E_MAC_FREQ_2400=0,
	E_MAC_FREQ_868=1,
	E_MAC_FREQ_915=2,
	E_MAC_FREQ_DEVICE_NOT_FOUND=0xFE,
	E_MAC_FREQ_UNKNOWN_ERROR=0xFF

} MAC_teRadioType;

/****************************/
/**** EXPORTED VARIABLES ****/
/****************************/

extern PUBLIC const uint32 ZPS_g_u32MacVersion;

/****************************/
/**** EXPORTED FUNCTIONS ****/
/****************************/

PUBLIC void
ZPS_vNwkHandleMcpsDcfmInd(void *pvNwk,
                          MAC_DcfmIndHdr_s *psMcpsDcfmInd);
PUBLIC MAC_teRadioType
ZPS_eMacRadioTypeFromChannelMask(uint32 u32ChannelMask);

PUBLIC teMacStatus
ZPS_eMacChannelsToChannelMask(uint8 u8Page, uint8 *pu8ChannelList, uint8 u8Size, uint32 *pu32Mask);

PUBLIC uint8
ZPS_u8MacChannelsFromChannelMask(uint32 u32Mask, uint8  *pu8Size, uint8 *pu8Page, uint8 *pu8ChannelList);


PUBLIC uint8
ZPS_u8MacGetChannelOffset(uint8 u8Page);

PUBLIC void
ZPS_vMacPibSetEBFilteringEnable(bool_t bEBFilteringEnabled);

PUBLIC void
ZPS_vMacPibSetEBR_Payload(const uint8* pu8EBRPayload, uint8 u8EBRPayloadLen);



#ifdef __cplusplus
};
#endif

#endif /* _zps_mac_h_ */

/* End of file $Id: zps_mac.h 77430 2016-02-29 10:57:17Z nxp29741 $ *******************************************/

