/*****************************************************************************
 *
 * MODULE:             ZPS NWK
 *
 * COMPONENT:          zps_nwk_mac_sap.h
 *
 * AUTHOR:             RCC
 *
 * DESCRIPTION:        ZPS NWK Public exported API
 *
 * $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/Zigbee%20Protocol%20Stack/Modules/NWK/Branches/ZBPRO_R20_v2.0_NoRTOS/Include/zps_nwk_mac_sap.h $
 *
 * $Revision: 77413 $
 *
 * $LastChangedBy: nxp29772 $
 *
 * $LastChangedDate: 2016-02-26 15:34:22 +0000 (Fri, 26 Feb 2016) $
 *
 * $Id: zps_nwk_mac_sap.h 77413 2016-02-26 15:34:22Z nxp29772 $
 *
 *****************************************************************************
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
 ****************************************************************************/

/**
 * @defgroup g_zps_nwk ZPS Network Layer infrastructure
 */

#ifndef _zps_nwk_mac_sap_h_
#define _zps_nwk_mac_sap_h_

#ifdef __cplusplus
extern "C" {
#endif

/***********************/
/**** INCLUDE FILES ****/
/***********************/

#include "jendefs.h"
#include "mac_sap.h"
#include "mac_vs_sap.h"

/************************/
/**** MACROS/DEFINES ****/
/************************/

/**************************/
/**** TYPE DEFINITIONS ****/
/**************************/

/****************************/
/**** EXPORTED VARIABLES ****/
/****************************/

/****************************/
/**** EXPORTED FUNCTIONS ****/
/****************************/

PUBLIC void
ZPS_vNwkHandleMlmeDcfmInd(void *pvNwk,
        MAC_DcfmIndHdr_s *psMlmeDcfmInd);

PUBLIC void
ZPS_vNwkHandleMcpsVsDcfmInd(void *pvNwk,
                            MAC_tsMcpsVsDcfmInd *psMcpsDcfmInd);

#ifdef __cplusplus
};
#endif

#endif /* _zps_nwk_mac_sap_h_ */

/* End of file $Id: zps_nwk_mac_sap.h 77413 2016-02-26 15:34:22Z nxp29772 $ *******************************************/
