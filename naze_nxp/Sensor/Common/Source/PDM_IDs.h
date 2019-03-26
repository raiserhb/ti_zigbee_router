/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          PDM_IDs.h
 *
 * DESCRIPTION:        Persistant Data Manager Id's
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

#ifndef  PDM_IDS_H_
#define  PDM_IDS_H_

#if defined __cplusplus
extern "C" {
#endif


/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
	// Ricky 
#define PDM_ID_APP_SENSOR                0x1
#define PDM_ID_POWER_ON_COUNTER          0xa
#define PDM_ID_APP_REPORTS               0xb
#define PDM_ID_APP_NTAG_NWK         	 0xc

/*  application ids start from 0Xa100 */
#define PDM_ID_APP_TL_ROUTER				(0xA100)
//#define PDM_ID_APP_SCENES_DATA				(0xA101)
#define PDM_ID_BDB_LOGICALCHANNEL			(0xA101)

#define PDM_ID_OTA_DATA						(0xA102)
#define PDM_ID_OTA_FILEOFFSET				(0xA103)
//#define PDM_ID_APP_CLD_GP_TRANS_TABLE		(0xA103)
//#define PDM_ID_APP_CLD_GP_SINK_PROXY_TABLE	(0xA104)
//#define PDM_ID_APP_REPORTS					0xA105
//#define PDM_ID_POWER_ON_COUNTER				0xA106
//#define PDM_ID_ZCL_ATTRIB					0xA107
//#define PDM_ID_APP_NTAG_NWK					0xA108
//#define PDM_ID_APP_JOIN_TABLE				0xA109
//#define PDM_ID_APP_SAVE_STATUS				0xA110
//#define PDM_ID_APP_ONOFF_STATUS				0xA111

#define PDM_ID_APP_POWERCONFIG				0xA112
#define PDM_ID_APP_BIND_UNLOCKCUT			0xA113
//#define PDM_ID_APP_PRODUCTCODE				0xA114

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif /* PDM_IDS_H_ */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
