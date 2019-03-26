/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          zcl_options.h
 *
 * DESCRIPTION:        ZCL Options Header for ZLO Occupancy Sensor
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

#ifndef ZCL_OPTIONS_H
#define ZCL_OPTIONS_H

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/*                      ZCL Specific initialization                         */
/****************************************************************************/
/* Disable APS acks */
#define ZCL_DISABLE_APS_ACK                                 1

/* This is the NXP manufacturer code. If creating new a manufacturer        */
/* specific command apply to the Zigbee alliance for an Id for your company */
/* Also update the manufacturer code in .zpscfg: Node Descriptor->misc      */
#define ZCL_MANUFACTURER_CODE                                0x117E //feibit

/*  Enabling various server and clients for command which act across entire  */
/*  profile                                                                  */
#define ZCL_ATTRIBUTE_READ_SERVER_SUPPORTED
#define ZCL_ATTRIBUTE_READ_CLIENT_SUPPORTED
#define ZCL_ATTRIBUTE_WRITE_SERVER_SUPPORTED
#define ZCL_ATTRIBUTE_REPORTING_SERVER_SUPPORTED
#define ZCL_ATTRIBUTE_REPORTING_CLIENT_SUPPORTED
#define ZCL_CONFIGURE_ATTRIBUTE_REPORTING_SERVER_SUPPORTED
#define ZCL_READ_ATTRIBUTE_REPORTING_CONFIGURATION_SERVER_SUPPORTED

/* Bind server related configuration */
#define ZCL_NUMBER_OF_REPORTS     2
#define ZLO_MIN_REPORT_INTERVAL   1
#define ZLO_MAX_REPORT_INTERVAL   0x00

/* Enable wild card profile */
#define ZCL_ALLOW_WILD_CARD_PROFILE
/****************************************************************************/
/*                             Enable Cluster                               */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to enable         */
/* cluster and their client or server instances                             */
/****************************************************************************/
#define CLD_BASIC
#define BASIC_SERVER

#define CLD_ZLL_COMMISSION
#define ZLL_COMMISSION_SERVER
/* Enabling groups client in case F&B groups is required */
#define CLD_GROUPS
#define GROUPS_CLIENT

//Ricky
#ifdef BUILD_OTA
#define CLD_OTA
#define OTA_CLIENT
#endif

#define CLD_POWER_CONFIGURATION
#define POWER_CONFIGURATION_SERVER

#define CLD_IDENTIFY
#define IDENTIFY_CLIENT
#define IDENTIFY_SERVER

#define CLD_OCCUPANCY_SENSING
#define OCCUPANCY_SENSING_SERVER

#define CLD_ALARMS
#define ALARMS_CLIENT
#define ALARMS_SERVER

#define CLD_TIME
#define TIME_CLIENT
#define TIME_SERVER

#define CLD_DOOR_LOCK
#define DOOR_LOCK_SERVER

#define CLD_IASWD
#define IASWD_SERVER
#define IASWD_CLIENT
//#define CLD_SCENES
//#define SCENES_SERVER


/****************************************************************************/
/*             Basic Cluster - Optional Attributes                          */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* attributes to the basic cluster.                                         */
/****************************************************************************/
#define   CLD_BAS_ATTR_APPLICATION_VERSION
#define   CLD_BAS_ATTR_STACK_VERSION
#define   CLD_BAS_ATTR_HARDWARE_VERSION
#define   CLD_BAS_ATTR_MANUFACTURER_NAME
#define   CLD_BAS_ATTR_MODEL_IDENTIFIER
#define   CLD_BAS_ATTR_DATE_CODE
#define   CLD_BAS_ATTR_DEVICE_ENABLED
#define   CLD_BAS_ATTR_SW_BUILD_ID
#define   CLD_BAS_ATTR_GENERIC_DEVICE_CLASS
#define   CLD_BAS_ATTR_GENERIC_DEVICE_TYPE
#define   CLD_BAS_ATTR_PRODUCT_CODE
#define   CLD_BAS_ATTR_PRODUCT_URL

#ifdef DB_DOORLOCK
	#define		CLD_BAS_ATTR_ID_IR_DATA
	#define		CLD_BAS_ATTR_ID_TRANSPRT_TRANS
	#define		CLD_BAS_ATTR_ID_TRANSPRT_TRANS_CNT
#endif

#ifdef YYH_CUSTOM
#define		CLD_BAS_MANUF_NAME_SIZE		(20)
#define		CLD_BAS_MODEL_ID_SIZE		(9)
#else
#define		CLD_BAS_MANUF_NAME_SIZE		(4)
#define		CLD_BAS_MODEL_ID_SIZE		(16)
#endif

#define		CLD_BAS_APP_VERSION			(1)
#define		CLD_BAS_STACK_VERSION		(2)
#define		CLD_BAS_HARDWARE_VERSION	(1)
#define		CLD_BAS_DATE_SIZE			(8)
#define		CLD_BAS_POWER_SOURCE        E_CLD_BAS_PS_BATTERY
#define		CLD_BAS_SW_BUILD_SIZE		(9)
#define		CLD_BAS_URL_SIZE			(12)
#define		CLD_BAS_PCODE_SIZE			(30)

#ifdef DB_DOORLOCK
#define		CLD_BAS_IRDATA_SIZE			(64)
#define		CLD_BAS_TRANSPRT_SIZE		(64)
#define		CLD_BAS_TRANSCNTT_SIZE		(64)
#endif

/****************************************************************************/
/*             Occupancy Sensing Cluster - Optional Attributes              */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* attributes to the Occupancy Sensing                                      */
/****************************************************************************/
#define CLD_OS_ATTR_PIR_OCCUPIED_TO_UNOCCUPIED_DELAY
#ifdef PIR_TYPE_PWM
    #define CLD_OS_ATTR_PIR_UNOCCUPIED_TO_OCCUPIED_DELAY
    #define CLD_OS_ATTR_PIR_UNOCCUPIED_TO_OCCUPIED_THRESHOLD
#endif

/****************************************************************************/
/*             Basic Cluster - Optional Commands                            */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* commands to the basic cluster.                                           */
/****************************************************************************/
#define CLD_BAS_CMD_RESET_TO_FACTORY_DEFAULTS

/****************************************************************************/
/*             Identify Cluster - Optional Commands                         */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* commands to the identify cluster.                                        */
/****************************************************************************/
#define   CLD_IDENTIFY_CMD_TRIGGER_EFFECT

/****************************************************************************/
/*             Get RSSI                                                     */
/*                                                                          */
/****************************************************************************/
#define CLD_DIAGNOSTICS
#define DIAGNOSTICS_CLIENT
#define DIAGNOSTICS_SERVER
#define CLD_DIAGNOSTICS_ATTR_ID_LAST_MESSAGE_LQI
#define CLD_DIAGNOSTICS_ATTR_ID_LAST_MESSAGE_RSSI


/****************************************************************************/
/*             OTA Cluster - Optional Attributes							*/
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* attributes to the OTA cluster.                                           */
/****************************************************************************/
#ifdef  CLD_OTA
    #define OTA_DEMO_TIMINGS                            // define this fior the fast timings for edemo purposes

    #define OTA_ACKS_ON FALSE
    #define OTA_MAX_CO_PROCESSOR_IMAGES             0
    #define OTA_CLD_ATTR_CURRENT_FILE_VERSION
    #define OTA_MAX_BLOCK_SIZE                      48      // in multiples of 16 (internal flash requirement)
#ifdef OTA_DEMO_TIMINGS
	// Ricky 3 -> 0
    #define OTA_TIME_INTERVAL_BETWEEN_RETRIES       0       // Valid only if OTA_TIME_INTERVAL_BETWEEN_REQUESTS not defined
    #define CLD_OTA_MAX_BLOCK_PAGE_REQ_RETRIES      5      // count of block reqest failure befiore abandoning download
#else
    #define OTA_TIME_INTERVAL_BETWEEN_REQUESTS      RND_u32GetRand(10,20)
    #define OTA_TIME_INTERVAL_BETWEEN_RETRIES       RND_u32GetRand(10,20)      // Valid only if OTA_TIME_INTERVAL_BETWEEN_REQUESTS not defined
    #define CLD_OTA_MAX_BLOCK_PAGE_REQ_RETRIES      240                        // count of block reqest failure befiore abandoning download
#endif
    #define OTA_STRING_COMPARE
    #define OTA_MAX_IMAGES_PER_ENDPOINT             1
#endif

/****************************************************************************/
/****************************************************************************/
/*             Touchlink Cluster - Optional Attributes                 */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* attributes to the Touchlink cluster.                                          */
/****************************************************************************/
/*
 * Use this if you have the ZLL Master Key
 */
//#define TL_SUPPORTED_KEYS ( TL_MASTER_KEY_MASK )
/*
 * Use the Test and Certification Keys
 */
#define TL_SUPPORTED_KEYS (TL_TEST_KEY_MASK | TL_CERTIFICATION_KEY_MASK | TL_MASTER_KEY_MASK)
#define TL_LEGACY_PROFILE_ID   (0xc05e);
#define TL_LEGACY_DEVICE_ID    (0x000A);
#define TL_LEGACY_VERSION_ID    (2)
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* ZCL_OPTIONS_H */
