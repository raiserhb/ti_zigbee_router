/*****************************************************************************
 *
 * MODULE:             ZPS NWK
 *
 * COMPONENT:          zps_nwk_slist.h
 *
 * AUTHOR:             RCC
 *
 * DESCRIPTION:        ZPS NWK Single Linked List
 *
 * $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/Zigbee%20Protocol%20Stack/Modules/NWK/Branches/ZBPRO_R20_v2.0_NoRTOS/Include/zps_nwk_slist.h $
 *
 * $Revision: 77413 $
 *
 * $LastChangedBy: nxp29772 $
 *
 * $LastChangedDate: 2016-02-26 15:34:22 +0000 (Fri, 26 Feb 2016) $
 *
 * $Id: zps_nwk_slist.h 77413 2016-02-26 15:34:22Z nxp29772 $
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

#ifndef _zps_nwk_slist_h_
#define _zps_nwk_slist_h_

#ifndef MODULE
#define MODULE /* no modifier implies global */
#endif

/***********************/
/**** INCLUDE FILES ****/
/***********************/

#include "jendefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************/
/**** MACROS/DEFINES ****/
/************************/

#define ZPS_NWK_SLIST_LIMBO_NODE ((zps_tsNwkSlistNode *)0xFFFFFFFF)

/**************************/
/**** TYPE DEFINITIONS ****/
/**************************/

/**
 * ZPS NWK Single linked list node
 * @ingroup g_zps_nwk_slist
 * @note
 * Based on Jeneric JennicStdLib but without the unnecessary stuff
 */
typedef struct zps_tsNwkSlistNode_tag zps_tsNwkSlistNode;
struct zps_tsNwkSlistNode_tag
{
    zps_tsNwkSlistNode *psNext;
};

/**
 * ZPS NWK Single linked list
 * @ingroup g_zps_nwk_slist
 * @note
 * Based on Jeneric JennicStdLib but without the unnecessary stuff
 */
typedef struct zps_tsNwkSlist_tag
{
    zps_tsNwkSlistNode *psHead;     /**< Head node in list (NULL if empty) */
    zps_tsNwkSlistNode *psTail;     /**< Tail node in list (NULL if empty) */
} zps_tsNwkSlist;

/****************************/
/**** IMPORTED FUNCTIONS ****/
/****************************/

/**************************/
/**** MODULE VARIABLES ****/
/**************************/

/****************************/
/**** EXPORTED FUNCTIONS ****/
/****************************/

MODULE void
zps_vNwkSlistInit(zps_tsNwkSlist *psList);

MODULE void
zps_vNwkSlistAddToHead(zps_tsNwkSlist *psList,
                       zps_tsNwkSlistNode *psNode);

MODULE void
zps_vNwkSlistAddToTail(zps_tsNwkSlist *psList,
                       zps_tsNwkSlistNode *psNode);

MODULE void
zps_vNwkSlistInsertAfter(zps_tsNwkSlist *psList,
                         zps_tsNwkSlistNode *psCurrentNode,
                         zps_tsNwkSlistNode *psNewNode);

MODULE zps_tsNwkSlistNode *
zps_psNwkSlistRemoveFromHead(zps_tsNwkSlist *psList);

MODULE zps_tsNwkSlistNode *
zps_psNwkSlistRemove(zps_tsNwkSlist *psList,
                     zps_tsNwkSlistNode *psNode);

MODULE void
zps_vNwkSlistMakeLimboNode(zps_tsNwkSlistNode *psNode);

MODULE bool_t
zps_bNwkSlistIsLimboNode(zps_tsNwkSlistNode *psNode);

#ifdef __cplusplus
};
#endif

#endif /* _mac_prv_h_ */

/* End of file $Id: zps_nwk_slist.h 77413 2016-02-26 15:34:22Z nxp29772 $ *******************************************/
