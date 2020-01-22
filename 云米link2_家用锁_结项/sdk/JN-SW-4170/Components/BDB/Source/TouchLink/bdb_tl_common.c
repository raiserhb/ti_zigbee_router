/*###############################################################################
#
# MODULE:      BDB
#
# COMPONENT:   bdb_tl_common.c
#
# AUTHOR:      
#
# DESCRIPTION: BDB Touchlink Common functionality
#              
#
# $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/Components/BDB/Trunk/Source/TouchLink/bdb_tl_common.c $
#
# $Revision: 74650 $
#
# $LastChangedBy: nxp46755 $
#
# $LastChangedDate: 2015-11-26 08:57:07 +0000 (Thu, 26 Nov 2015) $
#
# $Id: bdb_tl_common.c 74650 2015-11-26 08:57:07Z nxp46755 $
#
###############################################################################
#
# This software is owned by NXP B.V. and/or its supplier and is protected
# under applicable copyright laws. All rights are reserved. We grant You,
# and any third parties, a license to use this software solely and
# exclusively on NXP products [NXP Microcontrollers such as JN514x, JN516x, JN517x].
# You, and any third parties must reproduce the copyright and warranty notice 
# and any other legend of ownership on each  copy or partial copy of the software.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE. 
# 
# Copyright NXP B.V. 2015-2016. All rights reserved
#
###############################################################################*/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "bdb_api.h"
#include "bdb_tl.h"
#include "zll_commission.h"
#include "dbg.h"
#include <rnd_pub.h>
#include <string.h>
#include <stdlib.h>


/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define ADJUST_POWER        TRUE
#define TL_SCAN_LQI_MIN    (100)

#ifndef DEBUG_JOIN
#define TRACE_JOIN            FALSE
#else
#define TRACE_JOIN            TRUE
#endif

#ifndef DEBUG_COMMISSION
#define TRACE_COMMISSION      FALSE
#else
#define TRACE_COMMISSION      TRUE
#endif

#ifndef DEBUG_TL_NEGATIVE
#define TRACE_TL_NEGATIVE       FALSE
#else
#define TRACE_TL_NEGATIVE       TRUE
#endif

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
PRIVATE tsReg128 sMasterKey = {0x11223344, 0x55667788, 0x99aabbcc, 0xddeeff00 };
//extern PUBLIC uint8 au8TouchLinkKey[16];
PRIVATE tsReg128 sCertKey = {0xc0c1c2c3, 0xc4c5c6c7,0xc8c9cacb,0xcccdcecf};
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME:
 *
 * DESCRIPTION:
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC uint8 BDB_u8TlGetRandomPrimary(void)
{
#ifndef FIXED_CHANNEL
    uint32 u32NoOfBits = 0;
        uint32 u32ChannelMask = BDBC_TL_PRIMARY_CHANNEL_SET;
        uint32 u32RandomBitNo, u32Channel;
        int i;

        for (i=0; i<27; i++)
        {
            if (u32ChannelMask & 0x01)
            {
                u32NoOfBits++;
            }
            u32ChannelMask >>= 1;
        }
        if (u32NoOfBits > 1)
        {
            u32RandomBitNo = RND_u32GetRand( 0, u32NoOfBits);
            u32RandomBitNo++;
        }
        else
        {
            u32RandomBitNo = 1;
        }

        u32NoOfBits = 0;
        u32Channel = 0;
        u32ChannelMask = BDBC_TL_PRIMARY_CHANNEL_SET;
        for (i=0; i<27 && u32NoOfBits != u32RandomBitNo; i++)
        {
            if (u32ChannelMask & 0x01)
            {
                u32NoOfBits++;
                u32Channel = i;
            }

            u32ChannelMask >>= 1;
        }
        DBG_vPrintf(TRACE_JOIN, "PickChannel %d\n", u32Channel);
        return (uint8)u32Channel;


#else
    return FIXED_CHANNEL;
#endif
}

/****************************************************************************
 *
 * NAME: BDB_u8TlNewUpdateID
 *
 * DESCRIPTION: determines which of 2 network update ids is
 * the freshest
 *
 *
 * RETURNS: the frestest nwk update id
 *
 *
 ****************************************************************************/
PUBLIC uint8 BDB_u8TlNewUpdateID(uint8 u8ID1, uint8 u8ID2 )
{
    if ( (abs(u8ID1-u8ID2)) > 200) {
        return MIN(u8ID1, u8ID2);
    }
    return MAX(u8ID1, u8ID2);
}

/****************************************************************************
 *
 * NAME: BDB_u8TlEncryptKey
 *
 * DESCRIPTION: encrypt the nwk key before transmitting it
 *
 *
 * RETURNS:
 *
 *
 ****************************************************************************/
PUBLIC uint8 BDB_u8TlEncryptKey( uint8* au8InData,
                                  uint8* au8OutData,
                                  uint32 u32TransId,
                                  uint32 u32ResponseId,
                                  uint8 u8KeyIndex)
{
    tsReg128 sExpanded;
    tsReg128 sTransportKey;

    tsReg128 sDataIn,sDataOut;

    sExpanded.u32register0 = u32TransId;
    sExpanded.u32register1 = u32TransId;
    sExpanded.u32register2 = u32ResponseId;
    sExpanded.u32register3 = u32ResponseId;

    switch (u8KeyIndex)
    {
        case TL_TEST_KEY_INDEX:
            sTransportKey.u32register0 = 0x50684c69;
            sTransportKey.u32register1 = u32TransId;
            sTransportKey.u32register2 = 0x434c534e;
            sTransportKey.u32register3 = u32ResponseId;
            break;
        case TL_MASTER_KEY_INDEX:
            bACI_ECBencodeStripe( &sMasterKey,
                                  TRUE,
                                  &sExpanded,
                                  &sTransportKey);
            break;
        case TL_CERTIFICATION_KEY_INDEX:
            bACI_ECBencodeStripe( &sCertKey,
                                  TRUE,
                                  &sExpanded,
                                  &sTransportKey);
            break;

        default:
            return 3;
            break;
    }

    memcpy(&sDataIn,au8InData,0x10);
    memcpy(&sDataOut,au8OutData,0x10);
    bACI_ECBencodeStripe(&sTransportKey,
                         TRUE,
                         &sDataIn,
                         &sDataOut);
    memcpy(au8OutData,&sDataOut,0x10);

    return 0;
}

/****************************************************************************
 *
 * NAME: BDB_eTlDecryptKey
 *
 * DESCRIPTION: decrypt the received nwk key
 *
 *
 * RETURNS:
 *
 *
 ****************************************************************************/
PUBLIC uint8 BDB_eTlDecryptKey( uint8* au8InData,
                                  uint8* au8OutData,
                                  uint32 u32TransId,
                                  uint32 u32ResponseId,
                                  uint8 u8KeyIndex)
{
    tsReg128 sTransportKey;
    tsReg128 sExpanded;

    sExpanded.u32register0 = u32TransId;
    sExpanded.u32register1 = u32TransId;
    sExpanded.u32register2 = u32ResponseId;
    sExpanded.u32register3 = u32ResponseId;

    switch (u8KeyIndex)
    {
        case TL_TEST_KEY_INDEX:
            sTransportKey.u32register0 = 0x50684c69;
            sTransportKey.u32register1 = u32TransId;
            sTransportKey.u32register2 = 0x434c534e;
            sTransportKey.u32register3 = u32ResponseId;
            break;
        case TL_MASTER_KEY_INDEX:
            bACI_ECBencodeStripe( &sMasterKey,
                                  TRUE,
                                  &sExpanded,
                                  &sTransportKey);
            break;
        case TL_CERTIFICATION_KEY_INDEX:
            bACI_ECBencodeStripe( &sCertKey,
                                  TRUE,
                                  &sExpanded,
                                  &sTransportKey);
            break;

        default:
            DBG_vPrintf(TRACE_COMMISSION, "***Ooops***\n");
            return 3;
            break;
    }

    vECB_Decrypt( (uint8*)&sTransportKey,
                  au8InData,
                  au8OutData);
#ifdef SHOW_KEY
    int i;
    for (i=0; i<16; i++) {
        DBG_vPrintf(TRACE_COMMISSION, "%02x ", au8OutData[i]);
    }
    DBG_vPrintf(TRACE_COMMISSION, "\n");
#endif

    return 0;
}

/****************************************************************************
 *
 * NAME: BDB_bTlIsKeySupported
 *
 * DESCRIPTION: tests if the given key index matches a supported key
 *
 *
 * RETURNS: True if Key index is supported
 *
 *
 ****************************************************************************/
PUBLIC bool BDB_bTlIsKeySupported(uint8 u8KeyIndex)
{
    uint16 u16KeyMask = (1<<u8KeyIndex);
    return (bool)(u16KeyMask & TL_SUPPORTED_KEYS);
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

