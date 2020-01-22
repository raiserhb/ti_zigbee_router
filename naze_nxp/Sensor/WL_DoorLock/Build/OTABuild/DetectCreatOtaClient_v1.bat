# ####################################################################################################
#
# MODULE:      JN-AN-1189 ZigBee HA Demo
#
# DESCRIPTION: Batch file to build OTA client binary file
#
# ####################################################################################################
#
# This software is owned by NXP B.V. and/or its supplier and is protected
# under applicable copyright laws. All rights are reserved. We grant You,
# and any third parties, a license to use this software solely and
# exclusively on NXP products [NXP Microcontrollers such as  JN5168, JN5164,
# JN5161, JN5148, JN5142, JN5139]. 
# You, and any third parties must reproduce the copyright and warranty notice
# and any other legend of ownership on each copy or partial copy of the 
# software.
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
# Copyright NXP B.V. 2014. All rights reserved
#
# ####################################################################################################
# Change the path to the OTA Build folder.

# ####################################################################################################
# ###################################Build Unencrpted Client Binary ##################################################

# Add serialisation Data with ImageType = 0x0XXX - Indicates it is for Encrpted devices
JET.exe -m combine -f ZONE_JN5169_DR1199_CSW.bin -x configOTA_6x_Cer_Keys_HA_Switch.txt -v 4 -g 1 -k 0xffffffffffffffffffffffffffffffff -u 0x1037 -t 0x103 -j DR1199r1v2UNENCRYPTED00000JN5169

# Creat an Unencrpted Bootable Client with Veriosn as supplied
JET.exe -m otamerge --embed_hdr -c outputffffffffffffffff.bin -o ZONE_JN5169_DR1199_CSW_Client_v1.bin -v 4 -n 30 -u 0x1037 -t 0x103 -j DR1199r1v2UNENCRYPTED00000JN5169

# ###################Build OTA Unencrypted Upgarde Image from the Bootable Client  #########################
# Modify Embedded Header to reflect version as supplied 
JET.exe -m otamerge --embed_hdr -c ZONE_JN5169_DR1199_CSW_Client_v1.bin -o ZONE_JN5169_DR1199_CSW_v1_tmp.bin -v 4 -n 30 -u 0x1037 -t 0x103 -j DR1199r1v2UNENCRYPTED00000JN5169

# Wrap the Image with OTA header with version as supplied
JET.exe -m otamerge --ota -c ZONE_JN5169_DR1199_CSW_v1_tmp.bin -o ZONE_JN5169_DR1199_CSW_v1.bin -v 4 -n 30 -u 0x1037 -t 0x103 -j DR1199r1v2UNENCRYPTED00000JN5169

JET.exe -m otamerge --ota -c ZONE_JN5169_DR1199_CSW_v1_tmp.bin -o ZONE_JN5169_DR1199_CSW_v1.ota -p 1 -v 4 -n 30 -u 0x1037 -t 0x103 -j DR1199r1v2UNENCRYPTED00000JN5169

# ####################################################################################################
# #################################### Clean Up Imtermediate files##################################################
# rm ZONE_JN5169_DR1199_CSW.bin 
rm output*.bin
rm ZONE_JN5169_DR1199_CSW_v1_tmp.bin
rm ZONE_JN5169_DR1199_CSW_v1.bin
pause

#���ܣ�ZONE_JN5169_DR1199_CSW.binΪԴbin�ļ������ɴ�ota header��ZONE_JN5169_DR1199_CSW_Client_v1.bin�汾��ZONE_JN5169_DR1199_CSW_v1.otaΪota�����̼���-n��Ӱ汾��