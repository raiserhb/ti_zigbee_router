#
# $1 : the OTA buildd directory
# $2 : the SDK directory name
# $3 : the manufacturer code
# $4 : 32 byte OTA header string
# $5 : JET VERSION 4 - JN5169 and 5 for JN5179
# $6 : Jennic chip family like JN516x and JN517x
# $7 : OTA Device Id
# $8 : APP_SW_VERSION Ricky

# Change the path to the OTA Build folder.
cd $1

# ####################################################################################################
# ###################################Build Unencrpted Client Binary ##################################################

# Add serialisation Data with ImageType = 0x0XXX - Indicates it is for Encrpted devices
../../../../bstudio_nxp/SDK/$2/Tools/OTAUtils/JET.exe -m combine -f DoorLock.bin -x configOTA_$6_Cer_Keys_HA_DoorLock.txt -v $5 -g 1 -k 0xffffffffffffffffffffffffffffffff -u $3 -t $7 -j $4

# Creat an Unencrpted Bootable Client with Veriosn 1
../../../../bstudio_nxp/SDK/$2/Tools/OTAUtils/JET.exe -m otamerge --embed_hdr -c outputffffffffffffffff.bin -o DoorLock.bin -v $5 -n $8 -u $3 -t $7 -j $4

# ###################Build OTA Unencrypted Upgarde Image from the Bootable Client  #########################
# Modify Embedded Header to reflect version 2 
../../../../bstudio_nxp/SDK/$2/Tools/OTAUtils/JET.exe -m otamerge --embed_hdr -c DoorLock.bin -o UpGradeImagewithOTAHeaderV2.bin -v $5 -n $8 -u $3 -t $7 -j $4

# Wrap the Image with OTA header with version 2
../../../../bstudio_nxp/SDK/$2/Tools/OTAUtils/JET.exe -m otamerge --ota -c UpGradeImagewithOTAHeaderV2.bin -o ClientUpGradeImagewithOTAHeaderV2.bin -v $5 -n $8 -u $3 -t $7 -j $4
../../../../bstudio_nxp/SDK/$2/Tools/OTAUtils/JET.exe -m otamerge --ota -c UpGradeImagewithOTAHeaderV2.bin -o ClientUpGradeImagewithOTAHeaderV2.ota -p 1 -v $5 -n $8 -u $3 -t $7 -j $4

# Modify Embedded Header to reflect version 3 
../../../../bstudio_nxp/SDK/$2/Tools/OTAUtils/JET.exe -m otamerge --embed_hdr -c DoorLock.bin -o UpGradeImagewithOTAHeaderV3.bin -v $5 -n 3 -u $3 -t $7 j $4

# Wrap the Image with OTA header with version 3
../../../../bstudio_nxp/SDK/$2/Tools/OTAUtils/JET.exe -m otamerge --ota -c UpGradeImagewithOTAHeaderV3.bin -o ClientUpGradeImagewithOTAHeaderV3.bin -v $5 -n 3 -u $3 -t $7 -j $4
../../../../bstudio_nxp/SDK/$2/Tools/OTAUtils/JET.exe -m otamerge --ota -c UpGradeImagewithOTAHeaderV3.bin -o ClientUpGradeImagewithOTAHeaderV3.ota -p 1 -v $5 -n 3 -u $3 -t $7 -j $4

# ####################################################################################################
# #################################### Clean Up Imtermediate files##################################################

# rm DoorLock.bin 
rm output*.bin
rm UpGradeImagewithOTAHeader*.bin

chmod 777 DoorLock.bin 
chmod 777 ClientUpGradeImagewithOTAHeaderV2.bin
chmod 777 ClientUpGradeImagewithOTAHeaderV3.bin
