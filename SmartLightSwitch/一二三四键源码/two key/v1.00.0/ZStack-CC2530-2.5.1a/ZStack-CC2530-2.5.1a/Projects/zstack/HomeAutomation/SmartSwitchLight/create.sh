#!/bin/bash
#exit 1
PRO_NAME="OnOffLight"
PRO_NAME_1="onofflight"
PRO_NAME_2="ONOFFLIGHT"

find Source CC2530DB -type f | xargs sed -i "s/SampleLight/${PRO_NAME}/g"
find Source CC2530DB -type f | xargs sed -i "s/samplelight/${PRO_NAME_1}/g"
find Source CC2530DB -type f | xargs sed -i "s/SAMPLELIGHT/${PRO_NAME_2}/g"

cd Source

mv OSAL_SampleLight.c OSAL_${PRO_NAME}.c
mv zcl_samplelight.c zcl_${PRO_NAME_1}.c
mv zcl_samplelight_data.c zcl_${PRO_NAME_1}_data.c
mv zcl_samplelight.h zcl_${PRO_NAME_1}.h

cd ..

cd CC2530DB

mv SampleLight.ewd ${PRO_NAME}.ewd
mv SampleLight.ewp ${PRO_NAME}.ewp 
mv SampleLight.eww ${PRO_NAME}.eww

cd ..
