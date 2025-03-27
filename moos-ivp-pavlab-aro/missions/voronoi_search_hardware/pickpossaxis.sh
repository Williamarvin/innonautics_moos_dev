#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: pickpos.sh
#   Author: Michael Benjamin   
#     Date: Feb 4th 2020       
#    About: This call is common to both launch.sh and rlaunch.sh
#           One stop location for changing the polygons
#-------------------------------------------------------------- 

AMT=1
ITER="new"
CHECK1="start"
CHECK2="end"
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "pickpos.sh [AMOUNT]           " 
	echo "  --help, -h                       " 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$AMT" = 1 ]; then 
        AMT=$ARGI
    elif [ "${ARGI:0:7}" = "--iter=" ]; then
        ITER="${ARGI#--iter=*}"
    else 
	echo "Bad Argument: $ARGI"
	exit 0
    fi
done

if [ ! $AMT -ge 0 ] ; then
    echo "Vehicle amount must be >= 0. Exiting now."
    exit 1
fi
echo "ITER Argument: $ITER"
echo "AMT Argument: $AMT"
if [ "${ITER}" = "${CHECK1}" ]; then
    #pickpos --poly="-3000,-5270:-3000,-5340:-4000,-5340:-4000,3000:-3700,3000:-3700,-5270" \
	pickpos --poly="-5000,-3000:-4600,-3000:-4600,-3200:-5000,-3200" \
	--amt=1  > vpositionss1.txt 
elif [ "${ITER}" = "${CHECK2}" ]; then
    pickpos --poly="7000,600:7000,1000:8000,1000:8000,600"  \
	--amt=1  > vpositionss2.txt 
else 
	pickpos --poly="3850,-3650:3950,-3750:4650,-3100:5600,-1800:6000,-1050:5900,-1000"  \
	--poly="7000,350:6400,-475:7000,-450:7000,-350" \
	--poly="2650,-4650:2650,-5340:3650,-5270:3650,-4450" \
	--amt=$AMT --hdg=270:355  > vpositionss.txt  
fi


