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
    pickpos --poly="-120,-120:-125,-120:-125,-280:-120,-280" \
	--amt=1  --hdg=60:120 > vpositions1.txt 
elif [ "${ITER}" = "${CHECK2}" ]; then
    pickpos --poly="7000,600:7000,1000:8000,1000:8000,600"  \
	--amt=1  > vpositions2.txt 
else 
    pickpos --poly="-100,-100:100,-100:100,-300:-100,-300"  \
	--amt=$AMT --hdg=270:355  > vpositions.txt 
fi


