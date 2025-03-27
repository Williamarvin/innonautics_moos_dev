#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: pickpos.sh
#   Author: Michael Benjamin   
#     Date: Feb 4th 2020       
#    About: This call is common to both launch.sh and rlaunch.sh
#           One stop location for changing the polygons
#-------------------------------------------------------------- 

AMT=1
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "pickpos.sh [AMOUNT]           " 
	echo "  --help, -h                       " 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$AMT" = 1 ]; then 
        AMT=$ARGI
    else 
	echo "pickpos.sh: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

if [ ! $AMT -ge 0 ] ; then
    echo "Vehicle amount must be >= 0. Exiting now."
    exit 1
fi


pickpos --poly="-486800,7450 : -429536,60341 : -397415,3155 : -460262,-38305" \
        --poly="180785,-64040: 232460,18880  : 281340,-15431: 245030,-78335"  \
	--poly="179385,560722: 117935,507824 : 190560,490669: 222681,542137"  \
	--amt=$AMT --hdg=75,-100,0  --buffer=100 > vpositions.txt  
