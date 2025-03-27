#!/bin/bash -e
#--------------------------------------------------------------
#   Script: launch_shoreside.sh                                    
#   Author: Michael Benjamin  
#     Date: June 2022
#--------------------------------------------------------------  
#  Part 1: Set Exit actions and declare global var defaults
#--------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
SPASS_ARGS=""
VPASS_ARGS=""
JUST_MAKE="no"

#--------------------------------------------------------------  
#  Part 2: Check for and handle command-line arguments
#--------------------------------------------------------------  
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [SWITCHES] [time_warp]         "
	echo "  --help, -h                                     " 
	echo "    Display this help message                    "
	echo "  --just_make, -j                                " 
	echo "    Just make targ files, but do not launch      "
	echo "  --pavlab, -p                                   "
	echo "    Set region to be MIT pavlab                  " 
	echo "  --mission-pt2pt, -mp                           " 
	echo "    Use the Point-to-Point mision                "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	VPASS_ARGS+=" --just_make"
	SPASS_ARGS+=" --just_make"
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--pavlab" -o "${ARGI}" = "-p" ]; then
        VPASS_ARGS+=" $ARGI"
        SPASS_ARGS+=" $ARGI"
    elif [ "${ARGI}" = "--noconfirm" -o "${ARGI}" = "-nc" ]; then
        VPASS_ARGS+=" $ARGI"
        SPASS_ARGS+=" $ARGI"
    elif [ "${ARGI}" = "--misssion-pt2pt" -o "${ARGI}" = "-mp" ]; then
        VPASS_ARGS+=" $ARGI"
    else 
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done


#--------------------------------------------------------------  
#  Part 3: Actual launch
#--------------------------------------------------------------  
./launch_vehicle.sh   --auto $VPASS_ARGS $TIME_WARP --vname=sally
sleep 1
./launch_shoreside.sh --auto $SPASS_ARGS $TIME_WARP 

#--------------------------------------------------------------  
#  Part 4: Launch uMAC until mission quit
#--------------------------------------------------------------  
if [ "${JUST_MAKE}" != "yes" ]; then    
    uMAC targ_shoreside.moos
fi
