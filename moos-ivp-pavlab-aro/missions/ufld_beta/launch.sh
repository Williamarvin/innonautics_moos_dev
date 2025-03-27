#!/bin/bash -e
#--------------------------------------------------------------
#   Script: launch_shoreside.sh                                    
#   Author: Michael Benjamin  
#     Date: April 2020     
#--------------------------------------------------------------  
#  Part 1: Set Exit actions and declare global var defaults
#--------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
AUTO_LAUNCHED="no"


PASS_ARGS=""

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
	echo "  --verbose, -v                                  " 
	echo "    Verbose output, confirm before launching.    "
	echo "  --auto, -a                                     "
	echo "     Auto-launched by a script.                  "
	echo "     Will not launch uMAC as the final step.     "
	echo "  --pavlab, -p                                   "
	echo "    Set region to be MIT pavlab                  " 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="-j"
    elif [ "${ARGI}" = "--pavlab" -o "${ARGI}" = "-p" ]; then
        PASS_ARGS+=$ARGI
    else 
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done


#--------------------------------------------------------------  
#  Part 3: Pre-launch. Better to exit now if err building targs
#--------------------------------------------------------------  
./launch_shoreside.sh --auto $JUST_MAKE $PASS_ARGS $TIME_WARP
./launch_vehicle.sh   --auto $JUST_MAKE $PASS_ARGS $TIME_WARP --vname=abe

if [ "${JUST_MAKE}" = "-j" ] ; then
    exit 0
fi

#--------------------------------------------------------------  
#  Part 4: Actual launch
#--------------------------------------------------------------  
./launch_shoreside.sh --auto $PASS_ARGS $TIME_WARP
sleep 1
./launch_vehicle.sh   --auto $PASS_ARGS $TIME_WARP --vname=abe

#--------------------------------------------------------------  
#  Part 5: Launch uMAC until mission quit
#--------------------------------------------------------------  
uMAC targ_shoreside.moos
