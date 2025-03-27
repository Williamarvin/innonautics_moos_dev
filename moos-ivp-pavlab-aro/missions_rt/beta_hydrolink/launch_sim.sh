#!/bin/bash -e
#---------------------------------------------------------------
#   Script: launch_sim.sh
#  Mission: beta_hydrolink
#   Author: Ray Turrisi
#   LastEd: June 2023
#---------------------------------------------------------------
#  Part 1: Set global var defaults
#---------------------------------------------------------------

ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
VLAUNCH_ARGS=" --auto --sim --vname=$VNAME1 --index=1"
SLAUNCH_ARGS=" "

#---------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#---------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [SWITCHES] [time_warp]                                 "
	echo "  --help, -h         Show this help message                "
	echo "  --just_make, -j    Just make targ files, no launch       "
	echo "  --verbose, -v      Verbose output, confirm before launch "
	exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="-j"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="--verbose"
    elif [ "${ARGI}" = "--nologs" ]; then
	    VLAUNCH_ARGS+=" --nologs"
        SLAUNCH_ARGS+=" --nologs" 
    else
        echo "$ME: Bad arg:" $ARGI "Exit Code 1."
        exit 1
    fi
done

#---------------------------------------------------------------
#  Part 3: Initialize and Launch the vehicles
#---------------------------------------------------------------
echo "$ME: Launching Heron Simulation ..."
INDEX=1
./launch_heron.sh $VLAUNCH_ARGS $VERBOSE $JUST_MAKE $TIME_WARP --index=$INDEX

echo "$ME: Launching HydroLink Simulation ..."
INDEX=2
./launch_hydrolink.sh $VLAUNCH_ARGS $VERBOSE $JUST_MAKE $TIME_WARP --index=$INDEX

#---------------------------------------------------------------
#  Part 4: Launch the shoreside
#---------------------------------------------------------------
echo "$ME: Launching Shoreside ..."
INDEX=0
./launch_shoreside.sh --auto $VERBOSE $JUST_MAKE $TIME_WARP $SLAUNCH_ARGS

#---------------------------------------------------------------
# Part 5: Launch uMAC until the mission is quit
#---------------------------------------------------------------
if [ -z $JUST_MAKE ]; then 
    uMAC targ_shoreside.moos
    kill -- -$$
fi

