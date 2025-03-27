#!/bin/bash 
#---------------------------------------------------------------
#   Script: launch.sh
#  Mission: alpha_heron
#   Author: Tyler Paine
#   LastEd: 2023-Jun-19
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }


#---------------------------------------------------------------
#  Part 2: Set global var defaults
#---------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
VNAME1="abe"
VNAME2="ben"
VNAME3="cal"
VNAME4="deb"
AMT=8
REGION="pavlab"
CLEAN="no"
RANDSTART="true"


VLAUNCH_ARGS=" --auto --region=$REGION --sim  "
#VLAUNCH_ARGS=" --auto --sim --vname=$VNAME1 --index=1 --start=0,-10,180 "

#---------------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#---------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [SWITCHES] [time_warp]                                 "
	echo "  --help, -h         Show this help message                "
	echo "  --just_make, -j    Just make targ files, no launch       "
	echo "  --verbose, -v      Verbose output, confirm before launch "
	echo "  --clean, -cc       Run clean.sh and ktm prior to launch  "
	echo "  --amt=X            Number of vehicles to launch          "
	exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="-j"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="--verbose"
    elif [ "${ARGI}" = "--clean" -o "${ARGI}" = "-cc" ]; then
	CLEAN="yes"
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
        AMT="${ARGI#--amt=*}"
	if [ ! $AMT -ge 1 ]; then
	    echo "$ME: Vehicle amount must be >= 1."
	    exit 1
	fi
    else
        echo "$ME: Bad arg:" $ARGI "Exit Code 1."
        exit 1
    fi
done

VLAUNCH_ARGS+=" $VERBOSE $JUST_MAKE $TIME_WARP "

#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
# Not used Now


#-------------------------------------------------------------
# Part 5A: If Cleaning enabled, clean first
#-------------------------------------------------------------
if [ "${CLEAN}" = "yes" ]; then
    vecho "Running ./clean.sh and ktm prior to launch"
    ./clean.sh; ktm
fi

#-------------------------------------------------------------
# Part 5B: Generate random starting positions, speeds and vnames
#-------------------------------------------------------------
#vecho "Picking vehicle starting positions."
if [ "${RANDSTART}" = "true" -o  ! -f "vpositions.txt" ]; then
    ./pickpos.sh $AMT
fi

# vehicle names and colors are always deterministic
pickpos --amt=$AMT --vnames  > vnames.txt
pickpos --amt=$AMT --colors  > vcolors.txt

VEHPOS=(`cat vpositions.txt`)
VNAMES=(`cat vnames.txt`)
COLORS=(`cat vcolors.txt`)

#-------------------------------------------------------------
# Part 5C: Launch the vehicles
#-------------------------------------------------------------
echo "AMT=$AMT"
for INDEX in `seq 1 $AMT`;
do
    echo "INDEX=$INDEX"
    sleep 0.2
    echo "after sleep"
    ARRAY_INDEX=`expr $INDEX - 1`
    echo $ARRAY_INDEX
    START=${VEHPOS[$ARRAY_INDEX]}
    echo $START
    VNAME=${VNAMES[$ARRAY_INDEX]}
    COLOR=${COLORS[$ARRAY_INDEX]}

    echo "here"
    VLAUNCH_ARGS+=" $NOCONFIRM "
    IX_VLAUNCH_ARGS=$VLAUNCH_ARGS
    IX_VLAUNCH_ARGS+=" --index=$INDEX --start=$START     "
    IX_VLAUNCH_ARGS+=" --color=$COLOR                    " 
    IX_VLAUNCH_ARGS+=" --vname=$VNAME  --shore=localhost "

    echo "$ME: Launching $VNAME ..."
    ./launch_vehicle.sh $IX_VLAUNCH_ARGS	
done

#---------------------------------------------------------------
#  Part 6: Launch the shoreside
#---------------------------------------------------------------
echo "$ME: Launching Shoreside ..."
./launch_shoreside.sh --auto $VERBOSE $JUST_MAKE $TIME_WARP

#---------------------------------------------------------------
# Part 7: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_shoreside.moos
kill -- -$$

