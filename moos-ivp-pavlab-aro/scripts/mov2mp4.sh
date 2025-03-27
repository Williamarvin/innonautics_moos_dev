#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: vsp.sh    
#   Author: Michael Benjamin   
#   LastEd: June 2021
#-------------------------------------------------------------- 
#  Part 1: Set global var defaults
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
VERBOSE="no"
CMD_ARGS=""
OLD_FILE=""
NEW_FILE=""

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                      " 
        echo "  --help, -h                                     "
        echo "    Display this help message                    "
        echo "  --verbose, -v                                  "
        echo "    Verbose output, confirm before launching.    "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${OLD_FILE}" = "" ]; then
	if [[ "${ARGI}" = *.mov ]]; then
	    OLD_FILE="${ARGI}"
	elif [[ "${ARGI}" = *.MOV ]]; then
	    OLD_FILE="${ARGI}"
	fi
    elif [ "${NEW_FILE}" = "" ]; then
	if [[ "${ARGI}" = *.mp4 ]]; then
            NEW_FILE="${ARGI}"
	fi
    else
        echo "$ME Bad Arg:" $ARGI " Exit Code 1"
        exit 1
    fi
done

#---------------------------------------------------------------
#  Part 3: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then 
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]"
    echo "TIME_WARP =     [${TIME_WARP}]"
    echo "OLD_FILE =      [${OLD_FILE}]"
    echo "NEW_FILE =      [${NEW_FILE}]"
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi

#---------------------------------------------------------------
#  Part 4: Convert the file
#---------------------------------------------------------------

ffmpeg -i $OLD_FILE -vcodec h264 -acodec mp2 $NEW_FILE

# https://trac.ffmpeg.org/wiki/How%20to%20speed%20up%20/%20slow%20down%20a%20video

exit 0

#-------------------------------------------------------
#  Part 5: Launch the processes
#-------------------------------------------------------
echo "Launching shoreside MOOS Community. WARP is:" $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &

#---------------------------------------------------------------
#  Part 6: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 7: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_shoreside.moos
kill -- -$$
