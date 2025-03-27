#!/bin/bash
#--------------------------------------------------------------
#   Script: speedUpVid.sh                                     
#  Mission:                                                              
#   Author: Tyler Paine, from Mike Benjamin
#   LastEd: Feb 2025
#--------------------------------------------------------------

#--------------------------------------------------------------
#  Part 2: Set Global var defaults
#--------------------------------------------------------------
ME=`basename "$0"`
VERBOSE=""
VID_IN=""
VID_OUT=""
SPEED=1

#---------------------------------------------------------------
#  Part 3: Check for and handle command-line arguments            
#--------------------------------------------------------------- 
for ARGI; do
    CMD_ARGS+="${ARGI} "
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "Usage:                                               "
        echo "   $ME [OPTIONS]                                     "
        echo "                                                     "
        echo "Synopsis:                                            "
        echo "  $ME will speed up a video playback by a given      "
        echo "  amount using ffmpeg free tools.                    "
        echo "                                                     "
        echo "Options:                                             "
        echo "  --help, -h                                         "
        echo "    Show this help message                           "
        echo "  --verbose, -v                                      "
        echo "    Increase verbosity,  confirm before launch       "
        echo "  --vid_in=<filename>                                "
        echo "    Name of input video                              "
        echo "  --vid_out=<filename>                               "
        echo "    Name of output video. If not specified output is "
	echo "    <filename>_<speed>X<filename extension>          "
	echo "                                                     "
	echo "  --speed=<N>                                        "
        echo "    Playback rate                                    "
        echo "                                                     "
        echo "Examples:                                            "
        echo "  $ speedUpVid.sh --vid_in=in.mov --speed=4          "
        echo "                                                     "
        echo "Dependenices:                                        "
        echo "   ffmpeg                                            "
        exit 0
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE=$ARGI
    elif [ "${ARGI:0:9}" = "--vid_in=" ]; then
        VID_IN="${ARGI#--vid_in=*}"
    elif [ "${ARGI:0:10}" = "--vid_out=" ]; then
        VID_OUT="${ARGI#--vid_out=*}"
    elif [ "${ARGI:0:8}" = "--speed=" ]; then
        SPEED="${ARGI#--speed=*}"
    elif [ "${VID}" = "" ]; then
	VID_IN=$ARGI
    else
        echo "$ME: Bad arg:" $ARGI "Exit Code 1."
        exit 1
    fi

done

#---------------------------------------------------------------
#  Part 4: Sanity checks
#---------------------------------------------------------------
if [ "${VID_IN}" = "" ]; then
    echo "No video file provided. See --help. Exiting."
    exit 1
fi

echo $PATH

FFMPEG=`which ffmpeg`
if [ "$FFMPEG" = "" ]; then
    echo "Cannot find ffmpeg binary."
    exit 1
fi



#---------------------------------------------------------------
#  Part 5: If output name not specified, use a default name based on vid
#---------------------------------------------------------------
if [ "${VID_OUT}" = "" ]; then
    VID_BASE=`echo $VID_IN | cut -d '.' -f1`
    EXTENSION="${VID_IN##*.}"
    
    VID_OUT="${VID_BASE}_${SPEED}X.${EXTENSION}"
fi

#---------------------------------------------------------------
#  Part 6: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then
    echo "=================================="
    echo "  launch.sh SUMMARY               "
    echo "=================================="
    echo "$ME"
    echo "ARGS =  [${CMD_ARGS}]     "
    echo "VID_IN  =  [${VID_IN}]    "
    echo "VID_OUT =  [${VID_OUT}]   "
    echo "SPEED   =  [${SPEED}]     "
    echo -n "Hit the RETURN key to create the file"
    read ANSWER
fi


#---------------------------------------------------------------
#  Part 7: Create the vid
echo "${VID_IN}"
#---------------------------------------------------------------
ffmpeg -i "${VID_IN}" -filter:v "setpts=PTS/${SPEED}" "${VID_OUT}"
