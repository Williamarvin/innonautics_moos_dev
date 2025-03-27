#!/bin/bash
#--------------------------------------------------------------
#   Script: launch.sh                                     
#  Mission: legrun_heron                                                              
#   Author: Mike Benjamin
#   LastEd: Oct 2023
#--------------------------------------------------------------
#  Part 1: Define convenience functions
#--------------------------------------------------------------

function vinfo2() {
    ffprobe -v quiet -print_format flat -show_streams $1 | grep -E 'width|height|duration='
}

function vwid() {
    ffprobe -v quiet -print_format flat -show_streams $1 | grep -E 'coded_width=' | cut -d '=' -f2
}
function vhgt() {
    ffprobe -v quiet -print_format flat -show_streams $1 | grep -E 'coded_height=' | cut -d '=' -f2
}

#--------------------------------------------------------------
#  Part 2: Set Global var defaults
#--------------------------------------------------------------
ME=`basename "$0"`
VERBOSE=""
WID=0
HGT=0
VID=""
GIF=""
BEG=0
DUR=59
RATE=10

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
        echo "  $ME will convert a video into an animated GIF using"
        echo "  the ffmpeg and ffprobe free tools.                 "
        echo "  If a size is provided, the gifsicle tool will be    "
        echo "  invoked to ajust the gif size.                     "
        echo "                                                     "
        echo "Options:                                             "
        echo "  --help, -h                                         "
        echo "    Show this help message                           "
        echo "  --verbose, -v                                      "
        echo "    Increase verbosity,  confirm before launch       "
        echo "  --vid=<filename>                                   "
        echo "    Name of input video                              "
        echo "  --gif=<filename>                                   "
        echo "    Name of output animated gif                      "
        echo "  --beg=<N>                                          "
        echo "    N in {0,1,...,9} seconds offset from start       "
        echo "    Default is 0.                                    "
        echo "  --rate=<N>                                         "
        echo "    N frames per second. Default is 10.              "
        echo "  --dur=<N>                                          "
        echo "    N in {00,01,...,59} duration of gif. Default 59. "
        echo "    Gif dur will not be longer than original video.  "
        echo "                                                     "
        echo "Examples:                                            "
        echo "  $ agif.sh --vid=file.mov --gif=foobar.gif          "
        echo "  $ agif.sh file.mov --beg=4                         "
        echo "  $ agif.sh file.mov --dur=08                        "
        echo "  $ agif.sh file.mov                                 "
        echo "                                                     "
        echo "Dependenices:                                        "
        echo "  ffprobe, ffmpeg, gifsicle (only if resizing)       "
        exit 0
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE=$ARGI
    elif [ "${ARGI:0:6}" = "--vid=" ]; then
        VID="${ARGI#--vid=*}"
    elif [ "${ARGI:0:6}" = "--gif=" ]; then
        GIF="${ARGI#--gif=*}"
    elif [ "${ARGI:0:6}" = "--beg=" ]; then
        BEG="${ARGI#--beg=*}"
    elif [ "${ARGI:0:6}" = "--dur=" ]; then
        DUR="${ARGI#--dur=*}"
    elif [ "${ARGI:0:7}" = "--rate=" ]; then
        RATE="${ARGI#--rate=*}"
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$BEG" = "0" ]; then
	BEG=$ARGI
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$DUR" = "0" ]; then
	DUR=$ARGI
    elif [ "${VID}" = "" ]; then
	VID=$ARGI
    else
        echo "$ME: Bad arg:" $ARGI "Exit Code 1."
        exit 1
    fi

done

#---------------------------------------------------------------
#  Part 4: Sanity checks
#---------------------------------------------------------------
if [ "${VID}" = "" ]; then
    echo "No video file provided. See --help. Exiting."
    exit 1
fi

WID=`vwid ${VID}`
HGT=`vhgt ${VID}`

echo $PATH

FFMPEG=`which ffmpeg`
if [ "$FFMPEG" = "" ]; then
    echo "Cannot find ffmpeg binary."
    exit 1
fi

FFPROBE=`which ffprobe`
if [ "$FFPROBE" = "" ]; then
    echo "Cannot find ffprobe binary."
    exit 1
fi


#---------------------------------------------------------------
#  Part 5: If GIF not specified, use a default name based on vid
#---------------------------------------------------------------
if [ "${GIF}" = "" ]; then
    VID_BASE=`echo $VID | cut -d '.' -f1`
    GIF="${VID_BASE}.gif"
fi

#---------------------------------------------------------------
#  Part 6: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then
    echo "=================================="
    echo "  launch.sh SUMMARY               "
    echo "=================================="
    echo "$ME"
    echo "ARGS =  [${CMD_ARGS}]    "
    echo "VID  =  [${VID}]         "
    echo "GIF  =  [${GIF}]         "
    echo "WID  =  [${WID}]         "
    echo "HGT  =  [${HGT}]         "
    echo "BEG  =  [${BEG}]         "
    echo "DUR  =  [${DUR}]         "
    echo "RATE =  [${RATE}]        "
    echo -n "Hit the RETURN key to create the GIF"
    read ANSWER
fi


#---------------------------------------------------------------
#  Part 7: Create the gif
#---------------------------------------------------------------
ffmpeg -ss "00:00:0${BEG}.000" -i "${VID}" -pix_fmt rgb24 -r "${RATE}" \
       -s "${WID}x${HGT}" -t "00:00:${DUR}.000" "${GIF}"
