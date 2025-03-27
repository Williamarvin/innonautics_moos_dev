#!/bin/bash 
#------------------------------------------------------------
#   Script: init_field.sh
#   Author: M.Benjamin
#   LastEd: May 26 2024
#------------------------------------------------------------
#  Part 1: A convenience function for producing terminal
#          debugging/status output depending on verbosity.
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#------------------------------------------------------------
#  Part 2: Set global variable default values
#------------------------------------------------------------
ME=`basename "$0"`
VEHICLE_AMT="1"
VERBOSE=""
RAND_VPOS="no"

# custom
START="ring"
LEGS="yes"

#CIRCLE="x=50,y=-90,rad=40"
CIRCLE="x=23,y=-82,rad=32"

#------------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#------------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                      "
	echo "                                               "
	echo "Options:                                       "
	echo "  --amt=N            Num vehicles to launch    "
	echo "  --verbose, -v      Verbose, confirm values   "
	echo "  --rand, -r         Rand vehicle positions    "
	echo "                                               "
	echo "Options (custom):                              "
	echo "  --dock, -d         Vehicles start at dock    " 
	echo "  --nolegs,          Dont choose new legs      "
	echo "  --circle<spec>     Joust Circle              " 
       exit 0;
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
        VEHICLE_AMT="${ARGI#--amt=*}"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE=$ARGI
    elif [ "${ARGI}" = "--rand" -o "${ARGI}" = "-r" ]; then
        RAND_VPOS="yes"

    elif [ "${ARGI}" = "--dock" -o "${ARGI}" = "-d" ]; then
	START="dock"
    elif [ "${ARGI}" = "--nolegs" ]; then
	LEGS="no"
    elif [ "${ARGI:0:9}" = "--circle=" ]; then
        CIRLCE="${ARGI#--circle=*}"
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

#------------------------------------------------------------
#  Part 4: Source local coordinate grid if it exits
#------------------------------------------------------------

echo VEHICLE_AMT=$VEHICLE_AMT
echo LEGS=$LEGS

#------------------------------------------------------------
#  Part 5: Set starting positions, speeds, vnames, colors
#------------------------------------------------------------
if [ "${LEGS}" = "yes" ]; then
    echo VEHICLE_AMT=$VEHICLE_AMT
    pickjoust --circle=$CIRCLE --amt=$VEHICLE_AMT --file=joust.txt \
	      --spd=1.4,2.2 --maxtries=8000 --hdrs $REUSE	   \
              --ang_min_diff=15 --ang_max_diff=170 


#pickjoust --circle="x=50,y=-90,rad=40" --amt=4 --spd=1.8,2.2 --ang_min_diff=30  --ang_max_diff=170 --maxtries=8000 --hdrs 
fi

if [[ ! -f "joust.txt" ]]; then
    echo "Missing Joust.txt file. Exiting."
    exit 1
fi

if [ "${START}" = "dock" ]; then
    pickpos --amt=$VEHICLE_AMT --poly=pavlab --hdg=125:185 \
	    $REUSE --file=vpositions.txt  \
    
    if [[ ! -f "dock_start.txt" ]]; then
	echo "Missing dock_start.txt file. Exiting."
	exit 1
    fi
fi


#------------------------------------------------------------
#  Part 6: Set other aspects of the field, e.g., obstacles
#------------------------------------------------------------

#------------------------------------------------------------
#  Part 7: If verbose, show file contents
#------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then
    echo "--------------------------------------"
    echo "VEHICLE_AMT = $VEHICLE_AMT "
    echo "RAND_VPOS   = $RAND_VPOS   "
    echo "CIRCLE      = $CIRCLE      "
    echo "START       = $START       "
    echo "--------------------------------------(pos/spd)"
    echo "vpositions.txt:"; cat  vpositions.txt
    echo "--------------------------------------(custom)"
    echo "joust.txt:";      cat  joust.txt
    echo -n "Hit any key to continue"
    read ANSWER
fi

exit 0

