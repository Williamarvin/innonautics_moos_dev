#!/bin/bash 
#--------------------------------------------------------------
#   Script: launch.sh
#  Mission: ufld_joust_pavlab
#   Author: Michael Benjamin  
#   LastEd: March 2nd, 2024 
#------------------------------------------------------------
#  Part 1: A convenience function for producing terminal
#          debugging/status output depending on verbosity.
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

#------------------------------------------------------------
#  Part 2: Set global variable default values
#------------------------------------------------------------
ME=`basename "$0"`
CMD_ARGS=""
TIME_WARP=1
VERBOSE=""
JUST_MAKE=""
LOG_CLEAN=""
VAMT="2"
MAX_VAMT="10"
RAND_VPOS=""

# Monte
XLAUNCHED="no"
NOGUI=""

# Custom
REUSE=""
CIRCLE="x=50,y=-90,rad=40"
START=""
COLAVD=""
NOLEGS=""

#--------------------------------------------------------------  
#  Part 3: Check for and handle command-line arguments
#--------------------------------------------------------------  
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                      "
	echo "                                               "
	echo "Options:                                       "
	echo "  --help, -h         Show this help message    " 
	echo "  --verbose, -v      Verbose, confirm launch   "
	echo "  --just_make, -j    Only create targ files    " 
	echo "  --log_clean, -lc   Run clean.sh bef launch   " 
	echo "  --amt=N            Num vehicles to launch    "
	echo "  --rand, -r         Rand vehicle positions    "
	echo "                                               "
	echo "Options (monte):                               "
	echo "  --xlaunched, -x    Launched by xlaunch       "
	echo "  --nogui, -ng       Headless launch, no gui   "
	echo "                                               "
	echo "Options (custom):                              "
	echo "  --dock, -d         Vehicles start at dock    " 
	echo "  --nolegs           Don't recalc joust legs   " 
	echo "  --colregs, -c      Enable COLREGS colavd     " 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE=$ARGI
    elif [ "${ARGI}" = "--log_clean" -o "${ARGI}" = "-lc" ]; then
	LOG_CLEAN=$ARGI
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
        VAMT="${ARGI#--amt=*}"
	if [ $VAMT -lt 1 -o $VAMT -gt $MAX_VAMT ]; then
	    echo "$ME: Veh amt range: [1, $MAX_VAMT]. Exit Code 2."
	    exit 2
	fi
    elif [ "${ARGI}" = "--rand" -o "${ARGI}" = "-r" ]; then
        RAND_VPOS=$ARGI
    elif [ "${ARGI}" = "--xlaunched" -o "${ARGI}" = "-x" ]; then
	XLAUNCHED="yes"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-ng" ]; then
	NOGUI="--nogui"

    elif [ "${ARGI}" = "--dock" -o "${ARGI}" = "-d" ]; then
	START=$ARGI
    elif [ "${ARGI}" = "--nolegs" ]; then
	NOLEGS=$ARGI
    elif [ "${ARGI}" = "--colregs" -o "${ARGI}" = "-c" ]; then
	COLAVD=$ARGI
    elif [ "${ARGI}" = "--cpa" -o "${ARGI}" = "-cpa" ]; then
	COLAVD=$ARGI
    else 
        echo "$ME Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done

#------------------------------------------------------------
#  Part 4: Set starting positions, speeds, vnames, colors
#------------------------------------------------------------
INIT_VARS=" --amt=$VAMT $RAND_VPOS $VERBOSE "
INIT_VARS+=" --circle=$CIRCLE $START " #custom
./init_field.sh $INIT_VARS $NOLEGS


#------------------------------------------------------------
#  Part 5: If verbose, show vars and confirm before launching
#------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then
    echo "============================================"
    echo "  $ME SUMMARY                   (ALL)       "
    echo "============================================"
    echo "CMD_ARGS =      [${CMD_ARGS}]               "
    echo "TIME_WARP =     [${TIME_WARP}]              "
    echo "JUST_MAKE =     [${JUST_MAKE}]              "
    echo "LOG_CLEAN =     [${LOG_CLEAN}]              "
    echo "VAMT =          [${VAMT}]                   "
    echo "MAX_VAMT =      [${MAX_VAMT}]               "
    echo "RAND_VPOS =     [${RAND_VPOS}]              "
    echo "--------------------------------(VProps)----"
    echo "VNAMES =        [${VNAMES[*]}]              "
    echo "VCOLORS =       [${VCOLOR[*]}]              "
    echo "START_POS =     [${VEHPOS[*]}]              "
    echo "--------------------------------(Monte)-----"
    echo "XLAUNCHED =     [${XLAUNCHED}]              "
    echo "NOGUI =         [${NOGUI}]                  "
    echo "--------------------------------(Custom)----"
    echo "CIRCLE =        [${CIRCLE}]                 "
    echo "START =         [${START}]                  "
    echo "COLAVD =        [${COLAVD}]                 "
    echo "                                            "
    echo -n "Hit any key to continue launch           "
    read ANSWER
fi

#-------------------------------------------------------------
# Part 6: Launch the vehicles
#-------------------------------------------------------------
VARGS=" --sim --auto $TIME_WARP $JUST_MAKE $VERBOSE $COLAVD "
VNAMES=""
for IX in `seq 1 $VAMT`;
do
    sleep 0.2
    IVARGS="$VARGS --mport=900${IX}  --pshare=920${IX} "
    
    # PX1,PY1 is first vertex of legrun and option for startpos
    PX1=`pluck joust.txt $IX --fld=px1`
    PY1=`pluck joust.txt $IX --fld=py1`
    HDG=`pluck joust.txt $IX --fld=hdg`
    SPD=`pluck joust.txt $IX --fld=spd`
    VNAME=`pluck joust.txt $IX --fld=vname`
    COLOR=`pluck joust.txt $IX --fld=vcolor`
    P1POS="x=${PX1},y=${PY1},heading=${HDG}"
    
    STAPOS=$P1POS
    if [ "${START}" != "" ]; then
	DX1=`pluck vpositions.txt $IX --fld=x`
	DY1=`pluck vpositions.txt $IX --fld=y`
	HDG=`pluck vpositions.txt $IX --fld=heading`	
	STAPOS="x=${DX1},y=${DY1},heading=${HDG}"
    fi
    
    IVARGS+=" --stock_spd=$SPD --color=$COLOR  " 
    IVARGS+=" --start_pos=${STAPOS} "
    IVARGS+=" --stock_spd=${SPD} "
    IVARGS+=" --vname=${VNAME} "
    IVARGS+=" --color=${COLOR} "

    if [ "${VNAMES}" != "" ]; then
	VNAMES+=":"
    fi
    VNAMES+=$VNAME
    
    vecho "About to launch vehicle: $IVARGS"
    ./launch_vehicle.sh $IVARGS	
done

#------------------------------------------------------------
#  Part 7: Launch the Shoreside mission file
#------------------------------------------------------------
SARGS=" --auto --mport=9000 --pshare=9200 $NOGUI "
SARGS+=" $TIME_WARP $JUST_MAKE $VERBOSE --vnames=$VNAMES"
vecho "Launching shoreside: $SARGS"
./launch_shoreside.sh $SARGS 

if [ "${JUST_MAKE}" != "" ]; then
    echo "$ME: Targ files made; exiting without launch."
    exit 0
fi

#------------------------------------------------------------
#  Part 8: Unless auto-launched, launch uMAC until mission quit
#------------------------------------------------------------
if [ "${XLAUNCHED}" != "yes" ]; then
    uMAC --paused targ_shoreside.moos
    trap "" SIGINT
    kill -- -$$
fi

exit 0
