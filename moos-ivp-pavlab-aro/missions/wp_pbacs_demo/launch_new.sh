#!/bin/bash -e
#---------------------------------------------------------------
#   Script: launch.sh
#  Mission: alpha_heron
#   Author: Mike Benjamin
#   LastEd: 2021-Jun-08
#---------------------------------------------------------------
#  Part 1: Set global var defaults
#---------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
GRIDSIZE="-SG"
PATHMODE="-OM"
VNUM="2"
N="0"
GRIDSTART=""
START=""
LAWNMIDDLEX="55"
LAWNSTARTX="-70"
GRIDFILE="1"
MIRROR=""

#---------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#---------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [SWITCHES] [time_warp]                                 "
	echo "  --help, -h         Show this help message                "
	echo "  --just_make, -j    Just make targ files, no launch       "
	echo "  --verbose, -v      Verbose output, confirm before launch "
	echo "                                                           "
	echo "  --fullgrid, -FG    Use full grid size                    "
	echo "  --smallgrid, -SG   Use small grid size                   "
	echo "                                                           "
	echo "  --lawnmode, -LM    lawnmower mode                        "
	echo "  --greedymode, -GM  greedy mode                           "  
	echo "  --othermode, -OM   other mode                            "
	echo "                                                           "
	echo "  --vnum=<# of vehicles>  How many vehicles to use         "
	exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="-j"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="--verbose"

    elif [ "${ARGI}" = "--fullgrid" -o "${ARGI}" = "-FG" ]; then
	GRIDSIZE="-FG"
    elif [ "${ARGI}" = "--smallgrid" -o "${ARGI}" = "-SG" ]; then
	GRIDSIZE="-SG"

    elif [ "${ARGI}" = "--lawnmode" -o "${ARGI}" = "-LM" ]; then
	PATHMODE="-LM"
    elif [ "${ARGI}" = "--greedymode" -o "${ARGI}" = "-GM" ]; then
	PATHMODE="-GM"
    elif [ "${ARGI}" = "--othermode" -o "${ARGI}" = "-OM" ]; then
	PATHMODE="-OM"

    elif [ "${ARGI:0:7}" = "--vnum=" ]; then
        VNUM="${ARGI#--vnum=*}"

    elif [ "${ARGI:0:11}" = "--gridfile=" ]; then
        GRIDFILE="${ARGI#--gridfile=*}"
    elif [ "${ARGI}" = "--mirror" ]; then
	MIRROR="--mirror"

    else
        echo "$ME: Bad arg:" $ARGI "Exit Code 1."
        exit 1
    fi
done

#---------------------------------------------------------------
#  Part 3: Initialize and Launch the vehicles
#---------------------------------------------------------------
echo "VNUM = $VNUM"

#  Set the grid corners here
#   
#   (1) ----------- (4)
#    |               |
#    |               |
#    |               |
#   (2) ----------- (3)
#
#   Does not have to be horizontal!!


if [ $GRIDSIZE = "-SG" ]; then
    X_1="-75"
    Y_1="-50"
    X_2="-75"
    Y_2="-220"
    X_3="185"
    Y_3="-220"
    X_4="185"
    Y_4="-50"
    #X_1="12"
    #Y_1="-18"
    #X_2="44"
    #Y_2="-96"
    #X_3="158"
    #Y_3="-36"
    #X_4="114"
    #Y_4="37"
    
    
    CELLSIZE="10"
    STARTY="-45"
else
    X_1="-290"
    Y_1="-140"
    X_2="-110"
    Y_2="-606"
    X_3="590"
    Y_3="-350"
    X_4="400"
    Y_4="140"
    
    CELLSIZE="20"
    STARTY="-55"
fi
## Create grid spec to pass to other apps
GRIDSPECS="--x1=$X_1 --y1=$Y_1 --x2=$X_2 --y2=$Y_2 --x3=$X_3 --y3=$Y_3 --x4=$X_4 --y4=$Y_4 --cellsize=$CELLSIZE"

## Create another smaller grid inside this grid to use for the grid start and
## explore end calculations
#  Set the grid corners here
#   
#   (1) ----------- (4)
#    |(s1)-------(s4)|
#    | |          |  |
#    |(s2)-------(s3)|
#   (2) ----------- (3)
#
#   Does not have to be horizontal!!
LENGTH_1_TO_3=`echo "scale=2; sqrt( ($X_1 - $X_3)^2 + ($Y_1 - $Y_3)^2 )" | bc`
LENGTH_2_TO_4=`echo "scale=2; sqrt( ($X_2 - $X_4)^2 + ($Y_2 - $Y_4)^2 )" | bc`

Xs_1=`echo "scale=2;  $X_1 + 0.707 * $CELLSIZE / $LENGTH_1_TO_3 * ($X_3 - $X_1) " | bc -l`
Ys_1=`echo "scale=2;  $Y_1 + 0.707 * $CELLSIZE / $LENGTH_1_TO_3 * ($Y_3 - $Y_1) " | bc -l`
Xs_2=`echo "scale=2;  $X_2 + 0.707 * $CELLSIZE / $LENGTH_2_TO_4 * ($X_4 - $X_2) " | bc -l`
Ys_2=`echo "scale=2;  $Y_2 + 0.707 * $CELLSIZE / $LENGTH_2_TO_4 * ($Y_4 - $Y_2) " | bc -l`
Xs_3=`echo "scale=2;  $X_3 + 0.707 * $CELLSIZE / $LENGTH_1_TO_3 * ($X_1 - $X_3) " | bc -l`
Ys_3=`echo "scale=2;  $Y_3 + 0.707 * $CELLSIZE / $LENGTH_1_TO_3 * ($Y_1 - $Y_3) " | bc -l`
Xs_4=`echo "scale=2;  $X_4 + 0.707 * $CELLSIZE / $LENGTH_2_TO_4 * ($X_2 - $X_4) " | bc -l`
Ys_4=`echo "scale=2;  $Y_4 + 0.707 * $CELLSIZE / $LENGTH_2_TO_4 * ($Y_2 - $Y_4) " | bc -l`


# Calculate some lengths quickly for use later
LENGTH_1_TO_4=`echo "scale=2; sqrt(  ($X_1 - $X_4)^2 + ($Y_1 - $Y_4)^2 )" | bc`
LENGTH_1_TO_2=`echo "scale=2; sqrt(  ($X_2 - $X_1)^2 + ($Y_2 - $Y_1)^2 )" | bc`
LENGTH_2_TO_3=`echo "scale=2; sqrt(  ($X_3 - $X_2)^2 + ($Y_3 - $Y_2)^2 )" | bc`
LENGTH_4_TO_3=`echo "scale=2; sqrt(  ($X_4 - $X_3)^2 + ($Y_4 - $Y_3)^2 )" | bc`
LENGTH_s1_TO_s4=`echo "scale=2; sqrt(  ($Xs_1 - $Xs_4)^2 + ($Ys_1 - $Ys_4)^2 )" | bc`
LENGTH_s1_TO_s2=`echo "scale=2; sqrt(  ($Xs_2 - $Xs_1)^2 + ($Ys_2 - $Ys_1)^2 )" | bc`
LENGTH_s2_TO_s3=`echo "scale=2; sqrt(  ($Xs_3 - $Xs_2)^2 + ($Ys_3 - $Ys_2)^2 )" | bc`
LENGTH_s4_TO_s3=`echo "scale=2; sqrt(  ($Xs_4 - $Xs_3)^2 + ($Ys_4 - $Ys_3)^2 )" | bc`
LAWNW=`echo "scale=2; $LENGTH_s1_TO_s4 / $VNUM " | bc`



# Launch each vehicle.
# We need to cacluate:
# 1. a start position for each vehicle based on the number of vehicles
# 2. explore end is the endpoint for the initial explore, this doesn't
#    matter if you are using a lawn mower pattern, or greedy mode
#    because it will be overwritten by the calculated path at the start


######################################################
# Launch first vehicle
VNAME1="ida"
N="0"


if [ "${PATHMODE}" = "-GM" -o "${PATHMODE}" = "-LM" ]; then
    # set start location to corner of the grid and the grid
    # start to the first cell in that corner
    STARTX="$X_1"
    STARTY="$Y_1"
    START="--start=$STARTX,$STARTY,180"
    
    #GRIDSTARTX=`echo "scale=2;  $Xs_1 + 0.5 * $CELLSIZE / $LENGTH_s1_TO_s4 * ($Xs_4 - $Xs_1) " | bc -l`
    #GRIDSTARTY=`echo "scale=2;  $Ys_1 + 0.5 * $CELLSIZE / $LENGTH_s1_TO_s4 * ($Ys_4 - $Ys_1) " | bc -l`
    GRIDSTARTX="$Xs_1"
    GRIDSTARTY="$Ys_1"
    GRIDSTART="--gridstart=$GRIDSTARTX,$GRIDSTARTY"
    
else
    # start near the dock, and set the grid start to be the
    # first cell in the corner
    START="--start=-70,-45,180"
    
    #GRIDSTARTX=`echo "scale=2;  $Xs_1 + 0.5 * $CELLSIZE / $LENGTH_s1_TO_s4 * ($Xs_4 - $Xs_1) " | bc -l`
    #GRIDSTARTY=`echo "scale=2;  $Ys_1 + 0.5 * $CELLSIZE / $LENGTH_s1_TO_s4 * ($Ys_4 - $Ys_1) " | bc -l`
    GRIDSTARTX="$Xs_1"
    GRIDSTARTY="$Ys_1"
    GRIDSTART="--gridstart=$GRIDSTARTX,$GRIDSTARTY"

fi

# Regardless of mode, set the explore x and explore y the same
# cell which is the cell at the top right corner (4)

#EXPLOREX=`echo "scale=2;  $Xs_4 + 0.5 * $CELLSIZE / $LENGTH_s1_TO_s4 * ($Xs_1 - $Xs_4) " | bc -l`
#EXPLOREY=`echo "scale=2;  $Ys_4 + 0.5 * $CELLSIZE / $LENGTH_s1_TO_s4 * ($Ys_1 - $Ys_4) " | bc -l`
EXPLOREX="$Xs_4"
EXPLOREY="$Ys_4"
EXPLORE_END="$EXPLOREX,$EXPLOREY"

# 0,-10 start originally
echo "START = $START"
echo "GRIDSTART = $GRIDSTART"
echo "EXPLORE_END = $EXPLORE_END"
   
#VLAUNCH_ARGS=" --auto --sim --vname=$VNAME1 --index=9 --start=-56,-45,180 -NS --gridstart=-70,-55"
VLAUNCH_ARGS=" --auto --sim --vname=$VNAME1 --index=9  --v_N=$N --vnum=$VNUM --exploreend=$EXPLORE_END --cellsize=$CELLSIZE --gridfile=$GRIDFILE $MIRROR"
echo "$ME: Launching $VNAME1 ..."
./launch_vehicle.sh $VLAUNCH_ARGS $VERBOSE $JUST_MAKE $TIME_WARP $GRIDSIZE $PATHMODE $GRIDSTART $START $GRIDSPECS



######################################################
# Launch second vehicle

if [ "${VNUM}" = "2" -o "${VNUM}" = "3" -o "${VNUM}" = "4" ]; then 

    VNAME2="jing"
    N="1"
    SCALE=`echo "scale=2;  $N / ($VNUM - 1) " | bc -l`
    echo "SCALE = $SCALE"
       
    # 7,-6 start originally
    
    #LAWNSTARTX="$((MIN_X+LAWNW*N + (CELLSIZE/2)))"
    #echo $LAWNSTART

    if [ "${PATHMODE}" = "-GM" -o "${PATHMODE}" = "-LM" ]; then
	# set start location along the top edge, spaced as 
	# required depending on the number of vehicles
	# The grid start is right below that.
	STARTX=`echo "scale=2;  $X_1 + ($LAWNW * $N) / $LENGTH_1_TO_4 * ($X_4 - $X_1) " | bc -l`
	STARTY=`echo "scale=2;  $Y_1 + ($LAWNW * $N) / $LENGTH_1_TO_4 * ($Y_4 - $Y_1) " | bc -l`
	START="--start=$STARTX,$STARTY,180"
	
	GRIDSTARTX=`echo "scale=2;  $Xs_1 + ( $LAWNW * $N ) / $LENGTH_s1_TO_s4 * ($Xs_4 - $Xs_1) " | bc -l`
	GRIDSTARTY=`echo "scale=2;  $Ys_1 + ( $LAWNW * $N ) / $LENGTH_s1_TO_s4 * ($Ys_4 - $Ys_1) " | bc -l`
	GRIDSTART="--gridstart=$GRIDSTARTX,$GRIDSTARTY"
	
    else
	# start near the dock, and set the grid start to be the
	# cell along the right edge that is spaced
	# accordingly
	START="--start=-56,-40,180"

	GRIDSTARTX=`echo "scale=2;  $Xs_1 + $SCALE * ($Xs_2 - $Xs_1) " | bc -l`
	GRIDSTARTY=`echo "scale=2;  $Ys_1 + $SCALE * ($Ys_2 - $Ys_1) " | bc -l`
	GRIDSTART="--gridstart=$GRIDSTARTX,$GRIDSTARTY"

    fi


    # Regardless of mode, set the explore x and explore y depending
    # on the number of vehicles to be along the line from 3 to 4. 
    EXPLOREX=`echo "scale=2;  $Xs_4 + $SCALE * ($Xs_3 - $Xs_4) " | bc -l`
    EXPLOREY=`echo "scale=2;  $Ys_4 + $SCALE * ($Ys_3 - $Ys_4) " | bc -l`
    EXPLORE_END="$EXPLOREX,$EXPLOREY"

    echo "LAWNW = $LAWNW"
    echo "START = $START"
    echo "GRIDSTART = $GRIDSTART"
    echo "EXPLORE_END = $EXPLORE_END"

    VLAUNCH_ARGS=" --auto --sim --vname=$VNAME2 --index=10 --v_N=$N --vnum=$VNUM --exploreend=$EXPLORE_END --cellsize=$CELLSIZE --gridfile=$GRIDFILE $MIRROR"
    #VLAUNCH_ARGS=" --auto --sim --vname=$VNAME2 --index=10 -EW "
    echo "$ME: Launching $VNAME2 ..."
    ./launch_vehicle.sh $VLAUNCH_ARGS $VERBOSE $JUST_MAKE $TIME_WARP $GRIDSIZE $PATHMODE $GRIDSTART $START $GRIDSPECS

fi



######################################################
# Launch third vehicle


if [ "${VNUM}" = "3" -o "${VNUM}" = "4" ]; then
    
    VNAME3="gus"
    N="2"
    SCALE=`echo "scale=2;  $N / ($VNUM - 1) " | bc -l`
    echo "SCALE = $SCALE"

    
#    LAWNSTARTX="$((-70+LAWNW*N + (CELLSIZE/2)))"
#    echo $LAWNSTARTX
    
#    if [ "${PATHMODE}" = "-GM" -o "${PATHMODE}" = "-LM" ]; then 
#	GRIDSTART="--gridstart=$LAWNSTARTX,$GRIDSTARTY"
#	START="--start=$LAWNSTARTX,$STARTY,180"
#    else 
#	GRIDSTART="--gridstart=$MIN_X,$EXPLOREY"
#	START="--start=-44,-35,180"
#    fi

   if [ "${PATHMODE}" = "-GM" -o "${PATHMODE}" = "-LM" ]; then
	# set start location along the top edge, spaced as 
	# required depending on the number of vehicles
	# The grid start is right below that.
	STARTX=`echo "scale=2;  $X_1 + ($LAWNW * $N) / $LENGTH_1_TO_4 * ($X_4 - $X_1) " | bc -l`
	STARTY=`echo "scale=2;  $Y_1 + ($LAWNW * $N) / $LENGTH_1_TO_4 * ($Y_4 - $Y_1) " | bc -l`
	START="--start=$STARTX,$STARTY,180"
	
	GRIDSTARTX=`echo "scale=2;  $Xs_1 + ( $LAWNW * $N ) / $LENGTH_s1_TO_s4 * ($Xs_4 - $Xs_1) " | bc -l`
	GRIDSTARTY=`echo "scale=2;  $Ys_1 + ( $LAWNW * $N ) / $LENGTH_s1_TO_s4 * ($Ys_4 - $Ys_1) " | bc -l`
	GRIDSTART="--gridstart=$GRIDSTARTX,$GRIDSTARTY"
	
    else
	# start near the dock, and set the grid start to be the
	# cell along the right edge that is spaced
	# accordingly
        START="--start=-44,-35,180"

	GRIDSTARTX=`echo "scale=2;  $Xs_1 + $SCALE * ($Xs_2 - $Xs_1) " | bc -l`
	GRIDSTARTY=`echo "scale=2;  $Ys_1 + $SCALE * ($Ys_2 - $Ys_1) " | bc -l`
	GRIDSTART="--gridstart=$GRIDSTARTX,$GRIDSTARTY"
   fi
   

   #if [ "${VNUM}" = "3" ]; then 
   #    EXPLOREY="$((MAX_Y + (CELLSIZE/2)))"
   #else 
   #    EXPLOREY="$(( MIN_Y + (HEIGHTRANGE/(VNUM-1))*N + (CELLSIZE/2) ))"
   #EXPLOREY="$((((EXPLOREY + 5/2) / 5) * 5))"
   #fi
   #EXPLORE_END="$MAX_X,$EXPLOREY"
   

   # Regardless of mode, set the explore x and explore y depending
   # on the number of vehicles to be along the line from 3 to 4. 
   EXPLOREX=`echo "scale=2;  $Xs_4 + $SCALE * ($Xs_3 - $Xs_4) " | bc -l`
   EXPLOREY=`echo "scale=2;  $Ys_4 + $SCALE * ($Ys_3 - $Ys_4) " | bc -l`
   EXPLORE_END="$EXPLOREX,$EXPLOREY"


   echo "LAWNW = $LAWNW"
   echo "START = $START"
   echo "GRIDSTART = $GRIDSTART"
   echo "EXPLORE_END = $EXPLORE_END"
   
   #VLAUNCH_ARGS=" --auto --sim --vname=$VNAME3 --index=10 --start=190,-220,180 -EW -RV --gridstart=180,-215"
   VLAUNCH_ARGS=" --auto --sim --vname=$VNAME3 --index=10 --v_N=$N --vnum=$VNUM --exploreend=$EXPLORE_END --cellsize=$CELLSIZE --gridfile=$GRIDFILE $MIRROR"
   echo "$ME: Launching $VNAME3 ..."
   ./launch_vehicle.sh $VLAUNCH_ARGS $VERBOSE $JUST_MAKE $TIME_WARP $GRIDSIZE $PATHMODE $GRIDSTART $START $GRIDSPECS
    
fi

if [ "${VNUM}" = "4" ]; then
    
    VNAME4="kirk"
    N="3"
    SCALE=`echo "scale=2;  $N / ($VNUM - 1) " | bc -l`
    echo "SCALE = $SCALE" 


    #LAWNSTARTX="$((-70+LAWNW*N + (CELLSIZE/2)))"
    #echo $LAWNSTARTX

    #if [ "${PATHMODE}" = "-GM" -o "${PATHMODE}" = "-LM" ]; then 
    #	GRIDSTART="--gridstart=$LAWNSTARTX,$GRIDSTARTY"
    #	START="--start=$LAWNSTARTX,$STARTY,180"
    #else 
    #   GRIDSTART="--gridstart=$MIN_X,$EXPLOREY"
    #	START="--start=-34,-31,180"
    #fi

    if [ "${PATHMODE}" = "-GM" -o "${PATHMODE}" = "-LM" ]; then
	# set start location along the top edge, spaced as 
	# required depending on the number of vehicles
	# The grid start is right below that.
	STARTX=`echo "scale=2;  $X_1 + ($LAWNW * $N) / $LENGTH_1_TO_4 * ($X_4 - $X_1) " | bc -l`
	STARTY=`echo "scale=2;  $Y_1 + ($LAWNW * $N) / $LENGTH_1_TO_4 * ($Y_4 - $Y_1) " | bc -l`
	START="--start=$STARTX,$STARTY,180"
	
	GRIDSTARTX=`echo "scale=2;  $Xs_1 + ( $LAWNW * $N ) / $LENGTH_s1_TO_s4 * ($Xs_4 - $Xs_1) " | bc -l`
	GRIDSTARTY=`echo "scale=2;  $Ys_1 + ( $LAWNW * $N ) / $LENGTH_s1_TO_s4 * ($Ys_4 - $Ys_1) " | bc -l`
	GRIDSTART="--gridstart=$GRIDSTARTX,$GRIDSTARTY"
	
    else
	# start near the dock, and set the grid start to be the
	# cell along the right edge that is spaced
	# accordingly
        START="--start=-34,-31,180"
	
	GRIDSTARTX=`echo "scale=2;  $Xs_1 + $SCALE * ($Xs_2 - $Xs_1) " | bc -l`
	GRIDSTARTY=`echo "scale=2;  $Ys_1 + $SCALE * ($Ys_2 - $Ys_1) " | bc -l`
	GRIDSTART="--gridstart=$GRIDSTARTX,$GRIDSTARTY"
    fi

    
    #EXPLOREY="$((MAX_Y + (CELLSIZE/2)))"
    #EXPLORE_END="$MAX_X,$EXPLOREY"

    EXPLOREX=`echo "scale=2;  $Xs_4 + $SCALE * ($Xs_3 - $Xs_4) " | bc -l`
    EXPLOREY=`echo "scale=2;  $Ys_4 + $SCALE * ($Ys_3 - $Ys_4) " | bc -l`
    EXPLORE_END="$EXPLOREX,$EXPLOREY"
    
    
    echo "LAWNW = $LAWNW"
    echo "START = $START"
    echo "GRIDSTART = $GRIDSTART"
    echo "EXPLORE_END = $EXPLORE_END"
    
    
    #VLAUNCH_ARGS=" --auto --sim --vname=$VNAME4 --index=10 --start=190,-225,185 -NS -RV --gridstart=180,-215"
    VLAUNCH_ARGS=" --auto --sim --vname=$VNAME4 --index=10 --v_N=$N --vnum=$VNUM --exploreend=$EXPLORE_END --cellsize=$CELLSIZE --gridfile=$GRIDFILE $MIRROR"
    echo "$ME: Launching $VNAME4 ..."
    ./launch_vehicle.sh $VLAUNCH_ARGS $VERBOSE $JUST_MAKE $TIME_WARP $GRIDSIZE $PATHMODE $GRIDSTART $START $GRIDSPECS


fi


#---------------------------------------------------------------
#  Part 4: Launch the shoreside
#---------------------------------------------------------------
SLAUNCH_ARGS=" --vname1=$VNAME1 --vname2=$VNAME2 --vname3=$VNAME3 --vname4=$VNAME4 --vnum=$VNUM"
echo "$ME: Launching Shoreside ..."
./launch_shoreside.sh --auto $VERBOSE $JUST_MAKE $SLAUNCH_ARGS $TIME_WARP $GRIDSIZE

#---------------------------------------------------------------
# Part 5: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_shoreside.moos
kill -- -$$

