#!/bin/bash 

if [[ $# -eq 0 ]] ; then
    echo "specify mode=<directory name> and -LM, -GM --ucp, -GM --mvi, or -OM"
    exit 0
fi

logs=(LOG_*)
echo $logs
if [ -e $logs ]; then
    echo "clean old logs first"
    exit 0
fi

NUM_RUNS=1
VNUM_IN=2
FLOW_DOWN_ARGS=""
MODE="unspecified"
START_FILE=0
FILES="all"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                                                                                        "
	echo "                                                                                                                 "
        echo "This script auto runs simulations of the bathysurvey mission                                                     "
        echo "The script automatically processes the log files and moves log folders to /processed/<mode name>/file<#>/vnum<#> "
        echo "and moves copies of relevant .klog files to /sim_results/<mode name>/file<#>/vnum<#>/<run#>/<vname>              "
        echo "                                                                                                                 "
        echo "For example, the variables from gus for the first 2-vehicle lawnmower run of file 1 will be under:               "
        echo "sim_results/lawn/file1/vnum2/0/gus/*.klog                                                                        "
        echo "The log folder from that run will be under:                                                                      "
        echo "processed/lawn/file1/vnum2/LOG_GUS*                                                                              "
        echo "                                                                                                                 "
        echo "To run, specify --vnum=<#> --mode=<mode name (e.g.) ucb> <mode flag (e.g. -GM --ucb)>                            "
        echo "Optional flags are --start=<#> and --runs=<#>, and a time warp number                                            "
        echo "                                                                                                                 "
	echo "Options:                                                         "
        echo "  --help, -h                                                     "
        echo "    Display this help message                                    "
        echo "  --verbose, -v                                                  "
        echo "    Increase verbosity                                           "
	echo "  --just_make, -j                                                " 
	echo "    Just make the targ files, but do not launch.                 "
	echo "                                                                 " 
	echo "  --runs=<# of runs>                                             " 
	echo "    How many times to repeat                                     "
	echo "  --vnum=<# of vehicles>                                         "
	echo "    How many vehicles to use                                     "
	echo '    Examples: --vnum=2, --vnum="1 2 3 4"                         '
	echo "  --mode=<mode name>                                             "
	echo "    What the folder will be called                               "
	echo "  --files=<ids>                                                  "
	echo "    Ids of which files to run, from 0 (default) to 9             "
	echo "    Order: [1 2 2mirror 3 3mirror 4 4mirror 5 6 6mirror]         "
	echo "    Example: --files='1 5 9' to run files 1,5,9                  "
	echo "    Omit this argument to run all files 0-9                      "
	echo "                                                                 "
	echo "Examples:                                                        "
	echo '    ./zlaunch.sh --vnum="4 3 2 1" --mode=hybrid -OM 2            '
	echo '    ./zlaunch.sh --vnum=2 --mode=ucb -GM --ucb 2 --files="3 7 8" '
	echo '    ./zlaunch.sh --vnum="4 3" --mode=mvi -GM --mvi 2             '
	echo '    ./zlaunch.sh --vnum="4 3 2 1" --mode=lawn -LM 2              '
	exit 0;
    elif [ "${ARGI:0:7}" = "--runs=" ]; then
	NUM_RUNS="${ARGI#--runs=*}"
    elif [ "${ARGI:0:7}" = "--vnum=" ]; then
	VNUM_IN="${ARGI#--vnum=*}"
    elif [ "${ARGI:0:7}" = "--mode=" ]; then
        MODE="${ARGI#--mode=*}"
    elif [ "${ARGI:0:8}" = "--files=" ]; then
	FILES="${ARGI#--files=*}"
    else 
	FLOW_DOWN_ARGS+="${ARGI} "
    fi
done

#if [ "${VNUM_IN}" = "1" ]; then
#    declare -a file_args
#elif [ "${VNUM_IN}" = "1" ]; then
#    MODE="${ARGI#--mode=*}"
#elif [ "${VNUM_IN}" = "1" ]; then
#    START_FILE="${ARGI#--start=*}"
#fi
	

declare -a file_args=("--gridfile=1" "--gridfile=2" "--gridfile=2 --mirror" "--gridfile=3" "--gridfile=3 --mirror" "--gridfile=4" "--gridfile=4 --mirror" "--gridfile=5" "--gridfile=6" "--gridfile=6 --mirror")
declare -a file_title=("1" "2a" "2b" "3a" "3b" "4a" "4b" "5" "6a" "6b")

#arraylength=${#file_args[@]}
if [ "$FILES" = "all" ]; then
    FILE_I="0 1 2 3 4 5 6 7 8 9"
else
    FILE_I=$FILES
fi

echo $FILES $FILE_I

for (( i=1; i<=$NUM_RUNS; i++ ))
do
    
    for VNUM in $VNUM_IN
    do	

	echo $FILE_I
	#for (( j=$START_FILE; j<${arraylength}; j++ )); do
	for j in $FILE_I; do
	    echo xlaunch.sh $FLOW_DOWN_ARGS --speed=2.4 --vnum="$VNUM" -FG ${file_args[$j]}
	    xlaunch.sh $FLOW_DOWN_ARGS --speed=2.4 --vnum="$VNUM" -FG ${file_args[$j]}
	    
	    sleep 2
	    ktm
	    sleep 2
	    ktm
	    sleep 10
	    ktm
	    sleep 10

	    echo "processing data"
	    
	    FILE_TITLE=${file_title[$j]}

            echo sim_results/$MODE/file$FILE_TITLE/vnum$VNUM
	    
	    mkdir -p sim_results/$MODE/file$FILE_TITLE/vnum$VNUM
	    mkdir -p processed
	    
	    NUMBER=$(ls sim_results/$MODE/file$FILE_TITLE/vnum$VNUM | wc -l)

	    if [ "${VNUM}" = "1" ]; then
		NAMES="IDA"
	    elif [ "${VNUM}" = "2" ]; then
		NAMES="IDA JING"
	    elif [ "${VNUM}" = "3" ]; then
		NAMES="GUS IDA JING"
	    elif [ "${VNUM}" = "4" ]; then
		NAMES="GUS IDA JING KIRK"
	    fi
	    
	    for name in $NAMES; do
		LOGNAME=$(echo LOG_$name*)
		
		echo $LOGNAME
		
		alogsplit LOG_$name*/*.alog
		
		mkdir -p sim_results/$MODE/file$FILE_TITLE/vnum$VNUM/$NUMBER/$name
		
		cp $LOGNAME/${LOGNAME}_alvtmp/FINAL_CONSENSUS_DEPTH.klog sim_results/$MODE/file$FILE_TITLE/vnum$VNUM/$NUMBER/$name
		cp $LOGNAME/${LOGNAME}_alvtmp/FINAL_CONSENSUS_VARIANCE.klog sim_results/$MODE/file$FILE_TITLE/vnum$VNUM/$NUMBER/$name
		cp $LOGNAME/${LOGNAME}_alvtmp/GROUND_TRUTH_DEPTH.klog sim_results/$MODE/file$FILE_TITLE/vnum$VNUM/$NUMBER/$name
		cp $LOGNAME/${LOGNAME}_alvtmp/TIME_TO_ROUTE.klog sim_results/$MODE/file$FILE_TITLE/vnum$VNUM/$NUMBER/$name
		cp $LOGNAME/${LOGNAME}_alvtmp/FINAL_ROUTE.klog sim_results/$MODE/file$FILE_TITLE/vnum$VNUM/$NUMBER/$name
		cp $LOGNAME/${LOGNAME}_alvtmp/PATH_VECTOR.klog sim_results/$MODE/file$FILE_TITLE/vnum$VNUM/$NUMBER/$name

		mkdir -p processed/$MODE/vnum$VNUM/
		
		mv $LOGNAME processed/$MODE/vnum$VNUM/
		#rm -rf $LOGNAME
	    done

	    LOGNAME=$(echo LOG_SHORESIDE*)

	    mv $LOGNAME processed/$MODE/vnum$VNUM/
	    #rm -rf $LOGNAME
	    
	    sleep 2
	    
	done

	sleep 2

    done
done
