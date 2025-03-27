#!/bin/sh

VNUM="4"
MODE="lawn"

mkdir -p sim_results/$MODE/$VNUM
mkdir -p processed

NUMBER=$(ls sim_results/$MODE/$VNUM | wc -l)

NAMES="GUS IDA JING KIRK"

for name in $NAMES; do
    LOGNAME=$(echo LOG_$name*)

    echo $LOGNAME
	      
    alogsplit LOG_$name*/*.alog
    
    mkdir -p sim_results/$MODE/$VNUM/$NUMBER/$name
    
    cp $LOGNAME/${LOGNAME}_alvtmp/FINAL_CONSENSUS_DEPTH.klog sim_results/$MODE/$VNUM/$NUMBER/$name
    cp $LOGNAME/${LOGNAME}_alvtmp/FINAL_CONSENSUS_VARIANCE.klog sim_results/$MODE/$VNUM/$NUMBER/$name
    cp $LOGNAME/${LOGNAME}_alvtmp/GROUND_TRUTH_DEPTH.klog sim_results/$MODE/$VNUM/$NUMBER/$name
    cp $LOGNAME/${LOGNAME}_alvtmp/TIME_TO_ROUTE.klog sim_results/$MODE/$VNUM/$NUMBER/$name
    cp $LOGNAME/${LOGNAME}_alvtmp/FINAL_ROUTE.klog sim_results/$MODE/$VNUM/$NUMBER/$name

    mv $LOGNAME processed
done


