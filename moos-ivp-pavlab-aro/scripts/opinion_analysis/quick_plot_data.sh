#! /bin/bash

DATA_DIR="data"
LOG_DIR=""


for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [SWITCHES] [time_warp]                                    "
	echo "  --help, -h            Show this help message                "
	echo "  --log_dir=<LOG_***>   Specify the log directory             "
	echo "  --keys=<Key1,Key2,..>   Specify the MOOS variables to plot  "
	exit 0

    elif [ "${ARGI:0:10}" = "--log_dir=" ]; then
        LOG_DIR="${ARGI#--log_dir=*}"
    elif [ "${ARGI:0:7}" = "--keys=" ]; then
	string="${ARGI#--keys=*}"
	IFS=', ' read -ra keys <<< "$string"	
    else
        echo "$ME: Bad arg:" $ARGI "Exit Code 1."
        exit 1
    fi
done

if [ -z "${LOG_DIR}" ]; then
	echo "$ME: Log file not specified, exiting"
	exit 1
else 
	if test -d $LOG_DIR/$DATA_DIR; then
		echo "Existing data in $LOG_DIR/$DATA_DIR"
		echo "Clear it manually, I will not do it for you"
		echo "because it is too dangerous"
		exit 1

	fi
	mkdir -p $LOG_DIR/$DATA_DIR
fi

for key in "${keys[@]}"
do 
	aloggrep "$key" --tv -f -csc -sd $LOG_DIR/*.alog $LOG_DIR/$DATA_DIR/$key.klog	

done

# plot the variables
~/moos-ivp-pavlab/scripts/opinion_analysis/simple_plot_data.py "$LOG_DIR/$DATA_DIR"

# calculate things if desired
~/moos-ivp-pavlab/scripts/opinion_analysis/simple_calc_data.py "$LOG_DIR/$DATA_DIR"

