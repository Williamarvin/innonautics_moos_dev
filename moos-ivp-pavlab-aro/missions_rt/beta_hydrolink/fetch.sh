#Author: Ray Turrisi

#Objective: Fetch all remote files on machines which are participating in a mission, and group them with a mission hash

#Programming scope: Since the code is simple enough - write it such that code can be refactored on a per mission basis

#Preliminaries: Install SSH keys on devices - see pavlab_shorts for distributing keys. 
#                 I also am assuming you are using pavlab_shorts - I'm using environment variables I define in there

#Useage: 
# 1) Configure this script to point to the right directories and consider the relevant vehicles
# 2) Run after a successful mission - not after you've done several missions
# 3) exec
#   All relevant systems  - ./fetch 
#   Specific systems      - ./fetch o h


#Consequence: Uses rsync to synchronize all the new data found in this repository, on a remote machine. 
# - Generates a mission hash locally, makes the directory on the remote machine, moves all the latest logs into that directory, then synchronizes the remote log file with our system on shoreside

#If you move this script into your own mission, change these values
CWD="/Users/raymondturrisi/svn/moos-ivp-pavlab/missions/beta_hydrolink"
REMOTE_DIRECTORY="~/moos-ivp-pavlab/missions/beta_hydrolink"

#Should always be student2680
STD_USER="student2680"

#-- Start

#Generate a unique mission hash for this run
HASH="$(gen_ivphash)"

#Manipulate local data
mkdir $CWD/logs/$HASH
mv logs/LOG* logs/$HASH
mv targ* logs/$HASH

# If you are just using ./fetch, I'm assuming you are gathering data from all the systems
if [ "$#" -eq 0 ]; then
    set -- "o" "h"
fi

#Iterate through the systems, update the location of the files on the remote devices, and sync the file structure with our relevant logs here
for sys in "$@"; do
    if [[ $sys == "o" || $sys == "O" ]]; then 
        #ssh into the device and manipulate the data
        ssh $STD_USER@$OAKB "mkdir $REMOTE_DIRECTORY/logs/$HASH && mv $REMOTE_DIRECTORY/logs/LOG* $REMOTE_DIRECTORY/logs/$HASH && mv $REMOTE_DIRECTORY/targ* $REMOTE_DIRECTORY/logs/$HASH"
        #Now that the data is stored cohesively between systems, use rsync to one-way synchronize directories between devices
        #Also detach from this process since it may take some time
        rsync -av "$STD_USER@$OAKB:$REMOTE_DIRECTORY/logs/" "$CWD/logs" --timeout=5 || echo "Timeout - Could not connect: Oak" &
    elif [[ $sys == "h" || $sys == "H" ]]; then 
        ssh $STD_USER@$HLIP0 "mkdir $REMOTE_DIRECTORY/logs/$HASH && mv $REMOTE_DIRECTORY/logs/LOG* $REMOTE_DIRECTORY/logs/$HASH && mv $REMOTE_DIRECTORY/targ* $REMOTE_DIRECTORY/logs/$HASH"
        rsync -av "$STD_USER@$HLIP0:$REMOTE_DIRECTORY/logs/" "$CWD/logs" --timeout=5 || echo "Timeout - Could not connect: HydroLink" &
    fi 
done 

#Since we detached and have child processes, wait for all the child processes to complete
wait
