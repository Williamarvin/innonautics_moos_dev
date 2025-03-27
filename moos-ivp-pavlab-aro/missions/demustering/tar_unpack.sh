#!/bin/bash -e
#---------------------------------------------------------------
#   Script: tar_unpack.sh
#   Author: Filip Stromstad
#   LastEd: 2024-Aug-22




# # Check if the main folder is provided as an argument
# if [ -z "$1" ]; then
#     echo "Usage: $0 <main-folder>"
#     exit 1
# fi

# # Navigate to the main folder
# main_folder="$1"
# cd "$main_folder" || { echo "Failed to navigate to $main_folder"; exit 1; }

main_folder="logs"
cd "$main_folder" || { echo "Failed to navigate to $main_folder"; exit 1; }

# Iterate over each subfolder
for subfolder in */; do
    # Skip subfolders that contain "batch" in their name
    if [[ "$subfolder" == *"batch"* ]]; then
        continue
    fi

    # Navigate to the subfolder
    echo "Navigating to $subfolder"
    cd "$subfolder" || continue

    # Iterate over each subsubfolder
    for subsubfolder in */; do
        # Navigate to the subsubfolder
        echo "Navigating to $subsubfolder"
        cd "$subsubfolder" || continue

        # Iterate over each .tgz file in the subfolder
        for tgz_file in *.tgz; do
            # Run the tar command on the .tgz file
            echo "Running command: tar -xvzf $tgz_file"
            tar -xvzf "$tgz_file"
        done

        # Navigate back to the subfolder
        cd ..
    done

    # Navigate back to the main folder
    cd ..
done

echo "Extraction complete."