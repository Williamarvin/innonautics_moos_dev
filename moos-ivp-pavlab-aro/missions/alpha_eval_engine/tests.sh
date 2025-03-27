#!/bin/bash

# Several tests for the post processing capabilities of the EvalEngine
# If you've first clicked on this, lib_evalengine has utilities to support both realtime and post processing analysis of an agents performance

# This file provides several means of interacting with the post processing component for testing purposes
# or simply demonstrating its use

# 1) Run in realtime if you know what you want, obtaining instantaneous and cummulative datapoints
# 2) Run in post processing to update the parameters you applied to a mission previously, in generating a new alog file
# 3) Run in post processing with updated parameters to receive a concise serialized final output

# It is forced that you MUST pass a directory, not an alog file, since within this directory, we make another directory, which contains the updated
# eval engine's parameters and the new alog file if you are rewriting it. This is so the original alog is obvious and semantically a rewritten mission is 
# derived from the source directory/alog. I did not want to generate another alog file "next to" the original, or not write the updated configuration
# parameters at all, or write both another alog and a new moos file next to the actual mission files

mission_logs=(*Log*____*)
latest_mission_logs="${mission_logs[0]}"

# Tests for only post processing applications
# 1

# For a mission that was already running the realtime version of this evalengine, we pass a configuration file with presumeably updated parameters, and collect only the final output
postKayakEvalEngine "$latest_mission_logs" --pname=pKayakEvalEngine alpha.moos

# Apply a tag to some version of it - should this script get wrapped in a bash script, this is where you would differentiate the newly rewritten alogs
# A tag is not exactly necessary, since if the files which would be written to already exist, they are just overwritten
for i in {1..3}; do
    atag="v$i"
    postKayakEvalEngine "$latest_mission_logs" --pname=pKayakEvalEngine alpha.moos --tag="$atag" --rewrite
done

# Rewrite the alog file, instead of only collecting the final outputs

# Verbose currently doesn't do anything 

# Quiet also doesn't do anything