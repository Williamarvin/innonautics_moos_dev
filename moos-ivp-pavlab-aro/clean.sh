#!/bin/bash

rm -rf build/*
rm -rf lib/*
rm -rf bin/*
rm -f .DS_Store

DIR=$(pwd)
echo Cleaning DIR: $DIR

txtrst=$(tput setaf 0)  # Reset                                                           
txtred=$(tput setaf 1)  # Red                                                             
txtgrn=$(tput setaf 2)  # Green                                                           
txtblu=$(tput setaf 4)  # Blue                                                            
txtpur=$(tput setaf 5)  # Purple                                                          
txtcyn=$(tput setaf 6)  # Cyan                                                            

for file in *; do
   if [ -d $file ]; then
      cd $file;
      if [ -e clean.sh ]; then
          echo ${txtblu} Cleaning: $file ${txtrst}
          ./clean.sh
      else
          echo ${txtred} Nothing to clean within $file ${txtrst}
      fi
      cd ..
   fi
done
