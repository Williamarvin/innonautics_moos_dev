#! /bin/bash

echo "Starting to parse all alog files. "
DATA_DIR="data"
mkdir -p $DATA_DIR

for dir in */; do

    case $dir in

	LOG_A*)
	    echo "Found abe"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_A*/*.alog $DATA_DIR/abe_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_A*/*.alog $DATA_DIR/abe_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_A*/*.alog $DATA_DIR/abe_contacts.alog
	    ;;
	LOG_B*)
	    echo "Found ben"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_B*/*.alog $DATA_DIR/ben_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_B*/*.alog $DATA_DIR/ben_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_B*/*.alog $DATA_DIR/ben_contacts.alog
	    ;;
	LOG_C*)
	    echo "Found cal"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_C*/*.alog $DATA_DIR/cal_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_C*/*.alog $DATA_DIR/cal_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_C*/*.alog $DATA_DIR/cal_contacts.alog
	    ;;
	LOG_D*)
	    echo "Found deb"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_D*/*.alog $DATA_DIR/deb_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_D*/*.alog $DATA_DIR/deb_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_D*/*.alog $DATA_DIR/deb_contacts.alog
	    ;;
	LOG_E*)
	    echo "Found eve"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_E*/*.alog $DATA_DIR/eve_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_E*/*.alog $DATA_DIR/eve_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_E*/*.alog $DATA_DIR/eve_contacts.alog
	    ;;
	LOG_F*)
	    echo "Found fin"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_F*/*.alog $DATA_DIR/fin_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_F*/*.alog $DATA_DIR/fin_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_F*/*.alog $DATA_DIR/fin_contacts.alog
	    ;;
	LOG_G*)
	    echo "Found gil"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_G*/*.alog $DATA_DIR/gil_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_G*/*.alog $DATA_DIR/gil_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_G*/*.alog $DATA_DIR/gil_contacts.alog
	    ;;
	LOG_H*)
	    echo "Found hix"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_H*/*.alog $DATA_DIR/hix_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_H*/*.alog $DATA_DIR/hix_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_H*/*.alog $DATA_DIR/hix_contacts.alog
	    ;;
	LOG_I*)
	    echo "Found ike"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_I*/*.alog $DATA_DIR/ike_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_I*/*.alog $DATA_DIR/ike_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_I*/*.alog $DATA_DIR/ike_contacts.alog
	    ;;
	LOG_J*)
	    echo "Found jim"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_J*/*.alog $DATA_DIR/jim_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_J*/*.alog $DATA_DIR/jim_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_J*/*.alog $DATA_DIR/jim_contacts.alog
	    ;;
	LOG_K*)
	    echo "Found kim"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_K*/*.alog $DATA_DIR/kim_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_K*/*.alog $DATA_DIR/kim_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_K*/*.alog $DATA_DIR/kim_contacts.alog
	    ;;
	LOG_L*)
	    echo "Found lou"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_L*/*.alog $DATA_DIR/lou_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_L*/*.alog $DATA_DIR/lou_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_L*/*.alog $DATA_DIR/lou_contacts.alog
	    ;;
	LOG_M*)
	    echo "Found max"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_M*/*.alog $DATA_DIR/max_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_M*/*.alog $DATA_DIR/max_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_M*/*.alog $DATA_DIR/max_contacts.alog
	    ;;
	LOG_N*)
	    echo "Found ned"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_N*/*.alog $DATA_DIR/ned_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_N*/*.alog $DATA_DIR/ned_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_N*/*.alog $DATA_DIR/ned_contacts.alog
	    ;;
	LOG_O*)
	    echo "Found oak"
	    aloggrep OWN_OPINION_STATE --tv -f -csc -sd LOG_O*/*.alog $DATA_DIR/oak_opinions.alog
	    aloggrep OWN_ATTENTION --tv -f -csc -sd LOG_O*/*.alog $DATA_DIR/oak_attention.alog
	    aloggrep CONTACTS_LIST --tv -f -csc -sd LOG_O*/*.alog $DATA_DIR/oak_contacts.alog
	    ;;
	*)
	    echo "Found other directory, skipping"
	    ;;
    esac
done


# Plot these values?
while true; do
    read -p "Do you want to produce an opinion plot for this run? [y/n] " yn
    case $yn in
	[Yy]* ) ~/moos-ivp-pavlab/scripts/opinion_analysis/plot_data.py; break;;
	[Nn]* ) break;;
	* ) echo "Please answer yes or no.";;
    esac
done

# make animations?
while true; do
    read -p "Do you want to generate an animation for all vehicles? [y/n] " yn
    case $yn in
	[Yy]* ) ~/moos-ivp-pavlab/scripts/opinion_analysis/animate_data.py; break;;
	[Nn]* ) break;;
	* ) echo "Please answer yes or no.";;
    esac
done


