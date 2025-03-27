#!/usr/bin/env python3

import pandas as pd
import sys
import numpy as np


# returns the value in the first column it finds that contains
#         the param name.  (for a given row
def getParam(param_name, row_indx, df_in):
    for index, row in df_in.iterrows():
        if (index == row_indx):
            for column_name, value in row.items():
                if (param_name in column_name):
                    return(float(value))
    print('ERROR: Could not find entry in df that contains' + str(param_name))
    return(0.0)


# Adapted from CDC24 work
def calculateBdotScalar(y_1_obs, number_of_agents, y_1_hat):

    # is the difference less than 1/N, if so don't update b
    if ( abs(y_1_obs - y_1_hat) < (1 / number_of_agents) ):
        B_dot_scalar = 0.0 
    else:
        # adjust y_1_hat
        B_dot_scalar = y_1_obs - y_1_hat

    return B_dot_scalar


def  updatePayoffMatrix(payoff_matrix_hat, y_1, observed_payoff, t_per_game, adaptation_gain):
    # build regressor
    w = np.array([ [ y_1 * y_1 ], [2 * y_1 * (1 - y_1)], [(1 - y_1)*(1 - y_1)] ])
    # build parameter vec
    pi_hat = np.array([ [ payoff_matrix_hat[0,0]], [payoff_matrix_hat[0,1]], [payoff_matrix_hat[1,1]] ])

    #update law
    delta_R = np.matmul(w.transpose(), pi_hat) - observed_payoff
    print('Difference in expected vs actual payoff = ' + str(delta_R))

    pi_hat_dot = -1.0 * delta_R * np.matmul(adaptation_gain, w)

    #euler integration
    pi_hat = pi_hat + pi_hat_dot * t_per_game

    new_payoff_matrix_hat = np.zeros(payoff_matrix_hat.shape)
    new_payoff_matrix_hat[0,0] = pi_hat[0]
    new_payoff_matrix_hat[0,1] = pi_hat[1]
    new_payoff_matrix_hat[1,0] = pi_hat[1]
    new_payoff_matrix_hat[1,1] = pi_hat[2]

    return new_payoff_matrix_hat


def main():
    if len(sys.argv) != 2:
        print("Usage: python process_mission.py filename.csv")
        sys.exit(1)

    filename = sys.argv[1]
    print(filename)

    try:
        df = pd.read_csv(filename)
        #print(df)
    except FileNotFoundError:
        print("Error: File '{filename}' not found.")

    column_averages = df.mean()

    print("Column averages:")
    print(column_averages)

    # sort by date in the
    df.sort_values(by='alpha_job', inplace=True)
    # reset the indexes
    df.reset_index(drop=True, inplace=True)
    print(df)

    # STEP 1.  Find when all the batches start
    #          Check that the values are all the same accross all
    #          the vehicles.
    batch_start_indx = []
    last_search_val_imprv_gain = -111111.0
    last_sample_cost_imprv_gain = -111111.0
    last_explore_bias = -111111111.0
    
    for index, row in df.iterrows():

        new_batch_started_this_row = False
        all_vehicles_same_params = True

        this_row_search_val_imprv_gain = 0.0
        this_row_sample_cost_imprv_gain = 0.0
        this_row_explore_bias = 0.0
        found_search_val_imprv_gain = False
        found_sample_cost_imprv_gain = False
        found_explore_bias = False
        
        for column_name, value in row.items():
            
            if ('EXPLORE_BIAS' in column_name):
                # is this the first time we've seen this
                # var in this row?
                if (not found_explore_bias):
                    found_explore_bias = True
                    this_row_explore_bias = float(value)
                    
                # if not, is this a new value?
                elif (float(value) != this_row_explore_bias):
                    all_vehicles_same_params = False
                    
            elif ('SAMPLE_COST_IMPRV_GAIN' in column_name):
                # is this the first time we've seen this
                # var in this row?
                if (not found_sample_cost_imprv_gain):
                    found_sample_cost_imprv_gain = True
                    this_row_sample_cost_imprv_gain = float(value)
                    
                # if not, is this a new value?
                elif (float(value) != this_row_sample_cost_imprv_gain):
                    all_vehicles_same_params = False

            elif ('SEARCH_VALUE_IMPRV_GAIN' in column_name):
                # is this the first time we've seen this
                # var in this row?
                if (not found_search_val_imprv_gain):
                    found_search_val_imprv_gain = True
                    this_row_search_val_imprv_gain = float(value)
        
                # if not, is this a new value?
                elif (float(value) != this_row_search_val_imprv_gain):
                    all_vehicles_same_params = False
                      
        # Ok, no
        if (not all_vehicles_same_params):
            print('ERROR: found a run where vehicles did not have the same params!')
            print('       Bad run is index = ' + str(index))
        else:
            if (this_row_explore_bias != last_explore_bias):
                new_batch_started_this_row = True
            if (this_row_sample_cost_imprv_gain != last_sample_cost_imprv_gain):
                new_batch_started_this_row = True
            if (this_row_search_val_imprv_gain != last_search_val_imprv_gain):
                new_batch_started_this_row = True

        if (new_batch_started_this_row):
            print('Found new batch start at index = ' + str(index))
            batch_start_indx.append(index)
            last_explore_bias = this_row_explore_bias
            last_sample_cost_imprv_gain = this_row_sample_cost_imprv_gain
            last_search_val_imprv_gain = this_row_search_val_imprv_gain
    
    #           Now we know where each batch starts and the data is clean!
    # STEP 2.  For each batch compute the params for the next batch
    #

    # payoff matrix params
    payoff_matrix_hat = np.array([[0.6, 0.6],[0.6, 0.6]])
    y_1_hat = 0.5
    
    for batch_number, start_index in enumerate(batch_start_indx):
        # batch number is obvious, but the start index is the column in
        # the data where the batch number starts
        last_batch = (batch_number == len(batch_start_indx) - 1)
        if (last_batch):
            end_index = len(df.index)
        else:
            # end one index before the next bactch starts
            end_index = batch_start_indx[batch_number + 1] - 1
        
            
        # get the values for this batch:
        mixed_strat = float(df.iloc[start_index:end_index]['MIXED_STRAT_RNG_AVG'].mean())
        sampled_blooms_completed = float(df.iloc[start_index:end_index]['SAMPLED_BLOOMS_COMPLETED'].mean())
        detected_blooms_completed = float(df.iloc[start_index:end_index]['DETECTED_BLOOMS_COMPLETED'].mean())
        unsamp_undetect_blooms_completed = float(df.iloc[start_index:end_index]['UNSAMP_UNDETECT_BLOOMS_COMPLETED'].mean())
        if np.isnan(sampled_blooms_completed):
            print('Error: found nan, exiting')
            quit()
            
        # get the exisiting parameters
        this_batch_explore_bias = getParam('EXPLORE_BIAS', start_index, df)
        this_batch_sample_cost_imprv_gain = getParam('SAMPLE_COST_IMPRV_GAIN', start_index, df)
        this_batch_search_val_imprv_gain = getParam('SEARCH_VALUE_IMPRV_GAIN', start_index, df)


        # update the parameters
        
        # update the bias
        #  taken from CDC 24 paper
        y_1_obs = mixed_strat
        total_blooms_completed = float(sampled_blooms_completed + detected_blooms_completed + unsamp_undetect_blooms_completed)
        sampled_ratio = float(sampled_blooms_completed/ total_blooms_completed)
        detected_ratio = float((sampled_blooms_completed + detected_blooms_completed) / total_blooms_completed)
        observed_payoff = 0.6 * sampled_ratio + 0.4 * detected_ratio # get 40% of the reward just for detecting
       
      
        t_per_game = 5000
        adaptation_gain = np.diag([0.00075, 0.00125, 0.00025]) 
        payoff_matrix_hat = updatePayoffMatrix(payoff_matrix_hat, y_1_obs, observed_payoff, t_per_game, adaptation_gain)
        print('New estimate of payoff matrix:')
        print(payoff_matrix_hat)
        
        # update mixed strategy
        y_1_update_gain = 3.0
        y_vec_hat = np.array([[y_1_hat],[1-y_1_hat]])
        term3 = np.matmul(payoff_matrix_hat, y_vec_hat)
        y_1_hat_dot = float( y_1_hat * (term3[0]  - np.matmul(y_vec_hat.transpose(), term3) ) )
        y_1_hat = y_1_hat + y_1_hat_dot * y_1_update_gain
        if (y_1_hat > 1.0):
            y_1_hat = 1.0
        elif (y_1_hat < 0.0):
            y_1_hat = 0.0
            
        print('Estimate of NE mixed strategy = ' + str(y_1_hat))

        # update the bias
        b_update_gain = 0.1
        num_cols = len(df.columns)
        num_vehicles = float(np.floor((num_cols - 3) / 5))
        print('Found ' + str(num_vehicles) + ' vehicles')
        b_up = b_update_gain * calculateBdotScalar(y_1_obs, num_vehicles, y_1_hat)
        next_explore_bias = this_batch_explore_bias + b_up

        next_search_val_imprv_gain = this_batch_search_val_imprv_gain + 0.005 * 0.4 * float(unsamp_undetect_blooms_completed)
        next_sample_cost_imprv_gain = this_batch_sample_cost_imprv_gain + 0.005 * 0.6 * float(detected_blooms_completed)
        
        # Step 3. 
        #          If this batch was already done, check it
        #          If this is the last batch, and the next batch
        #          will run next, then save the params to a file.

        if (last_batch):
            
            # Open the file in write mode ('w')
            with open("params.txt", "w") as f:
                # Write each variable and its value to the file
                f.write("SEARCH_VALUE_IMPRV_GAIN=" + str(next_search_val_imprv_gain) +"\n")
                f.write("SAMPLE_COST_IMPRV_GAIN="  + str(next_sample_cost_imprv_gain) +"\n")
                f.write("EXPLORE_BIAS="            + str(next_explore_bias) +"\n")

        else:
            all_good = True

            next_batch_start_index = batch_start_indx[batch_number + 1]
            
            explore_bias_in_next_batch        = getParam('EXPLORE_BIAS', next_batch_start_index, df)
            sample_cost_imprv_gain_next_batch = getParam('SAMPLE_COST_IMPRV_GAIN', next_batch_start_index, df)
            search_val_imprv_gain_next_batch  = getParam('SEARCH_VALUE_IMPRV_GAIN', next_batch_start_index, df)

            if not (next_explore_bias == explore_bias_in_next_batch):
                all_good = False
            if not (next_sample_cost_imprv_gain == sample_cost_imprv_gain_next_batch ):
                all_good = False
            if not (next_search_val_imprv_gain == search_val_imprv_gain_next_batch):
                all_good = False

            if not (all_good):
                print('ERROR: The parameters in the next batch do not match')
                print('       the expected values')
                


                
if __name__ == "__main__":
    main()
