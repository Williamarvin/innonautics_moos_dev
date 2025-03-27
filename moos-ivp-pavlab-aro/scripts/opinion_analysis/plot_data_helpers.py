
# Parse a single line
# Ex: ['510.14264', 'abe:leader:wide_loiter=-10.6039:small_loiter=2.7532:return_to_charge=7.8507']
def parseOpinionLine(inputLine):
    opinion_names = []
    opinion_vals  = []
    v_name = ""
    group  = ""
    time   = inputLine[0]
    
    split_input = inputLine[1].split(":")
    v_name = split_input[0]
    
    for item in split_input:

        if item == "leader" or item == "follower":
            group = item

        if "=" in item:
            split_item = item.split("=")
            opinion_names.append(split_item[0])
            opinion_vals.append(float(split_item[1]))
            
    # augment if some opinions were not valid
    while len(opinion_vals) < 3:
        opinion_names.insert(0,"not_active")
        opinion_vals.append(0.0)
    
    return time, v_name, group, opinion_names, tuple(opinion_vals)


# clip a tuple
def clipTuple(tuple_in, min_, max_):
    clipped_vals = []

    # find the scale factor with the largest deviation
    scale = 1.0
    scale_factor = 1.0    # how far out is this coordinate
    for i in range(len(tuple_in)):
        
        if (tuple_in[i] < min_):
            scale = tuple_in[i] / min_
            if (scale > scale_factor):
                scale_factor = scale
        elif (tuple_in[i] > max_):
            scale = tuple_in[i] / max_
            if (scale > scale_factor):
                scale_factor = scale
    # scale all coordinates of the tuple and return
    for i in range(len(tuple_in)):
        clipped_vals.append( tuple_in[i] / scale_factor)

    return tuple(clipped_vals)
            

# check if any item in tuple is out of limits
def tupleInRange(tuple_in, min_, max_):
    
    for i in range(len(tuple_in)):
        if ( (tuple_in[i] < min_) or (tuple_in[i] > max_) ):
            return False

    return True
