# lib_eval_engine README

This library aims to provide a collection of utilities which make the development of EvalEngine-type processes and applications easier. You can run
a collection of evaluation functions, called kernels, both in realtime and post processing. 

 - Your moos app or your app will have an "EvalEngine" instance
 - You can use this EvalEngine instance as it is used in pKayakEvalEngine and app_postKayakEvalEngine in moos-ivp-pavlab/KayakEvalEngineUtils/
 - Generally, you will only need to copy this directory and change the kernels which are used, class names, and add any additional behavior should you need it
 - At this level, there is a thick layer of abstraction between your application and the kernels which are maintained by the EvalEngine class and the EvalEnginePostProcessor
    - The user can define there own kernels using the standards set by the EvalEngineKernelCore abstract class
    - Examples can be followed in std_ee_kernels. These are a collection of standard and commonly needed kernels. 
        - xCounter (simplest)
        - xStateTimer
        - xOdometry
        - xCollisionDetect (not implemented, but coming soon)
    - Currently, only the kernels necessary for development and testing were implemented, but more should be coming if this library holds up
 - A user of the library only needs to develop a kernel for evaluating something that they need, and then its functionality can automatically be applied both in post processing and in realtime
    - If you have a collection of alog files in which 1) evaluations were not made but are desired, or 2) evaluations and conditions which should have an associated visual presentation, can both be added after the fact
    - Kernels are self documenting, meaning that any collection of kernels can be used together, and they will generate their configuration block
        - See "pKayakEvalEngine -e"
        - What will also soon be implemented, is the interfaces will be self documenting, since all necessary information is present to describe the kernels

## Useage

Useage of the applications should be quite simple. Referencing KayakEvalEngineUtils, you can see - 

 - User can add a custom eval engine with i.e. "pKayakEvalEngine -e >> alpha.moos" and add the process to pAntler
    - This is to get it running in realtime
 - User can use an eval engine in post processing with the following arguments, i.e.
    - ./ee MOOSLogTargetDirectory config.moos --pname=pEvalEngine
        - Will look for the process name in config.moos named pEvalEngine, and then simulate as if it was streamed in realtime, and then reports the latest value (for cummulative evaluations)
    - ./ee MOOSLogTargetDirectory config.moos --pname=pEvalEngine {opt}--tag={default=eerev} --rewrite
        - Will do the same as the previous, but will make a new directory with a tag, within the target directory, containing a new alog file and moos configuration file for only the pEvalEngine's config section.
            Since an EvalEngine should not modify an agents behavior, you are only adding new information, insights, and visual artifacts to the original mission file. With this, any modified
            alog files are nested within the original mission directory. This is forced behavior to prevent missuse or confusion. 
    - ./ee MOOSLogTargetDirectory config.moos --pname=pEvalEngine {opt}--tag={default=eerev} --rewrite --verbose
        - If the user has a bad line in the alog file and they are trying to identify it, this prints what line number the the post processor is on until you hit your issue. 

- In both real time applications and post processing applications, if the user follows the templates in KayakEvalEngineUtils, the user can debug their kernels, or possibly any of the existing kernels
    by enabling debug in the standard EvalEngine configuration block, in which all components in the application will share a common text file output. The only layer in which this is kept on, is within
    the EvalEngine core class, to make it easier to debug a users kernel. 
    i.e. 

    In application or kernel useage
    ...
    dbg_print("<%s>: Im having an issue here with: %s", kernel_name.c_str(), varname.c_str());
    ...
    //--------------------------------------------------------
    // pKayakEvalEngine Example MOOS Configuration

    ProcessConfig = pKayakEvalEngine
    {

    // Engine Configuration Parameters
        AppTick = 10
        CommsTick = 10

            debug = true

    // Kernel-specific configuration parameters

    //    > xCounter1 (default values as shown)
            pub_to_var_1 = X_COUNTER
            var_to_count_1 = NAV_X
    //    > xCounter2 (default values as shown)
            pub_to_var_2 = X_COUNTER
            var_to_count_2 = NAV_X
    //    > xStateTimer (default values as shown)
            pub_nresets_to_var = XSTATETIMER_NRESETS
                pub_time_to_var = XSTATETIMER_TIME
            timer_is_cummulative = true
            timer_reset_condition = XSTATETIMER_RESET
                    var_condition = TRUE
                    var_to_time = DEPLOY
    //    > xSurfaceOdometry (default values as shown)
                    conversion_factor = 1.000000
                display_sample_points = true
                            identity = xSurfaceOdometry
                            input_x_var = NAV_X
                            input_y_var = NAV_Y
                            output_var = ODOMETRY
            sample_points_seglist_label = odometry_sample_points
                        seglist_output = VIEW_SEGLIST
                                units = meters

    }


Roadmap/todo
 - Add include functionality for EvalEngineCore {showHelpAndExit, showInterfaceAndExit, etc..}
    - Add self documenting IO for pubs/subs for each kernel
 - Add near xCollisionKernel
    - Allow the user to define two collision boundaries (near collision, collision), produce a visual artifact, count near collisions, and publish a collision update
 - Include a wrapper for EvalEngine's debug capabilities, where each kernel and layer up to the application layer can share a common debug file
 - Create an GenEvalEngine script which generates a directory which is setup with an example use case
 - Document/add comments/docstrings


