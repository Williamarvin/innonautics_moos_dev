Expected useage

postKayakEvalEngine --pname={process name in newconfig.moos} --moos={path/to/*.moos} --alog={path/to/*.alog}

 - Provide the process name as granted in the *.moos file - if running for the first time, the names only in the .moos file and pname must match.
    If you are not running this for the first time, and you are applying parameter updates to an alog file which has already had publications from 
    an EvalEngine application, the process names in the alog file (inconsequentially), the *.moos file, and the pname argument must match (consequentially). 
    If applying updated EvalEngine publications, the previous ones remove messages which were published by the previous app. 