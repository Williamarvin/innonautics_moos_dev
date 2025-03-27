/*
    Herein we define a class which wraps the standard procedure for using an EvalEngine in post-processing alog files

    At its core, it will maintain an accelerated state machine following the same events which arose in realtime.
    Generally, we have to assume that it was used as intended, which the Engine's contribution to the MOOSDB does not
    render any behavioral change in an agent which would have changed how a mission runs. 
    An EvalEngine is purely for:
        1 )real time evaluation and analysis, 
        2) displaying associated graphical changes in pMarineViewer (i.e. marking where and event occurred), 
        3) post processing an alog file which did not have real time evaluations running which can be added and viewed in alogview later, 
        4) obtaining the final results of any kernels (their latest publications)

    Desired interface and behavior

    ./ee target_directory config.moos {opt}--pname=pEvalEngine {opt}--tag={default=ee} --rewrite

    If rewriting and not reporting the latest values

    target_directory/
        *.alog
        *.blog
        *.ylog
        *.xlog
        *._bhv
        *._moos
        target_directory_{tag+_}{YYYY-MM-DD-HH:MM:SS}/
            *{_tag}.alog - rewritten alog file
            {configProcessName}._moos - The EvalEngine post processors configuration block (since all else is the same in the previous other moos file)
*/

#pragma once

#include "EvalEngine.h"
#include "MOOS/libMOOS/Utils/ProcessConfigReader.h"
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <iostream>
#include "LogUtils.h"
#include "MBUtils.h"
#include <iterator>
#include <sys/stat.h>
#include <dirent.h>
#include <set>
#include <cstdarg> //va_list, va_start, va_end

bool isDirectory(std::string dirname);

bool containsAlogFile(std::string dirname);

bool isValidDirectory(std::string dirname);

std::string getAlogFileName(std::string dirname);

class EvalEnginePostProcessor
{
public:
    EvalEnginePostProcessor(EvalEngine *engine, std::string appName);

    bool setParam(std::string param_name, std::string param_val);

    void process();

private: // State variables
    EvalEngine *m_engine;
    std::string m_appName;

    bool m_debug;
    FILE *m_debug_stream;
    static const uint16_t m_fname_buff_size = 255;
    char m_fname[m_fname_buff_size];

    double m_apptick;
    double m_commtick;

    // TODO: Engine Publications will be changing to a topic which maintains a list of all the kernels subscribed
    std::set<std::string> m_engineSubscriptions;

    // TODO: Engine Subscriptions will be changing to a topic which maintains a list of all the kernels publishing to them
    std::set<std::string> m_enginePublications;

    std::set<std::string> m_engineParameters;

    std::map<std::string, std::string> m_param_lookup;

    CProcessConfigReader m_MissionReader;
    // Verbose means we will print the active status while we are reading the alog file (Optional - Default false)
    std::string m_verbose_key;
    bool m_verbose;
    // Rewrite means we are going to rewrite a processed alog file which allows visualization tools
    // and cummulative observations to be plotted with the new parameters (Optional - Default false)
    std::string m_rewrite_key;
    bool m_rewrite;
    // Tag is how we will tag the rewritten file, if one is enabled (Optional - Default "_eerev")
    std::string m_tag_key;
    std::string m_tag;

    // Quiet means we will only report the final result and nothing else (Optional - Default false)
    std::string m_quiet_key;
    bool m_quiet;

    // The process name is what we are looking for in the configuration block (Mandatory - no default)
    std::string m_process_name_key;
    std::string m_process_name;

    // The alog file is what we will be reading (Mandatory - no default)
    std::string m_input_alog_file_key;
    std::string m_input_alogf;

    // The alog file is what we will be writing to (Optional and only if we are rewriting - defaults to the name of the original file + the tag)
    std::string m_output_alogf;

    // The MOOS file is where we will look for our configuration block (Mandatory - no default)
    std::string m_moos_file_key;
    std::string m_target_directory_key;
    std::string m_target_directory;
    std::string m_output_directory;
    std::string m_input_moosf;
    std::string m_output_moosf;

    std::string m_logstart;

    std::set<std::string> m_output_x_list;

    std::string m_cached_moosf;

protected: // Internal Methods
    bool dbg_print(const char *format, ...);

    bool check_line(std::string line);

    bool fileExists(std::string fname);

    bool makeDirectory(std::string dirname);

    EEMessage getMessage(std::string alog_line);

    int asAscii(char c);

    bool charIsNumber(char c);

    std::string getLogStartStr();

    std::string getLogHeader(std::string fname);

    std::string makeAlogFile();

    std::string makeMOOSFile();

    std::string getMOOSConfigAsStr(std::string moosfile);

    std::string xChars(std::string c, int nchars);

    void getConfigParams();
};
