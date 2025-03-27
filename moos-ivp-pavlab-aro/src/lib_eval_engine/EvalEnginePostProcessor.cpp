#include "EvalEnginePostProcessor.h"
#include <cerrno>

bool isDirectory(std::string dirname)
{
    DIR *dir = opendir(dirname.c_str());
    if (dir)
    {
        closedir(dir);
        return true;
    }
    else
    {
        return false;
    }
}

bool containsAlogFile(std::string dirname)
{
    DIR *dir = opendir(dirname.c_str());
    std::string alog_suffix = ".alog";
    if (dir)
    {
        struct dirent *entity;
        while ((entity = readdir(dir)) != nullptr)
        {
            std::string filename = entity->d_name;
            if (filename.size() > alog_suffix.size() &&
                filename.compare(filename.size() - alog_suffix.size(), alog_suffix.size(), alog_suffix) == 0)
            {
                closedir(dir);
                return true;
            }
        }
        closedir(dir);
    }
    return false;
}

bool isValidDirectory(std::string dirname)
{
    if (isDirectory(dirname))
    {
        return containsAlogFile(dirname);
    }
    else
    {
        return false;
    }
}

std::string getAlogFileName(std::string dirname)
{
    // To get this far, we already confirmed that an alog exists, so now we just extract is generic name
    DIR *dir = opendir(dirname.c_str());
    std::string alog_suffix = ".alog";
    struct dirent *entity;
    while ((entity = readdir(dir)) != nullptr)
    {
        std::string filename = entity->d_name;
        if (filename.size() > alog_suffix.size() &&
            filename.compare(filename.size() - alog_suffix.size(), alog_suffix.size(), alog_suffix) == 0)
        {
            closedir(dir);
            filename = dirname + filename;
            return filename;
        }
    }
    // TODO: Check error handling?
    return "";
}

EvalEnginePostProcessor::EvalEnginePostProcessor(EvalEngine *engine, std::string appName)
{
    // Receive the EvalEngine and name of the main application
    m_engine = engine;
    m_appName = appName;

    // Extract relevant information from the engine
    m_engineParameters = m_engine->getParameterKeys();
    m_engineSubscriptions = m_engine->getSubscriptions();
    m_enginePublications = m_engine->getPublications();

    // Default values
    m_debug = false;

    m_apptick = 4.0;
    m_commtick = 4.0;

    m_verbose_key = "verbose";
    m_verbose = false;

    m_rewrite_key = "rewrite";
    m_rewrite = false;

    m_tag_key = "tag";
    m_tag = "eerev";

    m_quiet_key = "quiet";
    m_quiet = false;

    m_process_name_key = "process_name";
    m_process_name = "NA";

    m_input_alog_file_key = "alog_file";
    m_input_alogf = "NA";
    m_output_alogf = "";

    m_moos_file_key = "moos_file";
    m_target_directory_key = "target_directory";
    m_target_directory = "Unassigned-target-directory";
    m_output_directory = "Unassigned-output-directory";
    m_input_moosf = "Unassigned-input-moos-file";
    m_output_moosf = "Unassigned-output-moos-file";
    m_logstart = "";

    // When outputting a final Evaluation/latest kernel publications, we exclude visual artifacts, as a convenience
    m_output_x_list.insert("TRAIL_RESET");
    m_output_x_list.insert("VIEW_CIRCLE");
    m_output_x_list.insert("VIEW_COMMS_PULSE");
    m_output_x_list.insert("VIEW_GRID");
    m_output_x_list.insert("VIEW_GRID_CONFIG");
    m_output_x_list.insert("VIEW_GRID_DELTA");
    m_output_x_list.insert("VIEW_POINT");
    m_output_x_list.insert("VIEW_POLYGON");
    m_output_x_list.insert("VIEW_SEGLIST");
    m_output_x_list.insert("VIEW_MARKER");
    m_output_x_list.insert("VIEW_RANGE_PULSE");
    m_output_x_list.insert("VIEW_VECTOR");
}

//---------------------------------------------------------
// dbg print

bool EvalEnginePostProcessor::dbg_print(const char *format, ...)
{
  // wrap formatted print statements to go to stdout, stderr, or a file
  // if its a file, handle it cleanly
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    if (m_fname[0] == '\0')
    {
      vfprintf(m_debug_stream, format, args);
    }
    else
    {
      m_debug_stream = fopen(m_fname, "a");
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
    }
    return true;
  }
  return false;
}

bool EvalEnginePostProcessor::setParam(std::string param_name, std::string param_val)
{
    //  Update the internal parameter dictionary
    bool handled = false;
    if (param_name == m_verbose_key)
    {
        m_verbose = true;
        handled = true;
    }
    else if (param_name == m_rewrite_key)
    {
        m_rewrite = true;
        handled = true;
    }
    else if (param_name == m_tag_key)
    {
        m_tag = param_val;
        m_param_lookup[m_tag_key] = m_tag;
        handled = true;
    }
    else if (param_name == m_quiet_key)
    {
        m_quiet = true;
        handled = true;
    }
    else if (param_name == m_process_name_key)
    {
        m_process_name = param_val;
        m_param_lookup[m_process_name_key] = m_process_name;
        handled = true;
    }
    else if (param_name == m_target_directory_key)
    {
        m_target_directory = param_val;
        if (m_target_directory.back() != '/')
        {
            m_target_directory += "/";
        }
        m_param_lookup[m_target_directory_key] = m_target_directory;
        handled = setParam(m_input_alog_file_key, getAlogFileName(m_target_directory));
    }
    else if (param_name == m_input_alog_file_key)
    {
        m_input_alogf = param_val;
        m_param_lookup[m_input_alog_file_key] = m_input_alogf;
        if (fileExists(m_input_alogf))
        {
            handled = true;
        }
        else
        {
            std::cerr << "ERR: .alog file does not exist < " << m_input_alogf << " >" << std::endl;
        }
    }
    else if (param_name == m_moos_file_key)
    {
        m_input_moosf = param_val;
        m_param_lookup[m_moos_file_key] = m_input_moosf;
        if (m_MissionReader.SetFile(m_input_moosf))
        {
            handled = true;
        }
        else
        {
            std::cerr << "ERR: Mission file not found <" << m_input_moosf << ">" << std::endl;
        }
    }
    return handled;
}

int getFileLength(std::string filename)
{
    std::ifstream file(filename);
    std::string line;
    int nlines;
    if (file.is_open())
    {
        while (std::getline(file, line))
        {
            nlines++;
        }
    }
    else
    {
        std::cerr << "ERR: Unable to open file <" << filename << ">" << std::endl;
        exit(1);
    }
    file.close();
    return nlines;
}

EEMessage EvalEnginePostProcessor::getMessage(std::string alog_line)
{
    EEMessage msg;
    msg.timestamp = stod(getTimeStamp(alog_line));
    msg.src = getSourceNameNoAux(alog_line);
    msg.name = getVarName(alog_line);
    msg.contents_str = getDataEntry(alog_line);
    return msg;
}

void EvalEnginePostProcessor::process()
{
    m_param_lookup[m_verbose_key] = m_tag;
    m_param_lookup[m_process_name_key] = m_process_name;
    m_param_lookup[m_input_alog_file_key] = m_input_alogf;
    m_param_lookup[m_moos_file_key] = m_input_moosf;

    // Use the mission reader and extract the configuration block for the associated EvalEngine
    getConfigParams();

    // Initialize EvalEngine
    m_engine->setIdentity(m_process_name);
    m_engine->setTime(0);
    bool handled = false;

    double apptick_time_s = 1 / m_apptick;
    double commtick_time_s = 1 / m_commtick;

    // The half the lesser of the two tick rates is the frequency in which we'll run our state machine
    double write_period_s = apptick_time_s < commtick_time_s ? apptick_time_s / 2.0 : commtick_time_s / 2.0;

    m_engine->Initialize();

    // If rewriting, make the directory and the file pointer, also dump the configuration block for the eval engine
    if (m_rewrite)
    {
        // Get the absolute time in which the log file was opened
        m_logstart = getLogStartStr();
        // We do this since you are not making a new mission, just rewriting what would have been realtime calculations, so it is still part of the original mission directory
        m_output_directory = m_target_directory + m_target_directory.substr(0, m_target_directory.size() - 1) + "_" + m_tag + "/";

        if (makeDirectory(m_output_directory) == false)
        {
            exit(1);
        }
        m_output_alogf = makeAlogFile();
        m_output_moosf = makeMOOSFile();
    }

    // Initialize states and latest updates for state machine

    std::map<std::string, EEMessage> mail;
    std::vector<std::pair<std::string, std::string> > updates;
    std::map<std::string, std::string> latest_updates_exclusive;
    std::map<std::string, std::string> prev_updates;

    bool done = false;
    std::ifstream file(m_input_alogf);
    std::string line;
    double next_mailupdate = 0;
    double next_evaluation = 0;
    double c_time = 0;
    bool first_read = true;

    // If the file is open, we proceed to go through the file
    int fileLength;
    if (m_verbose)
        fileLength = getFileLength(m_input_alogf);

    if (file.is_open())
    {
        int on_line = 0;
        while (std::getline(file, line))
        {
            on_line++;
            //  While we are not at the end of the file
            if (m_verbose)
            {
                // Verbose mode should only be used if the user is looking for a line number which is corrupt
                printf("%d/%d\n", on_line, fileLength);
            }
            // We remove lines which are comments or empty
            if (line.size() == 0 || line.at(0) == '%')
            {
                continue;
            }

            EEMessage msg = getMessage(line);

            // We check the message to see if it came from our EvalEngine process name argument, if it does, we ignore it

            if (msg.src == m_process_name)
            {
                // This line was published by the previous EvalEngine process, so we ignore it
                continue;
            }

            if (m_rewrite)
            {
                std::ofstream file(m_output_alogf, std::ios::app);
                if (file.is_open())
                {
                    file << line << std::endl;
                }
                else
                {
                    std::cerr << "Cannot open output alog file: " << m_output_alogf << std::endl;
                    exit(1);
                }
                file.close();
            }

            double time_stamp = msg.timestamp;

            if (time_stamp < 0)
                continue;

            mail[msg.name] = msg;

            // If it is time to handle new mail which has built up in the queue if we have new mail, then we update all the kernel variables and then schedule the next time in which we update variables
            // The state machine time also has to at least be in sync with the alog files previous logs in order to see if there have been viable updates

            // With this latest update in the buffer, let the state machine timer run until it is time to handle the new mail while processing back end updates

            // When running the approximate mailupdate, and if we have new mail, we let the kernels update all their variables
            if (c_time > time_stamp)
                c_time = time_stamp;
            do
            {
                if (time_stamp >= next_mailupdate && (mail.size() != 0))
                {
                    m_engine->setTime(c_time);
                    m_engine->UpdateVariables(mail);
                    mail.clear();
                    next_mailupdate = c_time + commtick_time_s;
                }

                // If it is time to write/publish another evaluation, then we write the latest updates to the file while scheduling the next evaluation time

                if (c_time >= next_evaluation)
                {
                    // Get all the new updates, and write them as an alogfile
                    std::vector<std::string> kernel_level_warnings;

                    updates = m_engine->ComputeEvaluations(kernel_level_warnings);
                    std::vector<std::pair<std::string, std::string> >::iterator it;

                    for (it = updates.begin(); it != updates.end(); ++it)
                    {
                        latest_updates_exclusive[it->first] = it->second;
                    }

                    // If we have runtime warnings and we are not in debug mode, then we need to alert the user of the runtime warnings
                    if (kernel_level_warnings.size() && !m_debug)
                    {
                        std::vector<std::string>::iterator warning_it;
                        for (warning_it = kernel_level_warnings.begin(); warning_it != kernel_level_warnings.end(); ++warning_it)
                        {
                            std::cerr << "ERR: Kernel Level Runtime Warning - " << *warning_it << std::endl;
                        }
                    }

                    if (m_rewrite)
                    {
                        std::string time = std::to_string(c_time);
                        std::stringstream newentry;

                        std::ofstream file(m_output_alogf, std::ios::app);
                        if (file.is_open())
                        {
                            for (it = updates.begin(); it != updates.end(); ++it)
                            {
                                // If this variable has previously been published to
                                if (prev_updates.count(it->first))
                                {
                                    // If the content of the message was the same, we continue
                                    if (prev_updates[it->first] == it->second)
                                        continue;
                                }
                                newentry << std::setw(15) << std::fixed << std::setprecision(5) << std::left << c_time << " "
                                         << std::setw(20) << std::left << it->first << " "
                                         << std::setw(15) << std::left << m_process_name << "  "
                                         << it->second << std::endl;
                                file << newentry.str();
                                prev_updates[it->first] = it->second;
                            }
                        }
                        else
                        {
                            std::cerr << "ERR: Cannot open output alog file: " << m_output_alogf << std::endl;
                            exit(1);
                        }
                        file.close();
                        next_evaluation = c_time + apptick_time_s;
                    }
                }
                c_time += write_period_s;
            } while (c_time <= time_stamp);
        }
    }
    else
    {
        std::cerr << "Unable to open input file: " << m_input_alogf << std::endl;
        exit(1);
    }

    // At this point, we now maintain the latest updates from all the kernels, and we write this to the stdout
    if (latest_updates_exclusive.size() != 0)
    {
        std::map<std::string, std::string>::iterator it;
        for (it = latest_updates_exclusive.begin(); it != latest_updates_exclusive.end(); ++it)
        {
            if (m_output_x_list.count(it->first))
                continue;
            std::cout << it->first << "=" << it->second;
            if (std::next(it) != latest_updates_exclusive.end())
            {
                std::cout << ", ";
            }
        }
        std::cout << std::endl;
    }
    else
    {
        std::cout << "Error: Kernel rendered no updates throughout Alog file" << std::endl;
        exit(1);
    }
}

bool EvalEnginePostProcessor::check_line(std::string line)
{
    // Might use?
    return true;
}

bool EvalEnginePostProcessor::fileExists(std::string fname)
{
    std::ifstream file(fname.c_str());
    return file.good();
}

bool EvalEnginePostProcessor::makeDirectory(std::string dirname)
{

    if (mkdir(dirname.c_str(), 0700) != 0)
    {
        switch (errno)
        {
        case EEXIST:
            return true;
            break;
        default:
        {
            std::cerr << "ERR: Unable to open directory <" << dirname << std::endl;
            return false;
            break;
        }
        }
    }
    return true;
}

int EvalEnginePostProcessor::asAscii(char c)
{
    return static_cast<int>(c);
};

bool EvalEnginePostProcessor::charIsNumber(char c)
{
    return ((asAscii(c) > asAscii('0')) && (asAscii(c) < asAscii('9')));
}

std::string EvalEnginePostProcessor::getLogStartStr()
{
    std::string line;
    std::ifstream file(m_input_alogf);
    std::string logstart_time = "";

    if (file.is_open())
    {
        // Extract the header - Terminate on an absense of '%'
        for (std::string line; (std::getline(file, line) && line[0] == '%');)
        {
            // Scan until we find the LOGSTART argument
            if (line.find("LOGSTART") != std::string::npos)
            {
                std::string tok = "LOGSTART";
                // gets everything after token
                int pos = line.find(tok);
                line = line.substr(pos + tok.size());
                // Once we find the logstart argument, we scan for all the legal numbers
                char c;
                for (int i = 0; i < line.size(); ++i)
                {
                    c = line[i];
                    if (!charIsNumber(c) && c != '.')
                        continue;
                    logstart_time += c;
                }
                break;
            }
            else
            {
                continue;
            }
        }
        if (logstart_time == "")
        {
            std::cerr << "Unable to obtain log start time from alog file\n";
            exit(1);
        }
        return logstart_time;
    }
    else
    {
        std::cerr << "Unable to open input alog log file\n";
        exit(1);
        return "";
    }
}

std::string EvalEnginePostProcessor::getLogHeader(std::string fname)
{
    std::string header;
    // Capture and generate the header of the alog log file
    std::string line;
    std::ifstream file(fname);
    int pos = 0;
    if (file.is_open())
    {
        // Extract the header - Terminate on an absense of '%'
        for (std::string line; (std::getline(file, line) && line[0] == '%');)
        {
            // Otherwise we need express that this is a manipulated and revised file
            // Format the new time in which it was opened
            header += line + "\n";
        }
    }
    else
    {
        std::cerr << "Unable to open file: " << fname << std::endl;
        exit(1);
    }
    return header;
}

std::string EvalEnginePostProcessor::makeAlogFile()
{
    // This will make a new alog log file
    std::string alogheader = getLogHeader(m_input_alogf);
    std::stringstream alogheaderss(alogheader);
    std::string new_alogheader;
    // Capture and generate the header of the alog log file
    int pos = 0;
    pos = m_input_alogf.find(".alog");
    std::string alog_output = m_output_directory + m_process_name + "_" + m_tag + ".alog";

    std::ofstream ofile(alog_output);

    if (alogheader.size())
    {
        // Extract the header - Terminate on an absense of '%'
        for (std::string line; std::getline(alogheaderss, line) && line[0] == '%';)
        {
            if (line.find("LOG FILE: ") != std::string::npos)
            {
                pos = line.find(m_input_alogf);
                new_alogheader += line.substr(0, pos) + alog_output;
            }
            else if (line.find("FILE OPENED ON") != std::string::npos)
            {
                pos = line.find("  ");
                std::string lead = line.substr(0, pos);
                // Update the file opened line
                new_alogheader += lead + "<Include new time here> " + "POST PROCESSED REWRITTEN FILE - Originally from " + line + "\n";
            }
            else
            {
                // Otherwise we need express that this is a manipulated and revised file
                // Format the new time in which it was opened
                new_alogheader += line + "\n";
            }
        }
    }
    else
    {
        std::cerr << "No contents in header: " << alogheader << std::endl;
        exit(1);
    }
    ofile << alogheader << std::endl;
    ofile.close();
    return alog_output;
}

std::string EvalEnginePostProcessor::makeMOOSFile()
{
    std::string new_moosheader;
    // Capture and generate the header of the alog log file
    int pos = 0;
    pos = m_input_moosf.find(".moos");
    std::string output_moos_fname = m_output_directory + m_input_moosf.substr(0, pos) + "_" + m_tag + "._moos";

    std::ofstream ofile(output_moos_fname);
    m_cached_moosf = getMOOSConfigAsStr(m_input_moosf);

    if (m_cached_moosf.size())
    {
        ofile << "//" << xChars("-", 59) << std::endl;
        ofile << "// Automatically generated by: <" << m_appName << "> when referencing the configuration for <" << m_process_name << ">" << std::endl;
        ofile << "//" << xChars("-", 59) << std::endl;
        ofile << m_cached_moosf << std::endl;
    }
    else
    {
        exit(1);
    }
    ofile.close();
    return output_moos_fname;
}

std::string EvalEnginePostProcessor::getMOOSConfigAsStr(std::string moosfile)
{
    std::string line;
    std::ifstream file(moosfile);
    std::stringstream moosfile_contents;
    bool found_start_to_config = false;
    int stack_height = 0;
    int stack_max = 0;
    if (file.is_open())
    {
        // Extract only the related configuration block in the .moos file to save it when rewriting
        while (std::getline(file, line))
        {
            if ((line.find("ProcessConfig") != std::string::npos) && (line.find(m_process_name) != std::string::npos))
            {
                found_start_to_config = true;
            }
            else if (!found_start_to_config)
            {

                continue;
            }

            // The stack must exceed one at least once and the next time it reaches zero, we've reached a full configuration block closed in brackets at the same level
            for (int i = 0; i < line.size(); ++i)
            {
                if (line[i] == '{')
                    stack_height++;
                if (line[i] == '}')
                    stack_height--;
                if (stack_height > stack_max)
                    stack_max = stack_height;
            }

            if (found_start_to_config)
            {
                moosfile_contents << line << std::endl;
            }
            if (found_start_to_config && stack_height == 0 && stack_max > 0)
                break;
        }
        return moosfile_contents.str();
    }
    else
    {
        std::cerr << "Unable to open input moos config file\n";
        exit(1);
    }
}

std::string EvalEnginePostProcessor::xChars(std::string c, int nchars)
{
    std::string str;
    for (int i = 0; i < nchars; ++i)
    {
        str += c;
    }
    return str;
}

void EvalEnginePostProcessor::getConfigParams()
{
    // Obtain parameter list from the process block name
    STRING_LIST sParams;
    m_MissionReader.EnableVerbatimQuoting(false);

    // Unlike a MOOS App, we need to manually obtain the apptick and commstick
    m_MissionReader.SetAppName(m_process_name);
    //m_MissionReader.GetConfigurationParam("APPTICK", m_apptick);
    //m_MissionReader.GetConfigurationParam("COMMSTICK", m_commtick);

    if (!m_MissionReader.GetConfiguration(m_process_name, sParams))
        std::cerr << "Unhandled configuration parameter: " << m_process_name << std::endl;

    // Similar to a MOOS App, we parse a mission with the mission reader
    STRING_LIST::iterator p;
    for (p = sParams.begin(); p != sParams.end(); ++p)
    {
        std::string orig = *p;
        std::string line = *p;
        std::string param = tolower(biteStringX(line, '='));
        std::string value = line;
        bool handled = false;
        if (m_engineParameters.count(param))
        {
            handled = m_engine->setParameter(param, value);
        }
        else if (param == "debug")
        {
            m_debug = (value == tolower("true")) ? true : false;

            if (m_debug)
            {
                time_t rawtime;
                struct tm *timeinfo;
                memset(m_fname, m_fname_buff_size, '\0');
                time(&rawtime);
                timeinfo = localtime(&rawtime);
                char fmt[m_fname_buff_size];
                memset(fmt, m_fname_buff_size, '\0');
                strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
                snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                         m_appName.c_str(), fmt);
                m_engine->set_debug_stream(m_fname);
            }
            handled = true;
        }
        else if (param == "commstick")
        {
            m_commtick = stod(value);
            handled = true;
        }
        else if (param == "apptick")
        {
            m_apptick = stod(value);
            handled = true;
        }
        if (!handled)
         {
            std::cerr << "Unhandled configuration parameter: " << orig << std::endl;
         }
    }
}
