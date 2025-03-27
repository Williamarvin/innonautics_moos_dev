#include "seatrac_beacon.hpp"

/*==============================================================================
CSeatrac Class Implimentation
==============================================================================*/
#ifdef SEATRAC_EN_DIAG
uint32 CSeatrac::_classCnt = 0;
double CSeatrac::_classTimebase = 0.0;
#endif

/*!-----------------------------------------------------------------------------
Constructor
*/
CSeatrac::CSeatrac()
{
    #ifdef SEATRAC_EN_DIAG
    //Incriment the class count
    _classCnt++;

    //Store the time the object was created
    if(_classTimebase == 0.0)
        _classTimebase = CTicks::GetSeconds();

    //Generate a class name
    char buf[32];
    snprintf(buf, sizeof(buf), "Seatrac%u", _classCnt);
    _diagName.assign(buf);

    //Disable diagnostic logging
    this->DiagLogFlagsSet(0);
    #endif

    //Create the command processor
    _cmdProc = new CSeatracCmdProc(ST_ENCODE_HEADER_CHAR, ST_DECODE_HEADER_CHAR, ST_MSG_LEN);
    _cmdProc->OnDecodeLine.AttachMethod(this, &CSeatrac::CmdProcDecodeLineEvent);
    _cmdProc->OnDecodeMsg.AttachMethod(this, &CSeatrac::CmdProcDecodeMsgEvent);
    _cmdProc->OnEncode.AttachMethod(this, &CSeatrac::CmdProcEncodeEvent);

    //Reset params to default values
    this->CmdReset();
}

/*!-----------------------------------------------------------------------------
Destructor
*/
CSeatrac::~CSeatrac()
{
    //Tidy up
    delete _cmdProc;
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::CmdDecode(string& str)
{
    //Pass the decode params into the Command Processor
    _cmdProc->Decode(str);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::CmdDecode(puint8 data, uint16 len)
{
    //Pass the decode params into the Command Processor
    _cmdProc->Decode(data, len);
}

/*!-----------------------------------------------------------------------------
Function that executes data from a buffer.
The Seatrac data message doesn't need a checksum at the end of the buffer
*/
void CSeatrac::CmdExecute(puint8 data, uint16 len)
{
    CSeatracCmdMsg msg;
    msg.WriteData(data, len);
    this->CmdExecute(&msg);
}

/*!-----------------------------------------------------------------------------
Function called when a message has been decoded and should be processed
*/
void CSeatrac::CmdExecute(PSeatracCmdMsg msg)
{
    //Create the execute params
    TSeatracExecuteParams paramsExe;
    TSeatracExecuteDoneParams paramsDone;

    if(!msg)
        return;

    //Lock the mutex to only allow one command to execute at a time
    SEATRAC_MUTEX_EXE_LOCK;

    //Set the message reader to the first received byte
    msg->SetReadIdx(0);

    //Get the CID command byte
    ESeatracCmdId id;
    bool success = msg->ReadCmdId(&id, ST_CID_INVALID);

    if(!success) {
        //The buffer must contain at least a command (and optional device address)
        paramsDone.ExeStatus = ST_EXE_PARAM_MISSING;
        paramsDone.Id = ST_CID_INVALID;
    }
    else {
        //Populate the message executions params
        paramsExe.Msg = msg;
        paramsExe.Id = id;
        paramsExe.Status = ST_EXE_OK;      //The handling function should set this to other values if an error occured.
        paramsExe.Handled = false;         //The handling function should set this to true

        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_EXECUTE_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_EXECUTE, paramsExe.Id);
            //diagMsg.AddParamString("Cmd", CSeatracUtils::ToStringCmdId(paramsExe.Id));
            diagMsg.AddParam("Length", "%u", paramsExe.Msg->GetLength());
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the external OnExecute event to allow outside code to process commands
        this->OnCmdExecute.Call(&paramsExe);

        //Now call the internal code to execute event (if the command isn't marked as handled)
        if(!paramsExe.Handled) {
            this->ExecuteCmd(&paramsExe);
        }

        //Update the done params with the results from the execution call
        paramsDone.Id = paramsExe.Id;
        paramsDone.ExeStatus = paramsExe.Status;

        if(!paramsExe.Handled) {
            //If the command isn't handled, then report it
            paramsDone.ExeStatus = ST_EXE_UNHANDLED;
        }
    }

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_EXECUTED_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_EXECUTED, paramsDone.Id, (paramsDone.ExeStatus != ST_EXE_OK));
        //diagMsg.AddParamString("Cmd", CSeatracUtils::ToStringCmdId(paramsDone.Id));
        diagMsg.AddParamString("Status", CSeatracUtils::ToStringExecuteStatus(paramsDone.ExeStatus));
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Raise the ExecuteDone event
    this->OnCmdExecuteDone.Call(&paramsDone);

    //Release the Mutex lock
    SEATRAC_MUTEX_EXE_UNLOCK;

    return;
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::CmdProcDecodeLineEvent(pointer context, PSeatracCmdDecodeLineParams params)
{
    FUNC_PARAM_UNUSED(context);

    #ifdef SEATRAC_EN_DIAG
    //If diagnostic logging is enabled for encoding then raise an event
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DECODE_BIT)) {
        string decodeStr;
        decodeStr.append("'");
        decodeStr.append((pchar)params->Buffer, params->Length);
        decodeStr.append("'");

        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_DECODE);
        diagMsg.AddParam("Length", "%u", params->Length + 2); //Acount for <CR><LF> that have been stripped out
        diagMsg.AddParamString("Data", decodeStr);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Raise the external Command Processor Decode Line event
    this->OnCmdDecodeLine.Call(params);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::CmdProcDecodeMsgEvent(pointer context, PSeatracCmdDecodeMsgParams params)
{
    FUNC_PARAM_UNUSED(context);

    //Raise the external Command Processor Decode Message event
    this->OnCmdDecodeMsg.Call(params);

    //Execute this message
    this->CmdExecute(params->Msg);
}

/*!-----------------------------------------------------------------------------
Virtual method that can be overriden by inheriting classes to add additional
functionality, such as serial transmission or logging of encoded data.
*/
void CSeatrac::CmdProcEncodeEvent(pointer context, PSeatracCmdEncodeParams params)
{
    FUNC_PARAM_UNUSED(context);

    #ifdef SEATRAC_EN_DIAG
    //If diagnostic logging is enabled for encoding then raise an event
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_ENCODE_BIT)) {
        //Remove the <CR><LF> characters from the Tx buffer, and append a string null char
        string encodeStr;
        encodeStr.append("'");
        encodeStr.append((pchar)params->Buffer, params->Length - 2);
        encodeStr.append("'");

        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ENCODE);
        diagMsg.AddParam("Length", "%u", params->Length);
        diagMsg.AddParamString("Data", encodeStr);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Raise the external Command Processor Encode event
    this->OnCmdEncode.Call(params);
}

/*!-----------------------------------------------------------------------------
Function called to perform a calibration action command.
*/
bool CSeatrac::CalAction(ESeatracCalAction action)
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_CAL_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_CAL_ACTION);
        diagMsg.AddParamString("CalAction", CSeatracUtils::ToStringCalAction(action));
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Cal Action command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_CAL_ACTION);
    msg.WriteUint8((uint8)action);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::CfgBeaconGet(uint16 destSerialNumber)
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_CFG_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_CFG_BEACON_GET);
        diagMsg.AddParam("DestSerialNumber", "%06u", destSerialNumber);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the CfgBeaconGet command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_CFG_BEACON_GET);
    msg.WriteUint16(destSerialNumber);
    
    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;	
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::CfgBeaconSet(uint16 destSerialNumber, ESeatracBeaconId beaconId, uint16 rangeTimeout, uint16 responseTurnaround)
{
	SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_CFG_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_CFG_BEACON_SET);
        diagMsg.AddParam("DestSerialNumber", "%06u", destSerialNumber);
        diagMsg.AddParamString("NewBeaconId", CSeatracUtils::ToStringBeaconId(beaconId));		
		diagMsg.AddParam("NewRangeTimeout", "%um", rangeTimeout);
		diagMsg.AddParam("NewResponseTurnaround", "%ums", responseTurnaround);
		this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the CfgBeaconGet command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_CFG_BEACON_SET);
    msg.WriteUint16(destSerialNumber);
    msg.AddBeaconId(beaconId);
    msg.WriteUint16(rangeTimeout);
    msg.WriteUint16(responseTurnaround);
	
    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}
		
/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::DatSend(ESeatracBeaconId destId, ESeatracAcoMsgType msgType, puint8 data, uint8 dataLen)
{
    //Abort if the destId is invalid
    if(!CSeatracUtils::IsBeaconIdPhysicalOrAll(destId))
        return false;

    //Abort is the message type isn't a request
    if(!CSeatracUtils::IsAcoMsgTypeRequest(msgType) && !CSeatracUtils::IsAcoMsgTypeOneWay(msgType))
        return false;

    //If no data specified, then force length to zero
    if(!data)
        dataLen = 0;

    //Abort is the data is invalid
    if(dataLen > ST_ACOMSG_PAYLOAD_LEN)
        return false;

    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_DAT_SEND);
        diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(destId));
        diagMsg.AddParamString("MsgType", CSeatracUtils::ToStringAcoMsgType(msgType));
        diagMsg.AddParamsBuffer("Packet", data, dataLen);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Echo Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_DAT_SEND);
    msg.AddBeaconId(destId);
    msg.AddAcoMsgType(msgType);
    msg.WriteUint8(dataLen);
    if(data)
        msg.WriteArray(data, dataLen);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::DatQueueSet(ESeatracBeaconId destId, puint8 data, uint8 dataLen)
{
    //Abort if the destId is invalid
    if(!CSeatracUtils::IsBeaconIdPhysicalOrAll(destId))
        return false;

    //If no data specified, then force length to zero
    if(!data)
        dataLen = 0;

    //Abort is the data is invalid
    if(dataLen > ST_ACOMSG_PAYLOAD_LEN)
        return false;

    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_DAT_QUEUE_SET);
        diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(destId));
        diagMsg.AddParamsBuffer("Packet", data, dataLen);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Echo Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_DAT_QUEUE_SET);
    msg.AddBeaconId(destId);
    msg.WriteUint8(dataLen);
    if(data)
        msg.WriteArray(data, dataLen);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::DatQueueClear(ESeatracBeaconId destId)
{
    //Abort if the destId is invalid
    if(!CSeatracUtils::IsBeaconIdPhysicalOrAll(destId))
        return false;

    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_DAT_QUEUE_CLR);
        diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(destId));
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Reboot command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_DAT_QUEUE_CLR);
    msg.AddBeaconId(destId);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::DatQueueStatus()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_DAT_QUEUE_STATUS);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Reboot command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_DAT_QUEUE_STATUS);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
Function that returns the current diagnostic flags settings.
@param mask specifies which bits should be cleared
@result The modified set of DiagLog flags
*/
#ifdef SEATRAC_EN_DIAG
uint32 CSeatrac::DiagLogFlagsGet()
{
    return _diagLogFlags;
}
#endif

/*!-----------------------------------------------------------------------------
Function that sets flags in the DiagLog flags variable, enableing
DiagLog events for specific actions to be generated
@param value specifies the new value of the flags
*/
#ifdef SEATRAC_EN_DIAG
void CSeatrac::DiagLogFlagsSet(uint32 value)
{
    _diagLogFlags = value;
}
#endif

/*!-----------------------------------------------------------------------------
Function that raises a DiagLog event with a diagnostic message
*/
#ifdef SEATRAC_EN_DIAG
void CSeatrac::DoDiagLog(CSeatracDiagMsg& diagMsg)
{
    double timeNow = CTicks::GetSeconds();
    double timestamp = timeNow - _classTimebase;

    //Make the params struct from the diag message
    TSeatracDiagLogParams params;
    diagMsg.MakeParams(params);
    params.Timestamp = timestamp;
    snprintf(params.NameStr, sizeof(params.NameStr), "%s", _diagName.c_str());

    //Raise the event
    SEATRAC_MUTEX_DIAGLOG_LOCK;
    this->OnDiagLog.Call(&params);
    SEATRAC_MUTEX_DIAGLOG_UNLOCK;
}
#endif

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::EchoSend(ESeatracBeaconId destId, ESeatracAcoMsgType msgType, puint8 data, uint8 dataLen)
{
    //Abort if the destId is invalid
    if(!CSeatracUtils::IsBeaconIdPhysical(destId))
        return false;

    //Abort is the message type isn't a request
    if(!CSeatracUtils::IsAcoMsgTypeRequest(msgType))
        return false;

    //If no data specified, then force length to zero
    if(!data)
        dataLen = 0;

    //Abort is the data is invalid
    if(dataLen > ST_ACOMSG_PAYLOAD_LEN)
        return false;

    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_ECHO_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_ECHO_SEND);
        diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(destId));
        diagMsg.AddParamString("MsgType", CSeatracUtils::ToStringAcoMsgType(msgType));
        diagMsg.AddParamsBuffer("Packet", data, dataLen);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Echo Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_ECHO_SEND);
    msg.AddBeaconId(destId);
    msg.AddAcoMsgType(msgType);
    msg.WriteUint8(dataLen);
    if(data)
        msg.WriteArray(data, dataLen);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
Function called when a command is being executed and functions derived from it.
This function is called after the OnExecute event is called, allowing external
trapping of command events which can mark the command as handled and
prevent internal execution.
*/
void CSeatrac::ExecuteCmd(PSeatracExecuteParams exe)
{
    switch(exe->Id) {
        //System Messages
        case ST_CID_SYS_ALIVE : { this->ExecuteCmdSysAlive(exe); break; }
        case ST_CID_SYS_INFO : { this->ExecuteCmdSysInfo(exe); break; }
        case ST_CID_SYS_REBOOT : { this->ExecuteCmdSysReboot(exe); break; }
        //case ST_CID_SYS_ENGINEERING : { this->ExecuteXXX(exe); break; }
        // Firmware Programming Messages
        case ST_CID_PROG_INIT : { this->ExecuteCmdProgInit(exe); break; }
        case ST_CID_PROG_BLOCK : { this->ExecuteCmdProgBlock(exe); break; }
        case ST_CID_PROG_UPDATE : { this->ExecuteCmdProgUpdate(exe); break; }
        // Status Messages
        case ST_CID_STATUS : { this->ExecuteCmdStatus(exe); break; }
        case ST_CID_STATUS_CFG_GET : { this->ExecuteCmdStatusCfgGet(exe); break; }
        case ST_CID_STATUS_CFG_SET : { this->ExecuteCmdStatusCfgSet(exe); break; }
        // Settings Messages
        case ST_CID_SETTINGS_GET : { this->ExecuteCmdSettingsGet(exe); break; }
        case ST_CID_SETTINGS_SET : { this->ExecuteCmdSettingsSet(exe); break; }
        case ST_CID_SETTINGS_LOAD : { this->ExecuteCmdSettingsLoad(exe); break; }
        case ST_CID_SETTINGS_SAVE : { this->ExecuteCmdSettingsSave(exe); break; }
        case ST_CID_SETTINGS_RESET : { this->ExecuteCmdSettingsReset(exe); break; }
        // Calibration Messages
        case ST_CID_CAL_ACTION : { this->ExecuteCmdCalAction(exe); break; }
        //case ST_CID_CAL_AHRS_GET : { this->ExecuteCmdCalAhrsGet(exe); break; }
        //case ST_CID_CAL_AHRS_SET : { this->ExecuteCmdCalAhrsSet(exe); break; }
        // Acoustic Transceiver Messages
        case ST_CID_XCVR_ANALYSE : { this->ExecuteCmdXcvrAnalyse(exe); break; }
        case ST_CID_XCVR_TX_MSG : { this->ExecuteCmdXcvrTxMsg(exe); break; }
        case ST_CID_XCVR_RX_ERR : { this->ExecuteCmdXcvrRxErr(exe); break; }
        case ST_CID_XCVR_RX_MSG : { this->ExecuteCmdXcvrRxMsg(exe); break; }
        case ST_CID_XCVR_RX_REQ : { this->ExecuteCmdXcvrRxReq(exe); break; }
        case ST_CID_XCVR_RX_RESP : { this->ExecuteCmdXcvrRxResp(exe); break; }
        case ST_CID_XCVR_RX_UNHANDLED : { this->ExecuteCmdXcvrRxUnhandled(exe); break; }
        case ST_CID_XCVR_USBL : { this->ExecuteCmdXcvrUsbl(exe); break; }
        case ST_CID_XCVR_FIX : { this->ExecuteCmdXcvrFix(exe); break; }
        case ST_CID_XCVR_STATUS : { this->ExecuteCmdXcvrStatus(exe); break; }
		case ST_CID_XCVR_TX_MSGCTRL_SET : { this->ExecuteCmdXcvrTxMsgCtrlSet(exe); break; }
        // PING Protocol Messages
        case ST_CID_PING_SEND : { this->ExecuteCmdPingSend(exe); break; }
        case ST_CID_PING_REQ : { this->ExecuteCmdPingReq(exe); break; }
        case ST_CID_PING_RESP : { this->ExecuteCmdPingResp(exe); break; }
        case ST_CID_PING_ERROR : { this->ExecuteCmdPingError(exe); break; }
        // ECHO Protocol Messages
        case ST_CID_ECHO_SEND : { this->ExecuteCmdEchoSend(exe); break; }
        case ST_CID_ECHO_REQ : { this->ExecuteCmdEchoReq(exe); break; }
        case ST_CID_ECHO_RESP : { this->ExecuteCmdEchoResp(exe); break; }
        case ST_CID_ECHO_ERROR : { this->ExecuteCmdEchoError(exe); break; }
        // NAV Protocol Messages
        case ST_CID_NAV_QUERY_SEND : { this->ExecuteCmdNavQuerySend(exe); break; }
        case ST_CID_NAV_QUERY_REQ : { this->ExecuteCmdNavQueryReq(exe); break; }
        case ST_CID_NAV_QUERY_RESP : { this->ExecuteCmdNavQueryResp(exe); break; }
        case ST_CID_NAV_ERROR : { this->ExecuteCmdNavError(exe); break; }
        case ST_CID_NAV_QUEUE_SET : { this->ExecuteCmdNavQueueSet(exe); break; }
        case ST_CID_NAV_QUEUE_CLR : { this->ExecuteCmdNavQueueClr(exe); break; }
        case ST_CID_NAV_QUEUE_STATUS : { this->ExecuteCmdNavQueueStatus(exe); break; }
        case ST_CID_NAV_STATUS_SEND : { this->ExecuteCmdNavStatusSend(exe); break; }
        case ST_CID_NAV_STATUS_RECEIVE : { this->ExecuteCmdNavStatusReceive(exe); break; }
        // DAT Protocol Messages
        case ST_CID_DAT_SEND : { this->ExecuteCmdDatSend(exe); break; }
        case ST_CID_DAT_RECEIVE : { this->ExecuteCmdDatReceive(exe); break; }
        case ST_CID_DAT_ERROR : { this->ExecuteCmdDatError(exe); break; }
        case ST_CID_DAT_QUEUE_SET : { this->ExecuteCmdDatQueueSet(exe); break; }
        case ST_CID_DAT_QUEUE_CLR : { this->ExecuteCmdDatQueueClr(exe); break; }
        case ST_CID_DAT_QUEUE_STATUS : { this->ExecuteCmdDatQueueStatus(exe); break; }
        // DEX Protocol Messages
        //case ST_CID_DEX_CLOSE : { this->ExecuteXXX(exe); break; }
        //case ST_CID_DEX_DEBUG : { this->ExecuteXXX(exe); break; }
        //case ST_CID_DEX_ENQUEUE : { this->ExecuteXXX(exe); break; }
        //case ST_CID_DEX_OPEN : { this->ExecuteXXX(exe); break; }
        //case ST_CID_DEX_RESET : { this->ExecuteXXX(exe); break; }
        //case ST_CID_DEX_SEND : { this->ExecuteXXX(exe); break; }
        //case ST_CID_DEX_SOCKETS : { this->ExecuteXXX(exe); break; }
        //case ST_CID_DEX_RECEIVE : { this->ExecuteXXX(exe); break; }
		// CFG Protocol Messages
		case ST_CID_CFG_BEACON_GET : { this->ExecuteCmdCfgBeaconGet(exe); break; }
        case ST_CID_CFG_BEACON_SET : { this->ExecuteCmdCfgBeaconSet(exe); break; }
        case ST_CID_CFG_BEACON_RESP : { this->ExecuteCmdCfgBeaconResp(exe); break; }

        default: { break; }
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdCalAction(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracCmdStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus, ST_CST_FAIL);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_CAL_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnSysReboot event
        this->OnCalAction.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
//void CSeatrac::ExecuteCmdCalAhrsGet(PSeatracExecuteParams exe)
//{
//    PRAGMA_TODO("Impliment function");
//}

/*!-----------------------------------------------------------------------------
*/
//void CSeatrac::ExecuteCmdCalAhrsSet(PSeatracExecuteParams exe)
//{
//    PRAGMA_TODO("Impliment function");
//}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdCfgBeaconGet(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracCfgBeaconAckParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadUint16(&params.SerialNumber);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_CFG_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParam("DestSerialNumber", "%06u", params.SerialNumber);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnCfgBeaconGet event
        this->OnCfgBeaconGet.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdCfgBeaconSet(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracCfgBeaconAckParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadUint16(&params.SerialNumber);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_CFG_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParam("DestSerialNumber", "%06u", params.SerialNumber);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnCfgBeaconGet event
        this->OnCfgBeaconSet.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdCfgBeaconResp(PSeatracExecuteParams exe)
{
	bool success = true;
    TSeatracCfgBeaconRespParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadUint16(&params.SerialNumber);
	success &= exe->Msg->ReadBeaconId(&params.BeaconId, ST_BEACON_UNKNOWN);
	success &= exe->Msg->ReadUint16(&params.RangeTimeout);
	success &= exe->Msg->ReadUint16(&params.ResponseTurnaround);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_CFG_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParam("BeaconSerialNumber", "%06u", params.SerialNumber);
			diagMsg.AddParamString("BeaconId", CSeatracUtils::ToStringBeaconId(params.BeaconId));
			diagMsg.AddParam("RangeTimeout", "%um", params.RangeTimeout);
			diagMsg.AddParam("ResponseTurnaround", "%ums", params.ResponseTurnaround);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnCfgBeaconGet event
        this->OnCfgBeaconResp.Call(&params);
    }
}
		
/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdDatError(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracDatReceiveParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Fix);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    params.LocalFlag = true;
    params.Ack = false;
    params.Payload.Length = 0;
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.Fix.Msg.SrcId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsAcoFixMsg(params.Fix.Msg);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatReceive event with the error
        this->OnDatReceive.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdDatQueueSet(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracDatQueueSetParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    bool destIdValid = exe->Msg->ReadBeaconId(&params.DestId, ST_BEACON_UNKNOWN);
    bool lengthValid = exe->Msg->ReadUint8(&params.Length, 0);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(params.DestId));
            diagMsg.AddParam("PacketLen", "%u", params.Length);

            if(!destIdValid) {
                diagMsg.AddNote("DestId missing (assumed BEACON_UNKNOWN) - upgrade firmware");
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
                exe->Status = ST_EXE_PARAM_MISSING;
            }
            if(!lengthValid) {
                diagMsg.AddNote("PacketLen missing (assumed 0) - upgrade firmware");
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
                exe->Status = ST_EXE_PARAM_MISSING;
            }

            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatQueueSet event
        this->OnDatQueueSet.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdDatQueueClr(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracDatQueueClrParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    bool destIdValid = exe->Msg->ReadBeaconId(&params.DestId, ST_BEACON_UNKNOWN);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(params.DestId));

            if(!destIdValid) {
                diagMsg.AddNote("DestId missing (assumed BEACON_UNKNOWN) - upgrade firmware");
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
                exe->Status = ST_EXE_PARAM_MISSING;
            }

            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatQueueClear event
        this->OnDatQueueClear.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdDatQueueStatus(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracDatQueueStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    params.CmdStatus = ST_CST_OK;
    //success &= exe->Msg->ReadUint8(&params.Length);   //Firmware prior to v1.12
    for(uint8 idx = 0; idx < ST_BEACONS; idx++) {
        success &= exe->Msg->ReadUint8(&params.Length[idx], 0);
    }

    //if(!success) {
    //    //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
    //    exe->Status = ST_EXE_PARAM_MISSING;
    //}
    //else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsArray("PacketLen", params.Length, ST_BEACONS);

            if(!success) {
                diagMsg.AddNote("PacketLen missing (assumed 0) - upgrade firmware");
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
                exe->Status = ST_EXE_PARAM_MISSING;
            }

            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatQueueStatus event
        this->OnDatQueueStatus.Call(&params);
    //}
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdDatReceive(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracDatReceiveParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Fix);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.Fix);
    success &= exe->Msg->ReadBool(&params.Ack, false);
    success &= exe->Msg->ReadAcoPayload(&params.Payload);
    bool localFlagValid = exe->Msg->ReadBool(&params.LocalFlag, false);    //Supported in firmware 1.12 onwards

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        params.CmdStatus = ST_CST_OK;

        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamBool("Ack", params.Ack);
            diagMsg.AddParamsAcoFix(params.Fix);
            diagMsg.AddParamsBuffer("Packet", params.Payload);
            diagMsg.AddParamBool("LocalFlag", params.LocalFlag);

            if(!localFlagValid) {
                diagMsg.AddNote("LocalFlag missing (assumed FALSE) - upgrade firmware");
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
                exe->Status = ST_EXE_PARAM_MISSING;
            }

            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatReceive event with the error
        this->OnDatReceive.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdDatSend(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracDatSendParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.DestId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_DAT_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(params.DestId));
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatSend event
        this->OnDatSend.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdEchoError(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracEchoResponseParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Response);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    params.Payload.Length = 0;
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.Response.Msg.SrcId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_ECHO_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsAcoFixMsg(params.Response.Msg);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnEchoResponse event with the error
        this->OnEchoResponse.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdEchoReq(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracEchoRequestParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Request);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.Request);
    success &= exe->Msg->ReadAcoPayload(&params.Payload);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_ECHO_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsAcoFix(params.Request);
            diagMsg.AddParamsBuffer("Packet", params.Payload);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnEchoRequest event with the error
        this->OnEchoRequest.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdEchoResp(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracEchoResponseParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Response);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.Response);
    success &= exe->Msg->ReadAcoPayload(&params.Payload);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        params.CmdStatus = ST_CST_OK;

        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_ECHO_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsAcoFix(params.Response);
            diagMsg.AddParamsBuffer("Packet", params.Payload);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnEchoResponse event with the error
        this->OnEchoResponse.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdEchoSend(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracEchoSendParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.DestId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_ECHO_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(params.DestId));
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnEchoSend event
        this->OnEchoSend.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
NavError can be generated by both the transceiver timeing out in the case of
a request/response issue, but also in the case of malformatted or data error
in the packets. For simplicity with other protocols, generate a Response
event with a timeout error code in the case of a transceiver error, otherwise
ignore the error (other that a diagnostic output).
*/
void CSeatrac::ExecuteCmdNavError(PSeatracExecuteParams exe)
{
    bool success = true;
    //ESeatracCmdStatus status;
    //ESeatracBeaconId beaconId;
    TSeatracNavQueryResponseParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Response);
    CSeatracUtils::InitNavQuery(params.Query);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    params.LocalFlag = true;
    params.Payload.Length = 0;
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.Response.Msg.SrcId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsAcoFixMsg(params.Response.Msg);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnNavQueryResponse event with the error
        this->OnNavQueryResponse.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdNavQuerySend(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracNavQuerySendParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.DestId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(params.DestId));
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnNavQuerySend event
        this->OnNavQuerySend.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdNavQueryReq(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracNavQueryRequestParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Request);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.Request);
    success &= exe->Msg->ReadUint8(&params.QueryFlags);
    bool packetValid = exe->Msg->ReadNavPacket(&params.Payload);             //Supported in firmware 1.12 onwards
    bool localFlagValid = exe->Msg->ReadBool(&params.LocalFlag, false);     //Supported in firmware 1.12 onwards

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsAcoFix(params.Request);
            diagMsg.AddParam("QueryFlags", "0x%02X", params.QueryFlags);
            diagMsg.AddParamsBuffer("Packet", params.Payload);
            diagMsg.AddParamBool("LocalFlag", params.LocalFlag);

            if(!packetValid) {
                diagMsg.AddNote("PacketLen missing (assumed 0) - upgrade firmware");
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
                exe->Status = ST_EXE_PARAM_MISSING;
            }
            if(!localFlagValid) {
                diagMsg.AddNote("LocalFlag missing (assumed FALSE) - upgrade firmware");
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
                exe->Status = ST_EXE_PARAM_MISSING;
            }

            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnNavQueryRequest event with the error
        this->OnNavQueryRequest.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdNavQueryResp(PSeatracExecuteParams exe)
{
    bool success = true;
    bool packetValid = false;
    TSeatracNavQueryResponseParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Response);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.Response);
    success &= exe->Msg->ReadUint8(&params.QueryFlags);
    success &= exe->Msg->ReadNavQuery(&params.Query, params.QueryFlags);
    if(IS_BIT_SET(params.QueryFlags, ST_NAVQUERY_FLAGS_DATA_BIT)) {
        packetValid = exe->Msg->ReadNavPacket(&params.Payload);             //Supported in firmware 1.12 onwards
    }
    else {
        //If no data is present, set packetValid to true, so we don't generate the
        //diagnostic warning below.
        packetValid = true;
    }
    bool localFlagValid = exe->Msg->ReadBool(&params.LocalFlag, false);     //Supported in firmware 1.12 onwards

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        params.CmdStatus = ST_CST_OK;

        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsAcoFix(params.Response);
            diagMsg.AddParam("QueryFlags", "0x%02X", params.Query.Flags);
            diagMsg.AddParamsNavQuery(params.Query);
            diagMsg.AddParamsBuffer("Packet", params.Payload);
            diagMsg.AddParamBool("LocalFlag", params.LocalFlag);

            if(!packetValid) {
                diagMsg.AddNote("PacketLen missing (assumed 0) - upgrade firmware");
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
                exe->Status = ST_EXE_PARAM_MISSING;
            }
            if(!localFlagValid) {
                diagMsg.AddNote("LocalFlag missing (assumed FALSE) - upgrade firmware");
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
                exe->Status = ST_EXE_PARAM_MISSING;
            }

            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnPingResponse event with the error
        this->OnNavQueryResponse.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdNavQueueSet(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracNavQueueSetParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.DestId);
    success &= exe->Msg->ReadUint8(&params.Length, 0);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(params.DestId));
            diagMsg.AddParam("PacketLen", "%u", params.Length);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatQueueSet event
        this->OnNavQueueSet.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdNavQueueClr(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracNavQueueClrParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.DestId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(params.DestId));
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatQueueClear event
        this->OnNavQueueClear.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdNavQueueStatus(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracNavQueueStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    params.CmdStatus = ST_CST_OK;
    for(uint8 idx = 0; idx < ST_BEACONS; idx++) {
        success &= exe->Msg->ReadUint8(&params.Length[idx], 0);
    }

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsArray("PacketLen", params.Length, ST_BEACONS);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatQueueStatus event
        this->OnNavQueueStatus.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdNavStatusReceive(PSeatracExecuteParams exe)
{
 bool success = true;
    TSeatracNavStatusReceiveParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Fix);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.Fix);
    success &= exe->Msg->ReadBeaconId(&params.BeaconId);
    success &= exe->Msg->ReadNavPacket(&params.Payload);
    success &= exe->Msg->ReadBool(&params.LocalFlag, false);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        //params.CmdStatus = ST_CST_OK;

        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsAcoFix(params.Fix);
            diagMsg.AddParamString("BeaconId", CSeatracUtils::ToStringBeaconId(params.BeaconId));
            diagMsg.AddParamsBuffer("Packet", params.Payload);
            diagMsg.AddParamBool("LocalFlag", params.LocalFlag);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatReceive event with the error
        this->OnNavStatusReceive.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdNavStatusSend(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracNavStatusSendParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.BeaconId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamString("BeaconId", CSeatracUtils::ToStringBeaconId(params.BeaconId));
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnDatQueueClear event
        this->OnNavStatusSend.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdPingError(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracPingResponseParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Response);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.Response.Msg.SrcId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_PING_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsAcoFixMsg(params.Response.Msg);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnPingResponse event with the error
        this->OnPingResponse.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdPingReq(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracPingRequestParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Request);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.Request);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_PING_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsAcoFix(params.Request);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnPingRequest event with the error
        this->OnPingRequest.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdPingResp(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracPingResponseParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.Response);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.Response);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        params.CmdStatus = ST_CST_OK;

        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_PING_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsAcoFix(params.Response);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnPingResponse event with the error
        this->OnPingResponse.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
Function called when a serial acknowledge of a CID_PIG_SEND command is received,
indicating a Ping has either been sent or failed.
*/
void CSeatrac::ExecuteCmdPingSend(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracPingSendParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadBeaconId(&params.DestId);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_PING_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(params.DestId));
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnPingSend event
        this->OnPingSend.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdProgBlock(PSeatracExecuteParams exe)
{
    PRAGMA_TODO("Impliment function");
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdProgInit(PSeatracExecuteParams exe)
{
    PRAGMA_TODO("Impliment function");
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdProgUpdate(PSeatracExecuteParams exe)
{
    PRAGMA_TODO("Impliment function");
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdSettingsGet(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracSettingsGetParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadSettings(&params.Settings);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SETTINGS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsSettings(params.Settings);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnStatus event
        this->OnSettingsGet.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdSettingsLoad(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracCmdStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SETTINGS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnStatus event
        this->OnSettingsLoad.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdSettingsReset(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracCmdStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SETTINGS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnStatus event
        this->OnSettingsReset.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdSettingsSave(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracCmdStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SETTINGS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnStatus event
        this->OnSettingsSave.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdSettingsSet(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracCmdStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SETTINGS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnStatus event
        this->OnSettingsSet.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdStatus(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadStatus(&params.Status);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_STATUS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsStatus(params.Status);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnStatus event
        this->OnStatus.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdStatusCfgGet(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracStatusCfgGetParams params;
    uint8 flags;
    uint8 mode;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadUint8(&flags);
    success &= exe->Msg->ReadUint8(&mode);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        params.Mode = (ESeatracStatusMode)(mode & 0x07);
        params.Flags = flags;

        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_STATUS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamString("Mode", CSeatracUtils::ToStringStatusMode(params.Mode));
            diagMsg.AddParam("Flags", "0x%02X", params.Flags);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnStatusCfgGet event
        this->OnStatusCfgGet.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdStatusCfgSet(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracCmdStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus, ST_CST_FAIL);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_STATUS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnStatusCfgGet event
        this->OnStatusCfgSet.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdSysAlive(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracSysAliveParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadUint32(&params.Seconds, 0);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SYS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParam("Seconds", "%us", params.Seconds);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnSysAlive event
        this->OnSysAlive.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdSysInfo(PSeatracExecuteParams exe)
{
    bool success = true;
	uint8 tmp8;
	uint16 tmp16;
    TSeatracSysInfoParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadUint32(&params.Seconds, 0);
    success &= exe->Msg->ReadAppType(&params.AppType, ST_APPTYPE_UNKNOWN);
    success &= exe->Msg->ReadInfoHw(&params.Hardware);
    success &= exe->Msg->ReadInfoFw(&params.BootApp);
    success &= exe->Msg->ReadInfoFw(&params.MainApp);
	//Decode additional main application params
	if(params.AppType == ST_APPTYPE_MAIN) {	
		success &= exe->Msg->ReadUint8(&tmp8);							//Should be 0xFF
		success &= exe->Msg->ReadUint8(&tmp8);							//Should be 0x01, Indicates Pressure Sensor Flags
		success &= exe->Msg->ReadUint8(&tmp8);							//Should be 0x00, reserved
		success &= exe->Msg->ReadUint16(&tmp16);						//Should be 0x0000, reserved	
		success &= exe->Msg->ReadPressureCal(&params.PressureCal);	
	}

    params.BeaconType = ST_BEACONTYPE_UNKNOWN;    

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        //Determine if the library supports the firmware version in the beacon
        uint32 curFirmware = (params.MainApp.VersionMajor * 1000) + params.MainApp.VersionMinor;
        uint32 minFirmware = (ST_FIRMWARE_MAIN_VERSION_MAJOR * 1000) + ST_FIRMWARE_MAIN_VERSION_MINOR;
        params.MainSupported = (curFirmware >= minFirmware);

        //Convert the part number to a beacon type
        params.BeaconType = CSeatracUtils::ToBeaconType(params.Hardware.PartNumber);

        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SYS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParam("Seconds", "%us", params.Seconds);
            diagMsg.AddParamString("AppType", CSeatracUtils::ToStringAppType(params.AppType));
            diagMsg.AddParamString("BeaconType", CSeatracUtils::ToStringBeaconType(params.BeaconType));
            diagMsg.AddParam("HardwarePart", "BP%05u.%06u", params.Hardware.PartNumber, params.Hardware.SerialNumber);
            diagMsg.AddParam("HardwareRev", "%u", params.Hardware.PartRev);
            diagMsg.AddParam("HardwareFlags", "{0x%04X,0x%04X}", params.Hardware.FlagsSys, params.Hardware.FlagsUser);
            diagMsg.AddParamBool("BootValid", params.BootApp.Valid);
            if(params.BootApp.Valid) {
                diagMsg.AddParam("BootPart", "BP%05u", params.BootApp.PartNumber);
                diagMsg.AddParamString("BootVersion", CSeatracUtils::ToStringVersion(params.BootApp.VersionMajor, params.BootApp.VersionMinor, params.BootApp.VersionBuild));
                diagMsg.AddParam("BootChecksum", "0x%08X", params.BootApp.Checksum);
            }
            diagMsg.AddParamBool("MainValid", params.MainApp.Valid);
            if(params.MainApp.Valid) {
                diagMsg.AddParam("MainPart", "BP%05u", params.MainApp.PartNumber);
                diagMsg.AddParamString("MainVersion", CSeatracUtils::ToStringVersion(params.MainApp.VersionMajor, params.MainApp.VersionMinor, params.MainApp.VersionBuild));
                diagMsg.AddParam("MainChecksum", "0x%08X", params.MainApp.Checksum);
            }
            diagMsg.AddParamBool("MainSupported", params.MainSupported);
			if(params.AppType == ST_APPTYPE_MAIN) {	
				diagMsg.AddParamsPressureCal(params.PressureCal);					
			}

            //Set the message as a warning if the firmware isn't supported, or the firmware isn't valid
            if(!params.BootApp.Valid || !params.MainApp.Valid || !params.MainSupported)
                diagMsg.RaiseStatus(ST_DIAG_STATUS_WARNING);
            //If the beacon type is unknown then raise an error
            if(params.BeaconType == ST_BEACONTYPE_UNKNOWN)
                diagMsg.RaiseStatus(ST_DIAG_STATUS_ERROR);

            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnSysReboot event
        this->OnSysInfo.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdSysReboot(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracCmdStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus, ST_CST_FAIL);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SYS_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnSysReboot event
        this->OnSysReboot.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrAnalyse(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrAnalyseParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus, ST_CST_FAIL);
    success &= exe->Msg->ReadNoise(&params.Noise);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsNoise(params.Noise);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrAnalyse event
        this->OnXcvrAnalyseNoise.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrFix(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrFixParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.AcoFix);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.AcoFix);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsAcoFix(params.AcoFix);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrFix event
        this->OnXcvrFix.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrRxErr(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrRxErrorParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.AcoFix);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
    success &= exe->Msg->ReadAcoFix(&params.AcoFix);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            diagMsg.AddParamsAcoFix(params.AcoFix);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrRxError event with the error
        this->OnXcvrRxError.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrRxMsg(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrMsgFixParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.AcoFix);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.AcoFix);
    success &= exe->Msg->ReadAcoMsg(&params.AcoMsg);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsAcoMsg(params.AcoMsg);
            diagMsg.AddParamsAcoFix(params.AcoFix);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrRxMsg event
        this->OnXcvrRxMsg.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrRxReq(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrMsgFixParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.AcoFix);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.AcoFix);
    success &= exe->Msg->ReadAcoMsg(&params.AcoMsg);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsAcoMsg(params.AcoMsg);
            diagMsg.AddParamsAcoFix(params.AcoFix);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrRxReq event
        this->OnXcvrRxReq.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrRxResp(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrMsgFixParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);
    CSeatracUtils::InitAcoFix(params.AcoFix);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoFix(&params.AcoFix);
    success &= exe->Msg->ReadAcoMsg(&params.AcoMsg);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsAcoMsg(params.AcoMsg);
            diagMsg.AddParamsAcoFix(params.AcoFix);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrRxReq event
        this->OnXcvrRxResp.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrRxUnhandled(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrMsgParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoMsg(&params.AcoMsg);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, true);
            diagMsg.AddParamsAcoMsg(params.AcoMsg);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrRxUnhandled event
        this->OnXcvrRxUnhandled.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrStatus(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrStatusParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id, (params.CmdStatus != ST_CST_OK));
            diagMsg.AddParamsCmdStatus(params.CmdStatus);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrRxError event with the error
        this->OnXcvrStatus.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrTxMsg(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrMsgParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadAcoMsg(&params.AcoMsg);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsAcoMsg(params.AcoMsg);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrTxMsg event
        this->OnXcvrTxMsg.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrTxMsgCtrlSet(PSeatracExecuteParams exe)
{
	bool success = true;	
    TSeatracXcvrTxMsgCtrlSetParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    //success &= exe->Msg->ReadAcoMsg(&params.AcoMsg);
	success &= exe->Msg->ReadCmdStatus(&params.CmdStatus);
	success &= exe->Msg->ReadUint8((puint8)&params.TxMsgCtrl);	

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
			diagMsg.AddParamsCmdStatus(params.CmdStatus);
			diagMsg.AddParamsXcvrTxMsgCtrl(params.TxMsgCtrl);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrTxMsgCtrlSet event
        this->OnXcvrTxMsgCtrlSet.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::ExecuteCmdXcvrUsbl(PSeatracExecuteParams exe)
{
    bool success = true;
    TSeatracXcvrUsblParams params;

    //Initialise structures
    CSeatracUtils::InitStruct(params);

    //Mark message as handled
    exe->Handled = true;

    //Decode command paramters
    success &= exe->Msg->ReadUsbl(&params.Usbl);

    if(!success) {
        //Indicate a decode error has occured, this will raise an ExecuteDoneEvent with an error code
        exe->Status = ST_EXE_PARAM_MISSING;
    }
    else {
        #ifdef SEATRAC_EN_DIAG
        //Write a debug log entry if enabled
        if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
            CSeatracDiagMsg diagMsg;
            diagMsg.SetType(ST_DIAG_TYPE_PROCESSING, exe->Id);
            diagMsg.AddParamsUsbl(params.Usbl);
            this->DoDiagLog(diagMsg);
        }
        #endif

        //Raise the OnXcvrUsbl event
        this->OnXcvrUsbl.Call(&params);
    }
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::NavQuerySend(ESeatracBeaconId destId, uint8 queryFlags, puint8 data, uint8 dataLen)
{
    //Abort if the destId is invalid
    if(!CSeatracUtils::IsBeaconIdPhysical(destId))
        return false;

    //If no data specified, then force length to zero
    if(!data)
        dataLen = 0;

    //Abort is the data is invalid
    if(dataLen > ST_NAV_PAYLOAD_LEN)
        return false;

    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_NAV_QUERY_SEND);
        diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(destId));
        diagMsg.AddParam("QueryFlags", "0x%02X", queryFlags);
        diagMsg.AddParamsBuffer("Packet", data, dataLen);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Echo Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_NAV_QUERY_SEND);
    msg.AddBeaconId(destId);
    msg.WriteUint8(queryFlags);
    msg.WriteUint8(dataLen);
    if(data)
        msg.WriteArray(data, dataLen);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::NavQueueSet(ESeatracBeaconId destId, puint8 data, uint8 dataLen)
{
    //Abort if the destId is invalid
    if(!CSeatracUtils::IsBeaconIdPhysicalOrAll(destId))
        return false;

    //If no data specified, then force length to zero
    if(!data)
        dataLen = 0;

    //Abort is the data is invalid
    if(dataLen > ST_NAV_PAYLOAD_LEN)
        return false;

    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_NAV_QUEUE_SET);
        diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(destId));
        diagMsg.AddParamsBuffer("Packet", data, dataLen);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Echo Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_NAV_QUEUE_SET);
    msg.AddBeaconId(destId);
    msg.WriteUint8(dataLen);
    if(data)
        msg.WriteArray(data, dataLen);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::NavQueueClear(ESeatracBeaconId destId)
{
    //Abort if the destId is invalid
    if(!CSeatracUtils::IsBeaconIdPhysicalOrAll(destId))
        return false;

    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_NAV_QUEUE_CLR);
        diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(destId));
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Reboot command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_NAV_QUEUE_CLR);
    msg.AddBeaconId(destId);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::NavQueueStatus()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_NAV_QUEUE_STATUS);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Reboot command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_NAV_QUEUE_STATUS);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::NavStatusSend(ESeatracBeaconId beaconId, puint8 data, uint8 dataLen)
{
    //Abort if the destId is invalid
    if(!CSeatracUtils::IsBeaconIdPhysicalOrAll(beaconId))
        return false;

    //If no data specified, then force length to zero
    if(!data)
        dataLen = 0;

    //Abort is the data is invalid
    if(dataLen > ST_NAV_PAYLOAD_LEN)
        return false;

    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_NAV_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_NAV_STATUS_SEND);
        diagMsg.AddParamString("BeaconId", CSeatracUtils::ToStringBeaconId(beaconId));
        diagMsg.AddParamsBuffer("Packet", data, dataLen);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Echo Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_NAV_STATUS_SEND);
    msg.AddBeaconId(beaconId);
    msg.WriteUint8(dataLen);
    if(data)
        msg.WriteArray(data, dataLen);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::PingSend(ESeatracBeaconId destId, ESeatracAcoMsgType msgType)
{
    //Abort if the destId is invalid
    if(!CSeatracUtils::IsBeaconIdPhysical(destId))
        return false;

    //Abort is the message type isn't a request
    if(!CSeatracUtils::IsAcoMsgTypeRequest(msgType))
        return false;

    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_PING_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_PING_SEND);
        diagMsg.AddParamString("DestId", CSeatracUtils::ToStringBeaconId(destId));
        diagMsg.AddParamString("MsgType", CSeatracUtils::ToStringAcoMsgType(msgType));
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Ping Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_PING_SEND);
    msg.AddBeaconId(destId);
    msg.AddAcoMsgType(msgType);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
Function that resets the SeaTrac interface back to its default settings
*/
void CSeatrac::CmdReset()
{
    _cmdProc->SetDecodeCsumEnable(true);
    _cmdProc->SetDecodeIgnoreBadChars(false);
    _cmdProc->DecodeReset();
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::SetDecodeCsumEnable(bool value)
{
    _cmdProc->SetDecodeCsumEnable(value);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatrac::SetDecodeIgnoreBadChars(bool value)
{
    _cmdProc->SetDecodeIgnoreBadChars(value);
}

/*!-----------------------------------------------------------------------------
Function that sets the name of the class (mainly for diagnostic purposes)
*/
#ifdef SEATRAC_EN_DIAG
void CSeatrac::DiagNameSet(string& value)
{
    _diagName.assign(value);
}
#endif

/*!-----------------------------------------------------------------------------
*/
#ifdef SEATRAC_EN_DIAG
void CSeatrac::DiagNameSet(pchar buf)
{
    _diagName.assign(buf);
}
#endif

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::SettingsGet()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SETTINGS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_SETTINGS_GET);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Ping Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_SETTINGS_GET);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::SettingsSet(TSeatracSettings& settings)
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_SETTINGS_SET);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Xcvr Analyse command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_SETTINGS_SET);
    msg.AddSettings(settings);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::SettingsLoad()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SETTINGS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_SETTINGS_LOAD);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Ping Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_SETTINGS_LOAD);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::SettingsSave()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SETTINGS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_SETTINGS_SAVE);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Ping Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_SETTINGS_SAVE);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::SettingsReset()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SETTINGS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_SETTINGS_RESET);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Ping Send command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_SETTINGS_RESET);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::StatusGet()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_STATUS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_STATUS);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Status command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_STATUS);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::StatusGet(uint8 flags)
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_STATUS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_STATUS);
        diagMsg.AddParam("Flags", "0x%02X", flags);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Status command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_STATUS);
    msg.WriteUint8(flags);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::StatusCfgGet()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_STATUS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_STATUS_CFG_GET);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Status command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_STATUS_CFG_GET);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::StatusCfgSet(ESeatracStatusMode mode, uint8 flags)
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_STATUS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_STATUS_CFG_SET);
        diagMsg.AddParamString("Mode", CSeatracUtils::ToStringStatusMode(mode));
        diagMsg.AddParam("Flags", "0x%02X", flags);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Status command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_STATUS_CFG_SET);
    msg.WriteUint8(flags);
    msg.WriteUint8((uint8)mode);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
Function that asks the beacon if its alive, it will respond with its uptime
*/
bool CSeatrac::SysAlive()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SYS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_SYS_ALIVE);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_SYS_ALIVE);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::SysInfo()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SYS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_SYS_INFO);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Reboot command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_SYS_INFO);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatrac::SysReboot()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_SYS_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_SYS_REBOOT);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Reboot command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_SYS_REBOOT);
    msg.WriteUint16(0x6A95);              //Magic reboot security code

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
Function that requests the acoustic transceiver performs a background noise analysis.
*/
bool CSeatrac::XcvrAnalyseNoise()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_XCVR_ANALYSE);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Xcvr Analyse command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_XCVR_ANALYSE);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
Function that requests the acoustic transceiver returns its current status
*/
bool CSeatrac::XcvrStatus()
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_XCVR_STATUS);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Xcvr Analyse command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_XCVR_STATUS);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

/*!-----------------------------------------------------------------------------
Function that changes the transmit message control of the transceiver
*/
bool CSeatrac::XcvrTxMsgCtrlSet(ESeaTracXcvrTxMsgCtrl value)
{
    SEATRAC_MUTEX_CMD_LOCK;

    #ifdef SEATRAC_EN_DIAG
    //Write a debug log entry if enabled
    if(IS_BIT_SET(_diagLogFlags, SEATRAC_DIAGLOG_XCVR_BIT)) {
        CSeatracDiagMsg diagMsg;
        diagMsg.SetType(ST_DIAG_TYPE_ACTION, ST_CID_XCVR_TX_MSGCTRL_SET);
		diagMsg.AddParamsXcvrTxMsgCtrl(value);
        this->DoDiagLog(diagMsg);
    }
    #endif

    //Build the Xcvr TxMsgCtrl command
    CSeatracCmdMsg msg;
    msg.AddCmdId(ST_CID_XCVR_TX_MSGCTRL_SET);
	msg.WriteUint8((uint8)value);

    //Encode and send the message
    //(return if the encode and transmit was successful)
    bool success = _cmdProc->Encode(&msg);

    SEATRAC_MUTEX_CMD_UNLOCK;
    return success;
}

//==============================================================================

