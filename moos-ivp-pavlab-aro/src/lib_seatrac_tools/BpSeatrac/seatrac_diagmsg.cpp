#include "seatrac_diagmsg.hpp"

//==============================================================================
// Class Implimentation...
//==============================================================================
// CSeatracDiagMsg
//==============================================================================
//Initialise static variables
uint32 CSeatracDiagMsg::_diagIdCnt = 0;

/*!-----------------------------------------------------------------------------
*/
CSeatracDiagMsg::CSeatracDiagMsg()
{
    //Assign a unique Id number
    _diagId = CSeatracDiagMsg::_diagIdCnt;

    //Incriment the diagnostic message id counter
    CSeatracDiagMsg::_diagIdCnt++;

    _status = ST_DIAG_STATUS_INFO;
    _type = ST_DIAG_TYPE_NONE;
    _cmdId = ST_CID_INVALID;
    _cmdIdValid = false;
}

/*!-----------------------------------------------------------------------------
*/
CSeatracDiagMsg::~CSeatracDiagMsg()
{
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddNote(const char* str, ...)
{
    //Initialise the arguments list
    va_list args;
    va_start(args, str);

    this->AddNoteArgs(str, args);

    //Tidy up
    va_end(args);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddNoteArgs(const char* str, va_list args)
{
    //Allocate a buffer for output characters
    char buf[4096];

    //Convert the variable-args and string into a character buffer
    vsnprintf(buf, sizeof(buf), str, args);

    //If the params string isn't empty, then append a comma seperator
    if(_paramStr.length() > 0)
        _paramStr += SEATRAC_DIAGMSG_SEPERATOR_CHAR;

    //Build the param string
    //_paramStr.append("\"");
    _paramStr.append(buf);
    //_paramStr.append("\"");

}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParam(const char* name, const char* value, ...)
{
    //Initialise the arguments list
    va_list args;
    va_start(args, value);

    this->AddParamArgs(name, value, args);

    //Tidy up
    va_end(args);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamArgs(const char* name, const char* value, va_list args)
{
    //Allocate a buffer for output characters
    char nameBuf[128];
    char valueBuf[1024];

    //Convert the variable-args and string into a character buffer
    vsnprintf(nameBuf, sizeof(nameBuf), name, args);

    //Convert the variable-args and string into a character buffer
    vsnprintf(valueBuf, sizeof(valueBuf), value, args);

    //Put the string into the param string
    this->AddParamString(nameBuf, valueBuf);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamBool(const char* name, bool value)
{
    if(value)
        this->AddParamString(name, "TRUE");
    else
        this->AddParamString(name, "FALSE");
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamString(const char* name, const string& value)
{
    this->AddParamString(name, value.c_str());
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamString(const char* name, const char* value)
{
    //If the params string isn't empth, then append a comma seperator
    if(_paramStr.length() > 0) {
        _paramStr += SEATRAC_DIAGMSG_SEPERATOR_CHAR;
    }

    //Build the param string
    _paramStr.append(name);
    _paramStr.append("=");
    _paramStr.append(value);
}

/*!-----------------------------------------------------------------------------
Function that adds the contents of an array as decimal values.
*/
void CSeatracDiagMsg::AddParamsArray(const char* name, puint8 data, uint8 len)
{
    //If no data specified, then force the length to 0
    if(!data)
        len = 0;

    uint8 buf[16];
    string dataStr = "{";
    for(uint8 idx = 0; idx < len; idx++) {
        //Add seperating commas
        if(idx > 0)
            dataStr.append(",");
        //Add the hex value
        snprintf((pchar)buf, sizeof(buf), "%u", data[idx]);
        dataStr.append((pchar)buf);
    }
    dataStr.append("}");

    this->AddParamString(name, dataStr);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsBuffer(const char* name, TSeatracAcoPayload& value)
{
    this->AddParamsBuffer(name, value.Data, value.Length);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsBuffer(const char* name, TSeatracNavPacket& value)
{
    this->AddParamsBuffer(name, value.Data, value.Length);
}

/*!-----------------------------------------------------------------------------
Function that adds the contents of a buffer (specified by a length and data array)
using hexadecimal values.
*/
void CSeatracDiagMsg::AddParamsBuffer(const char* name, puint8 data, uint8 len)
{
    char nameLen[64];
    char nameData[64];

    //If no data specified, then force the length to 0
    if(!data)
        len = 0;

    string dataStr = "{";
    if(len > 0) {
        uint8 buf[16];
        for(uint8 idx = 0; idx < len; idx++) {
            uint8 val = data[idx];
            //Add seperating commas
            if(idx > 0)
                dataStr.append(",");
            //Add the hex value
            snprintf((pchar)buf, sizeof(buf), "0x%02X", val);
            dataStr.append((pchar)buf);
            //Add ASCII character
            if(val >= ' ' && val < 127) {
                snprintf((pchar)buf, sizeof(buf), " '%c'", val);
                dataStr.append((pchar)buf);
            }
        }
    }
    dataStr.append("}");

    snprintf(nameLen, sizeof(nameLen), "%sLen", name);
    snprintf(nameData, sizeof(nameData), "%sData", name);

    this->AddParam(nameLen, "%u", len);
    this->AddParamString(nameData, dataStr);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsAcoFix(TSeatracAcoFix& value)
{
    this->AddParamsAcoFixMsg(value.Msg);
    this->AddParamsAcoFixLocal(value.Local);
    this->AddParamsAcoFixInfo(value.Info);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsAcoFixInfo(TSeatracAcoFixInfo& value)
{
    if(value.RangeValid) {
        this->AddParam("RangeDist", "%0.1fm", value.RangeDist);
    }
    if(value.UsblValid) {
        this->AddParam("UsblRssi", "{%0.1fdB,%0.1fdB,%0.1fdB,%0.1fdB}", value.UsblRssi[0], value.UsblRssi[1], value.UsblRssi[2], value.UsblRssi[3]);
        this->AddParam("UsblAzimuth", "%0.1f°", value.UsblAzimuth);
        this->AddParam("UsblElevation", "%0.1f°", value.UsblElevation);
        this->AddParam("UsblFitError", "%0.1f", value.UsblFitError);
    }
    if(value.PositionValid) {
        this->AddParam("PosEasting", "%0.1fm", value.PositionEasting);
        this->AddParam("PosNorthing", "%0.1fm", value.PositionNorthing);
        this->AddParam("PosDepth", "%0.1fm", value.PositionDepth);
        this->AddParamBool("PosEnhanced", value.PositionEnhanced);
        this->AddParamBool("PosFltError", value.PositionFilterError);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsAcoFixLocal(TSeatracAcoFixLocal& value)
{
    this->AddParam("LocalVos", "%0.0fms-1", value.Vos);
    if(value.DepthValid) {
        this->AddParam("LocalDepth", "%0.1fm", value.Depth);
    }
    if(value.AttitudeValid) {
        this->AddParam("LocalYaw", "%0.1f°", value.AttitudeYaw);
        this->AddParam("LocalPitch", "%0.1f°", value.AttitudePitch);
        this->AddParam("LocalRoll", "%0.1f°", value.AttitudeRoll);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsAcoFixMsg(TSeatracAcoFixMsg& value)
{
    this->AddParamString("MsgSrcId", CSeatracUtils::ToStringBeaconId(value.SrcId));
    this->AddParamString("MsgDestId", CSeatracUtils::ToStringBeaconId(value.DestId));
    this->AddParamString("MsgType", CSeatracUtils::ToStringAcoMsgType(value.Type));
    this->AddParam("MsgRssi", "%0.1fdB", value.Rssi);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsAcoMsg(TSeatracAcoMsg& value)
{
    this->AddParamString("HdrSrcId", CSeatracUtils::ToStringBeaconId(value.SrcId));
    this->AddParamString("HdrDestId", CSeatracUtils::ToStringBeaconId(value.DestId));
    this->AddParamString("HdrMsgType", CSeatracUtils::ToStringAcoMsgType(value.MsgType));
    this->AddParam("HdrDepth", "%0.1fm", value.MsgDepth);
    this->AddParamString("PayloadType", CSeatracUtils::ToStringAcoPayloadType(value.PayloadType));
    this->AddParamsBuffer("Payload", value.Payload);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsCmdStatus(ESeatracCmdStatus value)
{
    this->AddParamString("CmdStatus", CSeatracUtils::ToStringCmdStatus(value));
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsNavQuery(TSeatracNavQuery& value)
{
    if(value.DepthValid) {
        this->AddParam("DepthRemote", "%0.1fm", value.DepthRemote);
    }
    if(value.SupplyValid) {
        this->AddParam("SupplyRemote", "%0.1fV", value.SupplyRemote);
    }
    if(value.TempValid) {
        this->AddParam("TempRemote", "%0.1f°C", value.TempRemote);
    }
    if(value.AttitudeValid) {
        this->AddParam("AttitudeRemote", "{%0.1f°Y,%0.1f°P,%0.1f°R}", value.AttitudeYawRemote, value.AttitudePitchRemote, value.AttitudeRollRemote);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsNoise(TSeatracNoise& value)
{
    this->AddParam("AdcMean", "%d", value.AdcMean);
    this->AddParam("AdcPkPk", "%d", value.AdcPkPk);
    this->AddParam("AdcRms", "%d", value.AdcRms);
    this->AddParam("RxLevelPkPk", "%0.1fdB", value.RxLevelPkPk);
    this->AddParam("RxLevelRms", "%0.1fdB", value.RxLevelRms);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsPressureCal(TSeatracPressureCal& value)
{	
	this->AddParam("PressureId", "%d", value.Id);
    this->AddParam("PressureType", "%d", value.Type);
	this->AddParam("PressureMin", "%0.1f Bar", value.PressureMin);
	this->AddParam("PressureMax", "%0.1f Bar", value.PressureMin);	
	this->AddParam("PressureCalDate", "%u-%02u-%02u", value.CalYear, value.CalMonth, value.CalDay);	
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsSettings(TSeatracSettings& value)
{
    this->AddParamString("StatusMode", CSeatracUtils::ToStringStatusMode(value.StatusMode));
    this->AddParamBool("StatusOutputEnv", value.StatusOutputEnv);
    this->AddParamBool("StatusOutputAttitude", value.StatusOutputAttitude);
    this->AddParamBool("StatusOutputMagCal", value.StatusOutputMagCal);
    this->AddParamBool("StatusOutputAccCal", value.StatusOutputAccCal);
    this->AddParamBool("StatusOutputAhrsRaw", value.StatusOutputAhrsRaw);
    this->AddParamBool("StatusOutputAhrsComp", value.StatusOutputAhrsComp);
    this->AddParam("EnvPressureOffset", "{%s,%0.3fBar}", (value.EnvAutoPressureOffset ? "AUTO" : "MANUAL"), value.EnvPressureOffset);
    this->AddParam("EnvVos", "{%s,%0.1fms-1}", (value.EnvAutoCalcVos ? "AUTO" : "MANUAL"), value.EnvVos);
    this->AddParam("EnvSalinity", "%0.1fppt", value.EnvSalinity);
    this->AddParamBool("AhrsAutoCalMag", value.AhrsAutoCalMag);
    this->AddParam("AhrsOffset", "{%0.1f°Y,%0.1f°P,%0.1f°R}", value.AhrsYawOffset, value.AhrsPitchOffset, value.AhrsRollOffset);
    this->AddParamString("BeaconId", CSeatracUtils::ToStringBeaconId(value.XcvrBeaconId));
    this->AddParam("XcvrRangeTimeout", "%um", value.XcvrRangeTimeout);
    this->AddParam("XcvrResponseTime", "%ums", value.XcvrResponseTime);
    this->AddParamBool("XcvrOutputDiagMsgs", value.XcvrOutputDiagMsgs);
    this->AddParamBool("XcvrOutputFixMsgs", value.XcvrOutputFixMsgs);
    this->AddParamBool("XcvrOutputUsblMsgs", value.XcvrOutputUsblMsgs);		
	this->AddParamString("XcvrTxMsgCtrl", CSeatracUtils::ToStringXcvrTxMsgCtrl(value.XcvrTxMsgCtrl));
    this->AddParamBool("XcvrEnablePosFilter", value.XcvrEnablePosFilter);
    this->AddParamBool("XcvrUsblUseAhrs", value.XcvrUsblUseAhrs);
    this->AddParam("XcvrFixedAttitude", "{%0.1f°Y,%0.1f°P,%0.1f°R}", value.XcvrFixedYaw, value.XcvrFixedPitch, value.XcvrFixedRoll);
    this->AddParam("XcvrPosFilterVelocity", "{%ums-1,%u°,%us}", value.XcvrPosFilterVelocity, value.XcvrPosFilterAngularRate, value.XcvrPosFilterTimeout);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsStatus(TSeatracStatus& value)
{
    this->AddParam("Timestamp", "%0.3fs", value.Timestamp);
    if(value.EnvValid) {
        this->AddParam("EnvSupply", "%0.1fV", value.EnvSupply);
        this->AddParam("EnvTemp", "%0.1f°C", value.EnvTemp);
        this->AddParam("EnvPressure", "%0.1fBar", value.EnvPressure);
        this->AddParam("EnvDepth", "%0.1fm", value.EnvDepth);
        this->AddParam("EnvVos", "%0.1fms-1", value.EnvVos);
    }
    if(value.AttitudeValid) {
        this->AddParam("Attitude", "{%0.1f°Y,%0.1f°P,%0.1f°R}", value.AttitudeYaw, value.AttitudePitch, value.AttitudeRoll);
        //this->AddParam("AttitudeYaw", "%0.1f°", value.AttitudeYaw);
        //this->AddParam("AttitudePitch", "%0.1f°", value.AttitudePitch);
        //this->AddParam("AttitudeRoll", "%0.1f°", value.AttitudeRoll);
    }
    if(value.MagCalValid) {
        this->AddParam("MagCalBuf", "%d%%", value.MagCalBuf);
        this->AddParamBool("MagCalValid", value.MagCalValid);
        this->AddParam("MagCalAge", "%us", value.MagCalAge);
        this->AddParam("MagCalFit", "%d%%", value.MagCalFit);
    }
    if(value.AccValid) {
        this->AddParam("AccLimX", "{%d,%d}", value.AccLimMinX, value.AccLimMaxX);
        this->AddParam("AccLimY", "{%d,%d}", value.AccLimMinY, value.AccLimMaxY);
        this->AddParam("AccLimZ", "{%d,%d}", value.AccLimMinZ, value.AccLimMaxZ);
    }
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsUsbl(TSeatracUsbl& value)
{
    this->AddParam("XcorPeak", "%0.1f", value.XcorSignalPeak);
    this->AddParam("XcorThreshold", "%0.1f", value.XcorThreshold);
    this->AddParam("XcorCrossPoint", "%u", value.XcorCrossingPoint);
    this->AddParam("XcorCrossMag", "%0.1f", value.XcorCrossingMagnitude);
    this->AddParam("XcorDetect", "%u", value.XcorDetect);
    this->AddParam("XcorLen", "%u", value.XcorLength);
    this->AddParam("Channels", "%u", value.Channels);
    this->AddParam("ChannelRssi", "{%0.1fdB,%0.1fdB,%0.1fdB,%0.1fdB}", value.ChannelRssi[0], value.ChannelRssi[1], value.ChannelRssi[2], value.ChannelRssi[3]);
    this->AddParam("Baselines", "%u", value.Baselines);
    this->AddParam("PhaseAngles", "{%0.1f°,%0.1f°,%0.1f°,%0.1f°,%0.1f°,%0.1f°}", value.BaselinePhaseAngle[0], value.BaselinePhaseAngle[1], value.BaselinePhaseAngle[2], value.BaselinePhaseAngle[3], value.BaselinePhaseAngle[4], value.BaselinePhaseAngle[5]);
    this->AddParam("SigAzimuth", "%0.1f°", value.SignalAzimuth);
    this->AddParam("SigElevation", "%0.1f°", value.SignalElevation);
    this->AddParam("SigFitError", "%0.1f", value.SignalFitError);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::AddParamsXcvrTxMsgCtrl(ESeaTracXcvrTxMsgCtrl value)
{
    this->AddParamString("XcvrTxMsgCtrl", CSeatracUtils::ToStringXcvrTxMsgCtrl(value));
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::MakeParams(TSeatracDiagLogParams& params)
{
    params.Id = _diagId;
    params.Timestamp = 0.0;     //To be filled in by the caller.
    params.Status = _status;    //INFO, WARNING, ERROR etc.
    snprintf(params.StatusStr, sizeof(params.StatusStr), "%s", CSeatracUtils::ToStringDiagMsgStatus(_status).c_str());
    params.NameStr[0] = 0;      //To be filled in by the caller - null string.
    params.Type = _type;
    snprintf(params.TypeStr, sizeof(params.TypeStr), "%s", CSeatracUtils::ToStringDiagMsgType(_type).c_str());
    params.CmdIdValid = _cmdIdValid;
    params.CmdId = _cmdId;
    snprintf(params.CmdIdStr, sizeof(params.CmdIdStr), "%s", CSeatracUtils::ToStringCmdId(_cmdId).c_str());
    snprintf(params.ParamsStr, sizeof(params.ParamsStr), "%s", _paramStr.c_str());
}

/*!-----------------------------------------------------------------------------
Function that is used to raise the status type from INFO to WARNING then ERROR.
The status can never be decreased.
*/
void CSeatracDiagMsg::RaiseStatus(ESeatracDiagMsgStatus status)
{
    uint8 newStatus = (uint8)status;
    uint8 oldStatus = (uint8)_status;
    if(newStatus > oldStatus)
        _status = status;
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::SetType(ESeatracDiagMsgType type, bool error)
{
    _type = type;
    _cmdId = ST_CID_INVALID;
    _cmdIdValid = false;
    if(error)
        this->RaiseStatus(ST_DIAG_STATUS_ERROR);
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracDiagMsg::SetType(ESeatracDiagMsgType type, ESeatracCmdId cmdId, bool error)
{
    _type = type;
    _cmdId = cmdId;
    _cmdIdValid = true;
    if(error)
        this->RaiseStatus(ST_DIAG_STATUS_ERROR);
}

//==============================================================================
