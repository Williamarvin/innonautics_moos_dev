#include "seatrac_utils.hpp"

//==============================================================================
// Class implimentation...
//==============================================================================
// CSeatracUtils
//==============================================================================
/*!-----------------------------------------------------------------------------
*/
void CSeatracUtils::InitAcoFixMsg(TSeatracAcoFixMsg& value)
{
    CSeatracUtils::InitStruct(value);
    value.DestId = ST_BEACON_UNKNOWN;
    value.SrcId = ST_BEACON_UNKNOWN;
    value.Type = ST_AMSG_UNKNOWN;
    //value.Rssi = 0.0F;
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracUtils::InitAcoFixLocal(TSeatracAcoFixLocal& value)
{
    CSeatracUtils::InitStruct(value);
    /*
    value.AttitudeValid = false;
    value.AttitudeYaw = 0.0F;
    value.AttitudePitch = 0.0F;
    value.AttitudeRoll = 0.0F;
    value.DepthValid = false;
    value.Depth = 0.0F;
    value.Vos = 0.0F;
    */
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracUtils::InitAcoFixInfo(TSeatracAcoFixInfo& value)
{
    CSeatracUtils::InitStruct(value);
    /*
    value.RangeValid = false;
    value.RangeCount = 0;
    value.RangeTime = 0;
    value.RangeDist = 0;
    value.UsblValid = false;
    value.UsblChannels = 0;
    value.UsblAzimuth = 0.0F;
    value.UsblElevation = 0.0F;
    value.UsblFitError = 0.0F;
    value.PositionValid = false;
    value.PositionEnhanced = false;
    value.PositionFilterError = false;
    value.PositionEasting = 0.0F;
    value.PositionNorthing = 0.0F;
    value.PositionDepth = 0.0F;
    */
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracUtils::InitAcoFix(TSeatracAcoFix& value)
{
    CSeatracUtils::InitStruct(value);
    /*
    value.Flags = 0;
    CSeatracUtils::InitAcoFixMsg(value.Msg);
    CSeatracUtils::InitAcoFixLocal(value.Local);
    CSeatracUtils::InitAcoFixInfo(value.Info);
    */
}

/*!-----------------------------------------------------------------------------
*/
void CSeatracUtils::InitNavQuery(TSeatracNavQuery &value)
{
    CSeatracUtils::InitStruct(value);
    /*
    value.Flags = 0;
    value.DepthValid = false;
    value.DepthRemote = 0.0F;
    value.SupplyValid = false;
    value.SupplyRemote = 0.0F;
    value.TempValid = false;
    value.TempRemote = 0.0F;
    value.AttitudeValid = false;
    value.AttitudeYawRemote = 0.0F;
    value.AttitudePitchRemote = 0.0F;
    value.AttitudeRollRemote = 0.0F;
    */
}

/*!-----------------------------------------------------------------------------
Determines if the specified beacon Id is a valid address (in the range 1-15)
*/
bool CSeatracUtils::IsBeaconIdPhysical(ESeatracBeaconId beaconId)
{
    uint8 id = (uint8)beaconId;
    if(id < 1 || id >= ST_BEACONS)
        return false;
    else
        return true;
}

/*!-----------------------------------------------------------------------------
Determines if the specified beacon Id is a valid address (in the range 0-15),
including the BEACON_ALL value
*/
bool CSeatracUtils::IsBeaconIdPhysicalOrAll(ESeatracBeaconId beaconId)
{
    uint8 id = (uint8)beaconId;
    if(id >= ST_BEACONS)
        return false;
    else
        return true;
}

/*!-----------------------------------------------------------------------------
Determines if the specified Acoustic Message Type is for One-Way comms
*/
bool CSeatracUtils::IsAcoMsgTypeOneWay(ESeatracAcoMsgType msgType)
{
    if(msgType == ST_AMSG_OWAY || msgType == ST_AMSG_OWAYU)
        return true;
    else
        return false;
}

/*!-----------------------------------------------------------------------------
Determines if the specified Acoustic Message Type is a Request
*/
bool CSeatracUtils::IsAcoMsgTypeRequest(ESeatracAcoMsgType msgType)
{
    if(msgType == ST_AMSG_REQ || msgType == ST_AMSG_REQU || msgType == ST_AMSG_REQX)
        return true;
    else
        return false;
}

/*!-----------------------------------------------------------------------------
Function that returns the SeaTrac beacon part number from its type value.
*/
uint32 CSeatracUtils::ToBeaconPartNumber(ESeatracBeaconType value)
{
    switch(value) {
        case ST_BEACONTYPE_X010 : { return ST_HARDWARE_PARTNUMBER_X010; }
        case ST_BEACONTYPE_X110 : { return ST_HARDWARE_PARTNUMBER_X110; }
        case ST_BEACONTYPE_X150 : { return ST_HARDWARE_PARTNUMBER_X150; }
        default : { return 0; }
    }
}

/*!-----------------------------------------------------------------------------
Function that returns the Seatrac Beacon type from its part number.
*/
ESeatracBeaconType CSeatracUtils::ToBeaconType(uint32 value)
{
    switch(value) {
        case ST_HARDWARE_PARTNUMBER_X010 : { return ST_BEACONTYPE_X010; }
        case ST_HARDWARE_PARTNUMBER_X110 : { return ST_BEACONTYPE_X110; }
        case ST_HARDWARE_PARTNUMBER_X150 : { return ST_BEACONTYPE_X150; }
        default : { return ST_BEACONTYPE_UNKNOWN; }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringAcoMsgType(ESeatracAcoMsgType value)
{
    switch(value) {
        case ST_AMSG_OWAY : { return string("OWAY"); }
        case ST_AMSG_OWAYU : { return string("OWAYU"); }
        case ST_AMSG_REQ : { return string("REQ"); }
        case ST_AMSG_RESP : { return string("RESP"); }
        case ST_AMSG_REQU : { return string("REQU"); }
        case ST_AMSG_RESPU : { return string("RESPU"); }
        case ST_AMSG_REQX : { return string("REQX"); }
        case ST_AMSG_RESPX : { return string("RESPX"); }
        //case ST_AMSG_UNKNOWN : { return string("UNKNOWN"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringAcoPayloadType(ESeatracAcoPayloadType value)
{
    switch(value) {
        case ST_APLOAD_PING : { return string("PING"); }
        case ST_APLOAD_ECHO : { return string("ECHO"); }
        case ST_APLOAD_NAV : { return string("NAV"); }
        case ST_APLOAD_DAT : { return string("DAT"); }
        case ST_APLOAD_DEX : { return string("DEX"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringAppType(ESeatracAppType value)
{
    switch(value) {
        case ST_APPTYPE_BOOT : { return string("BOOTLOADER"); }
        case ST_APPTYPE_MAIN : { return string("MAIN"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringBeaconId(ESeatracBeaconId value)
{
    return CSeatracUtils::ToStringBeaconId((uint8)value);
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringBeaconId(uint8 value)
{
    uint8 buf[16];
    if(value == 0) {
        snprintf((pchar)buf, sizeof(buf), "BEACON[ALL]");
        return string((pchar)buf);
    }
    else if(value <= 15) {
        snprintf((pchar)buf, sizeof(buf), "BEACON[%d]", value);
        return string((pchar)buf);
    }
    else {
        return CSeatracUtils::ToStringUnknown(value);
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringBeaconType(ESeatracBeaconType value)
{
    switch(value) {
        case ST_BEACONTYPE_X010 : { return string("X010"); }
        case ST_BEACONTYPE_X110 : { return string("X110"); }
        case ST_BEACONTYPE_X150 : { return string("X150"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringCalAction(ESeatracCalAction value)
{
    switch(value) {
        case ST_CAL_ACC_DEFAULTS : { return string("ACC_DEFAULTS"); }
        case ST_CAL_ACC_RESET : { return string("ACC_RESET"); }
        case ST_CAL_ACC_CALC : { return string("ACC_CALC"); }
        case ST_CAL_MAG_DEFAULTS : { return string("MAG_DEFAULTS"); }
        case ST_CAL_MAG_RESET : { return string("MAG_RESET"); }
        case ST_CAL_MAG_CALC : { return string("MAG_CALC"); }
        case ST_CAL_PRES_OFFSET_RESET : { return string("PRES_OFFSET_RESET"); }
        case ST_CAL_PRES_OFFSET_CALC : { return string("PRES_OFFSET_CALC"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringCmdId(ESeatracCmdId value)
{
    switch(value) {
        case ST_CID_INVALID : { return string("INVALID"); }
        //System Messages
        case ST_CID_SYS_ALIVE : { return string("SYS_ALIVE"); }
        case ST_CID_SYS_INFO : { return string("SYS_INFO"); }
        case ST_CID_SYS_REBOOT : { return string("SYS_REBOOT"); }
        case ST_CID_SYS_ENGINEERING : { return string("ENGINEERING"); }
        // Firmware Programming Messages
        case ST_CID_PROG_INIT : { return string("PROG_INIT"); }
        case ST_CID_PROG_BLOCK : { return string("PROG_BLOCK"); }
        case ST_CID_PROG_UPDATE : { return string("PROG_UPDATE"); }
        // Status Messages
        case ST_CID_STATUS : { return string("STATUS"); }
        case ST_CID_STATUS_CFG_GET : { return string("STATUS_CFG_GET"); }
        case ST_CID_STATUS_CFG_SET : { return string("STATUS_CFG_SET"); }
        // Settings Messages
        case ST_CID_SETTINGS_GET : { return string("SETTINGS_GET"); }
        case ST_CID_SETTINGS_SET : { return string("SETTIGS_SET"); }
        case ST_CID_SETTINGS_LOAD : { return string("SETTIGS_LOAD"); }
        case ST_CID_SETTINGS_SAVE : { return string("SETTINGS_SAVE"); }
        case ST_CID_SETTINGS_RESET : { return string("SETTINGS_RESET"); }
        // Calibration Messages
        case ST_CID_CAL_ACTION : { return string("CAL_ACTION"); }
        case ST_CID_CAL_AHRS_GET : { return string("CAL_AHRS_GET"); }
        case ST_CID_CAL_AHRS_SET : { return string("CAL_AHRS_SET"); }
        // Acoustic Transceiver Messages
        case ST_CID_XCVR_ANALYSE : { return string("XCVR_ANALYSE"); }
        case ST_CID_XCVR_TX_MSG : { return string("XCVR_TX_MSG"); }
        case ST_CID_XCVR_RX_ERR : { return string("XCVR_RX_ERR"); }
        case ST_CID_XCVR_RX_MSG : { return string("XCVR_RX_MSG"); }
        case ST_CID_XCVR_RX_REQ : { return string("XCVR_RX_REQ"); }
        case ST_CID_XCVR_RX_RESP : { return string("XCVR_RX_RESP"); }
        case ST_CID_XCVR_RX_UNHANDLED : { return string("XCVR_RX_UNHANDLED"); }
        case ST_CID_XCVR_USBL : { return string("XCVR_USBL"); }
        case ST_CID_XCVR_FIX : { return string("XCVR_FIX"); }
        case ST_CID_XCVR_STATUS : { return string("XCVR_STATUS"); }
        // PING Protocol Messages
        case ST_CID_PING_SEND : { return string("PING_SEND"); }
        case ST_CID_PING_REQ : { return string("PING_REQ"); }
        case ST_CID_PING_RESP : { return string("PING_RESP"); }
        case ST_CID_PING_ERROR : { return string("PING_ERROR"); }
        // ECHO Protocol Messages
        case ST_CID_ECHO_SEND : { return string("ECHO_SEND"); }
        case ST_CID_ECHO_REQ : { return string("ECHO_REQ"); }
        case ST_CID_ECHO_RESP : { return string("ECHO_RESP"); }
        case ST_CID_ECHO_ERROR : { return string("ECHO_ERROR"); }
        // DAT Protocol Messages
        case ST_CID_DAT_SEND : { return string("DAT_SEND"); }
        case ST_CID_DAT_RECEIVE : { return string("DAT_RECEIVE"); }
        case ST_CID_DAT_ERROR : { return string("DAT_ERROR"); }
        case ST_CID_DAT_QUEUE_SET : { return string("DAT_QUEUE_SET"); }
        case ST_CID_DAT_QUEUE_CLR : { return string("DAT_QUEUE_CLR"); }
        case ST_CID_DAT_QUEUE_STATUS : { return string("DAT_QUEUE_STATUS"); }
        // NAV Protocol Messages
        case ST_CID_NAV_QUERY_SEND : { return string("NAV_QUERY_SEND"); }
        case ST_CID_NAV_QUERY_REQ : { return string("NAV_QUERY_REQ"); }
        case ST_CID_NAV_QUERY_RESP : { return string("NAV_QUERY_RESP"); }
        case ST_CID_NAV_ERROR : { return string("NAV_ERROR"); }
        case ST_CID_NAV_QUEUE_SET : { return string("NAV_QUEUE_SET"); }
        case ST_CID_NAV_QUEUE_CLR : { return string("NAV_QUEUE_CLR"); }
        case ST_CID_NAV_QUEUE_STATUS : { return string("NAV_QUEUE_STATUS"); }
        case ST_CID_NAV_STATUS_SEND : { return string("NAV_STATUS_SEND"); }
        case ST_CID_NAV_STATUS_RECEIVE : { return string("NAV_STATUS_RECEIVE"); }
        // DEX Protocol Messages
        //case ST_CID_DEX_CLOSE : { return string("DEX_CLOSE"); }
        //case ST_CID_DEX_DEBUG : { return string("DEX_DEBUG"); }
        //case ST_CID_DEX_ENQUEUE : { return string("DEX_ENQUEUE"); }
        //case ST_CID_DEX_OPEN : { return string("DEX_OPEN"); }
        //case ST_CID_DEX_RESET : { return string("DEX_RESET"); }
        //case ST_CID_DEX_SEND : { return string("DEX_SEND"); }
        //case ST_CID_DEX_SOCKETS : { return string("DEX_SOCKETS"); }
        //case ST_CID_DEX_RECEIVE : { return string("DEX_RECEIVE"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringCmdStatus(ESeatracCmdStatus value)
{
    switch(value) {
        case ST_CST_OK : { return string("OK"); }
        case ST_CST_FAIL : { return string("FAIL"); }
        case ST_CST_EEPROM_ERROR : { return string("EEPROM_ERROR"); }
        //Command Processor Status Codes
        case ST_CST_CMD_PARAM_MISSING : { return string("PARAM_MISSING"); }
        case ST_CST_CMD_PARAM_INVALID : { return string("PARAM_INVALID"); }
        //Firmware Programming Status Codes
        case ST_CST_PROG_FLASH_ERROR : { return string("PROG_FLASH_ERROR"); }
        case ST_CST_PROG_FIRMWARE_ERROR : { return string("PROG_FIRMWARE_ERROR"); }
        case ST_CST_PROG_SECTION_ERROR : { return string("PROG_SECTION_ERROR"); }
        case ST_CST_PROG_LENGTH_ERROR : { return string("PROG_LENGTH_ERROR"); }
        case ST_CST_PROG_DATA_ERROR : { return string("PROG_DATA_ERROR"); }
        case ST_CST_PROG_CHECKSUM_ERROR : { return string("PROG_CSUM_ERROR"); }
        //Acoustic Transceiver Status Codes
        case ST_CST_XCVR_BUSY : { return string("XCVR_BUSY"); }
        case ST_CST_XCVR_ID_REJECTED : { return string("XCVR_ID_REJECTED"); }
        case ST_CST_XCVR_CSUM_ERROR : { return string("XCVR_CSUM_ERROR"); }
        case ST_CST_XCVR_LENGTH_ERROR : { return string("XCVR_LENGTH_ERROR"); }
        case ST_CST_XCVR_RESP_TIMEOUT : { return string("XCVR_RESP_TIMEOUT"); }
        case ST_CST_XCVR_RESP_ERROR : { return string("XCVR_RESP_ERROR"); }
        case ST_CST_XCVR_RESP_WRONG : { return string("XCVR_RESP_WRONG"); }
        case ST_CST_XCVR_PLOAD_ERROR : { return string("XCVR_PLOAD_ERROR"); }
        case ST_CST_XCVR_STATE_STOPPED : { return string("XCVR_STATE_STOPPED"); }
        case ST_CST_XCVR_STATE_IDLE : { return string("XCVR_STATE_IDLE"); }
        case ST_CST_XCVR_STATE_TX : { return string("XCVR_STATE_TX"); }
        case ST_CST_XCVR_STATE_REQ : { return string("XCVR_STATE_REQ"); }
        case ST_CST_XCVR_STATE_RX : { return string("XCVR_STATE_RX"); }
        case ST_CST_XCVR_STATE_RESP : { return string("XCVR_STATE_RESP"); }
        //DEX Protocol Status Codes
        case ST_CST_DEX_SOCKET_ERROR : { return string("DEX_SOCKET_ERROR"); }
        case ST_CST_DEX_RX_SYNC : { return string("DEX_RX_SYNC"); }
        case ST_CST_DEX_RX_DATA : { return string("DEX_RX_DATA"); }
        case ST_CST_DEX_RX_SEQ_ERROR : { return string("DEX_RX_SEQ_ERROR"); }
        case ST_CST_DEX_RX_MSG_ERROR : { return string("DEX_RX_MSG_ERROR"); }
        case ST_CST_DEX_REQ_ERROR : { return string("DEX_REQ_ERROR"); }
        case ST_CST_DEX_RESP_TMO_ERROR : { return string("DEX_RESP_TMO_ERROR"); }
        case ST_CST_DEX_RESP_MSG_ERROR : { return string("DEX_RESP_MSG_ERROR"); }
        case ST_CST_DEX_RESP_REMOTE_ERROR : { return string("DEX_RESP_REMOTE_ERROR"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringDiagMsgStatus(ESeatracDiagMsgStatus value)
{
    switch(value) {
        case ST_DIAG_STATUS_INFO : { return string("INFO"); }
        case ST_DIAG_STATUS_WARNING : { return string("WARNING"); }
        case ST_DIAG_STATUS_ERROR : { return string("ERROR"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringDiagMsgType(ESeatracDiagMsgType value)
{
    switch(value) {
        case ST_DIAG_TYPE_NONE : { return string(""); }
        case ST_DIAG_TYPE_ACTION : { return string("ACTION"); }
        case ST_DIAG_TYPE_DECODE : { return string("DECODE"); }
        case ST_DIAG_TYPE_ENCODE : { return string("ENCODE"); }
        case ST_DIAG_TYPE_EXECUTE : { return string("EXECUTE"); }
        case ST_DIAG_TYPE_PROCESSING : { return string("PROCESSING"); }
        case ST_DIAG_TYPE_EXECUTED : { return string("EXECUTED"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringExecuteStatus(ESeatracExecuteStatus value)
{
    switch(value) {
        case ST_EXE_OK : { return string("OK"); }
        case ST_EXE_FAIL : { return string("FAIL"); }
        case ST_EXE_UNHANDLED : { return string("UNHANDLED"); }
        case ST_EXE_PARAM_MISSING : { return string("PARAM_MISSING"); }
        case ST_EXE_PARAM_INVALID : { return string("PARAM_INVALID"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
};

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringStatusMode(ESeatracStatusMode value)
{
    switch(value) {
        case ST_STATUS_MODE_MANUAL : { return string("MANUAL"); }
        case ST_STATUS_MODE_1Hz : { return string("1Hz"); }
        case ST_STATUS_MODE_2Hz5 : { return string("2.5Hz"); }
        case ST_STATUS_MODE_5Hz : { return string("5Hz"); }
        case ST_STATUS_MODE_10Hz : { return string("10Hz"); }

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

/*!-----------------------------------------------------------------------------
Function used to return a generic 'UNKNOWN[xx]' status message, where XX
is the unknown value in hex form.
*/
string CSeatracUtils::ToStringUnknown(uint8 value)
{
    uint8 buf[16];
    snprintf((pchar)buf, sizeof(buf), "UNKNOWN[0x%02X]", (uint8)value);
    return string((pchar)buf);
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringVersion(uint8 major, uint8 minor, uint16 build)
{
    uint8 buf[64];
    snprintf((pchar)buf, sizeof(buf), "v%u.%u.%u", major, minor, build);
    return string((pchar)buf);
}

/*!-----------------------------------------------------------------------------
*/
string CSeatracUtils::ToStringXcvrTxMsgCtrl(ESeaTracXcvrTxMsgCtrl value)
{
	switch(value) {
        case ST_XCVR_TXMSG_ALLOW_ALL : { return string("ALLOW_ALL"); }
        case ST_XCVR_TXMSG_BLOCK_RESP : { return string("BLOCK_RESPONSE"); }
        case ST_XCVR_TXMSG_BLOCK_ALL : { return string("BLOCK_ALL"); }
        case ST_XCVR_TXMSG_QUERY : { return string("QUERY"); }        

        default : { return CSeatracUtils::ToStringUnknown((uint8)value); }
    }
}

//==============================================================================
