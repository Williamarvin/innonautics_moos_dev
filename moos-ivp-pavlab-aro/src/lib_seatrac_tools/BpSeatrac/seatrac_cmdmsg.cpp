#include "seatrac_cmdmsg.hpp"

//==============================================================================
//Class Implementation...
//==============================================================================
//CCmdMsg
//==============================================================================
/*!-----------------------------------------------------------------------------
Constructor that initialises a message with the specified buffer size
*/
CSeatracCmdMsg::CSeatracCmdMsg(uint16 size) : base(size)
{
}

/*!-----------------------------------------------------------------------------
Constructor that initialises the message with the specified command code and
contents of the specified buffer
*/
CSeatracCmdMsg::CSeatracCmdMsg(ESeatracCmdId id, puint8 data, uint16 len) : base()
{
    //Initialise the message buffer
    _buf = NULL;
    this->Init(1 + len);	//Add one byte for the command id.

    //Populate the command
    this->AddCmdId(id);

    //Populate the message contents
    this->WriteData(data, len);
}

//------------------------------------------------------------------------------
bool CSeatracCmdMsg::AddAcoMsgType(ESeatracAcoMsgType value) { return this->WriteUint8((uint8)value); }

bool CSeatracCmdMsg::AddBeaconId(ESeatracBeaconId value) { return this->WriteUint8((uint8)value); }

bool CSeatracCmdMsg::AddCmdId(ESeatracCmdId value) { return this->WriteUint8((uint8)value); }

/*!-----------------------------------------------------------------------------
bool CSeatracCmdMsg::AddAcoPayload(TSeatracAcoPayload& value)
{
    return this->AddAcoPayload(value.Data, value.Length);
}
*/

/*!-----------------------------------------------------------------------------
bool CSeatracCmdMsg::AddAcoPayload(puint8 data, uint8 len)
{
    bool success = true;
    success &= this->AddUint8(len);
    success &= this->AddArray(data, len);
    return success;
}
*/

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::AddAhrsCal(TSeatracAhrsCal& value)
{
    bool success = true;
    success &= this->WriteInt16(value.AccMinX);
    success &= this->WriteInt16(value.AccMinY);
    success &= this->WriteInt16(value.AccMinZ);
    success &= this->WriteInt16(value.AccMaxX);
    success &= this->WriteInt16(value.AccMaxY);
    success &= this->WriteInt16(value.AccMaxZ);
    success &= this->WriteBool(value.MagValid);
    success &= this->WriteFloat(value.MagHardX);
    success &= this->WriteFloat(value.MagHardY);
    success &= this->WriteFloat(value.MagHardZ);
    success &= this->WriteFloat(value.MagSoftX);
    success &= this->WriteFloat(value.MagSoftY);
    success &= this->WriteFloat(value.MagSoftZ);
    success &= this->WriteFloat(value.MagField);
    success &= this->WriteFloat(value.MagError);
    success &= this->WriteInt16(value.GyroOffsetX);
    success &= this->WriteInt16(value.GyroOffsetY);
    success &= this->WriteInt16(value.GyroOffsetZ);
    return success;
}

/*!-----------------------------------------------------------------------------
Function that adds a Baud Rate specified by the beacon.
*/
bool CSeatracCmdMsg::AddBeaconBaud(uint32 value)
{
    ESeatracBeaconBaud baud;
    if(value >= 115200)
        baud = ST_BEACON_BAUD_115200;
    else if(value >= 57600)
        baud = ST_BEACON_BAUD_57600;
    else if(value >= 38400)
        baud = ST_BEACON_BAUD_38400;
    else if(value >= 19200)
        baud = ST_BEACON_BAUD_19200;
    else if(value >= 14400)
        baud = ST_BEACON_BAUD_14400;
    else if(value >= 9600)
        baud = ST_BEACON_BAUD_9600;
    else
        baud = ST_BEACON_BAUD_4800;
    return this->WriteUint8((uint8)baud);
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::AddSettings(TSeatracSettings& value)
{
    bool success = true;

    //Computer output values required
    uint8 statusFlags = (uint8)value.StatusMode & 0x07;
    uint8 statusOutput = 0;
    if(value.StatusOutputEnv) SET_BIT(statusOutput, ST_STATUS_FLAGS_ENVIRONMENT_BIT);
    if(value.StatusOutputAttitude) SET_BIT(statusOutput, ST_STATUS_FLAGS_ATTITUDE_BIT);
    if(value.StatusOutputMagCal) SET_BIT(statusOutput, ST_STATUS_FLAGS_MAG_CAL_BIT);
    if(value.StatusOutputAccCal) SET_BIT(statusOutput, ST_STATUS_FLAGS_ACC_CAL_BIT);
    if(value.StatusOutputAhrsRaw) SET_BIT(statusOutput, ST_STATUS_FLAGS_AHRS_RAW_BIT);
    if(value.StatusOutputAhrsComp) SET_BIT(statusOutput, ST_STATUS_FLAGS_AHRS_COMP_BIT);

    uint8 envFlags = 0;
    if(value.EnvAutoPressureOffset) SET_BIT(envFlags, ST_ENV_FLAGS_AUTO_PRESSURE_BIT);
    if(value.EnvAutoCalcVos) SET_BIT(envFlags, ST_ENV_FLAGS_AUTO_VOS_BIT);

    uint8 ahrsFlags = 0;
    if(value.AhrsAutoCalMag) SET_BIT(ahrsFlags, ST_AHRS_FLAGS_AUTO_CALC_MAG_BIT);

    uint8 xcvrFlags = 0;
    if(value.XcvrOutputDiagMsgs) SET_BIT(xcvrFlags, ST_XCVR_FLAGS_DIAG_MSGS_BIT);
    if(value.XcvrOutputFixMsgs) SET_BIT(xcvrFlags, ST_XCVR_FLAGS_FIX_MSGS_BIT);
    if(value.XcvrOutputUsblMsgs) SET_BIT(xcvrFlags, ST_XCVR_FLAGS_USBL_MSGS_BIT);
    if(value.XcvrEnablePosFilter) SET_BIT(xcvrFlags, ST_XCVR_FLAGS_POSFLT_ENABLE_BIT);
    if(value.XcvrUsblUseAhrs) SET_BIT(xcvrFlags, ST_XCVR_FLAGS_USBL_USE_AHRS_BIT);
	xcvrFlags |= (uint8)(((uint8)value.XcvrTxMsgCtrl << ST_XCVR_FLAGS_TX_MSGCTRL_SHIFT) & ST_XCVR_FLAGS_TX_MSGCTRL_MASK);

    //Encode Message Params
    success &= this->WriteUint8(statusFlags);
    success &= this->WriteUint8(statusOutput);

    success &= this->AddBeaconBaud(value.UartMainBaud);
    success &= this->AddBeaconBaud(value.UartAuxBaud);

    success &= this->WriteUint16(0);      //NetMacAddr[0:1] - Not used - just a dummy placeholder
    success &= this->WriteUint16(0);      //NetMacAddr[2:3] - Not used - just a dummy placeholder
    success &= this->WriteUint16(0);      //NetMacAddr[4:5] - Not used - just a dummy placeholder
    success &= this->WriteUint32(0);      //NetIpAddr - Not used - just a dummy placeholder
    success &= this->WriteUint32(0);      //NetIpSubnet - Not used - just a dummy placeholder
    success &= this->WriteUint32(0);      //NetIpGateway - Not used - just a dummy placeholder
    success &= this->WriteUint32(0);      //NetIpDns - Not used - just a dummy placeholder
    success &= this->WriteUint16(0);      //NetTcpPort - Not used - just a dummy placeholder

    success &= this->WriteUint8(envFlags);
    success &= this->WriteUint32((uint32)(value.EnvPressureOffset * 1000.0F));
    success &= this->WriteUint16((uint16)(value.EnvSalinity * 10.0F));
    success &= this->WriteUint16((uint16)(value.EnvVos * 10.0F));

    success &= this->WriteUint8(ahrsFlags);
    success &= this->AddAhrsCal(value.AhrsCal);
    success &= this->WriteInt16((int16)(value.AhrsYawOffset / 10.0F));
    success &= this->WriteInt16((int16)(value.AhrsPitchOffset / 10.0F));
    success &= this->WriteInt16((int16)(value.AhrsRollOffset / 10.0F));

    success &= this->WriteUint8(xcvrFlags);
    success &= this->AddBeaconId(value.XcvrBeaconId);
    success &= this->WriteUint16(value.XcvrRangeTimeout);
    success &= this->WriteUint16(value.XcvrResponseTime);
    success &= this->WriteInt16((int16)(value.XcvrFixedYaw / 10.0F));
    success &= this->WriteInt16((int16)(value.XcvrFixedPitch / 10.0F));
    success &= this->WriteInt16((int16)(value.XcvrFixedRoll / 10.0F));
    success &= this->WriteUint8(value.XcvrPosFilterVelocity);
    success &= this->WriteUint8(value.XcvrPosFilterAngularRate);
    success &= this->WriteUint8(value.XcvrPosFilterTimeout);

    return success;
}

//------------------------------------------------------------------------------
bool CSeatracCmdMsg::ReadAcoMsgType(ESeatracAcoMsgType* value) { return this->ReadUint8((puint8)value); }
bool CSeatracCmdMsg::ReadAcoMsgType(ESeatracAcoMsgType* value, ESeatracAcoMsgType defValue) { return this->ReadUint8((puint8)value, (uint8)defValue); }

bool CSeatracCmdMsg::ReadAcoPayloadType(ESeatracAcoPayloadType* value) { return this->ReadUint8((puint8)value); }
bool CSeatracCmdMsg::ReadAcoPayloadType(ESeatracAcoPayloadType* value, ESeatracAcoPayloadType defValue) { return this->ReadUint8((puint8)value, (uint8)defValue); }

bool CSeatracCmdMsg::ReadAppType(ESeatracAppType* value) { return this->ReadUint8((puint8)value); }
bool CSeatracCmdMsg::ReadAppType(ESeatracAppType* value, ESeatracAppType defValue) { return this->ReadUint8((puint8)value, (uint8)defValue); }

bool CSeatracCmdMsg::ReadBeaconId(ESeatracBeaconId* value) { return this->ReadUint8((puint8)value); }
bool CSeatracCmdMsg::ReadBeaconId(ESeatracBeaconId* value, ESeatracBeaconId defValue) { return this->ReadUint8((puint8)value, (uint8)defValue); }

bool CSeatracCmdMsg::ReadCmdId(ESeatracCmdId* value) { return this->ReadUint8((puint8)value); }
bool CSeatracCmdMsg::ReadCmdId(ESeatracCmdId* value, ESeatracCmdId defValue) { return this->ReadUint8((puint8)value, (uint8)defValue); }

bool CSeatracCmdMsg::ReadCmdStatus(ESeatracCmdStatus* value) { return this->ReadUint8((puint8)value); }
bool CSeatracCmdMsg::ReadCmdStatus(ESeatracCmdStatus* value, ESeatracCmdStatus defValue) { return this->ReadUint8((puint8)value, (uint8)defValue); }

/*!-----------------------------------------------------------------------------
Read an ACO_FIX structure
*/
bool CSeatracCmdMsg::ReadAcoFix(PSeatracAcoFix value)
{
    bool success = true;
    int16 attitudeYaw;                      /*!< The yaw angle (relative to magnetic north) of the local beacon when the fix was computed. Values are encoded as deci-Degrees, so divide by 10 for just degrees to a 0.1° resolution. */
    int16 attitudePitch;                    /*!< The pitch angle of the local beacon when the fix was computed. Values are encoded as deci-Degrees, so divide by 10 for just degrees to a 0.1° resolution. */
    int16 attitudeRoll;                     /*!< The roll angle of the local beacon when the fix was computed. Values are encoded as deci-Degrees, so divide by 10 for just degrees to a 0.1° resolution. */
    uint16 depthLocal;                      /*!< The reading from the local beacon depth sensor when the fix was calculated. Values are encoded in decimetres, so divide by 10 to obtain a value in metres to a 0.1m resolution. */
    uint16 vos;                             /*!< The velocity of sound value used for the computation of the remote beacon’s range based on timing information. Values are encoded in decimetres-per-second, so divide by 10 for a value in metres-per-second. */
    int16 rssi;                             /*!< The Received Signal Strength Indicator value for the received message, encoded in centibels. To decode, divide this value by 10 for decibels to a 0.1 dB resolution. */
    uint32 rangeCount;                      /*!< The number of 16kHz timer intervals that were counted between Request message transmission and Response message reception. */
    int32 rangeTime;                        /*!< The time in seconds derived from the RANGE_COUNT value, and with internal timing offsets and compensation applied. Values are encoded in 100 nanosecond multiples, so divide by 10000000 to obtain a value in seconds. */
    uint16 rangeDist;                       /*!< The resolved line-of-sight distance to the remote beacon, based on the RANGE_TIME and VOS values. Values are encoded in decimetres, so divide by 10 for a value in metres. */
    uint8 usblChannels;                     /*!< The number of USBL receiver channels being used to compute the signal angle. Typically this value is either 3 or 4. */
    int16 usblRssi[ST_USBL_CHANNELS];       /*!< An array of the received signal strengths for each of the USBL receiver channels, where “x” is the value defined by the CHANNELS field. Values are encoded in centi-Bels, so divide by 10 to obtain a value in decibels to a resolution of 0.1dB. */
    int16 usblAzimuth;                      /*!< The incoming signal azimuth angle from 0° to 360°. Values are encoded as deci-Degrees, so divide by 10 for just degrees to a 0.1° resolution. */
    int16 usblEvelvation;                   /*!< The incoming signal elevation angle from -90° to +90°. Values are encoded as deci-Degrees, so divide by 10 for just degrees to a 0.1° resolution. */
    int16 usblFitError;                     /*!< The fit error value returns a number that indicates the quality of fit (or confidence) of the signal azimuth and elevation values from the timing and phase-angle data available. Smaller values towards 0.0 indicate a better fit, while larger values (increasing above 2-3) indicate poorer fits and larger error tolerances. Values are dimensionless, but divide the value by 100 to obtain a signed floating-point value to a resolution of 0.01. */
    int16 positionEasting;                  /*!< The Easting distance component of the relative position of the remote beacon to the local beacon computed from the range, incoming signal angle, local beacon depth, attitude and magnetic heading. Values are encoded in decimetres, so divide by 10 for a value in metres. */
    int16 positionNorthing;                 /*!< The Northing distance component of the relative position of the remote beacon to the local beacon computed from the range, incoming signal angle, local beacon depth, attitude and magnetic heading. Values are encoded in decimetres, so divide by 10 for a value in metres. */
    int16 positionDepth;                    /*!< The vertical Depth distance component of the remote beacon from the surface - computed from the range, incoming signal angle, local beacon depth, attitude and magnetic heading. Values are encoded in decimetres, so divide by 10 for a value in metres. NB: If the ‘Fix’ has been obtained by a MSG_REQU (Usbl) type request, then this value is computed from the beacon’s attitude and incoming signal angle. If a MSG_REQX (Enhanced) type request has been used, then this value is the remotely transmitted beacon depth sensor value. */

    //Received Message Fields
    success &= this->ReadBeaconId(&value->Msg.DestId, ST_BEACON_UNKNOWN);
    success &= this->ReadBeaconId(&value->Msg.SrcId, ST_BEACON_UNKNOWN);
    success &= this->ReadUint8(&value->Flags, 0);
    success &= this->ReadAcoMsgType(&value->Msg.Type, ST_AMSG_UNKNOWN);
    success &= this->ReadInt16(&attitudeYaw, 0);
    success &= this->ReadInt16(&attitudePitch, 0);
    success &= this->ReadInt16(&attitudeRoll, 0);
    success &= this->ReadUint16(&depthLocal, 0);
    success &= this->ReadUint16(&vos, 0);
    success &= this->ReadInt16(&rssi, 0);

    value->Msg.Rssi = rssi / 10.0F;
    value->Local.Vos = vos / 10.0F;
    value->Local.DepthValid = true;                        //###TODO True at the moment, but allows support for beacons without a depth sensor
    value->Local.Depth = depthLocal / 10.0F;
    value->Local.AttitudeValid = true;                     //###TODO True at the moment, add support for beacons without an AHRX (X010=FALSE)
    value->Local.AttitudeYaw = attitudeYaw / 10.0F;
    value->Local.AttitudePitch = attitudePitch / 10.0F;
    value->Local.AttitudeRoll = attitudeRoll / 10.0F;
    value->Info.RangeValid = IS_BIT_SET(value->Flags, ST_ACOFIX_FLAGS_RANGE_VALID_BIT);
    value->Info.UsblValid = IS_BIT_SET(value->Flags, ST_ACOFIX_FLAGS_USBL_VALID_BIT);
    value->Info.PositionValid = IS_BIT_SET(value->Flags, ST_ACOFIX_FLAGS_POS_VALID_BIT);
    value->Info.PositionEnhanced = IS_BIT_SET(value->Flags, ST_ACOFIX_FLAGS_POS_ENHANCED_BIT);
    value->Info.PositionFilterError = IS_BIT_SET(value->Flags, ST_ACOFIX_FLAGS_POS_FLT_ERROR_BIT);

    //Read Range values if present
    if(value->Info.RangeValid) {
        success &= this->ReadUint32(&rangeCount);
        success &= this->ReadInt32(&rangeTime);
        success &= this->ReadUint16(&rangeDist);
        value->Info.RangeCount = rangeCount;
        value->Info.RangeTime = rangeTime / 10000000.0F;
        value->Info.RangeDist = rangeDist / 10.0F;
    }
    else {
        value->Info.RangeCount = 0;
        value->Info.RangeTime = 0.0F;
        value->Info.RangeDist = 0.0F;
    }

    //Read USBL values if present
    if(value->Info.UsblValid) {
        success &= this->ReadUint8(&usblChannels, 0);
        usblChannels = std::min(usblChannels, (uint8)ST_USBL_CHANNELS);
        success &= this->ReadArray(usblRssi, usblChannels, sizeof(usblRssi[0]));
        success &= this->ReadInt16(&usblAzimuth);
        success &= this->ReadInt16(&usblEvelvation);
        success &= this->ReadInt16(&usblFitError);
        value->Info.UsblChannels = usblChannels;
        for(uint8 idx = 0; idx < ST_USBL_CHANNELS; idx++)
            value->Info.UsblRssi[idx] = usblRssi[idx] / 10.0F;
        value->Info.UsblAzimuth = usblAzimuth / 10.0F;
        value->Info.UsblElevation = usblEvelvation / 10.0F;
        value->Info.UsblFitError = usblFitError / 100.0F;
    }
    else {
        value->Info.UsblChannels = 0;
        for(uint8 idx = 0; idx < ST_USBL_CHANNELS; idx++)
            value->Info.UsblRssi[idx] = 0.0F;
        value->Info.UsblAzimuth = 0.0F;
        value->Info.UsblElevation = 0.0F;
        value->Info.UsblFitError = 0.0F;
    }

    //Read position values if present
    if(value->Info.PositionValid) {
        success &= this->ReadInt16(&positionEasting);
        success &= this->ReadInt16(&positionNorthing);
        success &= this->ReadInt16(&positionDepth);
        value->Info.PositionEasting = positionEasting / 10.0F;
        value->Info.PositionNorthing = positionNorthing / 10.0F;
        value->Info.PositionDepth = positionDepth / 10.0F;
    }
    else {
        value->Info.PositionEasting = 0.0F;
        value->Info.PositionNorthing = 0.0F;
        value->Info.PositionDepth = 0.0F;
    }

    return success;
}

/*!-----------------------------------------------------------------------------
Read an ACO_MSG structure
*/
bool CSeatracCmdMsg::ReadAcoMsg(PSeatracAcoMsg value)
{
    bool success = true;
    success &= this->ReadBeaconId(&value->DestId, ST_BEACON_UNKNOWN);
    success &= this->ReadBeaconId(&value->SrcId, ST_BEACON_UNKNOWN);
    success &= this->ReadAcoMsgType(&value->MsgType, ST_AMSG_UNKNOWN);
    success &= this->ReadUint16(&value->MsgDepth, 0);
    success &= this->ReadAcoPayloadType(&value->PayloadType, ST_APLOAD_UNKNOWN);

    //success &= this->ReadUint8(&value->Payload.Length, 0);
    ////success &= this->ReadArray(&value->Payload.Data, ST_ACOMSG_PAYLOAD_LEN);
    //success &= this->ReadArray(&value->Payload.Data, value->Payload.Length, sizeof(value->Payload.Data[0]));
    success &= this->ReadAcoPayload(&value->Payload);

    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadAcoPayload(PSeatracAcoPayload value)
{
    bool success = true;
    success &= this->ReadUint8(&value->Length, 0);

    if(success && (value->Length > 0)) {
        //Clip the length
        value->Length = min(value->Length, (uint8)ST_ACOMSG_PAYLOAD_LEN);
        //Return the data
        success &= this->ReadArray(value->Data, value->Length, 1);
    }
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadAhrsCal(PSeatracAhrsCal value)
{
    bool success = true;
    success &= this->ReadInt16(&value->AccMinX, 0);
    success &= this->ReadInt16(&value->AccMinY, 0);
    success &= this->ReadInt16(&value->AccMinZ, 0);
    success &= this->ReadInt16(&value->AccMaxX, 0);
    success &= this->ReadInt16(&value->AccMaxY, 0);
    success &= this->ReadInt16(&value->AccMaxZ, 0);
    success &= this->ReadBool(&value->MagValid, false);
    success &= this->ReadFloat(&value->MagHardX, 0.0F);
    success &= this->ReadFloat(&value->MagHardY, 0.0F);
    success &= this->ReadFloat(&value->MagHardZ, 0.0F);
    success &= this->ReadFloat(&value->MagSoftX, 0.0F);
    success &= this->ReadFloat(&value->MagSoftY, 0.0F);
    success &= this->ReadFloat(&value->MagSoftZ, 0.0F);
    success &= this->ReadFloat(&value->MagField, 0.0F);
    success &= this->ReadFloat(&value->MagError, 0.0F);
    success &= this->ReadInt16(&value->GyroOffsetX, 0);
    success &= this->ReadInt16(&value->GyroOffsetY, 0);
    success &= this->ReadInt16(&value->GyroOffsetZ, 0);
    return success;
}

/*!-----------------------------------------------------------------------------
Function that reads a Baud Rate specified by the beacon.
*/
bool CSeatracCmdMsg::ReadBeaconBaud(puint32 value)
{
    uint8 baud;
    bool success = this->ReadUint8(&baud);
    if(success) {
        switch (baud) {
            case ST_BEACON_BAUD_4800  : { *value = 4800; break; }
            case ST_BEACON_BAUD_9600  : { *value = 9600; break; }
            case ST_BEACON_BAUD_14400 : { *value = 14400; break; }
            case ST_BEACON_BAUD_19200 : { *value = 19200; break; }
            case ST_BEACON_BAUD_38400 : { *value = 38400; break; }
            case ST_BEACON_BAUD_57600 : { *value = 57600; break; }
            case ST_BEACON_BAUD_115200 : { *value = 115200; break; }
            default : { *value = 0; break; }
        }
    }
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadBeaconBaud(puint32 value, uint32 defValue)
{
    bool success = this->ReadBeaconBaud(value);
    if(!success)
        *value = defValue;
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadInfoFw(PSeatracInfoFw value)
{
    bool success = true;
    success &= this->ReadBool(&value->Valid, false);
    success &= this->ReadUint16(&value->PartNumber, 0);
    success &= this->ReadUint8(&value->VersionMajor, 0);
    success &= this->ReadUint8(&value->VersionMinor, 0);
    success &= this->ReadUint16(&value->VersionBuild, 0);
    success &= this->ReadUint32(&value->Checksum, 0);
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadInfoHw(PSeatracInfoHw value)
{
    bool success = true;
    success &= this->ReadUint16(&value->PartNumber, 0);
    success &= this->ReadUint8(&value->PartRev, 0);
    success &= this->ReadUint32(&value->SerialNumber, 0);
    success &= this->ReadUint16(&value->FlagsSys, 0);
    success &= this->ReadUint16(&value->FlagsUser, 0);
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadNavPacket(PSeatracNavPacket value)
{
    bool success = true;
    success &= this->ReadUint8(&value->Length, 0);      //Supported in firmware 1.12 onwards

    if(success && (value->Length > 0)) {
        //Clip the length
        value->Length = min(value->Length, (uint8)ST_NAV_PAYLOAD_LEN);
        //Return the data
        success &= this->ReadArray(value->Data, value->Length, 1);
    }
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadNavQuery(PSeatracNavQuery value, uint8 queryFlags)
{
    bool success = true;
    int32 depth;
    uint16 supply;
    int16 temp;
    int16 attYaw;
    int16 attPitch;
    int16 attRoll;

    //Decode Flags
    value->Flags = queryFlags;
    value->DepthValid = IS_BIT_SET(queryFlags, ST_NAVQUERY_FLAGS_DEPTH_BIT);
    value->SupplyValid = IS_BIT_SET(queryFlags, ST_NAVQUERY_FLAGS_SUPPLY_BIT);
    value->TempValid = IS_BIT_SET(queryFlags, ST_NAVQUERY_FLAGS_TEMP_BIT);
    value->AttitudeValid = IS_BIT_SET(queryFlags, ST_NAVQUERY_FLAGS_ATTITUDE_BIT);

    //Decode Depth Fields
    if(value->DepthValid) {
        success &= this->ReadInt32(&depth, 0);

        value->DepthRemote = depth / 10.0F;
    }
    else {
        value->DepthRemote = 0.0F;
    }

    //Decode Supply Fields
    if(value->SupplyValid) {
        success &= this->ReadUint16(&supply, 0);

        value->SupplyRemote = supply / 1000.0F;
    }
    else {
        value->SupplyRemote = 0.0F;
    }

    //Decode Temp Fields
    if(value->TempValid) {
        success &= this->ReadInt16(&temp, 0);

        value->TempRemote = temp / 10.0F;
    }
    else {
        value->TempRemote = 0.0F;
    }

    //Decode Attitude Fields
    if(value->AttitudeValid) {
        success &= this->ReadInt16(&attYaw, 0);
        success &= this->ReadInt16(&attPitch, 0);
        success &= this->ReadInt16(&attRoll, 0);

        value->AttitudeYawRemote = attYaw / 10.0F;
        value->AttitudePitchRemote = attPitch / 10.0F;
        value->AttitudeRollRemote = attRoll / 10.0F;
    }
    else {
        value->AttitudeYawRemote = 0.0F;
        value->AttitudePitchRemote = 0.0F;
        value->AttitudeRollRemote = 0.0F;
    }

    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadNoise(PSeatracNoise value)
{
    bool success = true;
    int16 rxLevelPkPk;
    int16 rxLevelRms;

    success &= this->ReadInt16(&value->AdcMean, 0);
    success &= this->ReadUint16(&value->AdcPkPk, 0);
    success &= this->ReadUint32(&value->AdcRms, 0);
    success &= this->ReadInt16(&rxLevelPkPk, 0);
    success &= this->ReadInt16(&rxLevelRms, 0);

    value->RxLevelPkPk = rxLevelPkPk / 10.0F;
    value->RxLevelRms = rxLevelRms / 10.0F;

    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadPressureCal(PSeatracPressureCal value)
{
	bool success = true;
    int16 pressureMin;
    int16 pressureMax;
      
	success &= this->ReadUint32(&value->Id, 0);
	success &= this->ReadUint8(&value->Type, 0);
	success &= this->ReadInt16(&pressureMin, 0);
	success &= this->ReadInt16(&pressureMax, 0);
	success &= this->ReadUint8(&value->CalDay, 1);
	success &= this->ReadUint8(&value->CalMonth, 1);
	success &= this->ReadUint16(&value->CalYear, 2000);
	
	value->PressureMin = pressureMin / 10.0F;
    value->PressureMax = pressureMax / 10.0F;
	
    return success;
}

/*!-----------------------------------------------------------------------------
*/
bool CSeatracCmdMsg::ReadSettings(PSeatracSettings value)
{
    bool success = true;
    uint8 statusFlags;
    uint8 statusOutput;
    uint8 netMacAddr[6];        //Not used - just a dummy placeholder
    uint8 netIpAddr[4];         //Not used - just a dummy placeholder
    uint8 netIpSubnet[4];       //Not used - just a dummy placeholder
    uint8 netIpGateway[4];      //Not used - just a dummy placeholder
    uint8 netIpDns[4];          //Not used - just a dummy placeholder
    uint16 netTcpPort;          //Not used - just a dummy placeholder
    uint8 envFlags;
    uint32 envPressureOffset;
    uint16 envSalinity;
    uint16 envVos;
    uint8 ahrsFlags;
    int16 ahrsYawOffset;
    int16 ahrsPitchOffset;
    int16 ahrsRollOffset;
    uint8 xcvrFlags;
    int16 xcvrFixedYaw;
    int16 xcvrFixedPitch;
    int16 xcvrFixedRoll;

    //Decode message params
    success &= this->ReadUint8(&statusFlags, 0);
    success &= this->ReadUint8(&statusOutput, 0);
    success &= this->ReadBeaconBaud(&value->UartMainBaud, 115200);
    success &= this->ReadBeaconBaud(&value->UartAuxBaud, 115200);
    success &= this->ReadData(netMacAddr, sizeof(netMacAddr));
    success &= this->ReadData(netIpAddr, sizeof(netIpAddr));
    success &= this->ReadData(netIpSubnet, sizeof(netIpSubnet));
    success &= this->ReadData(netIpGateway, sizeof(netIpGateway));
    success &= this->ReadData(netIpDns, sizeof(netIpDns));
    success &= this->ReadUint16(&netTcpPort, 0);
    success &= this->ReadUint8(&envFlags, 0);
    success &= this->ReadUint32(&envPressureOffset, 0);
    success &= this->ReadUint16(&envSalinity, 0);
    success &= this->ReadUint16(&envVos, 0);
    success &= this->ReadUint8(&ahrsFlags, 0);
    success &= this->ReadAhrsCal(&value->AhrsCal);
    success &= this->ReadInt16(&ahrsYawOffset, 0);
    success &= this->ReadInt16(&ahrsPitchOffset, 0);
    success &= this->ReadInt16(&ahrsRollOffset, 0);
    success &= this->ReadUint8(&xcvrFlags, 0);
    success &= this->ReadBeaconId(&value->XcvrBeaconId, ST_BEACON_UNKNOWN);
    success &= this->ReadUint16(&value->XcvrRangeTimeout, 1000);
    success &= this->ReadUint16(&value->XcvrResponseTime, 10);
    success &= this->ReadInt16(&xcvrFixedYaw, 0);
    success &= this->ReadInt16(&xcvrFixedPitch, 0);
    success &= this->ReadInt16(&xcvrFixedRoll, 0);
    success &= this->ReadUint8(&value->XcvrPosFilterVelocity, 0);
    success &= this->ReadUint8(&value->XcvrPosFilterAngularRate, 0);
    success &= this->ReadUint8(&value->XcvrPosFilterTimeout, 0);

    //Compute return values;
    value->StatusMode = (ESeatracStatusMode)(statusFlags & 0x07);
    value->StatusOutputEnv = IS_BIT_SET(statusOutput, ST_STATUS_FLAGS_ENVIRONMENT_BIT);
    value->StatusOutputAttitude = IS_BIT_SET(statusOutput, ST_STATUS_FLAGS_ATTITUDE_BIT);
    value->StatusOutputMagCal = IS_BIT_SET(statusOutput, ST_STATUS_FLAGS_MAG_CAL_BIT);
    value->StatusOutputAccCal = IS_BIT_SET(statusOutput, ST_STATUS_FLAGS_ACC_CAL_BIT);
    value->StatusOutputAhrsRaw = IS_BIT_SET(statusOutput, ST_STATUS_FLAGS_AHRS_RAW_BIT);
    value->StatusOutputAhrsComp = IS_BIT_SET(statusOutput, ST_STATUS_FLAGS_AHRS_COMP_BIT);
    value->EnvAutoPressureOffset = IS_BIT_SET(envFlags, ST_ENV_FLAGS_AUTO_PRESSURE_BIT);
    value->EnvAutoCalcVos = IS_BIT_SET(envFlags, ST_ENV_FLAGS_AUTO_VOS_BIT);
    value->EnvPressureOffset = envPressureOffset / 1000.0F;
    value->EnvSalinity = envSalinity / 10.0F;
    value->EnvVos = envVos / 10.0F;
    value->AhrsAutoCalMag = IS_BIT_SET(ahrsFlags, ST_AHRS_FLAGS_AUTO_CALC_MAG_BIT);
    value->AhrsYawOffset = ahrsYawOffset / 10.0F;
    value->AhrsPitchOffset = ahrsPitchOffset / 10.0F;
    value->AhrsRollOffset = ahrsRollOffset / 10.0F;
    value->XcvrOutputDiagMsgs = IS_BIT_SET(xcvrFlags, ST_XCVR_FLAGS_DIAG_MSGS_BIT);
    value->XcvrOutputFixMsgs = IS_BIT_SET(xcvrFlags, ST_XCVR_FLAGS_FIX_MSGS_BIT);
    value->XcvrOutputUsblMsgs = IS_BIT_SET(xcvrFlags, ST_XCVR_FLAGS_USBL_MSGS_BIT);
    value->XcvrEnablePosFilter = IS_BIT_SET(xcvrFlags, ST_XCVR_FLAGS_POSFLT_ENABLE_BIT);
    value->XcvrUsblUseAhrs = IS_BIT_SET(xcvrFlags, ST_XCVR_FLAGS_USBL_USE_AHRS_BIT);
	value->XcvrTxMsgCtrl = (ESeaTracXcvrTxMsgCtrl)((xcvrFlags & (uint8)ST_XCVR_FLAGS_TX_MSGCTRL_MASK) >> ST_XCVR_FLAGS_TX_MSGCTRL_SHIFT);	
    value->XcvrFixedYaw = xcvrFixedYaw / 10.0F;
    value->XcvrFixedPitch = xcvrFixedPitch / 10.0F;
    value->XcvrFixedRoll = xcvrFixedRoll / 10.0F;

    return success;
}

/*!-----------------------------------------------------------------------------
Read an STATUS structure
*/
bool CSeatracCmdMsg::ReadStatus(PSeatracStatus value)
{
    bool success = true;
    uint64 timestamp;
    uint16 envSupply;
    int16 envTemp;
    int32 envPressure;
    int32 envDepth;
    uint16 envVos;
    int16 attYaw;
    int16 attPitch;
    int16 attRoll;

    //Decode command paramters
    success &= this->ReadUint8(&value->Flags, 0);
    success &= this->ReadUint64(&timestamp);

    value->Timestamp = timestamp / 1000.0F;
    value->EnvValid = IS_BIT_SET(value->Flags, ST_STATUS_FLAGS_ENVIRONMENT_BIT);
    value->AttitudeValid = IS_BIT_SET(value->Flags, ST_STATUS_FLAGS_ATTITUDE_BIT);
    value->MagValid = IS_BIT_SET(value->Flags, ST_STATUS_FLAGS_MAG_CAL_BIT);
    value->AccValid = IS_BIT_SET(value->Flags, ST_STATUS_FLAGS_ACC_CAL_BIT);
    value->AhrsRawValid = IS_BIT_SET(value->Flags, ST_STATUS_FLAGS_AHRS_RAW_BIT);
    value->AhrsCompValid = IS_BIT_SET(value->Flags, ST_STATUS_FLAGS_AHRS_COMP_BIT);

    //Decode Environment Fields
    if(value->EnvValid) {
        success &= this->ReadUint16(&envSupply, 0);
        success &= this->ReadInt16(&envTemp, 0);
        success &= this->ReadInt32(&envPressure, 0);
        success &= this->ReadInt32(&envDepth, 0);
        success &= this->ReadUint16(&envVos, 0);
        value->EnvSupply = envSupply / 1000.0F;
        value->EnvTemp = envTemp / 10.0F;
        value->EnvPressure = envPressure / 1000.0F;
        value->EnvDepth = envDepth / 10.0F;
        value->EnvVos = envVos / 10.0F;
    }
    else {
        value->EnvSupply = 0.0F;
        value->EnvTemp = 0.0F;
        value->EnvPressure = 0.0F;
        value->EnvDepth = 0.0F;
        value->EnvVos = 0.0F;
    }

    //Decode Attitude Fields
    if(value->AttitudeValid) {
        success &= this->ReadInt16(&attYaw, 0);
        success &= this->ReadInt16(&attPitch, 0);
        success &= this->ReadInt16(&attRoll, 0);
        value->AttitudeYaw = attYaw / 10.0F;
        value->AttitudePitch = attPitch / 10.0F;
        value->AttitudeRoll = attRoll / 10.0F;
    }
    else {
        value->AttitudeYaw = 0.0F;
        value->AttitudePitch = 0.0F;
        value->AttitudeRoll = 0.0F;
    }

    //Decode Magnetometer Calibration Fields
    if(value->MagValid) {
        success &= this->ReadUint8(&value->MagCalBuf, 0);
        success &= this->ReadBool(&value->MagCalValid, false);
        success &= this->ReadUint32(&value->MagCalAge, 0);
        success &= this->ReadUint8(&value->MagCalFit, 0);
    }
    else {
        value->MagCalBuf = 0;
        value->MagCalValid = false;
        value->MagCalAge = 0;
        value->MagCalFit = 0;
    }

    //Decode Accelerometer Calibration Fields
    if(value->AccValid) {
        success &= this->ReadInt16(&value->AccLimMinX, 0);
        success &= this->ReadInt16(&value->AccLimMaxX, 0);
        success &= this->ReadInt16(&value->AccLimMinY, 0);
        success &= this->ReadInt16(&value->AccLimMaxY, 0);
        success &= this->ReadInt16(&value->AccLimMinZ, 0);
        success &= this->ReadInt16(&value->AccLimMaxZ, 0);
    }
    else {
        value->AccLimMinX = 0; value->AccLimMaxX = 0;
        value->AccLimMinY = 0; value->AccLimMaxY = 0;
        value->AccLimMinZ = 0; value->AccLimMaxZ = 0;
    }

    //Decode Ahrs Raw Sensor Data Fields
    if(value->AhrsRawValid) {
        success &= this->ReadInt16(&value->AhrsRawAccX, 0);
        success &= this->ReadInt16(&value->AhrsRawAccY, 0);
        success &= this->ReadInt16(&value->AhrsRawAccZ, 0);
        success &= this->ReadInt16(&value->AhrsRawMagX, 0);
        success &= this->ReadInt16(&value->AhrsRawMagY, 0);
        success &= this->ReadInt16(&value->AhrsRawMagZ, 0);
        success &= this->ReadInt16(&value->AhrsRawGyroX, 0);
        success &= this->ReadInt16(&value->AhrsRawGyroY, 0);
        success &= this->ReadInt16(&value->AhrsRawGyroZ, 0);
    }
    else {
        value->AhrsRawAccX = 0; value->AhrsRawAccY = 0; value->AhrsRawAccZ = 0;
        value->AhrsRawMagX = 0; value->AhrsRawMagY = 0; value->AhrsRawMagZ = 0;
        value->AhrsRawGyroX = 0; value->AhrsRawGyroY = 0; value->AhrsRawGyroZ = 0;
    }

    //Decode Ahrs Compensated Sensor Data Fields
    if(value->AhrsCompValid) {
        success &= this->ReadFloat(&value->AhrsCompAccX, 0.0F);
        success &= this->ReadFloat(&value->AhrsCompAccY, 0.0F);
        success &= this->ReadFloat(&value->AhrsCompAccZ, 0.0F);
        success &= this->ReadFloat(&value->AhrsCompMagX, 0.0F);
        success &= this->ReadFloat(&value->AhrsCompMagY, 0.0F);
        success &= this->ReadFloat(&value->AhrsCompMagZ, 0.0F);
        success &= this->ReadFloat(&value->AhrsCompGyroX, 0.0F);
        success &= this->ReadFloat(&value->AhrsCompGyroY, 0.0F);
        success &= this->ReadFloat(&value->AhrsCompGyroZ, 0.0F);
    }
    else {
        value->AhrsCompAccX = 0.0F; value->AhrsCompAccY = 0.0F; value->AhrsCompAccZ = 0.0F;
        value->AhrsCompMagX = 0.0F; value->AhrsCompMagY = 0.0F; value->AhrsCompMagZ = 0.0F;
        value->AhrsCompGyroX = 0.0F; value->AhrsCompGyroY = 0.0F; value->AhrsCompGyroZ = 0.0F;
    }

    return success;
}

/*!-----------------------------------------------------------------------------
Read an USBL structure
*/
bool CSeatracCmdMsg::ReadUsbl(PSeatracUsbl value)
{
    bool success = true;
    int16 channelRssi[ST_USBL_CHANNELS];
    int16 signalAzimuth;
    int16 signalElevation;

    //Decode command paramters
    success &=this->ReadFloat(&value->XcorSignalPeak, 0);
    success &=this->ReadFloat(&value->XcorThreshold, 0.0F);
    success &=this->ReadUint16(&value->XcorCrossingPoint, 0);
    success &=this->ReadFloat(&value->XcorCrossingMagnitude, 0.0F);
    success &=this->ReadUint16(&value->XcorDetect, 0);
    success &=this->ReadUint16(&value->XcorLength, 0);
    value->XcorLength = std::min(value->XcorLength, (uint16)ST_USBL_XCOR_DATA_MAX);
    success &=this->ReadArray(value->XcorData, value->XcorLength, sizeof(value->XcorData[0]));
    success &=this->ReadUint8(&value->Channels, 0);
    value->Channels = std::min(value->Channels, (uint8)ST_USBL_CHANNELS);
    success &=this->ReadArray(channelRssi, value->Channels, sizeof(channelRssi[0]));
    success &=this->ReadUint8(&value->Baselines, 0);
    value->Baselines = std::min(value->Baselines, (uint8)ST_USBL_BASELINES);
    success &=this->ReadArray(value->BaselinePhaseAngle, value->Baselines, sizeof(value->BaselinePhaseAngle[0]));
    success &=this->ReadInt16(&signalAzimuth, 0);
    success &=this->ReadInt16(&signalElevation, 0);
    success &=this->ReadFloat(&value->SignalFitError, -1.0F);
    success &=this->ReadBeaconId(&value->DestId, ST_BEACON_UNKNOWN);
    success &=this->ReadBeaconId(&value->SrcId, ST_BEACON_UNKNOWN);

    for(uint8 idx = 0; idx < ST_USBL_CHANNELS; idx++)
        value->ChannelRssi[idx] = channelRssi[idx] / 10.0F;
    value->SignalAzimuth = signalAzimuth / 10.0F;
    value->SignalElevation = signalElevation / 10.0F;

    return success;
}

//==============================================================================
