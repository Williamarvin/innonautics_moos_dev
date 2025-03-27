/*==============================================================================
File that implements a serialisable class that holds messages used for communicating
with SeaTrac acoustic beacons.

29-11-2013 - Created V1.0 of the file (R.Sharphouse)
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef SEATRAC_CMD_MSG_HPP
#define SEATRAC_CMD_MSG_HPP

//==============================================================================
#include <algorithm>    //For min and max functions

//Include common type definitions and macros
#include "common.h"

//Include system libraries
#include "serializebuffer.hpp"

//Include SeaTrac libraries
#include "seatrac_types.h"

//==============================================================================
//Class Definition...
//==============================================================================
//------------------------------------------------------------------------------
/*!
Class that implements a buffer used to store Tx and Rx commands on, and provides
helper methods for reading and writing them.
*/
class CSeatracCmdMsg : public CSerializeBuffer {
    private :
        /*! Declare easy access to the parent class */
        typedef CSerializeBuffer base;

    public :
        //Construction and disposal
        CSeatracCmdMsg(uint16 size = ST_MSG_LEN);
        CSeatracCmdMsg(ESeatracCmdId id, puint8 data, uint16 len);

        //Methods
        bool AddAcoMsgType(ESeatracAcoMsgType value);
        //bool AddAcoPayload(TSeatracAcoPayload& value);
        //bool AddAcoPayload(puint8 data, uint8 len);
        bool AddAhrsCal(TSeatracAhrsCal& value);
        bool AddBeaconId(ESeatracBeaconId value);
        bool AddBeaconBaud(uint32 value);
        bool AddCmdId(ESeatracCmdId value);
        bool AddSettings(TSeatracSettings& value);

        bool ReadAcoMsgType(ESeatracAcoMsgType* value);
        bool ReadAcoMsgType(ESeatracAcoMsgType* value, ESeatracAcoMsgType defValue);
        bool ReadAcoPayloadType(ESeatracAcoPayloadType* value);
        bool ReadAcoPayloadType(ESeatracAcoPayloadType* value, ESeatracAcoPayloadType defValue);
        bool ReadAppType(ESeatracAppType* value);
        bool ReadAppType(ESeatracAppType* value, ESeatracAppType defValue);
        bool ReadBeaconBaud(puint32 value);
        bool ReadBeaconBaud(puint32 value, uint32 defValue);
        bool ReadBeaconId(ESeatracBeaconId* value);
        bool ReadBeaconId(ESeatracBeaconId* value, ESeatracBeaconId defValue);
        bool ReadCmdId(ESeatracCmdId* value);
        bool ReadCmdId(ESeatracCmdId* value, ESeatracCmdId defValue);
        bool ReadCmdStatus(ESeatracCmdStatus* value);
        bool ReadCmdStatus(ESeatracCmdStatus* value, ESeatracCmdStatus defValue);

        bool ReadAcoFix(PSeatracAcoFix value);
        bool ReadAcoMsg(PSeatracAcoMsg value);
        bool ReadAcoPayload(PSeatracAcoPayload value);
        bool ReadAhrsCal(PSeatracAhrsCal value);
        bool ReadInfoFw(PSeatracInfoFw value);
        bool ReadInfoHw(PSeatracInfoHw value);
        bool ReadNavPacket(PSeatracNavPacket value);
        bool ReadNavQuery(PSeatracNavQuery value, uint8 queryFlags);
        bool ReadNoise(PSeatracNoise value);
		bool ReadPressureCal(PSeatracPressureCal value);
        bool ReadSettings(PSeatracSettings value);
        bool ReadStatus(PSeatracStatus value);
        bool ReadUsbl(PSeatracUsbl value);
};

/*! Declare a pointer to a command buffer */
typedef CSeatracCmdMsg* PSeatracCmdMsg;

//==============================================================================
#endif
