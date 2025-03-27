/*==============================================================================
Class that impliments static utility/helper functions

2017-08-07 - v1, Created file (R.Sharphouse)
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef SEATRAC_UTILS_HPP
#define SEATRAC_UTILS_HPP

//==============================================================================
//Include common type definitions and macros
#include "common.h"

//Include SeaTrac libraries
#include "seatrac_types.h"

//==============================================================================
//Class Definition...
//==============================================================================
/*! Class that impliments generat static help functions for SeaTrac data types
*/
class CSeatracUtils
{
    private:
        static string ToStringUnknown(uint8 value);

    public:
        template <typename T>
        static void InitStruct(T& value);

        static void InitAcoFixMsg(TSeatracAcoFixMsg& value);
        static void InitAcoFixLocal(TSeatracAcoFixLocal& value);
        static void InitAcoFixInfo(TSeatracAcoFixInfo& value);
        static void InitAcoFix(TSeatracAcoFix& value);
        static void InitNavQuery(TSeatracNavQuery &value);

        static bool IsBeaconIdPhysical(ESeatracBeaconId beaconId);
        static bool IsBeaconIdPhysicalOrAll(ESeatracBeaconId beaconId);
        static bool IsAcoMsgTypeOneWay(ESeatracAcoMsgType msgType);
        static bool IsAcoMsgTypeRequest(ESeatracAcoMsgType msgType);

        static uint32 ToBeaconPartNumber(ESeatracBeaconType value);
        static ESeatracBeaconType ToBeaconType(uint32 value);

        static string ToStringAcoMsgType(ESeatracAcoMsgType value);
        static string ToStringAcoPayloadType(ESeatracAcoPayloadType value);
        static string ToStringAppType(ESeatracAppType value);
        static string ToStringBeaconId(ESeatracBeaconId value);
        static string ToStringBeaconId(uint8 value);
        static string ToStringBeaconType(ESeatracBeaconType value);
        static string ToStringCalAction(ESeatracCalAction value);
        static string ToStringCmdId(ESeatracCmdId value);
        static string ToStringCmdStatus(ESeatracCmdStatus value);
        static string ToStringDiagMsgStatus(ESeatracDiagMsgStatus value);
        static string ToStringDiagMsgType(ESeatracDiagMsgType value);
        static string ToStringExecuteStatus(ESeatracExecuteStatus value);
        static string ToStringStatusMode(ESeatracStatusMode value);				
        static string ToStringVersion(uint8 major, uint8 minor, uint16 build);
		static string ToStringXcvrTxMsgCtrl(ESeaTracXcvrTxMsgCtrl value);
};

//==============================================================================
// Class Implimentation of Templated Functions
//==============================================================================
/*!-----------------------------------------------------------------------------
Function that initialised the contents of a generic structure to zero
*/
template <typename T>
void CSeatracUtils::InitStruct(T& value)
{
    T* p = &value;
    uint32 size = sizeof(T);
    memset(p, 0, size);
}

//==============================================================================
#endif
