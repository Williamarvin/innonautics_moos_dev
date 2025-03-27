/*==============================================================================
File that impliments a class that manages Seatrac Diagnostic Messages
//==============================================================================
History:
2017-08-09 Created v1 of file (R.Sharphouse)
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef SEATRAC_DIAGMSG_HPP
#define SEATRAC_DIAGMSG_HPP

//==============================================================================
//Library Includes
//==============================================================================
//Include C++ Libraries
#ifdef __cplusplus
using namespace std;	//Allow use of the standard namespace without explicit declaration
#endif

//Include system libraries
#include <stdarg.h>		//For variable length arguments in functions
#include <string.h>		//For string manipulation functions
#include <string>

//Include common type definitions and macros
#include "common.h"

//Include SeaTrac libraries
#include "seatrac_types.h"
#include "seatrac_utils.hpp"

//==============================================================================
//Class Definitions...
//==============================================================================
#define SEATRAC_DIAGMSG_SEPERATOR_CHAR  '|'

class CSeatracDiagMsg
{
    private:
        uint32 _diagId;
        ESeatracDiagMsgType _type;
        ESeatracCmdId _cmdId;
        bool _cmdIdValid;
        ESeatracDiagMsgStatus _status;
        string _paramStr;

        void AddNoteArgs(const char* str, va_list args);
        void AddParamArgs(const char* name, const char* value, va_list args);

        //Private static variables
        static uint32 _diagIdCnt;

    public:
        //Construction and Disposal
        CSeatracDiagMsg();
        ~CSeatracDiagMsg();

        //Methods
        void AddNote(const char* str, ...);
        void AddParam(const char* name, const char* value, ...);
        void AddParamBool(const char* name, bool value);
        void AddParamString(const char* name, const string& value);
        void AddParamString(const char* name, const char* value);
        void AddParamsArray(const char* name, puint8 data, uint8 len);
        void AddParamsBuffer(const char* name, TSeatracAcoPayload& value);
        void AddParamsBuffer(const char* name, TSeatracNavPacket& value);
        void AddParamsBuffer(const char* name, puint8 data, uint8 len);
        void AddParamsAcoFix(TSeatracAcoFix& value);
        void AddParamsAcoFixInfo(TSeatracAcoFixInfo& value);
        void AddParamsAcoFixLocal(TSeatracAcoFixLocal& value);
        void AddParamsAcoFixMsg(TSeatracAcoFixMsg& value);
        void AddParamsAcoMsg(TSeatracAcoMsg& value);
        void AddParamsCmdStatus(ESeatracCmdStatus value);
        void AddParamsNavQuery(TSeatracNavQuery& value);
        void AddParamsNoise(TSeatracNoise& value);
		void AddParamsPressureCal(TSeatracPressureCal& value);		
        void AddParamsSettings(TSeatracSettings& value);
        void AddParamsStatus(TSeatracStatus& value);
        void AddParamsUsbl(TSeatracUsbl& value);
		void AddParamsXcvrTxMsgCtrl(ESeaTracXcvrTxMsgCtrl value);
        void MakeParams(TSeatracDiagLogParams& params);
        void RaiseStatus(ESeatracDiagMsgStatus status);
        void SetType(ESeatracDiagMsgType type, bool error = false);
        void SetType(ESeatracDiagMsgType type, ESeatracCmdId cmdId, bool error = false);
};

//==============================================================================
#endif
