/*==============================================================================
This file is part of <APPLICATION NAME>

<APPLICATION NAME> is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

//==============================================================================
History:
2017-08-01 Created v1 of file (R.Sharphouse)
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef SEATRAC_HPP
#define SEATRAC_HPP

//==============================================================================
//Library Includes
//==============================================================================
//Include C++ Libraries
#ifdef __cplusplus
using namespace std;	//Allow use of the standard namespace without explicit declaration
#endif

//*** These are at the project level above ***
//The following define allows compilation or everything relating to SeaTrac daignotic log messages and events
//#define SEATRAC_EN_DIAG
//The following define allows the use of Mutex objects to be inlucded in the command execution routines
//#define SEATRAC_EN_MUTEX

//------------------------------------------------------------------------------
//System libraries
#include <math.h>
#include <stdarg.h>		//For variable length arguments in functions
#include <string.h>		//For string manipulation functions

//------------------------------------------------------------------------------
//Base libraries
#include "common.h"

//------------------------------------------------------------------------------
#ifdef SEATRAC_EN_DIAG
    PRAGMA_INFO("SeaTrac Diagnostic Events and Messages ENABLED.")
#else
    PRAGMA_INFO("SeaTrac Diagnostic Events and Messages DISABLED.")
#endif

#ifdef SEATRAC_EN_MUTEX
    PRAGMA_INFO("SeaTrac Mutex Support ENABLED.")

    #define SEATRAC_MUTEX_CMD_LOCK          _mutexCmd.Lock()
    #define SEATRAC_MUTEX_CMD_UNLOCK        _mutexCmd.Unlock()

    #define SEATRAC_MUTEX_DIAGLOG_LOCK      _mutexDiagLog.Lock()
    #define SEATRAC_MUTEX_DIAGLOG_UNLOCK    _mutexDiagLog.Unlock()

    #define SEATRAC_MUTEX_EXE_LOCK          _mutexExe.Lock()
    #define SEATRAC_MUTEX_EXE_UNLOCK        _mutexExe.Unlock()

#else
    PRAGMA_INFO("SeaTrac Mutex Support DISABLED.")

    #define SEATRAC_MUTEX_CMD_LOCK          {}
    #define SEATRAC_MUTEX_CMD_UNLOCK        {}

    #define SEATRAC_MUTEX_DIAGLOG_LOCK      {}
    #define SEATRAC_MUTEX_DIAGLOG_UNLOCK    {}

    #define SEATRAC_MUTEX_EXE_LOCK          {}
    #define SEATRAC_MUTEX_EXE_UNLOCK        {}
#endif

//------------------------------------------------------------------------------
//System libraries
#include "events.hpp"

#ifdef SEATRAC_EN_DIAG
#include "ticks.hpp"
#endif

#ifdef SEATRAC_EN_MUTEX
#include "mutex.hpp"
#endif

//------------------------------------------------------------------------------
//SeaTrac Libraries
#include "seatrac_types.h"
#include "seatrac_utils.hpp"
#include "seatrac_cmdmsg.hpp"
#include "seatrac_cmdproc.hpp"

#ifdef SEATRAC_EN_DIAG
#include "seatrac_diagmsg.hpp"
#endif

//==============================================================================
//Event Definitions...
//==============================================================================
typedef CEvent0 CSeatracEvent;

//------------------------------------------------------------------------------
typedef CEvent1<PSeatracCmdStatusParams> CSeatracCmdStatusEvent;

//------------------------------------------------------------------------------
typedef CEvent1<PSeatracCfgBeaconAckParams> CSeatracCfgBeaconAckEvent;

typedef CEvent1<PSeatracCfgBeaconRespParams> CSeatracCfgBeaconRespEvent;
		
//------------------------------------------------------------------------------
typedef CEvent1<PSeatracDatSendParams> CSeatracDatSendEvent;

typedef CEvent1<PSeatracDatReceiveParams> CSeatracDatReceiveEvent;

typedef CEvent1<PSeatracDatQueueSetParams> CSeatracDatQueueSetEvent;

typedef CEvent1<PSeatracDatQueueClrParams> CSeatracDatQueueClrEvent;

typedef CEvent1<PSeatracDatQueueStatusParams> CSeatracDatQueueStatusEvent;

//------------------------------------------------------------------------------
#ifdef SEATRAC_EN_DIAG
/*! Define an event to handle DiagLog messages from the Seatrac engine */
typedef CEvent1<PSeatracDiagLogParams> CSeatracDiagLogEvent;
#endif

//------------------------------------------------------------------------------
typedef CEvent1<PSeatracEchoSendParams> CSeatracEchoSendEvent;

typedef CEvent1<PSeatracEchoRequestParams> CSeatracEchoRequestEvent;

typedef CEvent1<PSeatracEchoResponseParams> CSeatracEchoResponseEvent;

//------------------------------------------------------------------------------
/*!
Define a structure that is passed to methods/functions that are registered to
handle commands. The struct contains necessary pointers/info to allow command execution
*/
struct TSeatracExecuteParams {
    PSeatracCmdMsg Msg;                     /*!< Pointer to the message object where the decoded payload to be executed (without checksum) is stored. The read pointer will point to the first byte of paramter data after the command code (which has already been decoded) */
    ESeatracCmdId Id;                       /*!< The ID code of the message being handled */
    ESeatracExecuteStatus Status;           /*!< The status code of the command - if set to anything other that CST_OK and OnError event may be raised */
    bool Handled;                           /*!< Return flag that should be set to true to indicate the message was handled */
};

typedef TSeatracExecuteParams* PSeatracExecuteParams;

/*! Define an event type to handle decodeing of raw messages from the command processor */
typedef CEvent1<PSeatracExecuteParams> CSeatracExecuteEvent;

/*! Define an event type to handle sending of decoding status information from the command processor */
typedef CEvent1<PSeatracExecuteDoneParams> CSeatracExecuteDoneEvent;

//------------------------------------------------------------------------------
typedef CEvent1<PSeatracNavQuerySendParams> CSeatracNavQuerySendEvent;

typedef CEvent1<PSeatracNavQueryRequestParams> CSeatracNavQueryRequestEvent;

typedef CEvent1<PSeatracNavQueryResponseParams> CSeatracNavQueryResponseEvent;

typedef CEvent1<PSeatracNavQueueSetParams> CSeatracNavQueueSetEvent;

typedef CEvent1<PSeatracNavQueueClrParams> CSeatracNavQueueClrEvent;

typedef CEvent1<PSeatracNavQueueStatusParams> CSeatracNavQueueStatusEvent;

typedef CEvent1<PSeatracNavStatusSendParams> CSeatracNavStatusSendEvent;

typedef CEvent1<PSeatracNavStatusReceiveParams> CSeatracNavStatusReceiveEvent;

//------------------------------------------------------------------------------
typedef CEvent1<PSeatracPingSendParams> CSeatracPingSendEvent;

typedef CEvent1<PSeatracPingRequestParams> CSeatracPingRequestEvent;

typedef CEvent1<PSeatracPingResponseParams> CSeatracPingResponseEvent;

//------------------------------------------------------------------------------
typedef CEvent1<PSeatracSettingsGetParams> CSeatracSettingsGetEvent;

//------------------------------------------------------------------------------
typedef CEvent1<PSeatracStatusParams> CSeatracStatusEvent;

typedef CEvent1<PSeatracStatusCfgGetParams> CSeatracStatusCfgGetEvent;

//------------------------------------------------------------------------------
typedef CEvent1<PSeatracSysAliveParams> CSeatracSysAliveEvent;

typedef CEvent1<PSeatracSysInfoParams> CSeatracSysInfoEvent;

//------------------------------------------------------------------------------
typedef CEvent1<PSeatracXcvrAnalyseParams> CSeatracXcvrAnalyseEvent;

typedef CEvent1<PSeatracXcvrFixParams> CSeatracXcvrFixEvent;

typedef CEvent1<PSeatracXcvrMsgFixParams> CSeatracXcvrMsgFixEvent;

typedef CEvent1<PSeatracXcvrMsgParams> CSeatracXcvrMsgEvent;

typedef CEvent1<PSeatracXcvrRxErrorParams> CSeatracXcvrRxErrorEvent;

typedef CEvent1<PSeatracXcvrStatusParams> CSeatracXcvrStatusEvent;

typedef CEvent1<PSeatracXcvrTxMsgCtrlSetParams> CSeatracXcvrTxMsgCtrlSetEvent;

typedef CEvent1<PSeatracXcvrUsblParams> CSeatracXcvrUsblEvent;

//==============================================================================
//Class Definitions...
//==============================================================================
class CSeatrac {
    protected:
        //Private Members
        PSeatracCmdProc _cmdProc;   /*!< Command processor (encode/decoder) object */

        #ifdef SEATRAC_EN_DIAG
        uint32 _diagLogFlags;       /*!< Flags that specify what diagnostic logging is allowed */
        string _diagName;
        #endif

        #ifdef SEATRAC_EN_MUTEX
        CMutex _mutexCmd;
        #ifdef SEATRAC_EN_DIAG
        CMutex _mutexDiagLog;
        #endif
        CMutex _mutexExe;           /*!< Mutex that only allows one thread access to the execution engine at a time */
        #endif

        //Private Methods
        virtual void CmdProcDecodeLineEvent(pointer context, PSeatracCmdDecodeLineParams params);
        virtual void CmdProcDecodeMsgEvent(pointer context, PSeatracCmdDecodeMsgParams params);
        virtual void CmdProcEncodeEvent(pointer context, PSeatracCmdEncodeParams params);
        #ifdef SEATRAC_EN_DIAG
        void DoDiagLog(CSeatracDiagMsg& diagMsg);
        #endif
        virtual void ExecuteCmd(PSeatracExecuteParams exe);
        void ExecuteCmdCalAction(PSeatracExecuteParams exe);
        void ExecuteCmdCalAhrsGet(PSeatracExecuteParams exe);
        void ExecuteCmdCalAhrsSet(PSeatracExecuteParams exe);		
		void ExecuteCmdCfgBeaconGet(PSeatracExecuteParams exe);
		void ExecuteCmdCfgBeaconSet(PSeatracExecuteParams exe);
		void ExecuteCmdCfgBeaconResp(PSeatracExecuteParams exe);		
        void ExecuteCmdDatError(PSeatracExecuteParams exe);
        void ExecuteCmdDatQueueSet(PSeatracExecuteParams exe);
        void ExecuteCmdDatQueueClr(PSeatracExecuteParams exe);
        void ExecuteCmdDatQueueStatus(PSeatracExecuteParams exe);
        void ExecuteCmdDatReceive(PSeatracExecuteParams exe);
        void ExecuteCmdDatSend(PSeatracExecuteParams exe);
        void ExecuteCmdEchoError(PSeatracExecuteParams exe);
        void ExecuteCmdEchoReq(PSeatracExecuteParams exe);
        void ExecuteCmdEchoResp(PSeatracExecuteParams exe);
        void ExecuteCmdEchoSend(PSeatracExecuteParams exe);
        void ExecuteCmdNavError(PSeatracExecuteParams exe);
        void ExecuteCmdNavQuerySend(PSeatracExecuteParams exe);
        void ExecuteCmdNavQueryReq(PSeatracExecuteParams exe);
        void ExecuteCmdNavQueryResp(PSeatracExecuteParams exe);
        void ExecuteCmdNavQueueSet(PSeatracExecuteParams exe);
        void ExecuteCmdNavQueueClr(PSeatracExecuteParams exe);
        void ExecuteCmdNavQueueStatus(PSeatracExecuteParams exe);
        void ExecuteCmdNavStatusReceive(PSeatracExecuteParams exe);
        void ExecuteCmdNavStatusSend(PSeatracExecuteParams exe);
        void ExecuteCmdPingError(PSeatracExecuteParams exe);
        void ExecuteCmdPingReq(PSeatracExecuteParams exe);
        void ExecuteCmdPingResp(PSeatracExecuteParams exe);
        void ExecuteCmdPingSend(PSeatracExecuteParams exe);
        void ExecuteCmdProgBlock(PSeatracExecuteParams exe);
        void ExecuteCmdProgInit(PSeatracExecuteParams exe);
        void ExecuteCmdProgUpdate(PSeatracExecuteParams exe);
        void ExecuteCmdSettingsGet(PSeatracExecuteParams exe);
        void ExecuteCmdSettingsLoad(PSeatracExecuteParams exe);
        void ExecuteCmdSettingsReset(PSeatracExecuteParams exe);
        void ExecuteCmdSettingsSave(PSeatracExecuteParams exe);
        void ExecuteCmdSettingsSet(PSeatracExecuteParams exe);
        void ExecuteCmdStatus(PSeatracExecuteParams exe);
        void ExecuteCmdStatusCfgGet(PSeatracExecuteParams exe);
        void ExecuteCmdStatusCfgSet(PSeatracExecuteParams exe);
        void ExecuteCmdSysAlive(PSeatracExecuteParams exe);
        void ExecuteCmdSysInfo(PSeatracExecuteParams exe);
        void ExecuteCmdSysReboot(PSeatracExecuteParams exe);
        void ExecuteCmdXcvrAnalyse(PSeatracExecuteParams exe);
        void ExecuteCmdXcvrFix(PSeatracExecuteParams exe);
        void ExecuteCmdXcvrRxErr(PSeatracExecuteParams exe);
        void ExecuteCmdXcvrRxMsg(PSeatracExecuteParams exe);
        void ExecuteCmdXcvrRxReq(PSeatracExecuteParams exe);
        void ExecuteCmdXcvrRxResp(PSeatracExecuteParams exe);
        void ExecuteCmdXcvrRxUnhandled(PSeatracExecuteParams exe);
        void ExecuteCmdXcvrStatus(PSeatracExecuteParams exe);
        void ExecuteCmdXcvrTxMsg(PSeatracExecuteParams exe);
		void ExecuteCmdXcvrTxMsgCtrlSet(PSeatracExecuteParams exe);		
        void ExecuteCmdXcvrUsbl(PSeatracExecuteParams exe);

        static uint32 _classCnt;
        static double _classTimebase;         /*!< Timestamp when the object was created (in seconds) */

    public:
        //Construction & Disposal
        CSeatrac();
        virtual ~CSeatrac();

        //Methods
        void CmdDecode(string& str);
        void CmdDecode(puint8 data, uint16 len);
        void CmdExecute(PSeatracCmdMsg msg);
        void CmdExecute(puint8 data, uint16 len);
        void CmdReset();

        #ifdef SEATRAC_EN_DIAG
        uint32 DiagLogFlagsGet();
        void DiagLogFlagsSet(uint32 value);
        void DiagNameSet(string& value);
        void DiagNameSet(pchar buf);
        #endif

        void SetDecodeCsumEnable(bool value);
        void SetDecodeIgnoreBadChars(bool value);

        bool CalAction(ESeatracCalAction action);
		bool CfgBeaconGet(uint16 destSerialNumber);
		bool CfgBeaconSet(uint16 destSerialNumber, ESeatracBeaconId beaconId, uint16 rangeTimeout, uint16 responseTurnaround);
        bool DatSend(ESeatracBeaconId destId, ESeatracAcoMsgType msgType, puint8 data, uint8 dataLen);
        bool DatQueueSet(ESeatracBeaconId destId, puint8 data, uint8 dataLen);
        bool DatQueueClear(ESeatracBeaconId destId);
        bool DatQueueStatus();
        bool EchoSend(ESeatracBeaconId destId, ESeatracAcoMsgType msgType, puint8 data, uint8 dataLen);
        bool NavQuerySend(ESeatracBeaconId destId, uint8 queryFlags, puint8 data, uint8 dataLen);
        bool NavQueueSet(ESeatracBeaconId destId, puint8 data, uint8 dataLen);
        bool NavQueueClear(ESeatracBeaconId destId);
        bool NavQueueStatus();
        bool NavStatusSend(ESeatracBeaconId beaconId, puint8 data, uint8 dataLen);
        bool PingSend(ESeatracBeaconId destId, ESeatracAcoMsgType msgType);
        bool SettingsGet();
        bool SettingsSet(TSeatracSettings& settings);
        bool SettingsLoad();
        bool SettingsSave();
        bool SettingsReset();
        bool StatusGet();
        bool StatusGet(uint8 flags);
        bool StatusCfgGet();
        bool StatusCfgSet(ESeatracStatusMode mode, uint8 flags);
        bool SysAlive();
        bool SysInfo();
        bool SysReboot();
        bool XcvrAnalyseNoise();
        bool XcvrStatus();
		bool XcvrTxMsgCtrlSet(ESeaTracXcvrTxMsgCtrl value);

        //Events
        //Diagnostic Events
        #ifdef SEATRAC_EN_DIAG
        CSeatracDiagLogEvent OnDiagLog;
        #endif
        //Command Message Decoder, Encoder & Execution Events
        CSeatracCmdDecodeMsgEvent OnCmdDecodeMsg;
        CSeatracCmdDecodeLineEvent OnCmdDecodeLine;
        CSeatracCmdEncodeEvent OnCmdEncode;
        CSeatracExecuteEvent OnCmdExecute;
        CSeatracExecuteDoneEvent OnCmdExecuteDone;
        //System Events
        CSeatracSysAliveEvent OnSysAlive;
        CSeatracSysInfoEvent OnSysInfo;
        CSeatracCmdStatusEvent OnSysReboot;
        //Settings Events
        CSeatracSettingsGetEvent OnSettingsGet;
        CSeatracCmdStatusEvent OnSettingsSet;
        CSeatracCmdStatusEvent OnSettingsLoad;
        CSeatracCmdStatusEvent OnSettingsSave;
        CSeatracCmdStatusEvent OnSettingsReset;
        //Status Event
        CSeatracStatusEvent OnStatus;
        CSeatracStatusCfgGetEvent OnStatusCfgGet;
        CSeatracCmdStatusEvent OnStatusCfgSet;
        //Calibration Events
        CSeatracCmdStatusEvent OnCalAction;
        //Acoustic Transceiver Events
        CSeatracXcvrAnalyseEvent OnXcvrAnalyseNoise;
        CSeatracXcvrFixEvent OnXcvrFix;
        CSeatracXcvrRxErrorEvent OnXcvrRxError;
        CSeatracXcvrMsgFixEvent OnXcvrRxMsg;
        CSeatracXcvrMsgFixEvent OnXcvrRxReq;
        CSeatracXcvrMsgFixEvent OnXcvrRxResp;
        CSeatracXcvrMsgEvent OnXcvrRxUnhandled;
        CSeatracXcvrStatusEvent OnXcvrStatus;
        CSeatracXcvrMsgEvent OnXcvrTxMsg;
        CSeatracXcvrUsblEvent OnXcvrUsbl;
		CSeatracXcvrTxMsgCtrlSetEvent OnXcvrTxMsgCtrlSet;
        //PING Protocol Events
        CSeatracPingSendEvent OnPingSend;
        CSeatracPingRequestEvent OnPingRequest;
        CSeatracPingResponseEvent OnPingResponse;
        //ECHO Protocol Events
        CSeatracEchoSendEvent OnEchoSend;
        CSeatracEchoRequestEvent OnEchoRequest;
        CSeatracEchoResponseEvent OnEchoResponse;
        //DAT Protocol Events
        CSeatracDatSendEvent OnDatSend;
        CSeatracDatReceiveEvent OnDatReceive;
        CSeatracDatQueueSetEvent OnDatQueueSet;
        CSeatracDatQueueClrEvent OnDatQueueClear;
        CSeatracDatQueueStatusEvent OnDatQueueStatus;
        //NAV Protocol Events
        CSeatracNavQuerySendEvent OnNavQuerySend;
        CSeatracNavQueryRequestEvent OnNavQueryRequest;
        CSeatracNavQueryResponseEvent OnNavQueryResponse;
        CSeatracNavQueueSetEvent OnNavQueueSet;
        CSeatracNavQueueClrEvent OnNavQueueClear;
        CSeatracNavQueueStatusEvent OnNavQueueStatus;
        CSeatracNavStatusSendEvent OnNavStatusSend;
        CSeatracNavStatusReceiveEvent OnNavStatusReceive;
		//CFG Protocol Events		
		CSeatracCfgBeaconAckEvent OnCfgBeaconGet;
		CSeatracCfgBeaconAckEvent OnCfgBeaconSet;
		CSeatracCfgBeaconRespEvent OnCfgBeaconResp;
};


/*
Future Firmware To Do...

Support new BNO AHRS device
Support input (on 2nd serial port) for external AHRS - i.e. SBG
(redo calibration struct and settings struct for new AHRS)

Impliment the timing offset value needed by IST etc for using external AHRS

Add in ability to read PressureSensor limits, type and serial number via info or status type message
(this needs adding perhaps into Info or Status struct)

Add version number into settings and status structures.

Combine XcvrStatus message into general Status Message?

For DAT protocol, impliment hardware codes for DAT_RECEIVE_REQ and DAT_RECEIVE_RESP
    instead of just DAT_RECEIVE

Add smart API command that can either just PING when no data is required, or do a data
    transfer if needed, and perhaps a NAV position broadcast after echo received if
    required - this way data buffers will be processed bidirectionally if available,
    all beacons will get pos updates (in lat/long) all automatically.

Add remote query of firmware and settings structure on beacons, and perhaps ability
    to remotely adjust settings (i.e. beacon ID's, response times, range timeouts etc)

Fix BUG in both X010 and X150/X110 firmware that causes a reboot crash to occur when
    Xcvr Tx/Rx diagnostic messages are enabled from SeaTracTools.

Fix BUG in only X010 firmware where received checksums don't seem to be checked - i.e.
    sending a csum of 0000 always passes, whereas on X150/X110 firmware this isn't
    the case.

Consider revising the way protocols handle errors - should there be a CmdMsg output for
    error, like NavError or PingError, or should this just output the appropriate request/response
    event with an error code - after all the Xcvr will output an XcvrError message as well.
    Specifically see CmdNavError where we don't handle the PLOAD error.

Consider concatenating event types for PingSend, EchoSend and NavSend into one handler
    and data struct type.
*/

//!Define a pointer to a SeaTrac class
typedef CSeatrac* PSeatrac;

//==============================================================================
#endif
