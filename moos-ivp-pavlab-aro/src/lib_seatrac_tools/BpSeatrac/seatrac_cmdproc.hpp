/*==============================================================================
File that implements a modular Serial Command Processor designed for operation
at the surface (PC) end of the serial link.

The class includes Mutex's that allow the Decode and Encode functions to
be thread safe - only on thread can decode or encode at a time.

29-11-2013 - Created V1.0 of the file (R.Sharphouse)
30-08-2016 - Modified to v1.1 For SRS Thruster Board, simplified execution calling
20-07-2017 - Modified for use a the surface-end command processor for the SeaTrac SDK
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef SEATRAC_CMD_PROC_HPP
#define SEATRAC_CMD_PROC_HPP

//Include system libraries
#include <stdarg.h>		//For variable length arguments in functions
//#include <string.h>		//For string manipulation functions
//#include <new>			//For the 'nothrow' operator with 'new'
#include <algorithm>    //For min.max functions with templated definitions

//Include common type definitions and macros
#include "common.h"

//Include helper classes
#include "events.hpp"
#include "conversion.hpp"
#include "crc16.hpp"

#include "mutex.hpp"

//Include the Command Processor buffer
//Additional application specific commands should be defined locally in "cmd_defs.h"
#include "seatrac_types.h"
#include "seatrac_cmdmsg.hpp"

//==============================================================================
//Class Definition...
//==============================================================================
//------------------------------------------------------------------------------
//Predeclare the command processor classes
class CSeatracCmdProc;

/*! Define a pointer to a command processor */
typedef CSeatracCmdProc* PSeatracCmdProc;

//------------------------------------------------------------------------------
//! Enumeration specify the status of decoded messages
enum ECmdProcDecodeStatus {
    DECODE_OK = 0,
    DECODE_CHAR_ERROR,
    DECODE_OVERFLOW_ERROR,
    DECODE_CHECKSUM_ERROR,
};

/*!
Define a structure that is passed to methods/functions that are registered to
handle commands. The struct contains necessary pointers/info to allow command execution
*/
struct TSeatracCmdDecodeMsgParams {
    ECmdProcDecodeStatus Status;    /*!< The status of the decoded message */
    PSeatracCmdMsg Msg;             /*!< Pointer to the receive buffer where incoming commands are stored */
    puint8 Buffer;                  /*!< Pointer to data at the start of the message buffer */
    uint32 Length;                  /*!< The length of data in the message buffer */
};

/*! Define a pointer to a shell command structure */
typedef TSeatracCmdDecodeMsgParams* PSeatracCmdDecodeMsgParams;

/*! Define a delegate type to handle execution of shell commands */
typedef CEvent1<PSeatracCmdDecodeMsgParams> CSeatracCmdDecodeMsgEvent;

//------------------------------------------------------------------------------
/*! Define a delegate type to handle execution of shell commands */
typedef CEvent1<PSeatracCmdDecodeLineParams> CSeatracCmdDecodeLineEvent;

//------------------------------------------------------------------------------
/*! Define a delegate type to handle sending of data from the command processor */
typedef CEvent1<PSeatracCmdEncodeParams> CSeatracCmdEncodeEvent;

//------------------------------------------------------------------------------
/*!
Class that implements the base functionality for a serial command processor
*/
class CSeatracCmdProc {
    private:
        uint32          _bufLen;
        char            _decodeHeader;          /*!< Header character look for before starting to decode messages */
        bool            _decodeCsumEn;			/*!< True if receive messages should have a checksum at their end - false to ignore */
        bool            _decodeEn;
        uint8           _decodeNibble;
        PSeatracCmdMsg	_decodeMsg;             /*!< Buffer that holds received data from a message */
        uint8           _decodeValue;
        bool            _decodeIgnoreBadChars;	/*!< True to ignore invalid characters in a message */
        string          _decodeLine;            /*!< String that holds the Ascii readible contents of a line */
        char            _encodeHeader;          /*!< Header character to use at the start of encoded messages */

        CMutex          _mutexDecode;
        CMutex          _mutexEncode;

        //Private methods
        uint16 CalcMsgChecksum(PSeatracCmdMsg msg, uint16 csumInit = 0);
        void DoDecodeChar(uint8 ch);
        void DoDecodeMsg(PSeatracCmdMsg msg, ECmdProcDecodeStatus status);
        bool EncodePrintArgs(pchar str, va_list args);

    public:
        //Construction and disposal
        CSeatracCmdProc(char encodeHeader, char decodeHeader, uint32 bufLen = ST_MSG_LEN);
        ~CSeatracCmdProc();

        //Methods
        uint8 GetAddr();
        void Decode(string& str);
        void Decode(puint8 data, uint16 len);
        void Decode(uint8 ch);
        void DecodeReset();
        bool Encode(PSeatracCmdMsg msg);
        bool Encode(ESeatracCmdId id, puint8 data, uint16 len);
        bool EncodePrint(const string str, ...);
        bool EncodePrint(const pchar str, ...);
        void SetDecodeCsumEnable(bool value);
        void SetDecodeIgnoreBadChars(bool value);

        //Events
        CSeatracCmdDecodeMsgEvent OnDecodeMsg;
        CSeatracCmdDecodeLineEvent OnDecodeLine;
        CSeatracCmdEncodeEvent OnEncode;
};

//==============================================================================
#endif

