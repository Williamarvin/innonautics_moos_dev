#include "seatrac_cmdproc.hpp"

//==============================================================================
//Class Implementation...
//==============================================================================
//CCmdProc
//==============================================================================
/*!-----------------------------------------------------------------------------
Constructor for the class
*/
CSeatracCmdProc::CSeatracCmdProc(char encodeHeader, char decodeHeader, uint32 bufLen)
{
    //Initialise the CRC16 lookup table
    CCrc16::Init(CRC16_GEN_POLY);

    //Setup initial variables
    _decodeHeader = decodeHeader;
    _decodeCsumEn = true;
    _decodeIgnoreBadChars = false;
    _encodeHeader = encodeHeader;
    _bufLen = bufLen;

    //Initialise message decoder
    _decodeMsg = new CSeatracCmdMsg(_bufLen);
    this->DecodeReset();
}

/*!-----------------------------------------------------------------------------
Destructor
*/
CSeatracCmdProc::~CSeatracCmdProc()
{
    //Free up message buffer
    delete _decodeMsg;
}

/*!-----------------------------------------------------------------------------
Function that computes the checksum value for the specified message (which
should include the message id code, but not an existing checksum).
The CRC16-IBM polynomial is used.
*/
uint16 CSeatracCmdProc::CalcMsgChecksum(PSeatracCmdMsg msg, uint16 csumInit)
{
    uint16 csum = 0;
    uint16 len = msg->GetLength();
    puint8 buf = msg->GetBufPtr();
    csum = CCrc16::Calc(buf, 0, len, csumInit);
    return csum;
}

/*!-----------------------------------------------------------------------------
Function that is called to decode a received message in a buffer
@param data The pointer to the buffer to process
@param len  The number of bytes in the buffer to process
*/
void CSeatracCmdProc::Decode(puint8 data, uint16 len)
{
    if(data) {
        _mutexDecode.Lock();
        //Process each character in the buffer
        for(uint32 i = 0; i < len; i++) {
            //Pass the character to the message decoder
            this->DoDecodeChar(*data);
            data++;
        }
        _mutexDecode.Unlock();
    }
}

/*!-----------------------------------------------------------------------------
Function that is called to decode a received message, parsing a string character
at a time
@param str	The string to process
*/
void CSeatracCmdProc::Decode(string& str)
{
    uint32 len = (uint32)str.length();
    puint8 data = (puint8)str.c_str();
    this->Decode(data, len);
}

/*!-----------------------------------------------------------------------------
Function that is called to decode a received message, processing each character
at a time.
@param ch	The character to process
*/
void CSeatracCmdProc::Decode(uint8 ch)
{
    _mutexDecode.Lock();
    this->DoDecodeChar(ch);
    _mutexDecode.Unlock();
}

/*!-----------------------------------------------------------------------------
Function that is called to reset the state of the command processor
*/
void CSeatracCmdProc::DecodeReset()
{
    _mutexDecode.Lock();

    //Reset the command decoder variables
    _decodeEn = false;
    _decodeNibble = 0;
    _decodeValue = 0;

    //Clear the received line string
    _decodeLine.clear();

    _mutexDecode.Unlock();
}

/*!-----------------------------------------------------------------------------
Function that is called to decode a received message, processing each character
at a time.
@param ch	The character to process
*/
void CSeatracCmdProc::DoDecodeChar(uint8 ch)
{
    uint8 hex;	//Temporary values where hex characters are converted to binary values

    if(ch == 10) {
        //Output the recevied line of text
        TSeatracCmdDecodeLineParams params;
        snprintf(params.Buffer, sizeof(params.Buffer), "%s", _decodeLine.c_str());
        params.HasMsg = _decodeEn;
        params.Length = (uint32)_decodeLine.length();
        this->OnDecodeLine.Call(&params);

        _decodeLine.clear();
    }
    else if(_decodeLine.length() < _bufLen && ch >= ' ' && ch < 127) {
        //Add the character to the received line
        //Only add human readible chars
        _decodeLine += ch;
    }

    //Deocde incoming characters into messages
    if(ch == _decodeHeader) {
        //Start decoding a new message
        _decodeEn = true;
        _decodeNibble = 0;
        _decodeValue = 0;
        _decodeMsg->Clear();
    }
    else if(_decodeEn) {
        //Decoding is in progress, so handle the character
        if(ch == 13) {
            //Ignore <CR> characters
        }
        else if(ch == 10) {
            //Process the command on a<LF> (Line-Feed), as this is the last character in
            //the CR+LF sequence on half-duplex.
            uint16 csumRx;
            uint16 csumCalc;

            //Disable further decoding of this command, and clear the received line
            _decodeEn = false;

            //Validate and process the message...
            if(_decodeCsumEn) {
                //If using Rx command checksums, then obtain and validate...
                //Remove the received checksum from the end of the buffer
                csumRx = _decodeMsg->PopUint16();
                //Now calculate the checksum of what we received
                csumCalc = this->CalcMsgChecksum(_decodeMsg);
            }
            else {
                //If we're not using Rx command checksums, set both to 0 so we always execute...
                csumRx = 0;
                csumCalc = 0;
            }

            if(csumRx == csumCalc) {
                //The checksums match, so execute
                this->DoDecodeMsg(_decodeMsg, DECODE_OK);
            }
            else {
                //The checksums do not match, so raise an error
                this->DoDecodeMsg(_decodeMsg, DECODE_CHECKSUM_ERROR);
            }
        }
        else if(!CConversion::HexCharToUint8(ch, &hex)) {
            //An invalid hex char was received, so abort decoding
            if(!_decodeIgnoreBadChars) {
                _decodeEn = false;
                this->DoDecodeMsg(_decodeMsg, DECODE_CHAR_ERROR);
            }
        }
        else if(_decodeNibble == 0) {
            //A valid Hex char was received
            //Store the first hex pair char - the upper nibble
            _decodeValue = hex << 4;
            _decodeNibble = 1;
        }
        else {
            //A valid Hex char was received
            //Store the second hex pair char - the lower nibble
            _decodeValue |= hex;
            _decodeNibble = 0;

            //Store the complete byte on the buffer
            if(!_decodeMsg->WriteUint8(_decodeValue)) {
                //The buffer size has overflowed, so abort decoding
                _decodeEn = false;
                this->DoDecodeMsg(_decodeMsg, DECODE_OVERFLOW_ERROR);
            }
        }
    }
}

/*!-----------------------------------------------------------------------------
Function that executes the command on the specified buffer.
The command code is the first byte on the buffer, followed by the data bytes.
No checksum should be included on the buffer.
When the execute function calls the appropriate delegate functions handler
registers for the message, the message buffer read pointer will be setup to
point to the first byte of data AFTER the message command code.
@param msg	Pointer to the message buffer to execute
*/
void CSeatracCmdProc::DoDecodeMsg(PSeatracCmdMsg msg, ECmdProcDecodeStatus status)
{
    //Create the execute params
    TSeatracCmdDecodeMsgParams params;

    //Set the message reader to the first received byte
    msg->SetReadIdx(0);

    //Populate the message executions params
    params.Status = status;
    params.Msg = msg;
    params.Buffer = msg->GetBufPtr();
    params.Length = msg->GetLength();

    //Call the commands delegate function to execute it
    this->OnDecodeMsg.Call(&params);
}

/*!-----------------------------------------------------------------------------
Function that writes a message out of the command processor, using the specified
message.
@param	msg		Pointer to the message buffer to write
@result true if the OnEncode event handler did not modify Success variable to false.
*/
bool CSeatracCmdProc::Encode(PSeatracCmdMsg msg)
{
    TSeatracCmdEncodeParams params;
    params.Success = false;

    _mutexEncode.Lock();

    //Abort if no message is specified
    if(msg) {
        //Get the message length
        uint16 msgLen = msg->GetLength();

        //Only send the message if contains something
        if(msgLen > 0) {
            //Create a new serialisation buffer for the data to send
            uint32 bufChars = 1 + ((msgLen + 2) * 2) + 2 + (1);
            PSerializeBuffer sendBuf = new (std::nothrow) CSerializeBuffer(bufChars);

            if(sendBuf) {
                //Initialise the checksum
                uint16 csum = 0;

                //Get the buffer pointer
                puint8 msgBuf = msg->GetBufPtr();

                //Write a header character
                sendBuf->WriteUint8(_encodeHeader);

                //Write the message buffer (Id and payload)
                sendBuf->WriteHexData(msgBuf, msgLen);

                //Compute and write the message checksum
                csum = this->CalcMsgChecksum(msg, csum);
                sendBuf->WriteHexUint16(csum);

                //Write a <CR><LF> character pair
                sendBuf->WriteUint8(13);
                sendBuf->WriteUint8(10);

                //Populate the message executions params
                uint32 len = min((uint32)sizeof(params.Buffer), (uint32)sendBuf->GetLength());
                memcpy(params.Buffer, sendBuf->GetBufPtr(), len);
                params.Length = len;
                params.Success = true;  //This could be modified by the even handler

                //Raise the OnSend callback with the data
                this->OnEncode.Call(&params);

                //Tidy up
                delete sendBuf;
            }
        }
    }

    _mutexEncode.Unlock();

    //Return the success code
    return params.Success;
}

/*!-----------------------------------------------------------------------------
Function that writes a message out of the command processor, using the specified
id code, and data from the specified buffer
@param	id		The message identification code to use
@param	data	Pointer to where the message payload data is stored
@param	len		The number of bytes of data to write
@result true if the OnEncode event handler did not modify Success variable to false.
*/
bool CSeatracCmdProc::Encode(ESeatracCmdId id, puint8 data, uint16 len)
{
    //Make up a message
    CSeatracCmdMsg msg(id, data, len);

    //Send the message
    return this->Encode(&msg);
}

/*!-----------------------------------------------------------------------------
Function that implements a PrintF like capability to write text to the uart
output
@result true if the OnEncode event handler did not modify Success variable to false.
*/
bool CSeatracCmdProc::EncodePrint(const string str, ...)
{
    //Initialise the arguments list
    va_list args;
    va_start(args, str);

    //Call the print function for the arguments list
    bool success = this->EncodePrintArgs((pchar)str.c_str(), args);

    //Tidy up
    va_end(args);

    return success;
}

/*!-----------------------------------------------------------------------------
Function that implements a PrintF like capability to write text to the uart
output
@result true if the OnEncode event handler did not modify Success variable to false.
*/
bool CSeatracCmdProc::EncodePrint(const pchar str, ...)
{
    //Initialise the arguments list
    va_list args;
    va_start(args, str);

    //Call the print function for the arguments list
    bool success = this->EncodePrintArgs(str, args);

    //Tidy up
    va_end(args);

    return success;
}

/*!-----------------------------------------------------------------------------
Function that implements a PrintF like capability to write text to the uart
output
@param str Pointer to a null-terminated Format string of characters
@param args The variable args structure containing variables to insert in the format string
@result true if the OnEncode event handler did not modify Success variable to false.
*/
bool CSeatracCmdProc::EncodePrintArgs(pchar str, va_list args)
{
    _mutexEncode.Lock();

    //Populate the message executions params
    TSeatracCmdEncodeParams params;
    vsnprintf(params.Buffer, sizeof(params.Buffer), str, args);
    params.Length = (uint32)strlen(params.Buffer);
    params.Success = true;  //The OnEncode event handler can modify this to fail (i.e. if passing to a serial port etc)

    //Raise the OnSend callback with the data
    this->OnEncode.Call(&params);

    _mutexEncode.Unlock();

    //Return the success code (that can be modified by the event handler)
    return params.Success;
}

/*!-----------------------------------------------------------------------------
Function that determines weather the receiver process the checksum at the end
of the message
*/
void CSeatracCmdProc::SetDecodeCsumEnable(bool value)
{
    _mutexDecode.Lock();
    _decodeCsumEn = value;
    _mutexDecode.Unlock();
}

/*!-----------------------------------------------------------------------------
Function that determines weather the receiver ignores invalid Hex chars while
decoding a message. If bad chars arn't ignored, then message decoding aborts
with an error
*/
void CSeatracCmdProc::SetDecodeIgnoreBadChars(bool value)
{
    _mutexDecode.Lock();
    _decodeIgnoreBadChars = value;
    _mutexDecode.Unlock();
}

//==============================================================================
