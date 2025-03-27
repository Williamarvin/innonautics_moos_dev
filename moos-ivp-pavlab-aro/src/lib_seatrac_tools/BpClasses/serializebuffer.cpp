#include "serializebuffer.hpp"

//==============================================================================
//Class Implementation...
//==============================================================================
//CSerializeBuffer
//==============================================================================
/*!-----------------------------------------------------------------------------
Inheritance may require a default constructor
*/
CSerializeBuffer::CSerializeBuffer()
{
    _buf = NULL;
    this->Init(0);
}

/*!-----------------------------------------------------------------------------
Constructor that initialises a buffer with the specified size
*/
CSerializeBuffer::CSerializeBuffer(uint16 size)
{
    //Initialise the message buffer
    _buf = NULL;
    this->Init(size);
}

/*!-----------------------------------------------------------------------------
*/
CSerializeBuffer::~CSerializeBuffer()
{
    //Free up the buffer usage
    delete[] _buf;
}

/*!-----------------------------------------------------------------------------
*/
uint8& CSerializeBuffer::operator[](uint16 idx)
{
    return _buf[idx];
}

/*!-----------------------------------------------------------------------------
*/
void CSerializeBuffer::Clear()
{
    _bufLen = 0;
    _bufRdIdx = 0;
}

/*!-----------------------------------------------------------------------------
*/
puint8 CSerializeBuffer::GetBufPtr()
{
    return _buf;
}

/*!-----------------------------------------------------------------------------
*/
uint16 CSerializeBuffer::GetFree()
{
    return _bufSize - _bufLen;
}

/*!-----------------------------------------------------------------------------
*/
uint16 CSerializeBuffer::GetLength() const
{
    return _bufLen;
}

/*!-----------------------------------------------------------------------------
*/
uint16 CSerializeBuffer::GetReadIdx() const
{
    return _bufRdIdx;
}

/*!-----------------------------------------------------------------------------
Function that returns a pointer to the current memory location indicated by the
current Read Index position.
If a CheckLen param is specified as larger than zero, the pointer is only
returned if there is this many bytes available, otherwise NULL is returned.
*/
puint8 CSerializeBuffer::GetReadPtr(uint16 checkLen)
{
    if((checkLen == 0) ||(_bufRdIdx <= (_bufLen - checkLen)))
        return &_buf[_bufRdIdx];
    else
        return NULL;
}

/*!-----------------------------------------------------------------------------
*/
uint8& CSerializeBuffer::GetUint8(uint16 idx)
{
    return _buf[idx];
}

/*!-----------------------------------------------------------------------------
Function that is called to initalise the message buffer to the specified size
*/
void CSerializeBuffer::Init(uint16 size)
{
    //If buffer is already allocated, free it
    if(_buf)
        delete[] _buf;

    //Allocate memory for the buffer
    _bufSize = 0;
    if(size > 0) {
        _buf = new (nothrow) uint8[size];
        if(_buf) {
            //Memory was allocated
            _bufSize = size;
        }
    }

    //Clear buffer access pointers
    this->Clear();
}

/*!-----------------------------------------------------------------------------
Function that removes and returns the last Uint8 value from the buffer.
If there is not enought data on the buffer, the functon returns 0
This does not affect the position of the read pointer.
@result The last Uint8 value on the buffer
*/
uint8 CSerializeBuffer::PopUint8()
{
    if(_bufLen >= 1) {
        _bufLen--;
        return _buf[_bufLen];
    }
    else
        return 0;
}

/*!-----------------------------------------------------------------------------
Function that removes and returns the last Uint16 value from the buffer.
If there is not enought data on the buffer, the functon returns 0
This does not affect the position of the read pointer.
@result The last Uint8 value on the buffer
*/
uint16 CSerializeBuffer::PopUint16()
{
    if(_bufLen >= 2) {
        _bufLen -= 2;
        puint16 p = (puint16)&_buf[_bufLen];
        return *p;
    }
    else
        return 0;
}

/*!-----------------------------------------------------------------------------
Function that removes and returns the last Uint16 value from the buffer.
If there is not enought data on the buffer, the functon returns 0
This does not affect the position of the read pointer.
@result The last Uint8 value on the buffer
*/
uint32 CSerializeBuffer::PopUint32()
{
    if(_bufLen >= 4) {
        _bufLen -= 4;
        puint32 p = (puint32)&_buf[_bufLen];
        return *p;
    }
    else
        return 0;
}

/*!-----------------------------------------------------------------------------
*/
bool CSerializeBuffer::ReadData(puint8 data, uint16 len)
{
    /*
    if(_bufRdIdx <= (_bufLen - len)) {
        for(uint16 idx = 0; idx < len; idx++) {
            *data = _buf[_bufRdIdx];
            _bufRdIdx++;
            data++;
        }
        return true;
    }
    else {
        return false;
    }
    */

    if(data) {
        for(uint16 idx = 0; idx < len; idx++) {
            if(_bufRdIdx >= _bufLen)
                return false;
            *data = _buf[_bufRdIdx];
            _bufRdIdx++;
            data++;
        }
        return true;
    }
    else {
        return false;
    }
}

/*!-----------------------------------------------------------------------------
Function used to set the length of the buffer.
This function is applicable if the buffer is being filled from a source such
as memory stream etc (via pointers using the GetPtr method), that then needs to
be deserialised.
*/
void CSerializeBuffer::SetLength(uint16 value)
{
    //Set the buffer length
    if(value < _bufSize)
        _bufLen = value;
    else
        _bufLen = _bufSize;

    //Reset the read pointer to the start of the buffer
    this->SetReadIdx(0);
}

/*!-----------------------------------------------------------------------------
*/
void CSerializeBuffer::SetReadIdx(uint16 value)
{
    if(value < _bufLen)
        _bufRdIdx = value;
    else
        _bufRdIdx = _bufLen;
}

/*!-----------------------------------------------------------------------------
Appends the contents of the specified buffer onto this one
*/
bool CSerializeBuffer::WriteBuffer(PSerializeBuffer buf)
{
    puint8 data = buf->GetBufPtr();
    uint16 len = GetLength();
    return this->WriteData(data, len);
}

/*!-----------------------------------------------------------------------------
Function that adds an array of data ont the END of the buffer.
@result True if the array was added, false if the array could no be added as there is insufficiant room on the buffer
*/
bool CSerializeBuffer::WriteData(puint8 data, uint16 len)
{
    /*
    if(data && (len > 0) && ((_bufLen + len) <= _bufSize)) {
        for(uint16 idx = 0; idx < len; idx++) {
            _buf[_bufLen] = *data;
            _bufLen++;
            data++;
        }
        return true;
    }
    else
        return false;
    */

    if(data) {
        for(uint16 idx = 0; idx < len; idx++) {
            //Abort if buffer is full
            if(_bufLen >= _bufSize)
                return false;
            //Add next byte onto buffer
            _buf[_bufLen] = *data;
            _bufLen++;
            data++;
        }
        return true;
    }
    else
        return false;
}

//==============================================================================

