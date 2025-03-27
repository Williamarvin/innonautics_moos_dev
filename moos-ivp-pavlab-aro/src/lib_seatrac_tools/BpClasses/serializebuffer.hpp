/*==============================================================================
Class that impliments a byte buffer - a pre-allocated buffer with methods
that allow the sequential addtion of various data types, and a sequential
method to read data types back from the buffer.
Methods provide access to the underlying byte array.
Effectivly this is a Serializer/Deserialiser for bytes onto a buffer.
Userful for getting around compiler "unaligned structure member" warnings, where
packing of data fields into a byte array is required.

04-06-2014 - Created V1.0 of the file based on developed code in other projects (R.Sharphouse)
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef SERIALIZE_BUFFER_HPP
#define SERIALIZE_BUFFER_HPP

//Include system libraries
//#include <string.h>		//For string manipulation functions
#include <new>			//For the 'nothrow' operator with 'new'

//Include common type definitions and macros
#include "common.h"

//Include the serialisation library
#include "serialize.hpp"

//==============================================================================
//Class Definition...
//==============================================================================
//Predeclare the buffer
class CSerializeBuffer;

/*! Declare a pointer to a command buffer */
typedef CSerializeBuffer* PSerializeBuffer;

/*!
Class that implements a buffer of bytes that provides methods to sequentially add
(serialise) data types onto the buffer, and then read them off.
Pointer methods allow access to the serialised array of bytes held in the buffer.
*/
class CSerializeBuffer : public CSerialize {
    protected :
        puint8	_buf;
        uint16	_bufLen;		//The current number of bytes on the buffer
        uint16	_bufSize;		//The size of the buffer
        uint16	_bufRdIdx;		//The current read index from the buffer

    public :
        //Construction and disposal
        CSerializeBuffer();			//Required for some types of inheritance with their own initialisation constructors
        CSerializeBuffer(uint16 size);
        ~CSerializeBuffer();

        //Methods
        uint8& operator[](uint16 idx);
        bool WriteBuffer(PSerializeBuffer buf);
        void Clear();
        puint8 GetBufPtr();
        uint16 GetFree();
        uint16 GetLength() const;
        uint16 GetReadIdx() const;
        puint8 GetReadPtr(uint16 checkLen = 0);
        uint8& GetUint8(uint16 idx);
        void Init(uint16 size);
        uint8 PopUint8();
        uint16 PopUint16();
        uint32 PopUint32();
        bool ReadData(puint8 data, uint16 len);
        void SetLength(uint16 value);
        void SetReadIdx(uint16 value);
        bool WriteData(puint8 data, uint16 len);
};

//==============================================================================
#endif
