/*==============================================================================
Class that implements a byte buffer - a preallocated buffer with methods
that allow the sequential addition of various data types, and a sequential
method to read data types back from the buffer.
Methods provide access to the underlying byte array.
Effectively this is a Serialiser/Deserialiser for bytes onto a buffer.
Useful for getting around compiler "unaligned structure member" warnings, where
packing of data fields into a byte array is required.

04-06-2014 - Created V1.0 of the file based on developed code in other projects (R.Sharphouse)
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef SERIALIZE_HPP
#define SERIALIZE_HPP

//Include system libraries
//#include <string.h>		//For string manipulation functions
//#include <new>			//For the 'nothrow' operator with 'new'

//Include common type definitions and macros
#include "common.h"
#include "conversion.hpp"

//==============================================================================
//Class Definition...
//==============================================================================
/*!
Abstract class from which serialization objects should inherit, and implement
the AddData and ReadData methods.
*/
class CSerialize {
    protected :

    public :
        //Disposal
        virtual ~CSerialize() {}

        //Methods
        template <typename T>
        bool ReadArray(T* array, uint32 elements, uint32 elementSize);

        virtual bool ReadData(puint8 data, uint16 len) = 0;

        template <typename T>
        bool Read(T* value);

        template <typename T>
        bool Read(T* value, const T defValue);

        template <typename T>
        bool WriteArray(T* array, uint32 elements);

        virtual bool WriteData(puint8 data, uint16 len) = 0;

        template <typename T>
        bool Write(T value);

        template <typename T>
        bool Write(T* value);

    public :
        //Explicit methods to add values - no real need, except
        //used in code to make easier to read serialisation sizes.

        bool ReadBool(pbool value);
        bool ReadBool(pbool value, bool defValue);
        bool ReadDouble(pdouble value);
        bool ReadDouble(pdouble value, double defValue);
        bool ReadFloat(pfloat value);
        bool ReadFloat(pfloat value, float defValue);
        bool ReadInt8(pint8 value);
        bool ReadInt8(pint8 value, int8 defValue);
        bool ReadInt16(pint16 value);
        bool ReadInt16(pint16 value, int16 defValue);
        bool ReadInt32(pint32 value);
        bool ReadInt32(pint32 value, int32 defValue);
        bool ReadInt64(pint64 value);
        bool ReadInt64(pint64 value, int64 defValue);
        bool ReadStringZ(string& value, uint16 maxChars = 0);
        bool ReadStringZ(puint8 buf, uint16 maxChars = 0);
        bool ReadUint8(puint8 value);
        bool ReadUint8(puint8 value, uint8 defValue);
        bool ReadUint16(puint16 value);
        bool ReadUint16(puint16 value, uint16 defValue);
        bool ReadUint32(puint32 value);
        bool ReadUint32(puint32 value, uint32 defValue);
        bool ReadUint64(puint64 value);
        bool ReadUint64(puint64 value, uint64 defValue);

        bool WriteBool(bool value);
        bool WriteDouble(double value);
        bool WriteFloat(float value);
        bool WriteHexData(puint8 data, uint16 len);
        bool WriteHexUint8(uint8 data);
        bool WriteHexUint16(uint16 data);
        bool WriteHexUint32(uint32 data);
        bool WriteInt8(int8 value);
        bool WriteInt16(int16 value);
        bool WriteInt32(int32 value);
        bool WriteInt64(int64 value);
        bool WriteStringZ(const string& value, uint16 maxChars = 0);
        bool WriteStringZ(puint8 buf, uint16 maxChars = 0);
        bool WriteUint8(uint8 value);
        bool WriteUint16(uint16 value);
        bool WriteUint32(uint32 value);
        bool WriteUint64(uint64 value);
};

typedef CSerialize* PSerialize;

//==============================================================================
//==============================================================================
/*! Define the Interface that can serialise its contents to
a serialisation object, or deserialise its contents back from the object.
*/
class ISerializable {
    public :
        virtual ~ISerializable() {}

        //Method for serialising or deserializing the struct
        virtual bool Deserialize(PSerialize serialize) = 0;		/*!< Deserialise the struct contents from the serialise object */
        virtual bool Serialize(PSerialize serialize) = 0;		/*!< Serialise the struct to the serialise object */
};

//==============================================================================
//Class Implementation
//==============================================================================
/*!-----------------------------------------------------------------------------
Template function the adds a value of the specified type into the serialiser
by calling the overloadable method AddData.
*/
template <typename T>
bool CSerialize::Write(T value)
{
    return this->WriteData((puint8)&value, sizeof(T));
}

/*!-----------------------------------------------------------------------------
Template function the adds a value of the specified type into the serialiser
by calling the overloadable method AddData.
*/
template <typename T>
bool CSerialize::Write(T* value)
{
    return this->WriteData((puint8)value, sizeof(T));
}

/*!-----------------------------------------------------------------------------
Specialisation of the Add template function to write bools as 0xFF or 0x00
for True and False.
*/
template <>
inline bool CSerialize::Write<bool>(bool value)
{
    uint8 tmp = (value) ? 0xFF : 0x00;
    return this->WriteData(&tmp, sizeof(uint8));
}

/*!-----------------------------------------------------------------------------
Function that adds a specified number of elements from an array onto the serializer,
using the appropriate type serializer
*/
template <typename T>
bool CSerialize::WriteArray(T* array, uint32 elements)
{
    bool success = true;
    for(uint32 idx = 0; idx < elements; idx++) {
        success &= this->Write(array);
        array++;
    }
    return success;
}

/*!-----------------------------------------------------------------------------
Template function the reads a value of the specified type from the serialiser
by calling the overloadable method ReadData.
*/
template <typename T>
bool CSerialize::Read(T* value)
{
    return this->ReadData((puint8)value, sizeof(T));
}

/*!-----------------------------------------------------------------------------
Specialisation of the Read template function to return a boolean true for a non
zero value read.
*/
template <>
inline bool CSerialize::Read<bool>(bool* value)
{
    uint8 tmp;
    bool success = this->ReadData(&tmp, sizeof(uint8));
    *value = (tmp != 0);
    return success;
}

/*!-----------------------------------------------------------------------------
Template function the reads a value of the specified type from the serialiser
by calling the overloadable method ReadData.
However, if the read fails (not enough data), then the default value is instead
copied to the result value.
*/
template <typename T>
bool CSerialize::Read(T* value, const T defValue)
{
    bool success = this->Read(value);
    if(!success)
        *value = defValue;
    return success;
}

/*!-----------------------------------------------------------------------------
Function that reads a specified number of elements from the serializer onto an array,
using the appropriate type serializer
*/
template <typename T>
bool CSerialize::ReadArray(T* array, uint32 elements, uint32 elementSize)
{
    puint8 ptr = (puint8)array;
    bool success = true;
    for(uint32 idx = 0; idx < elements; idx++) {
        success &= this->ReadData(ptr, elementSize);
        ptr += elementSize;
    }
    return success;
}

//==============================================================================
#endif
