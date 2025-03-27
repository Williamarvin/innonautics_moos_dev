#include "serialize.hpp"

//==============================================================================
//Class Implementation
//==============================================================================
/*!-----------------------------------------------------------------------------
As the specialisation of the Add<bool> template must be defined first,
Implement the function body for using it here after the definition.
*/
bool CSerialize::WriteBool(bool value)
{
	return this->Write(value);
}

/*!-----------------------------------------------------------------------------
As the specialisation of the Read<bool> template must be defined first,
Implement the function body for using it here after the definition.
*/
bool CSerialize::ReadBool(bool* value)
{
	return this->Read(value);
}

//------------------------------------------------------------------------------
bool CSerialize::WriteDouble(double value) { return this->Write(value); }
bool CSerialize::WriteFloat(float value) { return this->Write(value); }
bool CSerialize::WriteInt8(int8 value) { return this->Write(value); }
bool CSerialize::WriteInt16(int16 value) { return this->Write(value); }
bool CSerialize::WriteInt32(int32 value) { return this->Write(value); }
bool CSerialize::WriteInt64(int64 value) { return this->Write(value); }
bool CSerialize::WriteUint8(uint8 value) { return this->Write(value); }
bool CSerialize::WriteUint16(uint16 value) { return this->Write(value); }
bool CSerialize::WriteUint32(uint32 value) { return this->Write(value); }
bool CSerialize::WriteUint64(uint64 value) { return this->Write(value); }

bool CSerialize::ReadBool(pbool value, bool defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadDouble(pdouble value) { return this->Read(value); }
bool CSerialize::ReadDouble(pdouble value, double defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadFloat(pfloat value) { return this->Read(value); }
bool CSerialize::ReadFloat(pfloat value, float defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadInt8(pint8 value) { return this->Read(value); }
bool CSerialize::ReadInt8(pint8 value, int8 defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadInt16(pint16 value) { return this->Read(value); }
bool CSerialize::ReadInt16(pint16 value, int16 defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadInt32(pint32 value) { return this->Read(value); }
bool CSerialize::ReadInt32(pint32 value, int32 defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadInt64(pint64 value) { return this->Read(value); }
bool CSerialize::ReadInt64(pint64 value, int64 defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadUint8(puint8 value) { return this->Read(value); }
bool CSerialize::ReadUint8(puint8 value, uint8 defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadUint16(puint16 value) { return this->Read(value); }
bool CSerialize::ReadUint16(puint16 value, uint16 defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadUint32(puint32 value) { return this->Read(value); }
bool CSerialize::ReadUint32(puint32 value, uint32 defValue) { return this->Read(value, defValue); }
bool CSerialize::ReadUint64(puint64 value) { return this->Read(value); }
bool CSerialize::ReadUint64(puint64 value, uint64 defValue) { return this->Read(value, defValue); }

/*!-----------------------------------------------------------------------------
Function that adds an Ascii-hex Uint8 encoded character pair.
Most significant nibble is written first.
*/
bool CSerialize::WriteHexUint8(uint8 data)
{
    bool success = true;
    success &= this->WriteUint8(CConversion::GetHexChar(data >> 4));
    success &= this->WriteUint8(CConversion::GetHexChar(data));
    return success;
}

/*!-----------------------------------------------------------------------------
Function that adds an Ascii-hex Uint16 encoded character sequence.
Least significant byte is written first
Most significant nibble is written first of each byte
*/
bool CSerialize::WriteHexUint16(uint16 data)
{
    bool success = true;
    puint8 pdata = (puint8)&data;
    success &= this->WriteHexUint8(*pdata);
    success &= this->WriteHexUint8(*(pdata + 1));
    return success;
}

/*!-----------------------------------------------------------------------------
Function that adds an Ascii-hex Uint32 encoded character sequence.
Least significant byte is written first
Most significant nibble is written first of each byte
*/
bool CSerialize::WriteHexUint32(uint32 data)
{
    bool success = true;
    puint8 pdata = (puint8)&data;
    success &= this->WriteHexUint8(*pdata);
    success &= this->WriteHexUint8(*(pdata + 1));
    success &= this->WriteHexUint8(*(pdata + 2));
    success &= this->WriteHexUint8(*(pdata + 3));
    return success;
}

/*!-----------------------------------------------------------------------------
Function that adds a buffer of Ascii-hex Uint8 encoded character pairs.
Most significant nibble is written first of each byte.
Bytes are written in sequential order of the buffer in.
*/
bool CSerialize::WriteHexData(puint8 data, uint16 len)
{
    bool success = true;
    for(uint16 idx = 0; idx < len; idx++) {
        success &= this->WriteHexUint8(*data);
        data++;
    }
    return success;
}

/*!-----------------------------------------------------------------------------
Adds a stream of null-terminated characters representing the string object.
@param maxChars specifies the maximum number of characters (excluding the Null Terminator) that can be written. A value of zero allows any length
*/
bool CSerialize::WriteStringZ(const string& value, uint16 maxChars)
{
	puint8 buf = (puint8)value.c_str();
	return this->WriteStringZ(buf, maxChars);
}

/*!-----------------------------------------------------------------------------
Adds a stream of null-terminated characters representing the buffer
@param maxChars specifies the maximum number of characters (excluding the Null Terminator) that can be written. A value of zero allows any length
*/
bool CSerialize::WriteStringZ(puint8 buf, uint16 maxChars)
{
	bool success = true;

	while(success) {
		uint8 ch = *buf;

		//Abort if the character is a null
		if(!ch)
			break;

		//Add the character to the buffer
		success &= this->WriteUint8(ch);
		buf++;

		//If MaxLen is bigger than 0, then decrease it and abort at 0
		if(maxChars > 0) {
			maxChars--;
			if(maxChars == 0)
				break;
		}
	}

	//Add the null terminator
	success &= this->WriteUint8(0);

	return success;
}

/*!-----------------------------------------------------------------------------
@param maxChars specifies the maximum number of characters (excluding the Null Terminator) that can be read into the buffer. A value of zero allows any length
*/
bool CSerialize::ReadStringZ(string& value, uint16 maxChars)
{
	uint8 ch;
	bool store = true;
	bool success = true;

	while(success) {
		success = this->ReadUint8(&ch, 0);
		if(success) {
			if(ch) {
				//A valid character, so store

				if(maxChars > 0) {
					maxChars--;
					store = (maxChars > 0);
				}
				if(store) {
					value.append((char*)&ch, 1);
				}
			}
			else {
				//A null terminator so abort
				break;
			}
		}
	}

	return success;
}

/*!-----------------------------------------------------------------------------
@param maxChars specifies the maximum number of characters (excluding the Null Terminator) that can be read into the buffer. A value of zero allows any length
*/
bool CSerialize::ReadStringZ(puint8 buf, uint16 maxChars)
{
	uint8 ch;
	bool store = true;
	bool success = true;

	while(success) {
		success = this->ReadUint8(&ch, 0);
		if(success) {
			if(ch) {
				//A valid character, so store
				if(maxChars > 0) {
					maxChars--;
					store = (maxChars > 0);
				}
				if(store) {
					*buf = ch;
					buf++;
				}
			}
			else {
				//A null terminator so abort
				break;
			}
		}
	}

	return success;
}

//==============================================================================
