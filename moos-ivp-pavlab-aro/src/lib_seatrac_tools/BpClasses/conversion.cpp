#include "conversion.hpp"

//==============================================================================
//Class Implementation...
//==============================================================================
//CConversion
//==============================================================================
/*!-----------------------------------------------------------------------------
Define a lookup table for quickly converting binary nibbles to ascii hex characters
*/
char CConversion::_hexChars[17] = "0123456789ABCDEF";	

/*!-----------------------------------------------------------------------------
Function that converts the specified nibble from an UInt into an ASCII hex
character.
@param value	The uint from which to extract and convert the nibble
*/
char CConversion::GetHexChar(uint8 value)
{		
	return CConversion::_hexChars[(uint8)(value & 0x0Fu)];
}

/*!-----------------------------------------------------------------------------
Function that converts the specified nibble from an UInt into an ASCII hex
character.
@param value	The uint from which to extract and convert the nibble
@param nibble	The nibble index (0 to 15) to convert.
*/
char CConversion::GetHexChar(uint32 value, uint8 nibble)
{
	//Check the nibble is valid	in the range 0 to 15
	nibble &= 0xF;	
	//Extract and mask the appropriate nibble
	uint8 val = (value >> (nibble * 4)) & 0xF;
	//Obtain the hex character for the nibble
	return CConversion::_hexChars[val];
}

/*!-----------------------------------------------------------------------------
Function that converts a hexadecimal character (0-9, A-F) in any chase to a
uint 8 value from 0 to 15
@param chVal 	The hexadecimal character to conver
@param binVal	Pointer to where the converted value should be stored
@result True if the conversion was sucessful, otherwise false
*/
bool CConversion::HexCharToUint8(char chVal, puint8 binVal)
{
	if(chVal >= '0' && chVal <= '9') {
		*binVal = (uint8)(chVal - '0');
		return true;
	}
	else if(chVal >= 'A' && chVal <= 'F') {				
		*binVal = (uint8)(chVal - 'A' + 10);
		return true;
	}
	else if(chVal >= 'a' && chVal <= 'f') {
		*binVal = (uint8)(chVal - 'a' + 10);
		return true;
	}
	else {
		//Unable to make the conversion
		return false;
	}	
}

/*!-----------------------------------------------------------------------------
Function that contains a sequence of 2 hex characters into a Unit8 value
@param buf		Pointer the the sequence of 2 characters to convert
@param value	Pointer to where the converted result should be stored
@result True if the conversion completed successfully, false if the value could not be converted
*/
bool CConversion::HexCharsToUint8(pchar buf, puint8 value)
{
	uint8 result;
	uint8 tmp;
	
	if(!CConversion::HexCharToUint8(*buf, &tmp))
		return false;
	result = tmp << 4;	
	buf++;
	if(!CConversion::HexCharToUint8(*buf, &tmp))
		return false;
	result |= tmp;
	*value = result;
	return true;	
}


/*!-----------------------------------------------------------------------------
*/
bool CConversion::HexCharsToUint8(const string& str, puint8 value)
{
    return CConversion::HexCharsToUint8((pchar)str.c_str(), value);
}

/*!-----------------------------------------------------------------------------
Function that contains a sequence of 4 hex characters into a Unit16 value
@param str Pointer the the sequence of characters to convert
@result The value converted from hex characters

uint16 CConversion::HexCharsToUint16(PChar pStr)
{
	uint8 result;
	result = (CConversion::HexCharToUint8(*pStr) << 12);
	pStr++;
	result += (CConversion::HexCharToUint8(*pStr) << 8);
	pStr++;
	result += (CConversion::HexCharToUint8(*pStr) << 4);
	pStr++;
	result += CConversion::HexCharToUint8(*pStr);
	return result;
}
*/

/*!-----------------------------------------------------------------------------

uint16 CConversion::HexCharsToUint16(const string& str)
{
	return CConversion::HexCharsToUint16(str.c_str());
}
*/

/*!-----------------------------------------------------------------------------
Function that contains a sequence of 8 hex characters into a Unit32 value
Assumes values are coded most-sig byte and nibble first.
@param str Pointer the the sequence of characters to convert
@result The value converted from hex characters

uint32 CConversion::HexCharsToUint32(PChar pStr)
{
	uint32 result;
	result = (CConversion::HexCharToUint8(*pStr) << 28);
	pStr++;
	result += (CConversion::HexCharToUint8(*pStr) << 24);
	pStr++;
	result += (CConversion::HexCharToUint8(*pStr) << 20);
	pStr++;
	result += (CConversion::HexCharToUint8(*pStr) << 16);
	pStr++;
	result += (CConversion::HexCharToUint8(*pStr) << 12);
	pStr++;
	result += (CConversion::HexCharToUint8(*pStr) << 8);
	pStr++;
	result += (CConversion::HexCharToUint8(*pStr) << 4);
	pStr++;
	result += CConversion::HexCharToUint8(*pStr);
	return result;
}
*/

/*!-----------------------------------------------------------------------------

uint32 CConversion::HexCharsToUint32(const string& str)
{
	return CConversion::HexCharsToUint32(str.c_str());
}	
*/

//==============================================================================
