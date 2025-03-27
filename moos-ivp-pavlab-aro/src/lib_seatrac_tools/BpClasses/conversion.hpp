/*==============================================================================

Examples of conversion code taken from...
http://stackoverflow.com/questions/200090/how-do-you-convert-a-c-string-to-an-int
http://stackoverflow.com/questions/194465/how-to-parse-a-string-to-an-int-in-c
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef CONVERSION_HPP
#define CONVERSION_HPP

//#include <stdlib.h>
//#include <cerrno>
//#include <climits>

//Include common type definitions and macros
#include "common.h"

//==============================================================================
//Class Definition...
//==============================================================================
/*!
Class that provides methods for converting between data types
*/
class CConversion {	
	private:		
		static char _hexChars[17];
		
	public:
		static char GetHexChar(uint8 value);
		static char GetHexChar(uint32 value, uint8 nibble);
		static bool HexCharToUint8(char chVal, puint8 binVal);
		static bool HexCharsToUint8(pchar buf, puint8 value);
		static bool HexCharsToUint8(const string& str, puint8 value);
		
		//static uint16 HexCharsToUint16(PChar pStr);
		//static uint16 HexCharsToUint16(const string& str);
		//static uint32 HexCharsToUint32(PChar pStr);		
		//static uint32 HexCharsToUint32(const string& str);
};

//==============================================================================
#endif
